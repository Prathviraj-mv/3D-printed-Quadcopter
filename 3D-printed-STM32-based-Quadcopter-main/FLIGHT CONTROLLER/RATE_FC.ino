#include <Wire.h>
#include <Servo.h>
#define NUM_CHANNELS 4  // Channels for Throttle, Roll, Pitch, Yaw
#define NUM_MOTORS 4    // Motors M1, M2, M3, M4
#define LED PC13

Servo esc1, esc2, esc3, esc4;

// Roll PID variables
float rollAngle;
float roll_error, roll_prev_error = 0, roll_integral = 0, roll_derivative, roll_output;
float Kp_roll = 2.5, Ki_roll = 0.00, Kd_roll = 0.02;

// Pitch PID variables
float pitchAngle;
float pitch_error, pitch_prev_error = 0, pitch_integral = 0, pitch_derivative, pitch_output;
float Kp_pitch = 2.5, Ki_pitch = 0.00, Kd_pitch = 0.02;

unsigned long prevTime = 0;

// Channel Pins (Receiver PWM Inputs)
const int pwmPins[NUM_CHANNELS] = { PB8, PB13, PB9, PA7 };
int motor1, motor2, motor3, motor4;
// ESC Output Pins
const int motorPins[NUM_MOTORS] = { PA0, PA1, PB0, PA6 };

// Variables for timing
volatile unsigned long riseTime[NUM_CHANNELS];
volatile unsigned long pwmValues[NUM_CHANNELS];
volatile bool newSignal[NUM_CHANNELS] = { false };

unsigned long previousTime = 0;
float dt;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
int RateCalibrationNumber = 0;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}



void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  esc1.attach(PA0);
  esc2.attach(PA1);
  esc3.attach(PB0);
  esc4.attach(PA6);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Serial.println("Calibrating Gyro...");
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;   
  Serial.println("Calibration done.");

  previousTime = micros();

  
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(pwmPins[i], INPUT);
    attachInterrupt(
      digitalPinToInterrupt(pwmPins[i]), [i]() {
        risingEdge(i);
      },
      RISING);
  }
  
  for (int i = 0; i < NUM_CHANNELS; i++) {
    pinMode(pwmPins[i], INPUT);
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  // Arm all ESCs: send minimum throttle signal
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(500);
  digitalWrite(LED, HIGH);  // Turn the LED on
  delay(500);  // Wait for ESCs to arm (3 seconds)
  digitalWrite(LED, LOW);  // Turn the LED on
  delay(500);
  digitalWrite(LED, HIGH);  // Turn the LED on
  delay(500);  // Wait for ESCs to arm (3 seconds)
  digitalWrite(LED, LOW);  // Turn the LED on
  delay(500);
  digitalWrite(LED, HIGH);  // Turn the LED on
  delay(500);  // Wait for ESCs to arm (3 seconds)
  digitalWrite(LED, LOW);  // Turn the LED on

  // Spin all motors slowly for 2 seconds
  esc1.writeMicroseconds(1200);
  esc2.writeMicroseconds(1200);
  esc3.writeMicroseconds(1250);
  esc4.writeMicroseconds(1250);
  delay(1000);
  digitalWrite(LED, HIGH);  // Turn the LED on

  // Stop motors
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(1000);
  digitalWrite(LED, LOW);  // Turn the LED on
}

void writeMotor(int motorPin, int value) {
  // Example call: esc1.writeMicroseconds(value);
  // You’ll need a switch-case or if-else:
  if (motorPin == PA0) esc1.writeMicroseconds(value);
  else if (motorPin == PA1) esc2.writeMicroseconds(value);
  else if (motorPin == PB0) esc3.writeMicroseconds(value);
  else if (motorPin == PA6) esc4.writeMicroseconds(value);
}

void computePID() {
  unsigned long now = micros();
  float dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  roll_error = rollAngle - RateRoll;
  roll_integral += roll_error * dt;
  roll_derivative = (roll_error - roll_prev_error) / dt;
  roll_output = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative;
  roll_prev_error = roll_error;

  pitch_error = pitchAngle - RatePitch;
  pitch_integral += pitch_error * dt;
  pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  pitch_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative;
  pitch_prev_error = pitch_error;
}
void risingEdge(int channel) {
  riseTime[channel] = micros();
  attachInterrupt(
    digitalPinToInterrupt(pwmPins[channel]), [channel]() {
      fallingEdge(channel);
    },
    FALLING);
}

void fallingEdge(int channel) {
  pwmValues[channel] = micros() - riseTime[channel];
  newSignal[channel] = true;
  attachInterrupt(
    digitalPinToInterrupt(pwmPins[channel]), [channel]() {
      risingEdge(channel);
    },
    RISING);
}
void motorMixing(int throttle) {
  motor3 = throttle + pitch_output + roll_output;
  motor1 = throttle + pitch_output - roll_output;
  motor2 = throttle - pitch_output - roll_output;
  motor4 = throttle - pitch_output + roll_output;

  motor1 = constrain(motor1, 1100, 1900);
  motor2 = constrain(motor2, 1100, 1900);
  motor3 = constrain(motor3, 1100, 1900);
  motor4 = constrain(motor4, 1100, 1900);

  int ThrottleCutOff=1000;
  if (throttle<1150) {
    motor1=ThrottleCutOff; 
    motor2=ThrottleCutOff;
    motor3=ThrottleCutOff; 
    motor4=ThrottleCutOff;

  }
    // Write PWM signals to ESC pins
  writeMotor(motorPins[0], motor1);
  writeMotor(motorPins[1], motor2);
  writeMotor(motorPins[2], motor3);
  writeMotor(motorPins[3], motor4);
}

void loop() {
  digitalWrite(LED, HIGH);

  unsigned long currentTime = micros();
  dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  int throttle = map(pwmValues[2], 1100, 2000, 1100, 2000);
  int roll = pwmValues[0];
  int pitch = pwmValues[1];
  int yaw = pwmValues[3];

  rollAngle = ((float)(roll - 1500) / 500.0) * 30.0;
  pitchAngle = ((float)(pitch - 1500) / 500.0) * 30.0;
  float yawRate = ((float)(yaw - 1500) / 500.0) * 30.0;

  computePID();
  motorMixing(throttle);

  Serial.print("RateRoll: ");
  Serial.print(RateRoll);
  Serial.print(" | RatePitch: ");
  Serial.print(RatePitch);
  Serial.print(" | RateYaw: ");
  Serial.print(RateYaw);
  Serial.print(" | Throttle: ");
  Serial.print(throttle);
  Serial.print(" | Roll Angle: ");
  Serial.print(rollAngle, 2);
  Serial.print(" | Pitch Angle: ");
  Serial.print(pitchAngle, 2);
  Serial.print(" | Yaw Rate: ");
  Serial.println(yawRate, 2);
  Serial.print(" | motor1: ");
  Serial.print(motor1);
  Serial.print(" | motor2: ");
  Serial.print(motor2);
  Serial.print(" | motor3: ");
  Serial.print(motor3);
  Serial.print(" | motor4: ");
  Serial.println(motor4);
}
