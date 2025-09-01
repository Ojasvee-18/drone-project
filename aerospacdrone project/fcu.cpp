//  Aerospace Drone Flight Control Unit (FCU) Code
//  Implements a basic PID controller for a quadcopter using MPU6050 IMU and PWM receiver inputs.
//  Uses ESP32 microcontroller and ESP32Servo library for motor control.
//  Check implementation details and tuning comments in the code. 

#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// Motor pins
const int mot1_pin = 13; // front right (CCW)
const int mot2_pin = 12; // rear right (CW)
const int mot3_pin = 14; // rear left (CCW)
const int mot4_pin = 27; // front left (CW)

// Receiver pins for channels 1 to 6
const int channelPins[6] = {34, 35, 32, 33, 25, 26};

// Servo objects for motors
Servo mot1, mot2, mot3, mot4;

// Receiver pulse width inputs (us)
volatile int ReceiverValue[6] = {1000, 1000, 1000, 1000, 1000, 1000};
volatile uint32_t timer[6];
volatile int lastChannelState[6] = {0,0,0,0,0,0};

// Timing
uint32_t loopTimer;
const float dt = 0.004; // 4 ms loop time

// Complementary filter variables (roll, pitch)
float angleRoll = 0, anglePitch = 0;
float gyroRollRate = 0, gyroPitchRate = 0, gyroYawRate = 0;
float accX = 0, accY = 0, accZ = 0;

// Calibration offsets (adjust based on MPU) Use MPU6050 calibration values
const float gyroOffsetRoll = 0.27;
const float gyroOffsetPitch = -0.85;
const float gyroOffsetYaw = -2.09;
const float accOffsetX = 0.03;
const float accOffsetY = 0.01;
const float accOffsetZ = -0.07;

// PID gains (tune these experimentally)
float PAngleRoll=2.0, IAngleRoll=0.5, DAngleRoll=0.007;
float PAnglePitch=2.0, IAnglePitch=0.5, DAnglePitch=0.007;

float PRateRoll = 0.625, IRateRoll = 2.1, DRateRoll = 0.0088;
float PRatePitch = 0.625, IRatePitch = 2.1, DRatePitch = 0.0088;
float PRateYaw = 4.0, IRateYaw = 3.0, DRateYaw = 0;

// PID state variables
float errorAngleRoll=0, prevErrorAngleRoll=0, integralAngleRoll=0;
float errorAnglePitch=0, prevErrorAnglePitch=0, integralAnglePitch=0;

float errorRateRoll=0, prevErrorRateRoll=0, integralRateRoll=0;
float errorRatePitch=0, prevErrorRatePitch=0, integralRatePitch=0;
float errorRateYaw=0, prevErrorRateYaw=0, integralRateYaw=0;

// Limits
const int ThrottleMin = 1000;
const int ThrottleMax = 1800;
const int ThrottleIdle = 1100;
const int ThrottleCutOff = 1000;

// Motor PWM Outputs (us)
float motorOut[4] = {ThrottleIdle, ThrottleIdle, ThrottleIdle, ThrottleIdle};

// INTERRUPT HANDLER for receiver PWM inputs
void IRAM_ATTR channelInterruptHandler(int ch) {
  uint32_t currTime = micros();
  if (digitalRead(channelPins[ch])) {
    if (lastChannelState[ch] == 0) {
      lastChannelState[ch] = 1;
      timer[ch] = currTime;
    }
  } else if (lastChannelState[ch] == 1) {
    lastChannelState[ch] = 0;
    int pulseWidth = currTime - timer[ch];
    // Basic sanity check on pulse width range (900 - 2100 us)
    if (pulseWidth > 850 && pulseWidth < 2200) {
      ReceiverValue[ch] = pulseWidth;
    }
  }
}

// Define 6 ISR functions for each channel
void IRAM_ATTR channelInterruptHandler0() { channelInterruptHandler(0); }
void IRAM_ATTR channelInterruptHandler1() { channelInterruptHandler(1); }
void IRAM_ATTR channelInterruptHandler2() { channelInterruptHandler(2); }
void IRAM_ATTR channelInterruptHandler3() { channelInterruptHandler(3); }
void IRAM_ATTR channelInterruptHandler4() { channelInterruptHandler(4); }
void IRAM_ATTR channelInterruptHandler5() { channelInterruptHandler(5); }

// Setup function
void setup() {
  Serial.begin(115200);

  // Attach motor pins to servo objects
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);

  // Set receiver pins and attach interrupts for each channel
  pinMode(channelPins[0], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPins[0]), channelInterruptHandler0, CHANGE);
  pinMode(channelPins[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPins[1]), channelInterruptHandler1, CHANGE);
  pinMode(channelPins[2], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPins[2]), channelInterruptHandler2, CHANGE);
  pinMode(channelPins[3], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPins[3]), channelInterruptHandler3, CHANGE);
  pinMode(channelPins[4], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPins[4]), channelInterruptHandler4, CHANGE);
  pinMode(channelPins[5], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(channelPins[5]), channelInterruptHandler5, CHANGE);

  // Initialize MPU6050
  Wire.begin();
  delay(100);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission();

  // Initialize gyro and accel config registers
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Gyro config
  Wire.write(0x08); // ±500 deg/sec full scale
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Accel config
  Wire.write(0x10); // ±8g full scale
  Wire.endTransmission();

  loopTimer = micros();

  // Set all motors to idle at startup
  for (int i = 0; i < 4; i++) {
    motorOut[i] = ThrottleIdle;
  }
  mot1.writeMicroseconds(ThrottleIdle);
  mot2.writeMicroseconds(ThrottleIdle);
  mot3.writeMicroseconds(ThrottleIdle);
  mot4.writeMicroseconds(ThrottleIdle);

  delay(2000);
}

// Function to read MPU6050 raw data and update gyro and accel variables
void readMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();

  int16_t rawTemp = Wire.read() << 8 | Wire.read(); // Not used

  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw to float values with scale and apply calibration offsets
  accX = (float)rawAccX / 4096.0f - accOffsetX;
  accY = (float)rawAccY / 4096.0f - accOffsetY;
  accZ = (float)rawAccZ / 4096.0f - accOffsetZ;

  gyroRollRate = (float)rawGyroX / 65.5f - gyroOffsetRoll;   // deg/s
  gyroPitchRate = (float)rawGyroY / 65.5f - gyroOffsetPitch; // deg/s
  gyroYawRate = (float)rawGyroZ / 65.5f - gyroOffsetYaw;     // deg/s
}

// Complementary filter update for roll and pitch
void updateAngles() {
  // Calculate accelerometer angles (degrees)
  float angleAccRoll = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 57.2958f;
  float angleAccPitch = -atan2(accX, sqrt(accY * accY + accZ * accZ)) * 57.2958f;

  // Apply complementary filter
  const float alpha = 0.98f;
  angleRoll = alpha * (angleRoll + gyroRollRate * dt) + (1 - alpha) * angleAccRoll;
  anglePitch = alpha * (anglePitch + gyroPitchRate * dt) + (1 - alpha) * angleAccPitch;

  // Limit angles to +/- 20 degrees for safety
  if (angleRoll > 20) angleRoll = 20;
  else if (angleRoll < -20) angleRoll = -20;
  if (anglePitch > 20) anglePitch = 20;
  else if (anglePitch < -20) anglePitch = -20;
}

// PID helper function
float pid(float error, float *prevError, float *integral, float P, float I, float D) {
  *integral += error * dt;
  float derivative = (error - *prevError) / dt;
  *prevError = error;

  // Anti-windup clamp on integral
  if (*integral > 400) *integral = 400;
  else if (*integral < -400) *integral = -400;

  float output = P * error + I * (*integral) + D * derivative;

  // Clamp output
  if (output > 400) output = 400;
  else if (output < -400) output = -400;

  return output;
}

void loop() {
  // Maintain fixed loop timing
  while (micros() - loopTimer < dt * 1000000);
  loopTimer = micros();

  // Read sensors
  readMPU();
  updateAngles();

  // Read receiver inputs and map throttle from 1000-1800us to 0-700us range over ThrottleIdle
  int throttleRaw = ReceiverValue[2];
  if (throttleRaw < ThrottleCutOff) throttleRaw = ThrottleCutOff;
  if (throttleRaw > ThrottleMax) throttleRaw = ThrottleMax;
  float throttle = map(throttleRaw, ThrottleCutOff, ThrottleMax, 0, ThrottleMax - ThrottleIdle);

  // Desired angles from stick input (channels 0 and 1)
  float desiredAngleRoll = 0.1f * (ReceiverValue[0] - 1500);
  float desiredAnglePitch = 0.1f * (ReceiverValue[1] - 1500);
  // Desired yaw rate from stick input (channel 3)
  float desiredYawRate = 0.15f * (ReceiverValue[3] - 1500);

  // Angle PID for roll and pitch (outer loop)
  errorAngleRoll = desiredAngleRoll - angleRoll;
  float pidAngleRoll = pid(errorAngleRoll, &prevErrorAngleRoll, &integralAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll);

  errorAnglePitch = desiredAnglePitch - anglePitch;
  float pidAnglePitch = pid(errorAnglePitch, &prevErrorAnglePitch, &integralAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch);

  // Desired rates are outputs of angle PID
  float desiredRateRoll = pidAngleRoll;
  float desiredRatePitch = pidAnglePitch;

  // Rate PID for roll, pitch, and yaw (inner loop)
  errorRateRoll = desiredRateRoll - gyroRollRate;
  float pidRateRoll = pid(errorRateRoll, &prevErrorRateRoll, &integralRateRoll, PRateRoll, IRateRoll, DRateRoll);

  errorRatePitch = desiredRatePitch - gyroPitchRate;
  float pidRatePitch = pid(errorRatePitch, &prevErrorRatePitch, &integralRatePitch, PRatePitch, IRatePitch, DRatePitch);

  errorRateYaw = desiredYawRate - gyroYawRate;
  float pidRateYaw = pid(errorRateYaw, &prevErrorRateYaw, &integralRateYaw, PRateYaw, IRateYaw, DRateYaw);

  // Calculate motor mixes
  // Motor order and directions:
  // mot1: front right (CCW)
  // mot2: rear right (CW)
  // mot3: rear left (CCW)
  // mot4: front left (CW)
  motorOut[0] = ThrottleIdle + throttle - pidRateRoll - pidRatePitch - pidRateYaw; // mot1
  motorOut[1] = ThrottleIdle + throttle - pidRateRoll + pidRatePitch + pidRateYaw; // mot2
  motorOut[2] = ThrottleIdle + throttle + pidRateRoll + pidRatePitch - pidRateYaw; // mot3
  motorOut[3] = ThrottleIdle + throttle + pidRateRoll - pidRatePitch + pidRateYaw; // mot4

  // Clamp motor outputs to valid range
  for (int i = 0; i < 4; i++) {
    if (motorOut[i] < ThrottleIdle) motorOut[i] = ThrottleIdle;
    if (motorOut[i] > 2000) motorOut[i] = 2000;
  }

  // Disarm motors if throttle is below cutoff
  if (ReceiverValue[2] < ThrottleCutOff) {
    for (int i = 0; i < 4; i++) motorOut[i] = ThrottleCutOff;
    // Reset PID integrals to avoid wind-up
    integralAngleRoll = 0; integralAnglePitch = 0;
    integralRateRoll = 0; integralRatePitch = 0; integralRateYaw = 0;
    prevErrorAngleRoll = 0; prevErrorAnglePitch = 0;
    prevErrorRateRoll = 0; prevErrorRatePitch = 0; prevErrorRateYaw = 0;
  }

  // Write motor outputs to ESCs
  mot1.writeMicroseconds((int)motorOut[0]);
  mot2.writeMicroseconds((int)motorOut[1]);
  mot3.writeMicroseconds((int)motorOut[2]);
  mot4.writeMicroseconds((int)motorOut[3]);

  // Optional: Debug printing - uncomment if needed
  /*
  Serial.print("Throttle:");
  Serial.print(throttle);
  Serial.print(" | Roll:");
  Serial.print(angleRoll);
  Serial.print(" | Pitch:");
  Serial.print(anglePitch);
  Serial.print(" | Mot1:");
  Serial.print((int)motorOut[0]);
  Serial.print(" Mot2:");
  Serial.print((int)motorOut[1]);
  Serial.print(" Mot3:");
  Serial.print((int)motorOut[2]);
  Serial.print(" Mot4:");
  Serial.println((int)motorOut[3]);
  */
}
