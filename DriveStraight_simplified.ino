#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4IMU imu;

uint32_t turnAngle = 0;

// Variables related to gyro
uint16_t gyroOffset;
int16_t turnRate;
uint16_t gyroLastUpdate = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  gyroOffset = total / 1024;

  // Reset gyro
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void loop() {
  driveStraight();
}

void driveStraight() {
    // update turn angle
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 14680064 / 17578125;

  // Calculate correction based on turn angle
    // Convert turn angle to degrees
  int32_t turnDegrees = (((int32_t)turnAngle >> 16) * 360) >> 16;
  int16_t correction = 5 * turnDegrees;


  // Adjust motors speeds to drive straight
  int leftSpeed = 200 + correction;
  int rightSpeed = 200 - correction;

  motors.setSpeeds(leftSpeed, rightSpeed);
}
