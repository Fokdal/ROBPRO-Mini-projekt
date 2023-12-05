#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

int speed = 220;

//Define custom brightness levels for increased precision of proximity sensors
uint16_t customBrightnessLevels[] = {0, 1, 2, 3, 4, 5, 6};

void setup() {
  proxSensors.initThreeSensors();

//set new brightness levels 
  proxSensors.setBrightnessLevels(customBrightnessLevels, sizeof(customBrightnessLevels) / sizeof(customBrightnessLevels[0]));

  Serial.begin(9600);
}

//experimentally found adjustment factor for accuracy
const float leftSensorAdjustmentFactor = 1.4;

//base motor speeds for motor adjustments
const int baseMotorSpeed = 60;

//speed adjustment factor for. Amplifies effect of sensor ratios on motor speeds
const int speedAdjustmentFactor = 4;

void loop() {
    proxSensors.read();
    Serial.print("Left: ");
    Serial.print(proxSensors.countsLeftWithLeftLeds());
    Serial.print("  Right: ");
    Serial.print(proxSensors.countsRightWithRightLeds());

    int leftSensorCount = proxSensors.countsLeftWithLeftLeds();
    int rightSensorCount = proxSensors.countsRightWithRightLeds();

    if (leftSensorCount > rightSensorCount) {
      // If left sensor count is higher, adjust motor speeds accordingly
      motors.setSpeeds(speed * int(float(leftSensorCount) / float(rightSensorCount)), baseMotorSpeed + speed * speedAdjustmentFactor * int(float((rightSensorCount) / float(leftSensorCount / leftSensorAdjustmentFactor))));
    }
    else if (leftSensorCount < rightSensorCount) {
      // If right sensor count is higher, adjust motor speeds accordingly
      motors.setSpeeds(baseMotorSpeed + speed * speedAdjustmentFactor * int(float((leftSensorCount) / float(rightSensorCount / leftSensorAdjustmentFactor))), speed * int(float((rightSensorCount) / leftSensorCount))));
    }
}

