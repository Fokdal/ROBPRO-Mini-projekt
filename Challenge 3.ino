#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

int speed = 220;

uint16_t customBrightnessLevels[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

void setup() {
  proxSensors.initThreeSensors();

  proxSensors.setBrightnessLevels(customBrightnessLevels, sizeof(customBrightnessLevels) / sizeof(customBrightnessLevels[0]));

  Serial.begin(9600);
}

void loop() {
    proxSensors.read();
    Serial.print("Left: ");
    Serial.print(proxSensors.countsLeftWithLeftLeds());
    Serial.print("  Right: ");
    Serial.print(proxSensors.countsRightWithRightLeds());
    Serial.println();

    if (proxSensors.countsLeftWithLeftLeds() > proxSensors.countsRightWithRightLeds()) {
      motors.setSpeeds(speed * int(float((proxSensors.countsLeftWithLeftLeds()) / float(proxSensors.countsRightWithRightLeds()))), 60 + speed * 4 * int(float((proxSensors.countsRightWithRightLeds()) / float(proxSensors.countsLeftWithLeftLeds()/1.4))));
    }
    else if (proxSensors.countsLeftWithLeftLeds() < proxSensors.countsRightWithRightLeds()) {
      motors.setSpeeds(60 + speed * 4 * int(float((proxSensors.countsLeftWithLeftLeds()) / float(proxSensors.countsRightWithRightLeds()/1.4))), speed * int(float((proxSensors.countsRightWithRightLeds()) / float(proxSensors.countsLeftWithLeftLeds()))));
    }
}

