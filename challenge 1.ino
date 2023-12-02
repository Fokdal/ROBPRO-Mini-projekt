#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED oled;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

int movementCommand = 0;
int movementParameter = 0;

const int countJump = 100;
const int maxDist = 100;
int speed = 100;
float wheelCirc = 13.0;
int stage = 0;
int chosenCommand = 0;

uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  switch (stage) {
    case 0:
    selectParameter();
    break;
    case 1:
    executeMovement();
    break;
  }
  delay(50);
}

void selectParameter() {
  readEncodersParameter();
  switch (chosenCommand) {
    case 0: 
      if (movementParameter < 0) {
        movementParameter = 0;
      } else if (movementParameter > maxDist) {
        movementParameter = maxDist;
      }
      OLEDSelectParameter();
      if (buttonA.isPressed()) {
        stage = 1;
        buttonA.waitForRelease(); 
      }
      break;
  }
}

void executeMovement() {
  switch (chosenCommand) {
    case 0:
      while (getDistance() < movementParameter) {
        forward();
        delay(100);
      }
      stop();
      movementCommand = 0;
      movementParameter = 0;
      stage = 0;
      chosenCommand = 0;
      break;
  }
 
}

void stop() {
  motors.setSpeeds(0, 0);
}

void forward() {
  motors.setSpeeds(speed, speed);
}

void OLEDSelectMovement() {
  switch (stage) {
    case 0:
      printOLED("Command>", "Forward");
      break;
  }
}

void OLEDSelectParameter() {
  switch (stage) {
    case 0 :
    printOLED("<Distance:", String(movementParameter));
    break;
    case 1:
     printOLED("<Distance:", String(getDistance()));

  }
}

void printOLED(String s0, String s1) {
  oled.clear();
  oled.print(s0);
  oled.gotoXY(0, 1);
  oled.print(s1);
}

float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL / 900.0 * wheelCirc;
  float distanceR = countsR / 900.0 * wheelCirc;

  return (distanceL + distanceR) / 2;
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}
