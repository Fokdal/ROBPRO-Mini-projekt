#include <Wire.h>
#include <Zumo32U4.h>


Zumo32U4Encoders encoders;
Zumo32U4OLED oled;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

int movementParameter = 0;  // used to select distance or angle

// control the flow of the program. 0 set desired distane
//                                  1 Find line
//                                  2 Allign Zumo
//                                  3 Drive distance
int stage = 0;

int targetCounts;
int targetMovement = 150;

int cLeft;
int cRight;

const int countJump = 5;


const int maxDist = 3000;  //an arbitrary max distance set, in mm.
int mSpeed = 200;          //Zumo driving speed.

const float mm2C_scalar = 7.7;

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void readEncodersParameter() {
  //  movementParameter = TODO;
  movementParameter = movementParameter + encoders.getCountsLeft();
  resetEncoders();
}

void selectParameter() {
  readEncodersParameter();

  if (movementParameter > countJump) {
    movementParameter = 0;
    targetMovement += 5;
    if (targetMovement > maxDist) {
      targetMovement = maxDist;
    }
  } else if (movementParameter < -countJump) {
    movementParameter = 0;
    targetMovement -= 5;
    if (targetMovement < 38) {
      targetMovement = 38;
    }
  }


  printOLED("Dst2Drv:", (String)targetMovement);
  if (buttonA.isPressed()) {
    stage++;
    targetCounts = round(targetMovement * mm2C_scalar);
    buttonA.waitForRelease();
    resetEncoders();
    delay(3000);
  }
}

void driveSetDist() {

  cLeft = encoders.getCountsLeft();
  cRight = encoders.getCountsRight();

  if (cLeft >= targetCounts) {
    motors.setLeftSpeed(0);
  } else if (cLeft > cRight) {
    motors.setLeftSpeed(mSpeed - 50);
  } else {
    motors.setLeftSpeed(mSpeed);
  }

  if (cRight >= targetCounts) {
    motors.setRightSpeed(0);
  } else if (cRight > cLeft) {
    motors.setRightSpeed(mSpeed - 50);
  } else {
    motors.setRightSpeed(mSpeed);
  }

  delay(10);

  delay(10);
  if (cLeft >= targetCounts && cRight >= targetCounts) {
    stage++;
  }
}

void printOLED(String s0, String s1) {
  oled.clear();
  oled.print(s0);
  oled.gotoXY(0, 1);
  oled.print(s1);
}

class Calibrator {
private:
  uint32_t turnAngle = 0;
  const float wheelCirc = 3.7 * 3.14;

  int state = 0;  //State 0: Driving up to line. state 1: drives forward untill both sensors have detected the line. state 2: rotates. state 3: driving back. state 4: finally found line.

  // The following variables are for the linesensor
  static constexpr int lineSensorsAmount = 5;                         //Amount of linesensors
  const int offset = 600;                                             // Light offset between light and dark
  const int calibrationAmount = 200;                                  // Amount of calibrations
  uint16_t calibrationValues[lineSensorsAmount] = { 0, 0, 0, 0, 0 };  // Calibration values when calibrating

  // Different variable for controlling the correction algorithm and
  const int calibrateSpeed = 100;              // Speed when calibrating
  const int maxCalibrationDistance = 50;       // The maximum distance, the robot may move before it just turns a bit.
  const float motorDifference = 38.78 / 38.8;  // There is a small difference in the diameter of each wheel. This is used here to adjust for this behaviour.
  const float angleThreshhold = 0.25;

  float encoderLength = 0;  // Storing the difference in a new variable, so the old one can be used again.
  float angle = 0;          // The predicted angle.
  float error = 0;          // Error for when driving straight.

  // Function returning the difference between the two encoders.
  float getEncoderDifference(int pin) {
    float encoderDifference = 0;
    while (!readLineSensor(pin)) {  // Untill it sees a line.
      encoderDifference += straightLineDriver(0);

      if (encoderDifference > maxCalibrationDistance) {     // If the robot has driven more than 50mm, it stops and
        encoderDifference = maxCalibrationDistance + 48.5;  // 48.5 is added to the distance. This is because it will now drive back to where it with one of the sensors detected a black line and rotate here.

        break;
      }
    }
    return encoderDifference;
  }

  // Function for calibrating. Returns true, when done. It must be called multiple times untill the robot is placed correctly.
  // It will return true constantly untill state = 0, then it will be able to run again and calibrate the robot to a new line.

  // Function converting the encoder value to a distance in mm.
  float calcDistance(float encoderValue) {
    return (encoderValue / 909.7 * 40 * 3.14159);
  }

  // Function for resetting the encoders.
  void resetEncoders() {
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    error = 0;
  }
public:
  int leftSensor = 0;
  int rightSensor = 0;
  int distanceBetweenEachSensor = 89;

  Calibrator(int lS = 0, int rS = 4, int_farptr_t dBES = 89) {
    leftSensor = lS;
    rightSensor = rS;
    distanceBetweenEachSensor = dBES;
  }

  // Calibrates the linesensors.
  bool lineCalibrator() {
    int turnDir = 1;
    switch (state) {
      // Drives forward untill one of the encoders detects a line.
      case 0:
        straightLineDriver(leftSensor);

        if (readLineSensor(leftSensor) || readLineSensor(rightSensor)) {
          resetEncoders();
          state = 1;
        }

        break;

      // Runs untill both encoders have have seen a line, and finds the length difference between the two sensors, to calculate the angle.
      case 1:
        // Gets the difference between the encoders.
        if (readLineSensor(leftSensor)) {
          encoderLength = getEncoderDifference(rightSensor);
          turnDir = -1;  // The direction must be changed to negative.
        } else if (readLineSensor(rightSensor)) {
          encoderLength = getEncoderDifference(leftSensor);
        }

        Serial.println(encoderLength);
        angle = atan((encoderLength) / distanceBetweenEachSensor) * turnDir;  // Calculates the angle. distanceBetweenEachSensor is the distance between the sensors.

        motors.setSpeeds(0, 0);
        resetEncoders();

        // If the angle is in the threshhold.
        if ((angle / 3.14159265 * 180) < angleThreshhold && (angle / 3.14159265 * 180) > -1 * angleThreshhold) {
          resetEncoders();
          resetEncoders();
          state = 4;
        } else {
          state = 2;
        }

        break;

      // State to turn the encoder a specified amount. Depending on the angle. But first it drives a bit, so the center of mass is above the line.
      case 2:
        straightLine(48.5 - encoderLength / 2);
        turnByEncoder(angle);

        state = 3;
        break;

      // Drives back and starts over again.
      case 3:
        straightLine(-75);

        state = 0;

        break;

      // Finally precise enough. Returns true constantly.
      case 4:
        return true;
        break;
    }

    return false;
  }

  void lineSensorCalibrate() {
    uint32_t lineSensorsAmountTotal[lineSensorsAmount] = { 0, 0, 0, 0, 0 };  // Variable to store the value

    for (int i = 0; i < calibrationAmount; i++) {           // First loop. This loops the amount of calibration points which must be taken.
      uint16_t lineSensorValues[lineSensorsAmount];         // Variable to store the values from the linesensor.
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // Reads the line sensor

      for (int j = 0; j < lineSensorsAmount; j++) {  // Loops trough each value from the linesensor reading and adds them to the lineSensorsAmountTotal.
        lineSensorsAmountTotal[j] += lineSensorValues[j];
      }
      delay(10);  //waits 10 ms.
    }

    for (int i = 0; i < lineSensorsAmount; i++) {  // Goes through each value and calculates the average.
      calibrationValues[i] = lineSensorsAmountTotal[i] / calibrationAmount;
      // Prints the final calibration value in the serial monitor.
      Serial.print(calibrationValues[i]);
      Serial.print(", ");
    }
    Serial.println("");
  }

  // Simple function for reading one of the line sensors. The input taken corresponds to the specified linesensor.
  bool readLineSensor(int no) {
    uint16_t lineSensorValues[lineSensorsAmount];         // Defines the variable holding the reading
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // Reads the line sensor

    // Returns true if detected a line, false if not.
    if (lineSensorValues[no] > calibrationValues[no] + offset) {
      return true;
    }
    return false;
  }

  // Driver for driving in a straight line. It takes a distance. Important note, it only uses the distance to determine the direction it must drive.
  // It returns the distance drived between last time the method was called.
  float straightLineDriver(float dist) {
    if (dist == 0) dist = 1;

    //Distance finder
    int dir = dist / abs(dist);
    int16_t encoderLeft = encoders.getCountsAndResetLeft() * dir;
    int16_t encoderRight = encoders.getCountsAndResetRight() * dir;

    error += encoderRight * motorDifference - encoderLeft * 1 / motorDifference;

    motors.setSpeeds(calibrateSpeed * dir + round(error) * dir, calibrateSpeed * dir - round(error) * dir);  // Adjusting depending on the encoders

    float encoderRelativeDist = (encoderLeft + encoderRight) / (2 * 909.7) * 40 * 3.14159 * dir;

    return encoderRelativeDist;
  }

  // Method to drive a specific amount in a straight line.
  void straightLine(float dist) {
    float encoderTotalDist = 0;
    resetEncoders();

    // While loop. It breaks, when the robot has driven the specified amount.
    while (encoderTotalDist < dist && dist > 0 || encoderTotalDist > dist && dist < 0) {
      encoderTotalDist += straightLineDriver(dist);
    }

    motors.setSpeeds(0, 0);
  }

  // Calibration software.
  void resetCalibrationFunction() {
    state = 0;
    resetEncoders();
  }

  // Function to turn the ZUMO using math and
  void turnByEncoder(float degrees) {
    resetEncoders();

    int direction = 1;
    float turnedDist = 0;

    if (degrees < 0) {
      direction = -1;
    }

    // While loop untill the robot has rotated the specified amount.
    while (turnedDist < 85 * 3.14159265 * abs(degrees) / (2 * 3.14159265)) {
      // The closer it is to being finished rotating, the slower it will go to prevent overshooting the specified distance.
      int deacceleration = 35 * 1 / (1 + 85 * 3.14159265 * abs(degrees) / (2 * 3.14159265) - abs(turnedDist));
      int turningSpeed = calibrateSpeed - deacceleration;

      //Turns in place.
      motors.setSpeeds(turningSpeed * direction, -1 * turningSpeed * direction);

      int16_t encoderLeft = encoders.getCountsLeft() * direction;
      int16_t encoderRight = -1 * encoders.getCountsRight() * direction;


      //Uses both encoders to determine the average turn.
      turnedDist = calcDistance((encoderLeft + encoderRight) / 2);
    }
    motors.setSpeeds(0, 0);
  }
};

Calibrator calibratorObj(1, 3, 19);

void setup() {
  lineSensors.initFiveSensors();  // Initialization
}

void loop() {
  delay(50);

  switch (stage) {
    case 0:
      selectParameter();
      break;
    case 1:
      calibratorObj.lineSensorCalibrate();  // Calibrating the sensors
      while (!calibratorObj.lineCalibrator())
        ;
      stage++;
      break;
    case 2:
      driveSetDist();
      break;
    case 3:
      printOLED("Done", ":)");
      delay(5000);
      stage = 0;
      break;
  }
}
