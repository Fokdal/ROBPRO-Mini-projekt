/*
Turn the desired angle, and stop at a line. 

Programmed by:
 - Rasmus Dueholm Kristiansen

*/
#include <Wire.h>
#include <Zumo32U4.h>


Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;

/*
 Class for calibrating the robot to a line. 
 The class is created such as it can be copy pasted alone into any programme and work. 
 The only thing which needs to be done, is to initialize the line sensors and defining the motors as an object called motors.
*/
class Calibrator {
  public:
    uint32_t turnAngle = 0;
    const float wheelCirc = 3.7*3.14;

    int state = 0;                                                              //State 0: Driving up to line. state 1: drives forward untill both sensors have detected the line. state 2: rotates. state 3: driving back. state 4: finally found line.
    
    // The following variables are for the linesensor 
    static const int lineSensorsAmount = 5;                                     // Amount of linesensors
    const int offset = 600;                                                     // Light offset between light and dark
    const int calibrationAmount = 200;                                          // Amount of calibrations
    const int distanceAfterCalibration = 25;                                    // Distance the robot drives forward after the calibration is done
    uint16_t calibrationValues[lineSensorsAmount] = {0, 0, 0, 0, 0};            // Calibration values when calibrating

    // Different variable for controlling the correction algorithm and 
    const int calibrateSpeed = 100;                                             // Speed when calibrating
    const int maxCalibrationDistance = 50;                                      // The maximum distance, the robot may move before it just turns a bit.
    const float motorDifference = 38.78/38.8;                                   // There is a small difference in the diameter of each wheel. This is used here to adjust for this behaviour.
    const float angleThreshhold = 0.25;                                         // Threshhold on how precise the robot needs to be before it may continue to move.

    float encoderLength = 0;                                                    // Storing the difference in a new variable, so the old one can be used again.
    float angle = 0;                                                            // The predicted angle.
    float error = 0;                                                            // Error for when driving straight.

    int leftSensor = 0;                                                         // Left sensor number
    int rightSensor = 4;                                                        // Right sensor number
    int distanceBetweenEachSensor = 89;                                         // Distance between the sensors

    // Constructor called when the class is created. The different variables are sat in this class.
    Calibrator(int lS = 0, int rS = 4, int_farptr_t dBES = 89) {
      leftSensor = lS;
      rightSensor = rS;
      distanceBetweenEachSensor = dBES;
    }

    // Function returning the difference between the two encoders.
    float getEncoderDifference(int pin) {
      float encoderDifference = 0;
      while (!readLineSensor(pin)) {                                            // Untill it sees a line.
        encoderDifference += straightLineDriver(0);

        if (encoderDifference > maxCalibrationDistance) {                       // If the robot has driven more than 50mm, it stops and
          encoderDifference = maxCalibrationDistance + 48.5;                    // 48.5 is added to the distance. This is because it will now drive back to where it with one of the sensors detected a black line and rotate here. 

          break;
        }
      }
      return encoderDifference;
    }

    // Function for calibrating. Returns true, when done. It must be called multiple times untill the robot is placed correctly.
    // It will return true constantly untill state = 0, then it will be able to run again and calibrate the robot to a new line.

    // Function converting the encoder value to a distance in mm.
    float calcDistance(float encoderValue) {
      return (encoderValue/909.7 * 40 * 3.14159);
    }

    // Function for resetting the encoders.
    void resetEncoders() {
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      error = 0;
    }

    // Calibrates the linesensors.
    bool lineCalibrator() {
      int turnDir = 1;

      switch(state) {
        // Drives forward untill one of the encoders detects a line.
        case 0:
          straightLineDriver(leftSensor);
          
          // Detects a line:
          if (readLineSensor(leftSensor) || readLineSensor(rightSensor)) {
            resetEncoders();
            state = 1;
          } 
          
          break;
        
        // Runs untill both encoders have have seen a line, and finds the length difference between the two sensors, to calculate the angle. Or Runs untill the robot has driven far enough.
        case 1:
          // Gets the difference between the encoders.
          if (readLineSensor(leftSensor)) {
            encoderLength = getEncoderDifference(rightSensor);
            turnDir = -1;                                                       // The direction must be changed to negative when turning one way around.

          } else if (readLineSensor(rightSensor)) {
            encoderLength = getEncoderDifference(leftSensor);
          }
          
          // Calculates the angle. distanceBetweenEachSensor is the distance between the sensors.
          angle = atan((encoderLength) / distanceBetweenEachSensor) * turnDir; 

          motors.setSpeeds(0, 0);
          resetEncoders();
          
          // If the angle is in the threshhold. The angle is also changed from being in radians to degreese. 
          if ((angle / 3.14159265 * 180) < angleThreshhold && (angle / 3.14159265 * 180) > -1 * angleThreshhold) {
            straightLine(distanceAfterCalibration);                             // Drives in a straight line for a specified distance, making it easy if a linedetector needs to be used just after calibration. 
            resetEncoders(); 
            
            state = 4;
          } else {
            state = 2;
          }

          break;

        // State to turn the encoder a specified amount. Depending on the angle. But first it drives a bit, so the center of mass is above the line.
        case 2:
          straightLine(48.5 - encoderLength / 2);                               // Calculates the distance, the robot needs to drive, so it is exactly above the line.
          turnByEncoder(angle);                                                 // Turns the calculated amount of degreese.

          state = 3;
          break;

        // Drives back and starts over again.
        case 3:
          straightLine(-75);                                    
          
          state = 0;
          
          break;
        
        // Finally precise enough. Returns true constantly, making this function ideal to put at the start of the loop in a while loop.
        case 4:
          return true;
          break;
      }

      return false;
    }

    // Function to calibrate the linesensors to the enviornment. The linesensors must be over the normal floor before the line it needs to detect.
    void lineSensorCalibrate() {
      uint32_t lineSensorsAmountTotal[lineSensorsAmount];                       // Variable to store the value

      for (int i = 0; i < calibrationAmount; i++) {                             // First loop. This loops the amount of calibration points which must be taken.
        uint16_t lineSensorValues[lineSensorsAmount];                           // Variable to store the values from the linesensor.
        lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);                    // Reads the line sensor
        
        for (int j = 0; j < lineSensorsAmount; j++) {                           // Loops trough each value from the linesensor reading and adds them to the lineSensorsAmountTotal.
          lineSensorsAmountTotal[j] += lineSensorValues[j];
        }
      }

      for (int i = 0; i < lineSensorsAmount; i++) {                             // Goes through each value and calculates the average.
        calibrationValues[i] = lineSensorsAmountTotal[i] / calibrationAmount;   // Calculating the average calibration value.
      }
    }

    // Simple function for reading one of the line sensors. The input taken corresponds to the specified linesensor.
    bool readLineSensor(int no) {
      uint16_t lineSensorValues[lineSensorsAmount];                             // Defines the variable holding the reading
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);                      // Reads the line sensor

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
      int dir = dist/abs(dist);
      int16_t encoderLeft = encoders.getCountsAndResetLeft() * dir; 
      int16_t encoderRight = encoders.getCountsAndResetRight() * dir;

      error += encoderRight * motorDifference - encoderLeft * 1/motorDifference;

      // Adjusting depending on the encoders to drive straight.
      int motorLeft = calibrateSpeed * dir + round(error) * dir;
      int motorRight = calibrateSpeed * dir - round(error) * dir;

      motors.setSpeeds(motorLeft, motorRight); 

      float encoderRelativeDist = (encoderLeft + encoderRight)/(2 * 909.7) * 40 * 3.14159 * dir; 
      
      return encoderRelativeDist;
    }

    // Method to drive a specific amount in a straight line.
    void straightLine(float dist) {
      resetEncoders();

      float encoderTotalDist = 0;
      
      // While loop. It breaks, when the robot has driven the specified amount. The if statement is both for the negative and postive values.
      while (encoderTotalDist < dist && dist > 0 || encoderTotalDist > dist && dist < 0) {
        encoderTotalDist += straightLineDriver(dist);
      }

      motors.setSpeeds(0,0);
      resetEncoders(); 
    }
    
    // Resetting the calibration software.
    void resetCalibrationFunction() {
      state = 0;
      resetEncoders();
    }

    // Function to turn the ZUMO using math and 
    void turnByEncoder(float degrees) {
      resetEncoders();
      
      // Direction and the distance already turned.
      int direction = 1;                                                      
      float turnedDist = 0;
    
      if (degrees < 0) {
        direction = -1;
      }

      // While loop untill the robot has rotated the specified amount. 
      while (turnedDist < 85 * 3.14159265 * abs(degrees) / (2 * 3.14159265)) {
        // The closer it is to being finished rotating, the slower it will go to prevent overshooting the specified distance.
        int deacceleration = 35 / (1 + 85 * 3.14159265 * abs(degrees)) / (2 * 3.14159265 - abs(turnedDist));
        int turningSpeed = calibrateSpeed - deacceleration;

        //Turns in place.
        motors.setSpeeds(turningSpeed * direction, -1 * turningSpeed * direction);
        
        // Gets how far each encoder has turned.
        int16_t encoderLeft = encoders.getCountsLeft() * direction;
        int16_t encoderRight = -1 * encoders.getCountsRight() * direction;


        //Uses both encoders to determine the average turn.
        turnedDist = calcDistance((encoderLeft + encoderRight)/2);
      }
      motors.setSpeeds(0, 0);
      resetEncoders(); 
    }
};

/*
The main code. A lot of the functions which is used, is from the class above.
*/

Calibrator calibratorObj(1, 3, 19);                                             // Defining the object used for calibrating the robot.

const int programmeLength = 3;

const int calibrationAmount = 200;                                              // Amount of calibrations
const int offset = 600;                                                         // Difference between the calibrated surface and the line.

const int StraightLineSpeed = 200;
const int lineSensorsAmount = calibratorObj.lineSensorsAmount;

int state = 0;                                                                  // Defined from the programme. 0: Drive straigt, 1: Turn, 2: Drive untill a black line is detected.

int16_t programmeDescription[programmeLength][2] = {{0, 45},{1, 40},{2, 0}};    //For each programme {programme, value}, where programme is describing the state.


void setup() {
  lineSensors.initFiveSensors();                                                // Initialization 
  calibratorObj.lineSensorCalibrate();                                          // Calibrates the linesensors in the class
  while (!calibratorObj.lineCalibrator());                                      // Runs untill the robot has calibrated itself to the line
}

// Loop function. Has the task of running the programme. 
void loop() {
  runProgramme();
}

// Function for running the programme. 
void runProgramme() {
  for (int i = 0; i < programmeLength; i++) {
    switch(programmeDescription[i][0]) {
      case 0:
        calibratorObj.straightLine(programmeDescription[i][1]);                 // As the class calibratorObj already have a build in straightLine, this is used.
        break;

      case 1:
        calibratorObj.turnByEncoder(programmeDescription[i][1]);                // As the class calibratorObj already have a build in turner, this is used.
        break;
      case 2:
        if (!blackTapeRegistered) {
          calibratorObj.straightLineDriver(1);                                  // A function which continiously needs to be called to drive the robot forward.
        } else {
          motors.setSpeeds(0, 0);
        }
        break;
    }
  } 
}

// Function to determine if any of the linesensors detect a black line.
bool blackTapeRegistered(uint16_t lineSensorValues[lineSensorsAmount]) {
  for (int i = 0; i < lineSensorsAmount; i++) {                                 // Looping through the line sensors and seeing if they are detecting a line. 
    if (calibratorObj.readLineSensor(i)) {
      return true;
    }
  }
  return false;
}
