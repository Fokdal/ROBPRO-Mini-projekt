/*
Line sensor following programme for ZUMO 32u4
*/


#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

const int lineSensorsAmount = 5; //Amount of linesensors
const float exponentFactor = 4.5; //For calculating how much the robot is off path, this formular is used: ax^n where this variable corresponds to n.
const float multiplier = 1.5; //Original 1.5 For calculating how much the robot is off path, this formular is used: ax^n where this variable corresponds to a.

const int maxSpeed = 400; // Maxiumum speed for the robot
const int maxSpeedWhenNoSensors = 400; //Maximum speed when no sensor is seeing a path (hopefully only happens, when the sensor is off path.)
const int minSpeed = 100; //Minimum speed. Must be set as the robto can't drive at very low speeds (below 100.). The absolute speed cant get below 100.
const int maxNegativeSpeed = -350; // Maximum negative speed.
const int speedFactor = 40; // Used to calculate the final speed. Here the formular speedFactor * lineSensorOutputSum + defaultSpeed is used, where lineSensorOutputSum = multiplier * sensorOffset^exponentFactor.

const int calibrationAmount = 200; // Amount of calibrations
const int offsetMin = 100; // Difference between the calibrated surface and the line.
const int offsetMax = 100;
const float offsetFactor = 1;


int defaultSpeed = maxSpeed; // Variable controlling the max speed.
int lineSensorLastSensorReading = 0; // If it doesn't detect anything with its linesensors, it performs the same movement based on what the previous line sensors read.

uint16_t calibrationValues[lineSensorsAmount] = {0, 0, 0, 0, 0}; // Calibration values when calibrating

// Setup function. Initialises the five sensors and calibrates them. 
void setup() {
  lineSensors.initFiveSensors(); // Initialization 
  Serial.begin(9600); // Beginning the serial port
  lineSensorCalibrate(); // Calibrates the sensors
  delay(2000);
}

void loop() {
  followLine(); // Calls a function which runs the linefollower.
}

// Function which returns an integer. The function gets the 
int lineFollowerGetLineSensorSum(uint16_t lineSensorValues[lineSensorsAmount]) {
  int lineSensorOutputSum = 0; // Output sum for linesensors. 
  int lineSensorActiveAmount = 0; // Amount of active linesensors.

  for (int i = 0; i < lineSensorsAmount; i++) { // Looping through the line sensors and seeing if they are detecting a line. 
    if (lineSensorValues[i] > calibrationValues[i] + offsetMin) {
      int lineSensorCenteringValue = 2 - i; // Finding the line sensor value (based on 5 line sensors, therefore no 2. must be the middle (as it is 0, 1, 2, 3, 4 which are the indexes for the line sensors))
      lineSensorActiveAmount++; // Adds one to active line sensor
      lineSensorOutputSum += (int)pow(abs(lineSensorCenteringValue * multiplier), exponentFactor) * offsetCalculation(lineSensorValues[i], calibrationValues[i]) * (1 - (-2 * (lineSensorCenteringValue >> 15))) * 1; 
      // Does some math which will be used later to determine how much the robot must turn in order to stay on the line. The power of the absolute lineSensorCenteringValue is being used. Therefore if the value was negative, it must be converted into a negative number again. Here a simple trick using bit manipulation is being used: (1 - (-2 * (lineSensorCenteringValue >> 15)))
    }
  }

  if (lineSensorActiveAmount == 0) { // Changes the speed if no linesensors have detected a black line.
    defaultSpeed = maxSpeedWhenNoSensors;
    return lineSensorLastSensorReading;
  } else {
    defaultSpeed = maxSpeed;
  }

  lineSensorLastSensorReading = lineSensorOutputSum;

  return lineSensorOutputSum;
}

float offsetCalculation(int lineSensorValue, int calibrationValue) {
  int lsvc = lineSensorValue - calibrationValue;
  if (lsvc > offsetMax-offsetMin) {lsvc = offsetMax-offsetMin;}
  
  //float val = pow(lsvc, offsetFactor) / pow((offsetMax-offsetMin) * 1.0, offsetFactor);
  float val = 1;
  return val;
}

// Calibration algorithm. Basically just two loops, looping through each sensor and storing the value
void lineSensorCalibrate() {
  uint32_t lineSensorsAmountTotal[lineSensorsAmount] = {0, 0, 0, 0, 0}; // Variable to store the value

  for (int i = 0; i < calibrationAmount; i++) { // First loop. This loops the amount of calibration points which must be taken.
    uint16_t lineSensorValues[lineSensorsAmount]; // Variable to store the values from the linesensor.
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON); // Reads the line sensor
    
    for (int j = 0; j < lineSensorsAmount; j++) { // Loops trough each value from the linesensor reading and adds them to the lineSensorsAmountTotal.
      lineSensorsAmountTotal[j] += lineSensorValues[j];
    }
    delay(10); //waits 10 ms.
  }
  for (int i = 0; i < lineSensorsAmount; i++) { // Goes through each value and calculates the average.
    calibrationValues[i] = lineSensorsAmountTotal[i] / calibrationAmount;
    // Prints the final calibration value in the serial monitor.
    Serial.print(calibrationValues[i]);
    Serial.print(", ");
  }
  Serial.println("");
}


// Function, returning an int. Uses different varaibles to set the speed.
int lineFollowerFixSpeed(int speed, int lineSensorOutputSum) {
  if (speed > defaultSpeed) {
    speed = defaultSpeed;
  }

  if (speed < maxNegativeSpeed) {
    speed = maxNegativeSpeed;
  }

  if (speed < minSpeed && speed > 0) {
    speed = 100;
    
  } else if (speed <= 0 && speed > -1 * minSpeed) {
    speed = 100;
  } 

  return speed;
}

// Line follower function, which calculates the speed based on the lineSensorOutputSum and sets the speed for the motors.
void lineFollowerDriver(int lineSensorOutputSum) {
  int speed_left = lineFollowerFixSpeed(-1 * speedFactor * lineSensorOutputSum + defaultSpeed, lineSensorOutputSum);
  int speed_right = lineFollowerFixSpeed(speedFactor * lineSensorOutputSum + defaultSpeed, lineSensorOutputSum);
  motors.setSpeeds(speed_left, speed_right);
}

// Main function for following a line.
void followLine() {
  uint16_t lineSensorValues[lineSensorsAmount]; // Defines the variable holding the reading
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON); // Gets a reading

  int lineSensorOutputSum = lineFollowerGetLineSensorSum(lineSensorValues); //Calculates the lineSensorOutputSum
  lineFollowerDriver(lineSensorOutputSum); // Gets the motors to turn
}
