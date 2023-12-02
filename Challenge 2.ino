/*
Line follower

Programmed by:
 - Rasmus Dueholm Kristiansen

*/
#include <Zumo32U4.h>


Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

uint64_t prevTime = 0;                                                                          // Time for previous adjustment. Used to get the delta time
float prevError = 0;                                                                            // Error for previous adjustment. Used to get the delta error.

const float mappingMaxValue = 100;                                                              // Before the programme is started, each sensor is calibrated. This is then when the programme is running mapped into a scale from 0 to 100.

// Variables for the PID controller
const float k_p = 0.55;
const float k_i = 0.8;
const float k_d = 20;

// Other variables controlling the movement.
const int defaultSpeed = 400;
const int minSpeed = 300;
const float defactorAcc = 400/(defaultSpeed * 1.0);
const float defactor = 200/(defaultSpeed * 1.0);

const float startOffset = 20;                                                                   // When the programme starts, the ZUMO is offsat, so it has to drive a shorter route.

const int calibrationSpeed = 200;                                                               // Speed for calibrating
const int calibrationAmount = 200;                                                              // Amount of calibrations.

// There are 5 sensors in total, 2 outer, two inner and one center. Because of the difference, the outer sensors will provide a more significant change in direction.
const float outerSensorFactor = 10;
const float innerSensorFactor = 1;

const int lineSensorAmount = 5;
int calibrationMinMax[lineSensorAmount][2] = {{0, 0},{0, 0},{0, 0},{0, 0},{0, 0}};              // Min and max calibration values

// Function using the calibration values, to map the sensorValue into a 0-100 number.
float sensorMap(uint16_t sensorValue, int sensorPosition) {
  int newSensorValue = map(sensorValue, calibrationMinMax[sensorPosition][0], calibrationMinMax[sensorPosition][1], 0, mappingMaxValue);
  
  // If the number is higher than 100 or lower than 0.
  if (newSensorValue > mappingMaxValue) newSensorValue = mappingMaxValue;
  if (newSensorValue < 0) newSensorValue = 0;

  return newSensorValue;
}

// Function using the sensor values to calculate the error, the robot must perform.
float pidController(uint16_t lineSensorValues[lineSensorAmount]) {
  float error = startOffset;
  
  // The first two errors are multiplied by -1 to take which side the sensors are on, into account.
  error += -1 * outerSensorFactor * (0 - sensorMap(lineSensorValues[0], 0));
  error += -1 * innerSensorFactor * (0 - sensorMap(lineSensorValues[1], 1));
  error += innerSensorFactor * (0 - sensorMap(lineSensorValues[3], 3));
  error += outerSensorFactor * (0 - sensorMap(lineSensorValues[4], 4));
  
  // Delta time being calculated since last time called.
  uint32_t deltaTime = micros() - prevTime;
  prevTime = micros();
  
  // Delta error.
  float deltaError = error - prevError;
  prevError = error;

  // Calculating each constant.
  int p = k_p * error;
  int i = k_i * error * deltaTime / 1000.0;
  int d = k_p * deltaError / deltaTime;

  // Returning them added together.
  return(p+i+d);
}

// Function for calibrating each sensor.
void calibrateSensors() {
  for (int i = 0; i < calibrationAmount; i++) {                                                 // First loop. This loops the amount of calibration points which must be taken.
    if (i < calibrationAmount / 2) {
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    } else {
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    }

    uint16_t lineSensorValues[lineSensorAmount];                                                // Variable to store the values from the linesensor.
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);                                        // Reads the line sensor
    
    for (int j = 0; j < lineSensorAmount; j++) {                                                // Loops trough each value from the linesensor reading and adds them to the lineSensorsAmountTotal.
      if (calibrationMinMax[j][0] > lineSensorValues[j]) calibrationMinMax[j][0] = lineSensorValues[j];
      else if (calibrationMinMax[j][1] < lineSensorValues[j]) calibrationMinMax[j][1] = lineSensorValues[j];
    }
    delay(10);                                                                                  //waits 10 ms for the robot to turn.
  }
  motors.setSpeeds(0,0);
}

// Setup function
void setup() {
  lineSensors.initFiveSensors();                                                                // Initialization 
  Serial.begin(9600);                                                                           // Beginning the serial port
  calibrateSensors();                                                                           // Calibrates the sensors

}

// Loop function
void loop() { 

  uint16_t lineSensorValues[lineSensorAmount];                                                  // Variable to store the values from the linesensor.
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);                                          // Reads the line sensor
  
  float error = pidController(lineSensorValues);                                                // Gets the error from the linesensors

  int speed = defaultSpeed - abs(error) / defactorAcc;                                          // Calculates the speed with a deacceleration factor based on the error
  if (speed < minSpeed) speed = minSpeed;                                                       // The speed must not get lower than minSpeed

  motors.setSpeeds(speed - error/defactor, speed + error/defactor);                             // Applies the speed together with the error.
}
