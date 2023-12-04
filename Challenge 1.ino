#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

const int maxDist = 41;
float dist= 0;
int speed = 100;
float wheelCirc = 13.0;
int threshold = 600;
bool tapeReached = false, doneAligning = false;
String sideSensor ="";

#define num_linesensors 3
uint16_t lineSensorValues[num_linesensors];

void setup(){
  lineSensors.initThreeSensors();
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  readLineSensors();
  Serial.println("0");

  if ( lineSensorValues[2] >= threshold && !tapeReached ){
    stop();
    Serial.println("1");
    tapeReached = true;
    sideSensor= "right";

  } else if ( lineSensorValues[0]>= threshold && !tapeReached ) {
    stop();
    Serial.println("2");
    tapeReached = true;
    sideSensor="left";
  }
  else if (lineSensorValues[0] < threshold && lineSensorValues[2] < threshold){
    forward();
    Serial.println("3");
  }
  else if (tapeReached == true && !doneAligning){
    alignZumo();
    doneAligning = true;
    Serial.println("4");
  }
  else if (doneAligning){
    resetEncoders();
    
    while (dist < maxDist){
      forward();
      dist += getDistance();
      
    }
    stop();

  }
}

void readLineSensors(){   //  Reads value of linesensors and insert the values into the array "lineSensorValues"
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void forward(){
  motors.setSpeeds(speed, speed);
}

void stop(){  //  Stops Zumo
  motors.setSpeeds(0, 0);
}

void alignZumo(){
  driveToTape();
  
  driveOverTape();

  driveBackToTape();
}

void driveToTape(){
  bool Run = true;

  if (sideSensor == "right"){
    motors.setSpeeds(100,0);
    while (Run){
      readLineSensors();
      if (lineSensorValues[0] >= threshold){
        stop();
        Run = false;
      }
    }

  }
  else if (sideSensor == "left"){
    motors.setSpeeds(0,100);
    while (Run){
      readLineSensors();
      if (lineSensorValues[2] >= threshold){
        stop();
        Run = false;
      }
    }

  }

}
void driveOverTape(){
  bool run = true;

  forward();
  
  while (run){
    readLineSensors();

    if (lineSensorValues[0] < threshold && lineSensorValues[2] < threshold){
      run = false;
    }

    if (lineSensorValues[0] < threshold){
      motors.setLeftSpeed(0);
    }
    
    if(lineSensorValues[2] < threshold){
      motors.setRightSpeed(0);
    }
  }
  
  
}
void driveBackToTape(){
  bool run = true;

  motors.setSpeeds(-50, -50);

  while(run){
    readLineSensors();

    if (lineSensorValues[0] >= threshold && lineSensorValues[2] >= threshold){
      run = false;
    }

    if (lineSensorValues[0] >= threshold){
      motors.setLeftSpeed(0);
    }
    
    if(lineSensorValues[2] >= threshold){
      motors.setRightSpeed(0);
    }
  }

}


void resetEncoders(){
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

float getDistance(){
  int countsL = encoders.getCountsAndResetLeft();
  int countsR = encoders.getCountsAndResetRight();

  float distanceL = countsL/900.0 * wheelCirc;
  float distanceR = countsR/900.0 * wheelCirc;

  
  return (distanceL + distanceR)/2;
}
