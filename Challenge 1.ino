#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED OLED;

const int maxDist = 41;
float dist= 0;
int speed = 100;
float wheelCirc = 11.94;
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
    sideSensor = "right";

  } else if ( lineSensorValues[0] >= threshold && !tapeReached ) {
    stop();
    Serial.println("2");
    tapeReached = true;
    sideSensor="left";
  }
  else if (lineSensorValues[0] < threshold && lineSensorValues[2] < threshold && !doneAligning){
    forward();
    Serial.println("3");
  }
  else if (tapeReached == true && !doneAligning){
    alignZumo();
    doneAligning = true;
    Serial.println("4");
    resetEncoders();
  }
  else {
    driveLength();
  }
  
  
}

void driveLength(){
  bool run = true;

  while (run){
    dist = getDistance();
    OLED.clear();
    OLED.print(dist);

    if (dist > maxDist){
      run = false;
      stop();
    }
    else {
      forward();
    }
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
    motors.setSpeeds(0, 100);
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
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();

  float distanceL = countsL/909.7 * wheelCirc;
  float distanceR = countsR/909.7 * wheelCirc;

  
  return (distanceL + distanceR)/2;
}

