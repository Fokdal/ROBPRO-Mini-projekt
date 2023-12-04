#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

//threshold value
const uint16_t I_black = 900;

void setup() {
  lineSensors.initFiveSensors();

}

void loop() {

  lineAdjust();

}


void lineAdjust() {

    // motors.setSpeeds(-70, -70);
    // delay(100);

  //loop that runs 5 times for line adjustment
  for (int i = 0; i<5; i++) {

    uint16_t lineSensorValues[5]; //array to store line sensor readings

    //move forward until sensor 2 or 4 detect values below threshold I_black
    while (lineSensorValues[1] < I_black && lineSensorValues[3] < I_black) {
      lineSensors.read(lineSensorValues);
      motors.setSpeeds(100, 100);
    } 

    motors.setSpeeds(0, 0);

    //adjust if left sensor detects a black line
    if (lineSensorValues[1] > I_black){ 
      motors.setSpeeds(-80, -80); //move backwards slightly
      delay(250);

      //drive with right motor until right sensor detects the line
      while (lineSensorValues[1] < I_black + 300) {
        lineSensors.read(lineSensorValues);
        Serial.println("left");
        motors.setSpeeds(0, 120);
      }
    }

    //adjust if right sensor + offset detects a black line
    else if (lineSensorValues[3] + 350 > I_black) {
      motors.setSpeeds(-80, -80); //move backwards slightly
      delay(250);

      //drive with left motor until left sensor detects the line
      while (lineSensorValues[1] < I_black) {
        lineSensors.read(lineSensorValues);
        Serial.println("right");
        motors.setSpeeds(120 , 0);
      }
    }

    lineSensors.read(lineSensorValues);
    Serial.println(lineSensorValues[1]);
    Serial.println(lineSensorValues[3]);

    Serial.println(i);
  }
  motors.setSpeeds(100, 100);
  delay(500);
}
