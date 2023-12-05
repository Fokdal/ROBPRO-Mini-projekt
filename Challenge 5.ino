 #include <Zumo32U4.h>
 #include <Wire.h>
 #include <Zumo32U4ProximitySensors.h>
Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

uint16_t lineSensorValues[5] = { 1, 2, 3, 4, 5 };

bool proxLeftActive; //Aktivering af venstre proximity sensor
bool proxFrontActive; //Aktivering af front proximity sensor
bool proxRightActive; //Aktivering af højre proximity sensor
long randNumber; //Tilfældigt tal
int speed = 50; //Den ønskede speed for motor, kan ændres efter ønske
int threshold = 500; //grænseværdi linesensor bruger til at se forskel på kørefladen og det sorte tape
int Laps = 0; // Antal runder robotten kører, sat til 0 fra start
void setup ()
{
 
  lineSensors.initThreeSensors(); //Initiereing af Linesensor
  proxSensors.initThreeSensors(); //Initiereing af Proximitysensor
  proxSensors.setPeriod(420); //Set frequency på IR pulse fra Prox sensorer
  proxSensors.setPulseOnTimeUs(421); //Mængde af tid pulsenen kører, når der måles i mikrosekunder
  proxSensors.setPulseOffTimeUs(578); //Mængde af tid pulsenen ikke kører, i mikrosekunder
  uint16_t levels[] = { 4, 15, 32, 55, 85, 120 };
  proxSensors.setBrightnessLevels(levels, sizeof(levels)/2); //Styrer brightness levels på LED'erne der bruges af prox sensors.
  
}
//Function for fremadrettet kørsel
void forward(){
  motors.setSpeeds(speed, speed);
}
//Function for tilfældigt venstre sving
void randomTurnLeft(){
  randNumber = random(300,500);
  motors.setSpeeds(-200,200);
  delay(randNumber);
  motors.setSpeeds(0,0);
}
//Function for tilfældigt højre sving
void randomTurnRight(){
  randNumber = random(300,500);
  motors.setSpeeds(200, -200);
  delay(randNumber);
  motors.setSpeeds(0,0);
}
//Main code, bestående af 2 functions, en der styrer kørsel, og en der printer værdien antallet af laps til displayet
void loop(){
  if (proxSensors.countsLeftWithLeftLeds() >= 50) {
    randomTurnLeft();
  }  else if (proxSensors.countsRightWithRightLeds() >= 50) {
    randomTurnRight();
  } else {
    forward();
  }

    if (lineSensorValues[0]<threshold && lineSensorValues[1] < threshold && lineSensorValues[2]< threshold) {
    Laps++;
  }
  else lcd.print("Laps");
  delay(100);
}
