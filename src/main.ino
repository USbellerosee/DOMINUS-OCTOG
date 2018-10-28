#include <LibRobus.h>
#include <qtrSensors.h>
#include <ADJDS311.h>
#include <math.h>


#define  NUM_SENSORS 8
#define TIMEOUT 2500
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN 2

    int lastError=0;
    #define Kp 0.05 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
    #define Kd 2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
    int rightMaxSpeed = 200; // max speed of the robot
    int leftMaxSpeed = 200; // max speed of the robot
    int rightBaseSpeed = 100; // this is the speed at which the motors should spin when the robot is perfectly on the line
    int leftBaseSpeed = 100;  


  QTRSensorsAnalog qtra((unsigned char[]) {3, 4,5, 6, 7, 8, 9, 10},
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
  unsigned int sensorValues[NUM_SENSORS];

  uint8_t ledPin = 52;
  ADJDS311 color(ledPin);

  void depart(int); 

  void sifflet(){}//Stop donner par victoria

  void couleur(){}//Find the color that you are in

  void getBackGoalie(){}//Find our zone if we are push out of it

  //3 options : Mur(2 not moving)/Ballon (down one)/Robot(2 moving towards you)

  void faireUnBut(){}//Follow and find a line to a goal then go until black line

  void couleurNoir(){}

  void suivreLigne(unsigned int position){
       
  int error = position-3500;//3500 car 1000*(8-1)/2

  int motorSpeed = Kp * error + Kd * (error - lastError);

    lastError = error;

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
      if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
      if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
      if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
      if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

    float realRigth=rightMotorSpeed/rightMaxSpeed;
    float realLeft=leftMotorSpeed/leftMaxSpeed;
    
    Serial.println(realRigth);
    Serial.println(realLeft);

    MOTOR_SetSpeed(0,realLeft/2);
    MOTOR_SetSpeed(1,realRigth/2);

    lastError=error;
  }


  void depart(int position){
  
    while(ENCODER_Read(0)<1200){
      suivreLigne(position);
      sifflet();
    }


  }


void setup(){
  
  BoardInit();
  Serial.begin(9600);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
}
/*
  BoardInit();
  color.init();
  color.ledOn();

   color.calibrate();  // first make sure the sensor faces a white surface at the focal point
   for(int i=0;i< 200;i++){
     qtra.calibrate();
   }


//delay(5000); Si il manque deux secondes
}*/

//LE CODE QUE JAI RAJOUTÉ DÉBUTE ICI JE NE CONNAIS PAS LE FONCTIONNEMENT AVANT CETTE SECTION

void distance(){
    float capteur_haut, capteur_bas;

   
    capteur_haut=28/(ROBUS_ReadIR(3)*0.0048828125) - 2;//formule de conversion des donnes brutes en cm
    capteur_bas=28/(ROBUS_ReadIR(2)*0.0048828125) - 2;//''
    if (capteur_haut< 30 && capteur_bas< 30)
    {
      MOTOR_SetSpeed(0, 0.4);//virage a deux roues vers la droite
      MOTOR_SetSpeed(1, -0.4);
    }
    else if (capteur_haut-capteur_bas>=20 && capteur_bas<70)
    {
      MOTOR_SetSpeed(0, 0.835);//avance avec asservissement
      MOTOR_SetSpeed(1, 0.8);
    }
    else
    {
      asservissement(300);
    }
    
   Serial.println(capteur_haut);
   Serial.println(capteur_bas);

delay(10);
  }

void loop() {
distance();
delay(230);
}


  /* code d'asservissement a jord*/
 
void asservissement(int dist)
{
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  while (ENCODER_Read(0) < dist)
  {
  float vd = 0.5, vgi = 0.55;
  float vg = vgi;
  float CE;
  #define kp  0.0001;
  #define ki  0.00002;

  MOTOR_SetSpeed(1, vd);
  MOTOR_SetSpeed(0, vg);
  delay(100);

  CE = ENCODER_Read(1) - ENCODER_Read(0);

  vg = vg + CE*kp;
 }
}

