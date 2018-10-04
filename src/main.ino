/*
Projet: Le nom du script
Equipe: P-15 S&UdeS
Auteurs: Mickaël Grisé-Roy
Description: Breve description du script
Date: 01 oct 2018
*/

#include <LibRobus.h> 
#include "move.h"


#define DELAY 50 // Delay in ms


uint32_t leftPulses;
uint32_t rightPulses;


float leftSpeed = 0.0;
float rightSpeed = 0.0;


float fSpeedAdjustment(){
  float fAdjustement = 0.0;
  int32_t i32DeltaPulse = ENCODER_Read(LEFT) - ENCODER_Read(RIGHT);
  int32_t i32DeltaPulseOvertime = leftPulses - rightPulses;
  if(i32DeltaPulse > MOVE_WHEEL_MAX_ENCODER_DELTA){
     i32DeltaPulse = MOVE_WHEEL_MAX_ENCODER_DELTA;
  }

  if(i32DeltaPulse < -1*MOVE_WHEEL_MAX_ENCODER_DELTA){
    i32DeltaPulse = -1*(MOVE_WHEEL_MAX_ENCODER_DELTA);
  }

  fAdjustement = i32DeltaPulse * MOVE_DERIVATIVE_ADJUSTEMENT_FACTOR;
  //fAdjustement += i32DeltaPulseOvertime * MOVE_INTEGRATIVE_ADJUSTEMENT_FACTOR;
  return fAdjustement;
}


//retourne distance parcourure en mm
int32_t MOVE_getDistanceP(int ID)
{
    int32_t d = ENCODER_Read(ID)*((float)1/MOVE_PULSE_PER_TURN)*MOVE_WHEEL_DIAMETER*PI;
    char tableau[100] = {0};
    sprintf(tableau,"distance : %i\n", d);
    Serial.print(tableau);
    return d;
}


void MOVE_vAcceleration(float initialSpeed, float finalSpeed, unsigned int time){
        leftPulses = ENCODER_Read(LEFT);
        rightPulses = ENCODER_Read(RIGHT);
        
        float deltaSpeed = finalSpeed - initialSpeed;
        leftSpeed = initialSpeed;
        rightSpeed = initialSpeed;
        MOTOR_SetSpeed(RIGHT,rightSpeed);
        MOTOR_SetSpeed(LEFT,leftSpeed);

        unsigned int wait = 20;
        float speedIncrementation = deltaSpeed * (float) wait / (float) time;

        if(initialSpeed < finalSpeed)
        while(leftSpeed < finalSpeed){
            leftSpeed += speedIncrementation;
            rightSpeed += speedIncrementation;
            MOTOR_SetSpeed(RIGHT,rightSpeed);
            MOTOR_SetSpeed(LEFT,leftSpeed);
            delay(wait);
        }
        if(finalSpeed < initialSpeed)
        while(leftSpeed > finalSpeed){
            leftSpeed += speedIncrementation;
            rightSpeed += speedIncrementation;
            MOTOR_SetSpeed(RIGHT,rightSpeed);
            MOTOR_SetSpeed(LEFT,leftSpeed);
            delay(wait);
        }
        leftSpeed = finalSpeed;
        rightSpeed = finalSpeed;
        
        MOTOR_SetSpeed(RIGHT,rightSpeed);
        MOTOR_SetSpeed(LEFT,leftSpeed);
        leftPulses = ENCODER_Read(LEFT) - leftPulses;
        rightPulses = ENCODER_Read(RIGHT) - rightPulses;

}
void MOVE_vAvancer(float fVitesse, int32_t i32Distance_mm){
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  leftPulses = 0;
  rightPulses = 0;
  MOVE_vAcceleration(leftSpeed,fVitesse,500);
  while(MOVE_getDistanceP(LEFT) < i32Distance_mm){
    rightSpeed = fVitesse + fSpeedAdjustment();
    delay(100);
  }

  MOTOR_SetSpeed(LEFT,0.0);
  MOTOR_SetSpeed(RIGHT,0.0);

}

void MOVE_Rotation1Roue(int angle, int ID){
  int pulseCount=0;
  float doPulses = angle * (2772/60160);
  if (ID==RIGHT)
    leftSpeed=0;
  else if (ID==LEFT)
    rightSpeed=0;
  pulseCount += ENCODER_Read(ID);
  if (pulseCount<doPulses)
    MOTOR_SetSpeed(ID,0.5);
  else if (pulseCount==doPulses)
    MOTOR_SetSpeed(ID,0.0);
}


void setup(){
  BoardInit();
  MOTOR_SetSpeed(0,0.0);
  MOTOR_SetSpeed(1,0.0);
  Serial.begin(9600);

}



void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  Serial.print("Forward");
  MOVE_vAvancer(0.7,2000);
  Serial.print("Stop");
  MOTOR_SetSpeed(LEFT,0.0);
  MOTOR_SetSpeed(RIGHT,0.0);
  delay(10000);
}
