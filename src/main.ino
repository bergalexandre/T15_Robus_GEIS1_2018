/*
Projet: Le nom du script
Equipe: P-15 S&UdeS
Auteurs: Mickaël Grisé-Roy
Description: Breve description du script
Date: 01 oct 2018
*/

#include <LibRobus.h> 
#include "move.h"

#define ACTION_TYPE_ACCELERATION 0
#define DELAY 50 // Delay in ms


uint32_t leftPulses;
uint32_t rightPulses;


float leftSpeed = 0.0;
float rightSpeed = 0.0;


float fSpeedAdjustment(){
  float fAdjustement = 0.0;
  int32_t i32DeltaPulse = ENCODER_Read(LEFT) - ENCODER_Read(RIGHT);
  
  if(i32DeltaPulse > MOVE_WHEEL_MAX_ENCODER_DELTA){
     i32DeltaPulse = MOVE_WHEEL_MAX_ENCODER_DELTA;
  }

  if(i32DeltaPulse < -1*MOVE_WHEEL_MAX_ENCODER_DELTA){
    i32DeltaPulse = -1*(MOVE_WHEEL_MAX_ENCODER_DELTA);
  }

  fAdjustement = i32DeltaPulse * MOVE_ADJUSTEMENT_FACTOR;
  return fAdjustement;
}


uint32_t MOVE_getDistanceP(int ID)
{
    uint32_t d = ENCODER_Read(ID)*(1/MOVE_PULSE_PER_TURN)*MOVE_WHEEL_DIAMETER*PI;
    return d;
}

void MOVE_vAcceleration(float initialSpeed, float finalSpeed, unsigned int time){
        
        
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
}
void MOVE_vAvancer(float fVitesse, uint32_t ui32Distance_mm){
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOVE_vAcceleration(leftSpeed,fVitesse,500);
  while(MOVE_getDistanceP(LEFT) < ui32Distance_mm){
    rightSpeed = fVitesse + fSpeedAdjustment();
    delay(10);
  }
}


void setup(){
  BoardInit();
  MOTOR_SetSpeed(0,0.0);
  MOTOR_SetSpeed(1,0.0);
}



void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  MOVE_vAvancer(0.7,2000);

  delay(2000);
}