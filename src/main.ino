/*
Projet: Le nom du script
Equipe: P-15 S&UdeS
Auteurs: Mickaël Grisé-Roy
Description: Breve description du script
Date: 01 oct 2018
*/

#include <LibRobus.h> 
#include <Arduino.h>
#include <QueueList.h>

#define ACTION_TYPE_ACCELERATION 0
#define DELAY 50 // Delay in ms

float leftSpeed;
float rightSpeed;






class Chrono{
  public:
  unsigned long time;
  unsigned long getTime(){
    return millis() - time;
  }
  void restart(){
    time = millis();
  }
  Chrono(){
      time = millis();
  }
};

class Action{
  public:
  unsigned int type;
  Action (unsigned int type){
    this->type = type;
  }
  virtual bool motion() = 0; // Returns true if action is finished. Once action is finished, the program can go to the next one.
};

class Acceleration : public Action{
  public:
  bool isFirstTime;
 
  unsigned int time;// time in ms.
  unsigned int timeElapsed;// time elapsed since beginning of acceleration
  Chrono chrono;
  float deltaSpeed;
  
  Acceleration (float initialSpeed, float finalSpeed, unsigned int time) : Action(ACTION_TYPE_ACCELERATION){
    this->time = time;
    deltaSpeed = finalSpeed - initialSpeed;
    this->isFirstTime = true;
    timeElapsed = 0;
  }
  

  bool motion(){
    if(isFirstTime){
      isFirstTime = false;
      chrono.restart();
      
    }
    
      if(timeElapsed < time){
        
        timeElapsed += chrono.getTime();
        float speedIncrementation = deltaSpeed * (float) chrono.getTime() / (float) time;
        leftSpeed += speedIncrementation;
        rightSpeed += speedIncrementation;
        chrono.restart();
        return false;
      }
      
       return true;
  
  }

};



QueueList<Action*> actionQueue; // help : https://playground.arduino.cc/Code/QueueList

void setup(){
  BoardInit();
  leftSpeed = 0.0;
  rightSpeed = 0.0;
  actionQueue.push(new Acceleration(leftSpeed,0.5,10000));
}



void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  if(!actionQueue.isEmpty())
  if(actionQueue.peek()->motion()){
    delete actionQueue.pop();
  }
  
  MOTOR_SetSpeed(0,leftSpeed);
  MOTOR_SetSpeed(1,rightSpeed);


  delay(DELAY);
}