/*
Projet: Le nom du script
Equipe: P-15 S&UdeS
Auteurs: Mickaël Grisé-Roy
Description: Breve description du script
Date: 01 oct 2018
*/

/*******************************************************************************
 * Include
*******************************************************************************/

#include <LibRobus.h> 
#include "move.h"
#include <stdarg.h> //Pour imprimer sur le port série comme si c'était un printf

/*******************************************************************************
 * Define
*******************************************************************************/

#define DELAY 50 // Delay in ms

/*******************************************************************************
 * Prototypes locaux
*******************************************************************************/
void SerialPrintf(const char *fmt, ...);
char* strFloat(float valeur);

/*******************************************************************************
 * Variables locales
*******************************************************************************/

float g_leftSpeed = 0.0;
float g_rightSpeed = 0.0;
char floatbuffer[8]; //mémoire reservé pour afficher des floats

/*******************************************************************************
 * fonctions
*******************************************************************************/

/**
 * @brief Imprimme jusqu'à 256 charactères
 * 
 * @param {type} fmt string comme dans un printf
 * @param {type} ... arguments pour %i, %f, etc
 */
void SerialPrintf(const char *fmt, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsprintf(buffer, fmt, args);
  va_end(args);
  Serial.print(buffer);
}

/**
 * @brief quand on fait %f, ça affiche des ? à la place. Work around avec %s et cette fonction.
 *  La fonction devrait afficher tout les nombres voulu, à deux décimal.
 */
char* strFloat(float valeur)
{
  //20 car 2 pour décimal, 1 pour la virgule, 1 pour le signe et 1 pour le caractères NULL. Ça laisse 3 byte pour le reste.
  memset(floatbuffer, 0, sizeof(floatbuffer));
  if(valeur < 100.99 && valeur > -100.99)
  {
    memset(floatbuffer, 0, sizeof(floatbuffer));
    dtostrf((double)valeur, sizeof(floatbuffer)-2, 2, floatbuffer);
  }
  return floatbuffer;
}

float fSpeedAdjustment(){
  float fAdjustement = 0.0;
  int32_t i32DeltaPulse = ENCODER_Read(LEFT) - ENCODER_Read(RIGHT);
  
  if(i32DeltaPulse > MOVE_WHEEL_MAX_ENCODER_DELTA){
    i32DeltaPulse = MOVE_WHEEL_MAX_ENCODER_DELTA;
  }

  if(i32DeltaPulse < -1*MOVE_WHEEL_MAX_ENCODER_DELTA){
    i32DeltaPulse = -1*(MOVE_WHEEL_MAX_ENCODER_DELTA);
  }

  fAdjustement = (float)i32DeltaPulse * MOVE_DERIVATIVE_ADJUSTEMENT_FACTOR;
  SerialPrintf("vitesse ajuste: %s\n", strFloat(fAdjustement));
  return fAdjustement;
}

/**
 * @brief retourne distance parcourure en mm
 * 
 * @param ID encode à mesurer.
 * @return distance en millimètre
 */
int32_t MOVE_getDistanceMM(int ID)
{
  int32_t d = ENCODER_Read(ID)*((float)1/MOVE_PULSE_PER_TURN)*MOVE_WHEEL_DIAMETER*PI;
  return d;
}

void MOVE_vAccelerationSingleWheel(float finalSpeed, unsigned int time, int ID)
{
  float *currentSpeed; //Pointeur vers la variable de vitesse. Quand on le modifie, ça modifie l'autre également.

  if(ID == LEFT)
  {
    currentSpeed = &g_leftSpeed; //Addresse de g_leftSpeed
  }
  else
  {
    currentSpeed = &g_rightSpeed; //Addresse de g_rightSpeed
  }

  unsigned int wait = 20;
  float deltaSpeed = finalSpeed - *currentSpeed;
  float speedIncrementation = deltaSpeed * (float) wait / (float) time;

  unsigned int iCount;
  for(iCount =0; iCount <= ((time/wait)+((time%wait)>0)); iCount++)
  {
    char buffer1[8] = {0};
    char buffer2[8] = {0}; 
    strcpy(buffer1, strFloat(*currentSpeed));
    strcpy(buffer2, strFloat(finalSpeed));    
    SerialPrintf("Acceleration %s sur %s\n", buffer1, buffer2);
    *currentSpeed += speedIncrementation;
    MOTOR_SetSpeed(ID, *currentSpeed);
  }
  //Set la vitesse à cause que les floats ne sont pas très précis (on ne sera jamais à la consigne exacte sinon)
  *currentSpeed = finalSpeed;
  MOTOR_SetSpeed(ID, *currentSpeed);
}

/**
 * @brief Accélère les deux roue en fonction de la vitesse initial de la roue gauche.
 * 
 * @param finalSpeed 
 * @param time 
 */
void MOVE_vAcceleration(float finalSpeed, unsigned int time)
{
  float deltaSpeed = finalSpeed - g_leftSpeed;
  unsigned int wait = 20;
  float speedIncrementation = deltaSpeed * (float) wait / (float) time;

  //S'assure que la roue droite soit à la même vitesse que la roue gauche.
  g_rightSpeed = g_leftSpeed;
  unsigned int iCount;
  for(iCount =0; iCount <= ((time/wait)+((time%wait)>0)); iCount++)
  {
    char buffer1[20] = {0};
    char buffer2[20] = {0}; 
    strcpy(buffer1, strFloat(g_leftSpeed));
    strcpy(buffer2, strFloat(finalSpeed));    
    SerialPrintf("Acceleration %s sur %s\n", buffer1, buffer2);
    g_leftSpeed += speedIncrementation;
    g_rightSpeed += speedIncrementation;
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    MOTOR_SetSpeed(LEFT, g_leftSpeed);
    delay(wait);
  }

  //Set la vitesse à cause que les floats ne sont pas très précis (on ne sera jamais à la consigne exacte sinon)
  g_leftSpeed = finalSpeed;
  g_rightSpeed = finalSpeed;
  MOTOR_SetSpeed(LEFT, g_leftSpeed);
  MOTOR_SetSpeed(RIGHT, g_leftSpeed);
}

void MOVE_vAvancer(float fVitesse, int32_t i32Distance_mm){
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOVE_vAcceleration(fVitesse, 200);
  while(MOVE_getDistanceMM(LEFT) < i32Distance_mm)
  {
    SerialPrintf("Distance fait = %i mm\n", MOVE_getDistanceMM(LEFT));
    g_rightSpeed = fVitesse + fSpeedAdjustment();
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    delay(100);
  }
}

/**
 * @brief Fait pivoter le robot sur lui même.
 * 
 * @param angle en degré (L'angle doit être positif)
 * @param Coté vers lequel le robot va tourner 
 */
void MOVE_Rotation1Roue(unsigned int angle, int iRotationDirection)
{
  // Avance du coté opposé à la direction.
  int ID = iRotationDirection == LEFT ? RIGHT: LEFT;

  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = (2*PI*MOVE_LARGEUR_ROBOT*angle)/360;
  //Arrête les mouvements avant de tourner
  MOVE_vAcceleration(0.0, 200);
  //Reset les encodeurss
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOVE_vAccelerationSingleWheel(0.9, 200 ,ID);
  int32_t distanceActuelle = MOVE_getDistanceMM(ID);

  while( distanceActuelle < angleEnDistance)
  {
    SerialPrintf("Virage à %s de %i degré, distance = %i sur %i\n", 
      iRotationDirection == LEFT ? "LEFT": "RIGTH", angle, distanceActuelle, angleEnDistance);
    delay(50);
    distanceActuelle = MOVE_getDistanceMM(iRotationDirection == LEFT ? RIGHT: LEFT);
  }
  MOVE_vAccelerationSingleWheel(0, 300, ID);
}


void setup(){
  BoardInit();
  Serial.begin(9600);
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, 0.0);
  MOTOR_SetSpeed(RIGHT, 0.0);
}



void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  SerialPrintf("test %i sur %i\n", 2, 5);
  SerialPrintf("Avance pour 2m\n");
  MOVE_vAvancer(0.7,4000);
  SerialPrintf("Pivote de 180 degre\n");
  MOVE_Rotation1Roue(180,RIGHT);
  SerialPrintf("Avance pour 2m\n");
  MOVE_vAvancer(0.7,2000);
  MOVE_vAcceleration(0, 300);
  delay(2000);
  while(1)delay(200);
}
