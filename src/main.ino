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
#define MS_PER_SECOND 1000

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
 * @brief Imprimme jusqu'à 256 charactères sur le port série. 
 * 
 * @param format string comme dans un printf
 * @param ... arguments pour %i, %f, etc
 * 
 * AB: Faîte attention ça dirait que la fonction marche pas quand on lui donne les arguments d'une autre fonction. 
 * Il faudra que je check pourquoi ça print 0 quand on fait ça.
 * 
 */
void SerialPrintf(const char *format, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
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

/**
 * @brief Retourne une valeur entre -0.1 et 0.1 en fonction de la différence entre les deux encodeurs.
 * 
 * @return float Ajustement à faire sur la vitesse de la roue droite.
 */
float fSpeedAdjustment(){
  float fAdjustement = 0.0;
  int32_t i32DeltaPulse = abs(ENCODER_Read(LEFT)) - abs(ENCODER_Read(RIGHT));
  
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
float MOVE_getGuessDistancecMM(uint32_t timeMs, float speed)
{
  //Mesurer le temps
  float estimate = ((float)timeMs/MS_PER_SECOND)*(float)MOVE_WHEEL_DIAMETER*speed;
  return estimate;
}

/**
 * @brief Essait d'estimer la distance parcouru pour accélérer ou ralentir.
 * 
 * @param finalSpeed Vitesse que le robot va atteindre.
 * @param initialSpeed Vitesse actuel du robot.
 * @param time Temps sur le quel il va accélérer
 * @return int32_t Nouvelle distance, ne prend pas en compte les décimals de millimètre.
 */
int32_t MOVE_GuessDecelerationDistance(float finalSpeed, float initialSpeed, uint32_t time)
{
  int wait = MOVE_WAIT;
  float deltaSpeed = finalSpeed - initialSpeed;
  float speedIncrementation = deltaSpeed * (float) wait / (float) time;

  float totalDistance = 0;
  unsigned int iCount;
  for(iCount =0; iCount < ((time/wait)+((time%wait)>0)); iCount++)
  {
    totalDistance += MOVE_getGuessDistancecMM(wait, initialSpeed + (iCount*speedIncrementation));
  }
  return (uint32_t)totalDistance;
}

/**
 * @brief Distance que le robot à parcouru depuis la dernière fois que l'encoder s'est fait reset.
 * 
 * @param ID ID de l'encodeur à lire.
 * @return int32_t Retourne la distance en millimètre.
 */
int32_t MOVE_getDistanceMM(int ID)
{
  int32_t d = ENCODER_Read(ID)*((float)1/MOVE_PULSE_PER_TURN)*MOVE_WHEEL_DIAMETER*PI;
  return d;
}

/**
 * @brief Accélère les deux roue inversément.
 * 
 * @param {type} finalSpeed 
 * @param {type} time 
 */
void MOVE_vAccelerationInverted(float finalSpeed, unsigned int time)
{
  float deltaSpeed = finalSpeed - g_leftSpeed;
  unsigned int wait = MOVE_WAIT;
  float speedIncrementation = deltaSpeed * (float) wait / (float) time;

  //S'assure que la roue droite soit à la même vitesse que la roue gauche.
  g_rightSpeed = g_leftSpeed;
  unsigned int iCount;
  for(iCount =0; iCount < ((time/wait)+((time%wait)>0)); iCount++)
  {
    char buffer1[20] = {0};
    char buffer2[20] = {0};     
    g_leftSpeed += speedIncrementation;
    g_rightSpeed = -g_leftSpeed + fSpeedAdjustment();
    strcpy(buffer1, strFloat(g_leftSpeed));
    strcpy(buffer2, strFloat(g_rightSpeed));
    SerialPrintf("Roue gauche = %s, roue droite = %s\n", buffer1, buffer2);
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    MOTOR_SetSpeed(LEFT, g_leftSpeed);
    delay(wait);
  }

  g_leftSpeed = finalSpeed;
  g_rightSpeed = finalSpeed;
  MOTOR_SetSpeed(LEFT, g_leftSpeed);
  MOTOR_SetSpeed(RIGHT, g_leftSpeed);
}

/**
 * @brief Accélère/ralenti pour une seule roue.
 * 
 * @param {type} finalSpeed Vitesse final du robot
 * @param {type} time Temps pour lequel le robot doit accélérer/ralentir.
 * @param {type} ID ID de la roue à bouger.
 */
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

  unsigned int wait = MOVE_WAIT;
  float deltaSpeed = finalSpeed - *currentSpeed;
  float speedIncrementation = deltaSpeed * (float) wait / (float) time;

  unsigned int iCount;
  for(iCount =0; iCount < ((time/wait)+((time%wait)>0)); iCount++)
  {
    char buffer1[8] = {0};
    char buffer2[8] = {0}; 
    strcpy(buffer1, strFloat(*currentSpeed));
    strcpy(buffer2, strFloat(finalSpeed));    
    SerialPrintf("Acceleration %s sur %s\n", buffer1, buffer2);
    *currentSpeed += speedIncrementation;
    MOTOR_SetSpeed(ID, *currentSpeed);
  }
  /*Set la vitesse à cause que les floats ne sont pas très précis (on ne sera jamais à la consigne exacte sinon). 
  * C'est un problème quand on veut une vitesse de 0.*/
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
  unsigned int wait = MOVE_WAIT;
  float speedIncrementation = deltaSpeed * (float) wait / (float) time;

  //S'assure que la roue droite soit à la même vitesse que la roue gauche.
  g_rightSpeed = g_leftSpeed;
  unsigned int iCount;
  for(iCount =0; iCount < ((time/wait)+((time%wait)>0)); iCount++)
  {
    char buffer1[20] = {0};
    char buffer2[20] = {0}; 
    strcpy(buffer1, strFloat(g_leftSpeed));
    strcpy(buffer2, strFloat(finalSpeed));    
    SerialPrintf("Acceleration %s sur %s\n", buffer1, buffer2);
    g_leftSpeed += speedIncrementation;
    g_rightSpeed = g_leftSpeed + fSpeedAdjustment();
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

  MOVE_vAcceleration(fVitesse, 125);
  while(MOVE_getDistanceMM(LEFT) < i32Distance_mm)
  {
    SerialPrintf("Distance fait = %i mm\n", MOVE_getDistanceMM(LEFT));
    g_rightSpeed = fVitesse + fSpeedAdjustment();
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    delay(100);
  }
  // Arrête le mouvement
  MOVE_vAcceleration(0.0, 100);
}

/**
 * @brief Fait pivoter le robot sur lui même.
 * 
 * @param angle en degré (L'angle doit être positif)
 * @param Coté vers lequel le robot va tourner 
 */
void MOVE_Rotation1Roue(float angle, int iRotationDirection)
{
  // Avance du coté opposé à la direction. Pour ceux qui connaissent pas, le ? C'est un opérateur conditionel.
  //Condition == TRUE ? Fait ça si TRUE: Sinon fait ça si FALSE;
  int ID = iRotationDirection == LEFT ? RIGHT: LEFT;

  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = (2*PI*MOVE_LARGEUR_ROBOT*angle)/360;
  angleEnDistance -= MOVE_GuessDecelerationDistance(0.0, MOVE_MAX_SPEED, 50);//Estimation de la distance requis pour ralentir.
  
  //Reset les encodeurss
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOVE_vAccelerationSingleWheel(0.5, 100 ,ID);
  int32_t distanceActuelle = MOVE_getDistanceMM(ID);

  while( distanceActuelle < angleEnDistance)
  {
    SerialPrintf("Virage à %s de %i degré, distance = %i sur %i\n", 
      iRotationDirection == LEFT ? "LEFT": "RIGTH", angle, distanceActuelle, angleEnDistance);
    delay(5);
    distanceActuelle = MOVE_getDistanceMM(iRotationDirection == LEFT ? RIGHT: LEFT);
  }

  //Stop la roue qui tourne.
  MOVE_vAccelerationSingleWheel(0, 50, ID);
}

void MOVE_Rotation2Roues(float angle)
{
  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = ((2*PI*MOVE_LARGEUR_ROBOT*angle)/360)/2;
  angleEnDistance -= MOVE_GuessDecelerationDistance(0.0, 0.4, 100);//Estimation de la distance requis pour ralentir.
  //Reset les encodeurss
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOVE_vAccelerationInverted(0.4, 100);
  int32_t distanceActuelleL = MOVE_getDistanceMM(LEFT);

  while( distanceActuelleL < angleEnDistance )
  {
    distanceActuelleL = MOVE_getDistanceMM(LEFT);
    //ajuste la vitesse de la roue droite
    g_rightSpeed = -0.4 + fSpeedAdjustment();
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    SerialPrintf("Distance fait: %imm\n", distanceActuelleL);
  }
  //Stop la roue qui tourne.
  MOVE_vAccelerationInverted(0.0, 100);
}


void setup(){
  BoardInit();
  Serial.begin(9600);
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, 0.0);
  MOTOR_SetSpeed(RIGHT, 0.0);


  while(!ROBUS_IsBumper(3)){
  }
  
}



void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  while(1)
  {
    // début de l'aller
    MOVE_vAvancer(MOVE_MAX_SPEED,2000);
    MOVE_Rotation1Roue(90,LEFT);
    MOVE_vAvancer(0.5,300);
    MOVE_Rotation1Roue(90,RIGHT);
    MOVE_vAvancer(0.5,200);
    MOVE_Rotation1Roue(90,RIGHT);
    MOVE_vAvancer(0.5,150);
    MOVE_Rotation1Roue(45,LEFT);
    MOVE_vAvancer(0.5,550);
    MOVE_Rotation1Roue(90,LEFT);
    MOVE_vAvancer(0.5,580);
    MOVE_Rotation1Roue(45,RIGHT);
    MOVE_vAvancer(0.5,180);
    MOVE_Rotation1Roue(10,RIGHT);
    MOVE_vAvancer(0.5,1000);

    MOVE_Rotation2Roues(178);

    // début du retour

    MOVE_vAvancer(0.5,1000);
    MOVE_Rotation1Roue(10,LEFT);
    MOVE_vAvancer(0.5,180);
    MOVE_Rotation1Roue(45,LEFT);
    MOVE_vAvancer(0.5,550);
    MOVE_Rotation1Roue(90,RIGHT);
    MOVE_vAvancer(0.5,550);
    MOVE_Rotation1Roue(45,RIGHT);
    MOVE_vAvancer(0.5,150);
    MOVE_Rotation1Roue(90,LEFT);
    MOVE_vAvancer(0.5,200);
    MOVE_Rotation1Roue(90,LEFT);
    MOVE_vAvancer(0.5,300);
    MOVE_Rotation1Roue(93, RIGHT);
    MOVE_vAvancer(MOVE_MAX_SPEED,2000);
    delay(50000);
  }
}
