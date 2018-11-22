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
#include <stdarg.h> //Pour imprimer sur le port série comme si c'était un printf
#include <Wire.h>
#include "SparkFunISL29125.h"
#include "move.h"
#include "Sortie.h"
#include <Pixy2.h>
#include <Arduino.h>

/*******************************************************************************
 * Define
*******************************************************************************/

#define DELAY 50 // Delay in ms
#define MS_PER_SECOND 1000
#define TIMER_ID_KICK 1
#define TIMER_ID_KICK_DISABLE 2
#define TIMER_ID_STATE 3
#define TIMER_ID_SIFFLET 4
#define TIMER_ID_GRAB 5
#define TIMER_ID_DROP 6
#define KICKER 0
#define GOALER 1
Pixy2 pixy;

//enum

typedef enum
{
  avance,
  stop,
  pivote,
  kick_ball,
  recule
} state_t;

/*******************************************************************************
 * Prototypes locaux
*******************************************************************************/
void SerialPrintf(const char *fmt, ...);
char* strFloat(float valeur);

int sifflet = 0;
/*******************************************************************************
 * Variables locales
*******************************************************************************/

float g_leftSpeed = 0.0;
float g_rightSpeed = 0.0;
char floatbuffer[8]; //mémoire reservé pour afficher des floats
int kick = 0;
int currently_carrying = 0;
SFE_ISL29125 RGB_sensor;
int robot;

int deltaTime = millis();
int voltage = 0;
bool siffletFirstTime = true;
state_t robot_state = avance;

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
void SerialPrintf(const char *format, ...){
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
 // SerialPrintf("vitesse ajuste: %s\n", strFloat(fAdjustement));
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
void MOVE_vAccelerationInverted(float finalSpeed, unsigned int time){
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
void MOVE_vAccelerationSingleWheel(float finalSpeed, unsigned int time, int ID, int32_t distanceMM){
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
    int32_t currentDistance = ENCODER_Read(ID);
    if(currentDistance >= distanceMM && distanceMM != 0)
    {
      break;
    }
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
void MOVE_vAcceleration(float finalSpeed, unsigned int time){
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

void MOVE_vAvancer(float fVitesse, int32_t i32Distance_mm,unsigned int accelerationTime = 125){
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  

  g_leftSpeed = fVitesse;
  MOTOR_SetSpeed(LEFT,g_leftSpeed);
  if(i32Distance_mm  > 0){
  while(MOVE_getDistanceMM(LEFT) < i32Distance_mm)
  {
    checkForSifflet();
    SerialPrintf("Distance fait = %i mm\n", MOVE_getDistanceMM(LEFT));
    g_rightSpeed = fVitesse + fSpeedAdjustment();
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    delay(50);
  }
  }
  else{
    while(MOVE_getDistanceMM(LEFT) > i32Distance_mm)
  {
    checkForSifflet();
    SerialPrintf("Distance fait = %i mm\n", MOVE_getDistanceMM(LEFT));
    g_rightSpeed = fVitesse - fSpeedAdjustment();
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    delay(50);
  }
  }

  // Arrête le mouvement
  MOVE_vAcceleration(0.0, 100 );
}

void MOVE_finDuVirage(int ID, int32_t distanceMM, float speed){
  distanceMM -= 10;
  float *currentSpeed = ID == LEFT? &g_leftSpeed: &g_rightSpeed;
  int32_t currentDistance = MOVE_getDistanceMM(ID);
  while(currentDistance < distanceMM)
  {
    float facteur = (((float)distanceMM-(float)currentDistance)/distanceMM);
    *currentSpeed = speed * facteur;
    if(*currentSpeed < 0.125)
    {
      *currentSpeed = 0.125;
    }
    strFloat(speed);
    MOTOR_SetSpeed(ID, *currentSpeed);
    SerialPrintf("Virage à %s, distance = %i sur %i, vitesse = %s\n",
    ID == RIGHT ? "LEFT": "RIGTH", (int)currentDistance, distanceMM, floatbuffer);
    delay(1);
    currentDistance = MOVE_getDistanceMM(ID);
  }
  *currentSpeed = 0.0;
  MOTOR_SetSpeed(ID, *currentSpeed);
  SerialPrintf("Distance final est de %i sur %i\n", (int)MOVE_getDistanceMM(ID), (int)distanceMM);
}

/**
 * @brief Fait pivoter le robot sur lui même.
 * 
 * @param angle en degré (L'angle doit être positif)
 * @param Coté vers lequel le robot va tourner 
 */
void MOVE_Rotation1Roue(float angle, int iRotationDirection){
  // Avance du coté opposé à la direction. Pour ceux qui connaissent pas, le ? C'est un opérateur conditionel.
  //Condition == TRUE ? Fait ça si TRUE: Sinon fait ça si FALSE;
  int ID = iRotationDirection == LEFT ? RIGHT: LEFT;
  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = (2*PI*MOVE_LARGEUR_ROBOT*angle)/360;
  //Reset les encodeurss
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  float speed = angle > 20? 0.5: 0.2;
  MOVE_vAccelerationSingleWheel(0.5, 100 ,ID, angleEnDistance);
  int32_t distanceActuelle = MOVE_getDistanceMM(ID);

  //Turn at max speed
  while( ((distanceActuelle)*100/angleEnDistance) < MOVE_SLOW_AT_PERCENT)
  {
    SerialPrintf("Virage à %s de %i degré, distance = %i sur %i\n", 
    iRotationDirection == LEFT ? "LEFT": "RIGTH", (int)angle, (int)distanceActuelle, angleEnDistance);
    distanceActuelle = MOVE_getDistanceMM(iRotationDirection == LEFT ? RIGHT: LEFT);
    delay(5);
  }
  //Tourne lentement juste à ce que le robot soit à la consigne.
  MOVE_finDuVirage(ID, angleEnDistance, speed);
}

void MOVE_FinduPivot(int32_t distanceMM, float speed){
  distanceMM -= 10;
  int32_t currentDistance[2];
  currentDistance[LEFT] = MOVE_getDistanceMM(LEFT);
  currentDistance[RIGHT] = MOVE_getDistanceMM(RIGHT);
  while(currentDistance[LEFT] < distanceMM || abs(currentDistance[RIGHT]) < distanceMM)
  {
    float facteur[2];
    facteur[LEFT] = (((float)distanceMM-(float)currentDistance[LEFT])/distanceMM);
    facteur[RIGHT] = (((float)distanceMM-(float)abs(currentDistance[RIGHT]))/distanceMM);
    g_leftSpeed = speed * facteur[LEFT];
    g_rightSpeed = -speed * facteur[RIGHT];

    if(g_leftSpeed < 0.125)
    {
      g_leftSpeed = 0.125;
    }
    if(g_rightSpeed > -0.125)
    {
      g_rightSpeed = -0.125;
    }
    
    MOTOR_SetSpeed(LEFT, currentDistance[LEFT] < distanceMM ? g_leftSpeed: 0.0);
    MOTOR_SetSpeed(RIGHT, currentDistance[RIGHT] < distanceMM ? g_rightSpeed: 0.0);

    SerialPrintf("Pivot, distance = %i et %i sur %i, vitesse LEFT = ", (int)currentDistance[LEFT], (int)currentDistance[RIGHT], (int)distanceMM);
    Serial.print(g_leftSpeed);
    Serial.print(" et vitesse RIGHT = ");
    Serial.print(g_rightSpeed);
    Serial.print("\n");
    delay(1);
    currentDistance[LEFT] = MOVE_getDistanceMM(LEFT);
    currentDistance[RIGHT] = MOVE_getDistanceMM(RIGHT);
  }
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, g_leftSpeed);
  MOTOR_SetSpeed(RIGHT, g_rightSpeed);
  SerialPrintf("Distance final est de %i et %i sur %i\n", (int)MOVE_getDistanceMM(LEFT), (int)MOVE_getDistanceMM(RIGHT), (int)distanceMM);
}

void MOVE_Rotation2Roues(float angle){
  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = ((2*PI*MOVE_LARGEUR_ROBOT*angle)/360)/2;
  float speed = 0.7;
  angleEnDistance -= MOVE_GuessDecelerationDistance(0.0, 0.4, 100);//Estimation de la distance requis pour ralentir.
  //Reset les encodeurss
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOVE_vAccelerationInverted(speed, 100);
  int32_t distanceActuelleL = MOVE_getDistanceMM(LEFT);

  while((distanceActuelleL*100/angleEnDistance) < MOVE_SLOW_AT_PERCENT)
  {
    distanceActuelleL = MOVE_getDistanceMM(LEFT);
    //ajuste la vitesse de la roue droite
    g_rightSpeed = -speed;
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    SerialPrintf("Distance fait: %imm\n", distanceActuelleL);
  }

  MOVE_FinduPivot(angleEnDistance, speed);

}

/******************* Fonction pour servomoteur  ************************/

void Ball_kick(){
  if (kick ==0){
    SERVO_Enable(0);
    SERVO_Enable(1);
    SERVO_SetAngle(0,110);
    SERVO_SetAngle(1,0);
    SOFT_TIMER_Enable(TIMER_ID_KICK);
    Serial.print("Kicked the ball.\n");
    kick = 1;
    // ajouter ignore de la lecture du IR sensor du bas
  }
}  

void Kick_return(){
  SERVO_SetAngle(0,0);
  SERVO_SetAngle(1,110);
  Serial.print("Unkicked the ball.\n");
  kick = 0;
  SOFT_TIMER_Enable(TIMER_ID_KICK_DISABLE);
  // enlever l'ignore dans le kick
}

void ballGrab(){
  SERVO_Enable(0);
  SERVO_SetAngle(0, 90);
  SOFT_TIMER_Enable(TIMER_ID_GRAB);
  Serial.print("Picked up golf ball.\n");
  currently_carrying = 1;
}

void ballDrop(){
  SERVO_SetAngle(0,0);
  SOFT_TIMER_Enable(TIMER_ID_DROP);
  Serial.print("Dropped the ball.");
  currently_carrying = 0;
}

void saveBatteriesByDisablingServos(){
  SERVO_Disable(0);
}

/*********************** Fonction pour les capteurs  ************************/

/**
 * @brief Lit le capteur pour la pin analogique donnée
 * 
 * @param ID 
 * @return int32_t 
 */

int32_t CAPTEUR_distanceIR(int ID) 
{
  //Verifie qu'on a un capteur IR
  double distance = 0;
  if(ID == CAPTEUR_IR_DISTANCE_BAS || ID == CAPTEUR_IR_DISTANCE_HAUT)
  {
    int lectureCapteur = analogRead(ID);
    //Formule: https://www.upgradeindustries.com/product/58/Sharp-10-80cm-Infrared-Distance-Sensor-(GP2Y0A21YK0F)
    distance = 123438.5 * pow(lectureCapteur,-1.15);
    if(DEBUG_CAPTEUR)
    {
      Serial.print("Distance(mm): ");
      Serial.print(distance);
      Serial.print("tension(v): ");
      Serial.println(((float)lectureCapteur/0x3ff)*5);
    }
  }
  if(distance > 800)
  {
    distance = 800;
  }
  //Perte des dixième de millimètre
  return (int32_t)distance;
}
void Kick_disable(){
  SERVO_Disable(1);
  SERVO_Disable(0);
}


bool CAPTEUR_detecteurDeLigne(int ID)
{
  bool bRet = false;
  if(ID == CAPTEUR_SUIVEUR_LIGNE_GAUCHE || ID == CAPTEUR_SUIVEUR_LIGNE_MILIEU || ID == CAPTEUR_SUIVEUR_LIGNE_DROIT)
  {
    int lectureCapteur = analogRead(ID);
    float tension = (float)lectureCapteur/0x3ff * 5.0;

    Serial.print("Tension = ");
    Serial.println(tension);

    //Si la tension est supérieur à deux volts, on dit qu'une ligne est vu.
    if(tension >= 2.0)
    {
      bRet = true;
    }
  }
  else
  {
    Serial.print("Mauvaise ID pour sensor de ligne\n");
  }
  return bRet;  
}

void resetSifflet()
{
  sifflet = 0;
}

void checkForSifflet(){

if(analogRead(SIFFLET_PIN) > 410)
  {
    sifflet++;
    Serial.println(sifflet);
  }
  if(sifflet == 1)
  {
    SOFT_TIMER_Enable(TIMER_ID_SIFFLET); 
  }
  if(sifflet >= 27)
  {
    MOTOR_SetSpeed(0,0.) ;
    MOTOR_SetSpeed(1,0.);
    sifflet = 0;
    delay(10000);
  }
}

/******************* Fonctions pour tester les capteurs ***************************/

void test_CapteurIR()
{
  int32_t distance[2];
  distance[0] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_BAS);
  distance[1] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_HAUT);
  Serial.print("Capteur haut(mm)  = "); Serial.println(distance[1]);
  Serial.print("Capteur bas(mm) = "); Serial.println(distance[0]);
}

void test_CapteurCouleur()
{
  unsigned int rouge = RGB_sensor.readRed();
  unsigned int vert = RGB_sensor.readGreen();
  unsigned int bleu = RGB_sensor.readBlue();
  Serial.print("Couleur Rouge = "); Serial.println(rouge, HEX); 
  Serial.print("Couleur Verte = "); Serial.println(vert, HEX); 
  Serial.print("Couleur Bleu = "); Serial.println(bleu, HEX);
}

void test_CapteurLigne()
{
  //Vérifie les lignes
  Serial.print("Detecteur de ligne A13 = "); 
  Serial.println(CAPTEUR_detecteurDeLigne(CAPTEUR_SUIVEUR_LIGNE_GAUCHE) == true?"TRUE": "FALSE"); 
  Serial.print("Detecteur de ligne A14 = "); 
  Serial.println(CAPTEUR_detecteurDeLigne(CAPTEUR_SUIVEUR_LIGNE_MILIEU) == true?"TRUE": "FALSE"); 
  Serial.print("Detecteur de ligne A15 = "); 
  Serial.println(CAPTEUR_detecteurDeLigne(CAPTEUR_SUIVEUR_LIGNE_DROIT) == true?"TRUE": "FALSE"); 
}

/***************** MAIN ****************/

void setup_timers()
{
  SOFT_TIMER_SetDelay(TIMER_ID_KICK, 300);
  SOFT_TIMER_SetRepetition(TIMER_ID_KICK, 1);
  SOFT_TIMER_SetCallback(TIMER_ID_KICK, &Kick_return);
  SOFT_TIMER_SetDelay(TIMER_ID_STATE, 1500);
  SOFT_TIMER_SetRepetition(TIMER_ID_STATE, 1);
  SOFT_TIMER_SetCallback(TIMER_ID_STATE, &changeMode);
  SOFT_TIMER_SetDelay(TIMER_ID_SIFFLET, 3500);
  SOFT_TIMER_SetRepetition(TIMER_ID_SIFFLET, 1);
  SOFT_TIMER_SetCallback(TIMER_ID_SIFFLET, &resetSifflet);
  SOFT_TIMER_SetDelay(TIMER_ID_DROP, 200);
  SOFT_TIMER_SetCallback(TIMER_ID_DROP, &saveBatteriesByDisablingServos);
}

void setup_Sorties()
{
  //Configure les PINS de capteurs IR comme entré sans pull up.
  pinMode(CAPTEUR_IR_DISTANCE_BAS, INPUT);
  pinMode(CAPTEUR_IR_DISTANCE_HAUT, INPUT);

  //Configure les PINS de capteur de ligne comme étant des entrés.
  pinMode(CAPTEUR_SUIVEUR_LIGNE_DROIT, INPUT);
  pinMode(CAPTEUR_SUIVEUR_LIGNE_MILIEU, INPUT);
  pinMode(CAPTEUR_SUIVEUR_LIGNE_GAUCHE, INPUT);
}

int setup_ISL29125()
{
  //Initialise le capteur de couleur ISL29125.
  int defenseur;
  if (RGB_sensor.init())
  {
    Serial.println("Capteur de couleur initialisation: Success\n\r");
    defenseur = GOALER;
  }
  else
  {
    Serial.println("Capteur de couleur initialisation: Failure\n\r");
    defenseur = KICKER;
  }
  return defenseur;
}

void setup_Moteurs(){
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, 0.0);
  MOTOR_SetSpeed(RIGHT, 0.0);

}

bool IsEncodeurStuck(int ID){
  bool ret = false;
  static unsigned long startTime[2] = {millis(), millis()};
  unsigned long currentTime = millis();
  static int32_t previousEncoder[2];
  memset(previousEncoder, 0, sizeof(previousEncoder));
  int32_t currentEncoder = ENCODER_Read(ID);

  if(previousEncoder[ID] > currentEncoder)
  {
    previousEncoder[ID] = 0;
  }
  else
  {
    if(previousEncoder[ID] <= (currentEncoder+100))
    {
      if(currentTime-startTime[ID] > 3000)
      {
        ret = true;
        startTime[ID] = millis();
      }
    }
    else
    {
      startTime[ID] = millis();
    }
  }
  previousEncoder[ID] = currentEncoder;
  return ret;
}

bool IsObstacle(int distance_bas, int distance_haut){
  bool ret = false;
  if(distance_bas <= 250 && distance_haut <= 250)
  {
    ret = true;
  }
  return ret;
}

bool IsBalle(int distance_bas, int distance_haut){
  bool ret = false;
  Serial.print("Distance_bas = "); Serial.println(distance_bas);
  if(distance_bas < 150)
  {
    ret = true;
  }
  return ret;
}

void changeMode(){
  robot_state = avance;
  ENCODER_Reset(RIGHT);
  ENCODER_Reset(LEFT);
}

void attaquant(){
  int distance[2] = {0};
  static state_t previousState = avance;
  static int counter = 0;
  distance[0] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_BAS);
  distance[1] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_HAUT);
  //checkForSifflet();
  checkForSifflet();

  if(robot_state == avance && robot_state != recule)
  {
    Serial.println(counter);
    if(counter > 100)
    {
      robot_state == recule;
      counter = 0;
    }
    //Si un mur ou robot
    else if(IsObstacle(distance[0], distance[1]))
    {
      //Coupe les moteurs
      robot_state = stop;
      Serial.print("Obstacle!\n");
    }
    //Si une balle
    else if(IsBalle(distance[0], distance[1]))
    {
      robot_state = kick_ball;
      Serial.print("Balle!\n");
    }
    //Mode par défaut
    else
    {
      robot_state = avance;
     // Serial.print("Avance!\n");
    }
  }

  switch(robot_state)
  {
    case avance:
      MOTOR_SetSpeed(LEFT, 0.7);
      MOTOR_SetSpeed(RIGHT, 0.7+fSpeedAdjustment());
      counter++;
      break;
    case stop:
      MOTOR_SetSpeed(LEFT, 0.0);
      MOTOR_SetSpeed(RIGHT, 0.0);
      break;
    case kick_ball:
      Ball_kick();
      robot_state = avance;
      break;
    case pivote:
      if(previousState != robot_state)
      {
        SOFT_TIMER_Enable(TIMER_ID_STATE);
      }
      MOTOR_SetSpeed(LEFT, 0.3);
      MOTOR_SetSpeed(RIGHT, -0.3);
      break;
    case recule:
      if(previousState != robot_state)
      {
        SOFT_TIMER_Enable(TIMER_ID_STATE);
      }
      MOTOR_SetSpeed(LEFT, -0.4);
      MOTOR_SetSpeed(RIGHT, -0.7);
      break;
  }
  delay(20);
  previousState = robot_state;
  if(robot_state == stop)
  {
    robot_state = pivote;
  }
}

void goaler(){
  MOVE_vAvancer(0.3, 350);
  MOVE_vAvancer(-0.3,-350);
}


void pirUS(){
	delay(100);
	bool lastPos = LEFT; // derniere position de l'objet a partir de la camera
	if (pixy.ccc.getBlocks(true, 1) > 0)
	{
		float ratio;

		int blockPos = 0;
		for (int i = 0; i < pixy.ccc.numBlocks; i++)
		{
			if (pixy.ccc.blocks[i].m_signature == 0)
				blockPos = i;
		}

		if (pixy.ccc.blocks[blockPos].m_height < 60)
		{
			g_leftSpeed = g_rightSpeed = 0.15;
		}

		if (pixy.ccc.blocks[blockPos].m_height > 80)
		{
			g_leftSpeed = g_rightSpeed = -0.15;
		}

		ratio = ((float)pixy.ccc.blocks[blockPos].m_x) / ((float)(pixy.frameWidth / 2.0));
		Serial.print("ratio non mod : ");
		Serial.println(ratio);
		Serial.println();
		if (pixy.ccc.blocks[blockPos].m_height <= 80 && pixy.ccc.blocks[blockPos].m_height >= 60)
		{
			if (ratio >= 1.1)
			{
				g_leftSpeed = 0.1;
				g_rightSpeed = -g_leftSpeed;
			}
			if (ratio <= 0.9)
			{
				g_leftSpeed = -0.1;
				g_rightSpeed = 0.1;
			}
			if (ratio < 1.1 && ratio > 0.9)
				g_leftSpeed = g_rightSpeed = 0.0;
		}

		if (ratio > 1.0)
			lastPos = RIGHT;
		else
			lastPos = LEFT;


		if ((ratio > 1.1 || ratio < 0.9) && (g_leftSpeed > 0.0 && g_rightSpeed > 0.0))
		{
			float ratioDivider = 2.0;
			ratio /= ratioDivider;
			ratio += 0.5;
			g_leftSpeed *= ratio;
		}
		Serial.print("ratio mod : ");
		Serial.println(ratio);
		Serial.println();
		Serial.println();

		
	}
	else
	{
		if (lastPos == LEFT)
		{
			g_leftSpeed = -0.13;
			g_rightSpeed = 0.13;
		}
		else
		{
			g_leftSpeed = 0.13;
			g_rightSpeed = - 0.13;
		}
	}

	MOTOR_SetSpeed(LEFT, g_leftSpeed);
	MOTOR_SetSpeed(RIGHT, g_rightSpeed);
}  

void setup(){
  BoardInit();
  Serial.begin(9600);
  setup_Moteurs();
  setup_Sorties();
  setup_timers();
  MOTOR_SetSpeed(RIGHT,0);
  MOTOR_SetSpeed(LEFT,0);
  
  delay(5000);
}

void loop() {
SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
static int i=0;
if (i==5){
  ballGrab();
  }
if (i==15){
  ballDrop();
  i=0;
}
i++;
delay(100);
}

