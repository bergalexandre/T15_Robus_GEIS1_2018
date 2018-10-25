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

/*******************************************************************************
 * Define
*******************************************************************************/

#define DELAY 50 // Delay in ms
#define MS_PER_SECOND 1000
#define TIMER_ID_KICK 1

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
int kick = 0;
SFE_ISL29125 RGB_sensor;

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
void MOVE_vAccelerationSingleWheel(float finalSpeed, unsigned int time, int ID, int32_t distanceMM)
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

void MOVE_vAvancer(float fVitesse, int32_t i32Distance_mm,unsigned int accelerationTime = 125){
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOVE_vAcceleration(fVitesse, accelerationTime);
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

void MOVE_finDuVirage(int ID, int32_t distanceMM, float speed)
{
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
void MOVE_Rotation1Roue(float angle, int iRotationDirection)
{
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

void MOVE_FinduPivot(int32_t distanceMM, float speed)
{
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

void MOVE_Rotation2Roues(float angle)
{
  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = ((2*PI*MOVE_LARGEUR_ROBOT*angle)/360)/2;
  float speed = 0.5;
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
    SERVO_SetAngle(0,90);
    SOFT_TIMER_Enable(TIMER_ID_KICK);
    Serial.print("Kicked the ball.\n");
    kick = 1;
  }
}  

void Kick_return(){
  SERVO_SetAngle(0,0);
  Serial.print("Unkicked the ball.\n");
  kick = 0;
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

/******************* Fonctions pour tester les capteurs ***************************/

void test_CapteurIR()
{
  int32_t distance[2];
  distance[BAS] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_BAS);
  distance[HAUT] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_HAUT);
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
  SOFT_TIMER_SetDelay(TIMER_ID_KICK, 500);
  SOFT_TIMER_SetRepetition(TIMER_ID_KICK, 1);
  SOFT_TIMER_SetCallback(TIMER_ID_KICK, &Kick_return);
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

void setup_ISL29125()
{
  //Initialise le capteur de couleur ISL29125.
  if (RGB_sensor.init())
  {
    Serial.println("Capteur de couleur initialisation: Success\n\r");
  }
  else
  {
    Serial.println("Capteur de couleur initialisation: Failure\n\r");
  }
}

void setup_Moteurs()
{
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, 0.0);
  MOTOR_SetSpeed(RIGHT, 0.0);
  SERVO_Enable(0);
}

void setup(){
  BoardInit();
  //Ne pas changer la valeur puisque le capteur de couleur communique en 115200 Bauds.
  Serial.begin(115200);
  setup_ISL29125();
  setup_Moteurs();
  setup_Sorties();
  setup_timers();
}

void loop() {
  SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  int distance[2] = {0};
  distance[0] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_BAS);
  distance[1] = CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_HAUT);

  //Vérifie que le capteur du haut et du bas retourne une distance différente de 10cm
  if((distance[1]-distance[0]) > 100)
  {
    //Vérifie que la balle est à proximiter de 15cm.
    if(distance[0] < 150)
    {
      Ball_kick();
    }
  }
  delay(100);
}
