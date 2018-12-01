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
#include <Pixy2SPI_SS.h>
#include <Arduino.h>

/*******************************************************************************
 * Define
*******************************************************************************/

#define TIMER_ID_STATE 1
#define TIMER_ID_DROP 2
#define TIMER_ID_GO 3

//Define pour suiveur de ligne.
// 0,0 en haut à gauche
#define LINE_HEIGHT 51
#define LINE_WIDTH 78
#define NB_VECTOR_TO_COMPARE 1
#define SLAVE_PIN_PIXY 40

#define FULL_TURN (2*PI*MOVE_LARGEUR_ROBOT*180)/360

/*******************************************************************
 * Typedef / enum / struct
 *******************************************************************/

typedef enum
{
  camera_suiveur_ligne,
  camera_detecteur_blocks
} camera_mode_t;

typedef enum
{
   ligne_gauche,
   ligne_centre,
   ligne_tres_au_centre,
   ligne_droite
} position_ligne_t;

typedef enum
{
   Golfotron_idle,
   Golfotron_Seek,
   Golfotron_Ambush_golf_ball,
   Golfotron_Eliminate,
   Golfotron_Fin_De_Ligne,
   Golfotron_retour_a_la_base
} Golfotron_state_t;

typedef enum
{
   seek_Balle_trouve,
   seek_Recherche_en_cours,
   seek_Pas_de_balle
} Seek_GolfBall_t;

typedef enum
{
   suiveur_but,
   suiveur_depart,
   suiveur_rien,
   suiveur_Check_20cm
} Suiveur_position_t;
/*******************************************************************************
 * Prototypes locaux
*******************************************************************************/

void changeMode(camera_mode_t mode);
void updateGolfotron_State();

/*******************************************************************************
 * Variables/classe locales
*******************************************************************************/

Pixy2SPI_SS pixy;
float g_leftSpeed = 0.0;
float g_rightSpeed = 0.0;
int currently_carrying = 0;
int angle_From_Line = 0;
int distance_From_Line = 0;
Golfotron_state_t Golfotron_state = Golfotron_idle;
Golfotron_state_t Golfotron_PreviousState = Golfotron_idle;


/*******************************************************************************
 * fonctions
*******************************************************************************/


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
 * @brief Distance que le robot à parcouru depuis la dernière fois que l'encoder s'est fait reset.
 * 
 * @param ID ID de l'encodeur à lire.
 * @return int32_t Retourne la distance en millimètre.
 */
int32_t MOVE_getDistanceMM(int ID)
{
   int32_t d = ENCODER_Read(ID)*((float)1/MOVE_PULSE_PER_TURN)*MOVE_WHEEL_DIAMETER*PI;
   if(DEBUG_CAPTEUR)
   {
      Serial.print("distance = ");
      Serial.println(d);
   }
   return d;
}

void MOVE_vSetSpeed()
{
   MOTOR_SetSpeed(LEFT, g_leftSpeed);
   MOTOR_SetSpeed(RIGHT, g_rightSpeed);
}

void MOVE_vStop()
{
   g_leftSpeed = 0.0;
   g_rightSpeed = 0.0;
   MOTOR_SetSpeed(LEFT, g_leftSpeed);
   MOTOR_SetSpeed(RIGHT, g_rightSpeed);
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
  float estimate = ((float)timeMs/1000)*(float)MOVE_WHEEL_DIAMETER*speed;
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
    g_leftSpeed += speedIncrementation;
    g_rightSpeed = -g_leftSpeed + fSpeedAdjustment();
    MOTOR_SetSpeed(RIGHT, g_rightSpeed);
    MOTOR_SetSpeed(LEFT, g_leftSpeed);
    delay(wait);
  }

  g_leftSpeed = finalSpeed;
  g_rightSpeed = finalSpeed;
  MOTOR_SetSpeed(LEFT, g_leftSpeed);
  MOTOR_SetSpeed(RIGHT, g_leftSpeed);
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

   if(DEBUG_CAPTEUR)
   {
      Serial.print(g_leftSpeed);
      Serial.print(" et vitesse RIGHT = ");
      Serial.print(g_rightSpeed);
      Serial.print("\n");
   }
    delay(1);
    currentDistance[LEFT] = MOVE_getDistanceMM(LEFT);
    currentDistance[RIGHT] = MOVE_getDistanceMM(RIGHT);
  }
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, g_leftSpeed);
  MOTOR_SetSpeed(RIGHT, g_rightSpeed);
}

void MOVE_Rotation2Roues(float angle)
{
  //Consigne de distance pour la roue opposé au virage afin d'arriver à l'angle voulu. 
  int32_t angleEnDistance = ((2*PI*MOVE_LARGEUR_ROBOT*angle)/360)/2;
  float speed = 0.15;
  angleEnDistance -= MOVE_GuessDecelerationDistance(0.0, 0.15, 100);//Estimation de la distance requis pour ralentir.
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
  }

  MOVE_FinduPivot(angleEnDistance, speed);

}


void MOVE_vAvancer(float fVitesse, int32_t i32Distance_mm,unsigned int accelerationTime = 125)
{
   ENCODER_Reset(0);
   ENCODER_Reset(1);

   g_leftSpeed = fVitesse;
   MOTOR_SetSpeed(LEFT,g_leftSpeed);

   if(fVitesse < 0)
   {
      if(i32Distance_mm > 0)
      {
         i32Distance_mm = i32Distance_mm;
      }
   }

   while(abs(MOVE_getDistanceMM(LEFT)) < i32Distance_mm)
   {
      if(DEBUG_CAPTEUR)
      {
         Serial.print("Distance fait = ");
         Serial.print(MOVE_getDistanceMM(LEFT));
         Serial.println(" mm");
      }
      g_rightSpeed = fVitesse + fSpeedAdjustment();
      MOTOR_SetSpeed(RIGHT, g_rightSpeed);
      delay(50);
   }
  
   MOVE_vStop();
}

/******************* Fonction pour servomoteur  ************************/

void ballGrab(int angle = 58){
  SERVO_Enable(0);
  SERVO_SetAngle(0, angle);
  SOFT_TIMER_Enable(TIMER_ID_DROP);
  Serial.print("Picked up golf ball.\n");
  currently_carrying = 1;
}

void ballDrop(){
  SERVO_Enable(0);
  SERVO_SetAngle(0,0);
  SOFT_TIMER_Enable(TIMER_ID_DROP);
  Serial.print("Dropped the ball.\n");
  currently_carrying = 0;
}

void saveBatteriesByDisablingServos(){
  SERVO_Disable(0);
}

bool getBall(){
   bool bRet = true;
   ENCODER_Reset(0);
   ENCODER_Reset(1);
   while(CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_BAS)>75){
      MOTOR_SetSpeed(LEFT,0.15);
      MOTOR_SetSpeed(RIGHT,0.15+fSpeedAdjustment());
      if(MOVE_getDistanceMM(LEFT) > 300)
      {
         //On a probablement perdu la balle.
         Serial.print("Where did the ball go?\n");
         bRet = false;
         MOVE_vAvancer(-0.2, 300);
         break;
      }
   }
   if(bRet == true)
   {
      Serial.print("La balle est icitte meyn!\n");
      delay(80);
      ballGrab(58);
      delay(100);
      MOVE_vStop();
   }
   else
   {
      MOVE_vStop();
   }
   distance_From_Line += ENCODER_Read(LEFT);  
   retourLigne(angle_From_Line, distance_From_Line);
   return bRet;
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

/***************** Demo ****************/

void demo_claw()
{
   static int i=0;
   if (i==5){
   ballDrop();
   }
   if (i==15){
   ballGrab(60);
   }
   if (i==20){
   ballDrop();
   i=0;
   }

   i++;
}

void demo_spin_claw()
{
   SOFT_TIMER_Update();
   ballGrab(70);
   delay(1000);
   SOFT_TIMER_Update();
  while(1)
  {
      MOTOR_SetSpeed(LEFT, 0.30);
      MOTOR_SetSpeed(RIGHT, -0.30);
      delay(5000);
      MOTOR_SetSpeed(LEFT, -0.30);
      MOTOR_SetSpeed(RIGHT, 0.30);
      delay(5000);
  }
}

/***************** Suiveur de ligne *******************/


float ValueFromPercent(float value, int percent)
{
   return ((float)value*percent)/100.0;
}

bool isVectorHorizontal(Vector ligne)
{
   int vecteur_x = abs(ligne.m_x1-ligne.m_x0);
   int vecteur_y = abs(ligne.m_y1-ligne.m_y0);
   bool bRet = false;
   if(vecteur_y == 0)
   {
      return bRet;
   }
   float ratio_x_on_y = (float)vecteur_x/vecteur_y;
   Serial.print("Vecteur, ratio x/y = ");
   Serial.println(ratio_x_on_y);
   if(ratio_x_on_y >= 3.0)
   {
      bRet = true;
   }
   return bRet;
}

bool isVectorVertical(Vector ligne)
{
   int vecteur_x = abs(ligne.m_x1-ligne.m_x0);
   int vecteur_y = abs(ligne.m_y1-ligne.m_y0);
   bool bRet = false;
   if(vecteur_x == 0)
   {
      return bRet;
   }
   float ratio_y_on_x = (float)vecteur_y/vecteur_x;
   Serial.print("Vecteur, ratio y/x = ");
   Serial.println(ratio_y_on_x);
   if(ratio_y_on_x >= 3.0)
   {
      bRet = true;
   }
   return bRet;
}

/**
 * @brief Ajuste la vitesse de la roue gauche en fonction de 
 * 
 * @param ligne Le vecteur Pixy qui doit être analysé
 * @return int 
 */

position_ligne_t IsRobotCenter(int value)
{
   //Trop à gauche
   position_ligne_t position = ligne_centre;
   if(value < ValueFromPercent(LINE_WIDTH, 50) && value < ValueFromPercent(LINE_WIDTH, 20))
   {
      Serial.print("Robot trop à gauche\n");
      position = ligne_gauche;
   }
   else if(value > ValueFromPercent(LINE_WIDTH, 50) && value > ValueFromPercent(LINE_WIDTH, 80))
   {
      Serial.print("Robot trop à droite\n");
      position = ligne_droite;
   }
   else
   {
      Serial.print("Robot est au centre\n");
      position = ligne_centre;
   }

   return position;
}

/**
 * @brief Compare 2 vecteurs
 * 
 * @param ligne1 
 * @param ligne2 
 * @return true si assez similaire
 */
bool bLigneCompare(Vector ligne1, Vector ligne2)
{
   bool bRet = false;
   //les lignes doivent être proche de 8 pixels;
   int threshold = 10;
   //int X_threshold = ValueFromPercent(LINE_WIDTH, threshold);
   //int Y_threshold = ValueFromPercent(LINE_HEIGHT, threshold);

   if(abs(ligne1.m_x0 - ligne2.m_x0) <= threshold &&
      abs(ligne1.m_x1 - ligne2.m_x1) <= threshold &&
      abs(ligne1.m_y0 - ligne2.m_y0) <= threshold &&
      abs(ligne1.m_y1 - ligne2.m_y1) <= threshold) 
   {
      bRet = true;
   }
   return bRet;
}

bool pixyGetLigne(Vector* ligne_retour)
{
   bool bRet = false;
	char buf[128] = {0};
   Vector Vecteur[NB_VECTOR_TO_COMPARE] = {0};
	// print all vectors
   for(int i = 0; i < NB_VECTOR_TO_COMPARE; i++)
   {
      int8_t pixy_line = pixy.line.getMainFeatures(LINE_VECTOR, true);
      if(pixy_line != PIXY_RESULT_BUSY && pixy_line != PIXY_RESULT_ERROR)
      {
         if(pixy_line & LINE_VECTOR)
         {
            //Pour suivre la ligne toujours dans le même sens, on vérifie que m_x0 et m_y0 sont en bas de la caméra
            if(pixy.line.vectors->m_y0 < pixy.line.vectors->m_y1)
            {
               Vector oldVecteur;
               memcpy(&oldVecteur, pixy.line.vectors, sizeof(Vector));
               pixy.line.vectors->m_y0 = oldVecteur.m_y1;
               pixy.line.vectors->m_y1 = oldVecteur.m_y0;
               pixy.line.vectors->m_x0 = oldVecteur.m_x1;
               pixy.line.vectors->m_x1 = oldVecteur.m_x0;
            }
            memcpy(&Vecteur[i], pixy.line.vectors, sizeof(Vector));

            bRet = true;
         }
      }
   }

   //Compare les trois vecteurs, ils doivent avoir les mêmes coordoné à + ou - 20%. Sinon le robot risque de freak out 
   //si il n'y a pas de ligne.
   //Pour éviter les faux positif on compare le vecteur trois fois.
   if(bRet)
   {
      bRet = false;
      int vecteurSimilaire = 0;
      for(int i = 1; i < NB_VECTOR_TO_COMPARE; i++)
      {
         vecteurSimilaire += bLigneCompare(Vecteur[0], Vecteur[i]);
      }
      if(vecteurSimilaire > 5)
      {
         bRet = true;
         *ligne_retour = Vecteur[0];
         sprintf(buf, "ligne: ");
         Serial.print(buf);
         Vecteur[0].print();
      }
      else if(NB_VECTOR_TO_COMPARE == 1)
      {
         bRet = true;
         *ligne_retour = Vecteur[0];
         sprintf(buf, "ligne: ");
         Serial.print(buf);
         Vecteur[0].print();
      }
      
   }

   return bRet;
}

/**
 * @brief Get the Pixy Bar Code object
 * 
 * @param depart 
 * @return true 
 * @return false 
 */
bool GetPixyBarCode(Barcode* depart)
{
   bool bRet = false;
   int8_t pixy_line = pixy.line.getMainFeatures(LINE_BARCODE, true);
   if(pixy_line & LINE_BARCODE)
   {
      for(int i = 0; i < pixy.line.numBarcodes; i++)
      {
         if(pixy.line.barcodes[i].m_code == 6)
         {
            bRet = true;
            memcpy(depart, &pixy.line.barcodes[i], sizeof(Barcode));
            pixy.line.barcodes[i].print();
            break;
         }
      }
   }

   return bRet;
}

/**
 * @brief Vérifie si le barcode est à au moins (20cm?) de distance.
 * 
 */
bool bIsBarCodeClose(Barcode depart)
{
   bool bRet = false;
   if(depart.m_y > ValueFromPercent(LINE_HEIGHT, 80))
   {
      Serial.print("Je suis proche du barcode\n");
      bRet = true;
   }
   return bRet;
}

/**
 * @brief La fonction doit se rapprocher de la ligne juste à temps qu'elle n'est plus visible.
 * 
 */
void getCloserToLine()
{
   Vector ligne;
   bool lignePerdu = false;
   while(lignePerdu == false)
   {
      if(pixyGetLigne(&ligne) == true)
      {
         g_leftSpeed = 0.2;
         g_rightSpeed = 0.2;
         //Essaie de rester perpendiculaire à la ligne.
         g_rightSpeed -= ((float)(ligne.m_x0-ligne.m_x1)/100);
         if(g_rightSpeed < 0.15)
         {
            g_rightSpeed = 0.15;
         }
         else if(g_rightSpeed > 0.25)
         {
            g_rightSpeed = 0.25;
         }
      }
      else
      {
         lignePerdu = true;
         Serial.print("Je suis en avant de la ligne\n");
         MOVE_vStop();
      }
      MOVE_vSetSpeed();
      delay(20);
   }
}

void retourLigne(int angle, int distance)
{
   Serial.print("retour a la ligne avec Angle//Distance = ");
   Serial.print(angle);
   Serial.print(" // ");
   Serial.print(distance);
   //MOVE_Rotation2Roues(360-angle);
   //MOVE_vAvancer(0.20,distance);
}

void GetAjustementX0(float* left, float* right, Vector ligne)
{
   //En dessous de 1 ligne à gauche, sinon à droite   
   float position_ligne = (float)ligne.m_x0/(ValueFromPercent(LINE_WIDTH, 50));
   float vitesseMax = 0.20;

   Serial.print("Position ligne = ");
   Serial.println(position_ligne);
   if(position_ligne < 1.0)
   {
      //Ligne est à gauche
      *left = (float)((float)position_ligne)*(vitesseMax);
      *right = vitesseMax;
   }
   else
   {
      //Ligne est à droite
      *left = vitesseMax;
      *right = (float)((float)2.0-position_ligne)*vitesseMax;
   }
   
}

bool findLine(bool avance = true)
{
   Serial.print("fonction findline\n");
   Vector ligne = {0};
   int iCompteur = 0;
   bool ligneTrouve = false;
   if(avance == true)
   {
      MOVE_vAvancer(0.2, 200);
   }

   ENCODER_Reset(LEFT);
   ENCODER_Reset(RIGHT);
   while(abs(MOVE_getDistanceMM(LEFT)) < FULL_TURN)
   {
      Serial.print("Entrain de tourner: ");
      Serial.print(MOVE_getDistanceMM(LEFT));
      Serial.print(" ");
      Serial.println(FULL_TURN);
      g_leftSpeed = -0.13;
      g_rightSpeed = 0.13;
      MOVE_vSetSpeed();
      if(pixyGetLigne(&ligne))
      {
         if(isVectorVertical(ligne))
         {
            ligneTrouve = true;
            Serial.print("Le robot est aligne avec la ligne\n");
            return ligneTrouve;
         }
         else if(avance == false && iCompteur > 4)
         {
            Serial.print("avance\n");
            while(isVectorVertical(ligne) == false)
            {
               if(pixyGetLigne(&ligne))
               {
                  GetAjustementX0(&g_leftSpeed, &g_rightSpeed, ligne);
                  MOVE_vSetSpeed();
                  delay(100);
               }
            }
            ENCODER_Reset(LEFT);
            ENCODER_Reset(RIGHT);
            MOVE_vStop();
            iCompteur = 0;
            return true;
         }
         else
         {
            iCompteur++;
         }
      }
      delay(50);
   }
   MOVE_vStop();
   return ligneTrouve;
}

void AucunVecteur(Vector previousLine, int* angle)
{
   static bool bMem = false;
   printf("aucun vecteur\n");
   //On ne voit plus de ligne
   if(bMem == true)
   {
      MOVE_vAvancer(0.2, 100);
      ENCODER_Reset(LEFT);
      ENCODER_Reset(RIGHT);
      *angle = 0;
      bMem = false;
   }
   else
   {
      bMem = findLine(false);
   }
}

Suiveur_position_t mode_ligne()
{
   Suiveur_position_t suiveur_ligne = suiveur_rien;
   static int angle_PasDeLigne = 0;       //Angle fait depuis que la ligne n'est pas vu.
   static Vector previousLine = {0};
   Barcode depart;
   Vector ligne = {0};
	// print all vectors
   if(MOVE_getDistanceMM(LEFT) > 800 && currently_carrying == false)
   {
      ENCODER_Reset(LEFT);
      ENCODER_Reset(RIGHT);
      return suiveur_Check_20cm;
   }

   if(pixyGetLigne(&ligne) == true)
   {
      if(isVectorVertical(ligne))
      {
         GetAjustementX0(&g_leftSpeed, &g_rightSpeed, ligne);
         Serial.print("Ajuste la vitesse gauche et droite: ");
         Serial.print(g_leftSpeed);
         Serial.print(" // ");
         Serial.print(g_rightSpeed);
      }
      else if(isVectorHorizontal(ligne))
      {
         //Robot doit revenir vers la ligne
         Serial.print("Le robot est trop loin de la ligne\n");
         //getCloserToLine();
         findLine(false);
         MOVE_vStop();
      }
      else
      { 
         findLine(false);
      }
      previousLine = ligne;
   }
   else
   {
      findLine(false);
   }

   //Ajouter les barcodes ici
   if((GetPixyBarCode(&depart) == true) && (currently_carrying == true))
   {
      suiveur_ligne = suiveur_depart;
   }
   MOVE_vSetSpeed();
   delay(100);
   return suiveur_ligne;
}

/***************** block ***************/

bool GetBlock(Block* block, int signature)
{
   bool bRet = false;
   if (pixy.ccc.getBlocks(true, signature) > 0) //rentre s'il y a un objet de détecté
	{
		for (int i = 0; i < pixy.ccc.numBlocks; i++) // trouve la position de la balle dans l'array
		{
         pixy.ccc.blocks[i].print();
            //Compare le ratio largeur/hauteur
            // if(((pixy.ccc.blocks[i].m_width >= ValueFromPercent(pixy.ccc.blocks[i].m_height, 60)) && 
            // (pixy.ccc.blocks[i].m_width <= pixy.ccc.blocks[i].m_height)) ||
            // ((pixy.ccc.blocks[i].m_height >= ValueFromPercent(pixy.ccc.blocks[i].m_width, 60)) && 
            // (pixy.ccc.blocks[i].m_height <= pixy.ccc.blocks[i].m_width)))
            // {
         bRet = true;
         *block = pixy.ccc.blocks[i];
            // }
		}
   }
   return bRet;
}

/**
 * @brief Trouve la balle de golf
 * 
 * @return true Vrai si repéré et robot centré sur la belle.
 * @return false Faux si pas de balle.
 */
Seek_GolfBall_t Find_Golf_Ball(){
   Seek_GolfBall_t tRet = seek_Recherche_en_cours;
   Block balle;

	if (GetBlock(&balle, CCC_SIG1) == true) //rentre s'il y a un objet de détecté
	{
		float ratio;
      Serial.print("Balle trouvee.\n");

		if (balle.m_y < 170) // si la balle est trop loin avance
		{
         Serial.print("Balle trop loin.\n");
			g_rightSpeed = 0.15;
         g_leftSpeed = g_rightSpeed;

         distance_From_Line = MOVE_getDistanceMM(LEFT);
		}

		if (balle.m_y > 190)// si trop proche recule
		{
         Serial.print("Balle trop proche.\n");
			g_rightSpeed = -0.15;
         g_leftSpeed = g_rightSpeed;
		}

		ratio = ((float)balle.m_x) / ((float)(pixy.frameWidth / 2.0)); //ratio de la postion en x de la ball va de 0 à 2 0 étant a gauche et 2 a droite
	
		if (balle.m_y <= 190 && balle.m_y >= 170) // si la balle est dans le bon threshold de distance, recentre la balle
		{
         Serial.print("On recentre la balle.\n");
			if (ratio >= 1.1)
			{
            Serial.print("Balle trop a droite.\n");
				g_leftSpeed = 0.1;
				g_rightSpeed = 0;
			}
			else if (ratio <= 0.9)
         {
            Serial.print("Balle trop a gauche.\n");
				g_leftSpeed = 0.1;
				g_rightSpeed = 0;
			}
			else if (ratio < 1.1 && ratio > 0.9){
				Serial.print("Balle centree.\n");
            g_leftSpeed = g_rightSpeed = 0.0;
            tRet = seek_Balle_trouve;
         }
		}	
	}
	else
	{
      g_leftSpeed = -0.13;
      g_rightSpeed = 0.13;
      Serial.print(MOVE_getDistanceMM(LEFT));
      if(abs(MOVE_getDistanceMM(LEFT)) > FULL_TURN)
      {
         Serial.print("Pas de balle trouve, tour complet de fait\n");
         tRet = seek_Pas_de_balle;
      }
      angle_From_Line = (abs(MOVE_getDistanceMM(LEFT))*360)/(2*PI*MOVE_LARGEUR_ROBOT);
	}

	MOVE_vSetSpeed();
   delay(10);
   return tRet;
}

void CheckGolfBall()
{
   Block balle;
   Serial.print("Check for golf ball\n");
   if(GetBlock(&balle, CCC_SIG1))
   {
      Serial.print("Reset timer_id_go\n");
      SOFT_TIMER_Enable(TIMER_ID_GO);
   }
   delay(500);
}

void ReleaseBall()
{
   MOVE_vStop();
   MOVE_Rotation2Roues(300);
   MOVE_vAvancer(0.2, 200);
   delay(200);
   ballDrop();
   MOVE_vAvancer(-0.2, 200);
}

bool StopWithGreen()
{
   bool bRet = false;
   Block vert;
   float vitesseMax = 0.2;
   if(GetBlock(&vert, CCC_SIG2) == true)
   {
      //Largeur = 110, hauteur = 50 
      if((vert.m_width < 110) && (vert.m_width < 50))
      {
         float position = ((float)vert.m_x/(ValueFromPercent(pixy.frameWidth,50)));
         Serial.println(position);
         Serial.println((ValueFromPercent(pixy.frameWidth,50)));
         if(position > 1.0)
         {
            g_leftSpeed = (float)((float)2.0-position)*vitesseMax;
            g_rightSpeed = vitesseMax;
         }
         else
         {
            g_leftSpeed = (float)((float)position)*vitesseMax;
            g_rightSpeed = vitesseMax;
         }      
      }
      else
      {
         bRet = true;
         g_leftSpeed = 0.2;
         g_rightSpeed = 0.2 + fSpeedAdjustment();
      }
   }
   else
   {
      g_leftSpeed = vitesseMax;
      g_rightSpeed = vitesseMax + fSpeedAdjustment();
   }
   Serial.println(g_leftSpeed);
   Serial.println(g_rightSpeed);
   delay(20);
   //MOVE_vSetSpeed();
   return bRet;
}

/***************** MAIN ****************/

void updateGolfotron_State()
{
   Serial.print("Check timer: ");
   Serial.println(MOVE_getDistanceMM(LEFT));

   if(((abs((MOVE_getDistanceMM(LEFT)))) > 200) && (currently_carrying == false) && (Golfotron_state == Golfotron_Seek))
   {
      Serial.print("Cherche la balle\n");
      Golfotron_state = Golfotron_Ambush_golf_ball;
   }
   //SOFT_TIMER_Enable(TIMER_ID_STATE);
}

void Golfotron_depart()
{
   Golfotron_state = Golfotron_Seek;
}


void changeMode(camera_mode_t mode)
{
   int changeProg;
   int changeLight;
	switch(mode)
	{
	case camera_suiveur_ligne:
		changeProg = pixy.changeProg("line");
      Serial.print("Mode suiveur de ligne\n");
      delay(20);
      changeLight = pixy.setCameraBrightness(40);
      Serial.print("Mode suiveur de ligne 2\n");
		break;
	case camera_detecteur_blocks:
		pixy.changeProg("color_connected_components");
      Serial.print("Mode block\n");
      delay(20);
      pixy.setCameraBrightness(40);
		break;
	default:
		pixy.changeProg("color_connected_components");
      Serial.print("Mode block\n");
      delay(20);
      pixy.setCameraBrightness(28);
		break;
   }
   if(changeProg != PIXY_RESULT_OK)
   {
      Serial.print("Erro while changing prog\n");
      //changeMode(mode);
   }
   if(changeLight == PIXY_RESULT_ERROR)
   {
      Serial.print("Erro while changing light\n");
      //changeMode(mode);
   }
   Serial.print("Mode suiveur de ligne 3\n");
}

void setup_timers()
{
  SOFT_TIMER_SetDelay(TIMER_ID_DROP, 200);
  SOFT_TIMER_SetCallback(TIMER_ID_DROP, &saveBatteriesByDisablingServos);
  SOFT_TIMER_SetRepetition(TIMER_ID_DROP, 1);
  SOFT_TIMER_SetDelay(TIMER_ID_GO, 5000);
  SOFT_TIMER_SetCallback(TIMER_ID_GO, &Golfotron_depart);
  SOFT_TIMER_SetRepetition(TIMER_ID_GO, 1);
  SOFT_TIMER_SetDelay(TIMER_ID_STATE, 500);
  SOFT_TIMER_SetRepetition(TIMER_ID_STATE, -1);
  SOFT_TIMER_SetCallback(TIMER_ID_STATE, &updateGolfotron_State);
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

void setup_Moteurs(){
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, 0.0);
  MOTOR_SetSpeed(RIGHT, 0.0);

}

void setup(){
   distance_From_Line = 0;
   pinMode(SLAVE_PIN_PIXY, OUTPUT);
   digitalWrite(SLAVE_PIN_PIXY, 1);
   BoardInit();
   Serial.begin(9600);
   setup_Moteurs();
   setup_Sorties();
   setup_timers();
   SOFT_TIMER_Update();
   ballDrop();
   //Attend que le pixy soit ready:
   while(pixy.init(SLAVE_PIN_PIXY) == PIXY_RESULT_TIMEOUT)
   {
      SOFT_TIMER_Update();
      delay(100);
   };
   changeMode(camera_detecteur_blocks); 
   //MOVE_vAvancer(0.2, 600);
}


void logique()
{
   static int i = 0;
   Serial.print("Golfotron Mode = ");
   Serial.println(Golfotron_state);
   switch(Golfotron_state)
   {
      //Attend que la balle soit potter
      case Golfotron_idle:
      {
         if(Golfotron_PreviousState != Golfotron_state)
         {
            changeMode(camera_detecteur_blocks);
         }
         CheckGolfBall();
         break;
      }
      //Mode suiveur de ligne
      case Golfotron_Seek:
      {
         //Configuration initiale du mode
         if(Golfotron_PreviousState != Golfotron_state)
         {
            bool bLigne = false;
            changeMode(camera_suiveur_ligne);
            Serial.print("try to reset the encoder\n");
            ENCODER_Reset(LEFT);
            ENCODER_Reset(RIGHT);
            Serial.print("Encoder reseted\n");
            //Ne trouve pas de ligne à suivre :(
            while((bLigne == false) && (MOVE_getDistanceMM(LEFT) > FULL_TURN) && (Golfotron_PreviousState == Golfotron_idle))
            {
               bLigne = findLine(false);
            }
            if(bLigne == true)
            {
               Serial.print("Aucune ligne de trouve\n");
               Golfotron_state = Golfotron_idle;
               break; 
            }
            SOFT_TIMER_Enable(TIMER_ID_STATE);
         }
         Suiveur_position_t suiveur_position = mode_ligne();
         if(suiveur_position == suiveur_but && currently_carrying == false)
         {
            if(Find_Golf_Ball() == seek_Balle_trouve)
            {
               Golfotron_state = Golfotron_Eliminate;
            }
            else
            {
               currently_carrying = true;
            }
         }
         else if(suiveur_position == suiveur_but)
         {
            Golfotron_PreviousState = Golfotron_Fin_De_Ligne;
         }
         else if(suiveur_position == suiveur_depart)
         {
            //Make a 90 degre turn to the opposite of the barcodes
            Serial.print("Change en mode carre vert\n");
            Golfotron_state = Golfotron_retour_a_la_base;
         }
         else if(suiveur_position == suiveur_Check_20cm)
         {
            Golfotron_state = Golfotron_Ambush_golf_ball;
         }
         break;
      }
      //Le robot doit détecter la balle de gofl
      case Golfotron_Ambush_golf_ball:
      {
         //Configuration initiale du mode
         if(Golfotron_PreviousState != Golfotron_state)
         {
            changeMode(camera_detecteur_blocks);
            ENCODER_Reset(LEFT);
            ENCODER_Reset(RIGHT);
         }

         Seek_GolfBall_t seek = Find_Golf_Ball();
         
         if(seek == seek_Balle_trouve)
         {
            Golfotron_state = Golfotron_Eliminate;
         }
         else if(seek == seek_Pas_de_balle)
         {
            Golfotron_state = Golfotron_Seek;
         }
         break;
      }
      //Le robot doit grab la balle
      case Golfotron_Eliminate:
      {
         int Essai = 0;
         if( currently_carrying == false )
         {
            getBall();
            Essai++;
         }
         if( currently_carrying == false && Essai > 0)
         {
            Golfotron_state = Golfotron_Ambush_golf_ball;
         }
         else
         {
            Golfotron_state = Golfotron_Seek;
         }
         break;
      }
      //Le robot doit passer en mode cherche balle
      case Golfotron_Fin_De_Ligne:
      {
         Golfotron_state = Golfotron_Ambush_golf_ball;
         MOVE_Rotation2Roues(180);
         break;
      }
      case Golfotron_retour_a_la_base:
      {
         if(Golfotron_PreviousState != Golfotron_retour_a_la_base)
         {
            ENCODER_Reset(LEFT);
            ENCODER_Reset(RIGHT);
            changeMode(camera_detecteur_blocks);
         }
         if(StopWithGreen() == true)
         {
            ReleaseBall();
            Golfotron_state = Golfotron_idle;   
         }
         break;
      }
      //Mode inconnu
      default:
      {
         Golfotron_state = Golfotron_idle;
         break;
      }
   }
    Golfotron_PreviousState = Golfotron_state;
}

void loop() {
  
   SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
   //mode_ligne();
   logique();
   //demo_claw();
   //delay(100);
   //mode_ligne();
   // if(currently_carrying == false)
   // {
   //    if(Find_Golf_Ball() == seek_Balle_trouve)
   //    {
   //       MOVE_vStop();
   //       delay(2000);
   //       getBall();
   //    }
   // }
   // else
   // {
   //    MOVE_vStop();
   // }
}

