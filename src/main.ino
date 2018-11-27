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

#define TIMER_ID_STATE 1
#define TIMER_ID_DROP 2


//Define pour suiveur de ligne.
// 0,0 en haut à gauche
#define LINE_HEIGHT 51
#define LINE_WIDTH 78
#define NB_VECTOR_TO_COMPARE 10

/*******************************************************************
 * Typedef / enum / struct
 *******************************************************************/

typedef enum
{
  suiveur_ligne,
  detecteur_blocks
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
   Golfotron_Eliminate
} Golfotron_state_t;

typedef enum
{
   seek_Balle_trouve,
   seek_Recherche_en_cours,
   seek_Pas_de_balle
} Seek_GolfBall_t;
/*******************************************************************************
 * Prototypes locaux
*******************************************************************************/

void changeMode(camera_mode_t mode);
void updateGolfotron_State();

/*******************************************************************************
 * Variables/classe locales
*******************************************************************************/

Pixy2 pixy;
float g_leftSpeed = 0.0;
float g_rightSpeed = 0.0;
int currently_carrying = 0;
Golfotron_state_t Golfotron_state = Golfotron_idle;

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

void MOVE_vAvancer(float fVitesse, int32_t i32Distance_mm,unsigned int accelerationTime = 125)
{
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  g_leftSpeed = fVitesse;
  MOTOR_SetSpeed(LEFT,g_leftSpeed);

   if(i32Distance_mm  > 0)
   {
      while(MOVE_getDistanceMM(LEFT) < i32Distance_mm)
      {
         Serial.print("Distance fait = ");
         Serial.print(MOVE_getDistanceMM(LEFT));
         Serial.println(" mm");
         g_rightSpeed = fVitesse + fSpeedAdjustment();
         MOTOR_SetSpeed(RIGHT, g_rightSpeed);
         delay(50);
      }
  }
  else
  {
      while(MOVE_getDistanceMM(LEFT) > i32Distance_mm)
      {
         Serial.print("Distance fait = ");
         Serial.print(MOVE_getDistanceMM(LEFT));
         Serial.println(" mm");
         g_rightSpeed = fVitesse - fSpeedAdjustment();
         MOTOR_SetSpeed(RIGHT, g_rightSpeed);
         delay(50);
      }
  }
   MOVE_vStop();
}

/******************* Fonction pour servomoteur  ************************/

void ballGrab(int angle){
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

void getBall(){
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  while(CAPTEUR_distanceIR(CAPTEUR_IR_DISTANCE_BAS)>75){
    MOTOR_SetSpeed(LEFT,0.15);
    MOTOR_SetSpeed(RIGHT,0.15);
  }
  Serial.print("La balle est icitte meyn!\n");
  MOTOR_SetSpeed(LEFT,0);
  MOTOR_SetSpeed(RIGHT,0);
  delay(50);
  ballGrab(90);
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
   ballGrab(70);
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
   float ratio_x_on_y = (float)vecteur_x/vecteur_y;
   Serial.print("Vecteur, ratio x/y = ");
   Serial.println(ratio_x_on_y);
   if(ratio_x_on_y >= 3)
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
   float ratio_y_on_x = (float)vecteur_y/vecteur_x;
   Serial.print("Vecteur, ratio y/x = ");
   Serial.println(ratio_y_on_x);
   if(ratio_y_on_x >= 3)
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
         delay(5000);
      }
      MOVE_vSetSpeed();
      delay(20);
   }
}

void findLine()
{
   Vector ligne;
   bool ligneTrouve = false;
   MOVE_vAvancer(0.2, 150);
   int temps_max = 20;
   int i = 0;

   for(i = 0; i < temps_max; i++)
   {
      g_leftSpeed = 0;
      g_rightSpeed = 0.15;
      MOVE_vSetSpeed();
      if(pixyGetLigne(&ligne))
      {
         if(isVectorVertical(ligne))
         {
            ligneTrouve = true;
            Serial.print("Le robot est aligne avec la ligne\n");
            break;
         }
      }
      delay(50);
   }
   MOVE_vStop();
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

void mode_ligne()
{
   Vector ligne;
	// print all vectors
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
      else if(isVectorHorizontal)
      {
         Serial.print("Le robot est trop loin de la ligne\n");
         //TODO
         //Robot doit revenir vers la ligne
         //Get closer to the line
         getCloserToLine();
         findLine();
         MOVE_vStop();
      }
      else
      { 
         //essait de voir si en pivotant il comprend la ligne.
         Serial.print("ligne etrange, pivote\n");
         //Tourne vers où il pense la ligne serait perpendiculaire.
         if(ligne.m_x0 < ligne.m_x1)
         {
            Serial.print("Va à droite\n");
            //Pivote vers la droite
            g_rightSpeed = 0.0;
            g_leftSpeed = 0.15;
         }
         else
         {
            Serial.print("Va à gauche\n");
            g_leftSpeed = 0.0;
            g_rightSpeed = 0.15;
         }
      }
   }
   else
   {
      //TODO
      MOVE_vStop();
      Serial.print("Pas de vecteur\n");
   }
   MOVE_vSetSpeed();
   delay(100);
}

/***************** block ***************/

/**
 * @brief Trouve la balle de golf
 * 
 * @return true Vrai si repéré et robot centré sur la belle.
 * @return false Faux si pas de balle.
 */
Seek_GolfBall_t Find_Golf_Ball(){
   static int rotation_mm = 0;
   Seek_GolfBall_t tRet = seek_Recherche_en_cours;
	bool lastPos = LEFT; // derniere position de l'objet a partir de la camera
	if (pixy.ccc.getBlocks(true, 1) > 0) //rentre s'il y a un objet de détecté
	{
		float ratio;
		int blockPos = 0;
      Serial.print("Balle trouvee.\n");
		for (int i = 0; i < pixy.ccc.numBlocks; i++) // trouve la position de la balle dans l'array
		{
			if (pixy.ccc.blocks[i].m_signature == 0)
				blockPos = i;
		}

		if (pixy.ccc.blocks[blockPos].m_y < 180) // si la balle est trop loin avance
		{
         Serial.print("Balle trop loin.\n");
			g_rightSpeed = 0.15;
         g_leftSpeed = g_rightSpeed;
		}

		if (pixy.ccc.blocks[blockPos].m_y > 190)// si trop proche recule
		{
         Serial.print("Balle trop proche.\n");
			g_rightSpeed = -0.15;
         g_leftSpeed = g_rightSpeed;
		}

		ratio = ((float)pixy.ccc.blocks[blockPos].m_x) / ((float)(pixy.frameWidth / 2.0)); //ratio de la postion en x de la ball va de 0 à 2 0 étant a gauche et 2 a droite
	
		if (pixy.ccc.blocks[blockPos].m_y <= 190 && pixy.ccc.blocks[blockPos].m_y >= 180) // si la balle est dans le bon threshold de distance, recentre la balle
		{
         Serial.print("On recentre la balle.\n");
			if (ratio >= 1.2)
			{
            Serial.print("Balle trop a droite.\n");
				g_leftSpeed = 0.1;
				g_rightSpeed = 0;
			}
			else if (ratio <= 0.8)
         {
         Serial.print("Balle trop a gauche.\n");
				g_leftSpeed = 0.1;
				g_rightSpeed = 0;
			}
			else if (ratio < 1.2 && ratio > 0.8){
				Serial.print("Balle centree.\n");
            g_leftSpeed = g_rightSpeed = 0.0;
            tRet = seek_Balle_trouve;
         }
		}

		if (ratio > 1.0)
			lastPos = RIGHT;
		else
			lastPos = LEFT;
	
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

   return bRet;
}

/***************** MAIN ****************/

void updateGolfotron_State()
{
   
}


void changeMode(camera_mode_t mode)
{
	switch(mode)
	{
	case suiveur_ligne:
		pixy.changeProg("line");
      Serial.print("Mode suiveur de ligne\n");
      pixy.setCameraBrightness(0);
		break;
	case detecteur_blocks:
		pixy.changeProg("color_connected_components");
      Serial.print("Mode block\n");
      pixy.setCameraBrightness(35);
		break;
	default:
		pixy.changeProg("color_connected_components");
      Serial.print("Mode block\n");
		break;
   }
}

void setup_timers()
{
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

void setup_Moteurs(){
  g_leftSpeed = 0;
  g_rightSpeed = 0;
  MOTOR_SetSpeed(LEFT, 0.0);
  MOTOR_SetSpeed(RIGHT, 0.0);

}

void setup(){
  BoardInit();
  SOFT_TIMER_Update();
  Serial.begin(9600);
  setup_Moteurs();
  setup_Sorties();
  setup_timers();
  ballDrop();
  //Attend que le pixy soit ready:
  int pixy_answer;
  
  do
  {
      pixy_answer = pixy.getVersion();
      SOFT_TIMER_Update();
      delay(100);
  }while(pixy_answer == PIXY_RESULT_BUSY || pixy_answer == PIXY_RESULT_ERROR);
  changeMode(suiveur_ligne);
}

void logique()
{
   
}

void loop() {
  
  SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  mode_ligne();
}

