#include "move.h"
#include "LibRobus.h"
/**
 * @brief Configure la vitesse d'un moteur. Ça va être utilisé par fGetMotorSpeed plus tard.
 * 
 * @param {type} iMotorID 
 * @param {type} iSpeed 
 */
void vSetMotorSpeed(int iMotorID, float iSpeed)
{

}

/**
 * @brief La fonction est incomplète, mais l'idée est qu'elle retourne une vitesse lègerement moins vite pour la roue plus rapide.  
 * 
 * @param {type} iMotorID 
 * @return float retourne la vitesse pour la nouvelle roue
 */
float fGetMotorSpeed(int iMotorID)
{
    int32_t i32_DeltaWheelLeft;
    int32_t i32_DeltaWheelRight;

    //Compare la distance parcouru entre les deux roues.
    i32_DeltaWheelLeft = ENCODER_Read(LEFT) - ENCODER_Read(RIGHT);
    i32_DeltaWheelLeft =  ENCODER_Read(RIGHT) - ENCODER_Read(LEFT);

    if(i32_DeltaWheelLeft > MOVE_WHEEL_THERSHOLD)
    {
        //Le moteur a trop avancé à gauche, réduire la vitesse de la roue gauche
    }
    else if(i32_DeltaWheelRight > MOVE_WHEEL_THERSHOLD)
    {
        //Le moteur a trop avancé à droite, réduire la vitesse de la roue droite
    }

    //Retourne la nouvelle consigne de vitesse.
    return 0;
}

