#include "move.h"

/**
 * @brief Configure la vitesse d'un moteur. Ça va être utilisé par fGetMotorSpeed plus tard.
 * 
 * @param {type} iMotorID 
 * @param {type} iSpeed 
 */
/*void vSetMotorSpeed(int iMotorID, float fSpeed)
{
    MOTOR_SetSpeed(iMotorID,fSpeed);
}*/

/**
 * @brief La fonction est incomplète, mais l'idée est qu'elle retourne une vitesse lègerement moins vite pour la roue plus rapide.  
 * 
 * @param {type} iMotorID 
 * @return float retourne la vitesse pour la nouvelle roue
 */
/*float fGetSlaveSpeed(int iMotorID) //Il faudra appeler la fonction pour savoir quel moteur doit être réduit.
{
    int32_t i32_DeltaWheelSlave;
    
    //Compare la distance parcouru entre les deux roues.
    i32_DeltaWheelSlave = ENCODER_Read(MOVE_MASTER) - ENCODER_Read(MOVE_SLAVE);

    if(i32_DeltaWheelSlave > MOVE_WHEEL_THERSHOLD  || i32_DeltaWheelSlave < -MOVE_WHEEL_THERSHOLD){
        return i32_DeltaWheelSlave;
    }

    //Retourne la nouvelle consigne de vitesse.
    return 0;
}
void vMoveStraight(float speed){
    float speed2= speed;
    if(speed > 0.9)
        speed2 = 0.9;
    if(speed < -0.9)
        speed2 = -0.9;
    MOTOR_SetSpeed(LEFT, speed2);
    MOTOR_SetSpeed(RIGHT, speed2);    
}

*/