#ifndef MOVE_H 
#define MOVE_h
#include <LibRobus.h> 


/**
 * @brief 
 * 
 * @file move.h
 * @author your name
 * @date 2018-09-25
 */

/**
<<<<<<< HEAD
 * @def Diamètre de la roue en millimètre.
=======
 * @def Diamètre de la roue en millimetres.
>>>>>>> 57fc3e9a1b3adeff7d0b4d53ed2ee63f4f290345
 * 
 */
#define MOVE_WHEEL_DIAMETER 77

/**
 * @def Nombre de "Pulse" que l'encodeur génère pour chaque tour de roue complet.
 * 
 */
#define MOVE_PULSE_PER_TURN 3200

/**
 * @def Différence permis en nombre de pulse avant de corriger la trajectoire.
 * 
 */
#define MOVE_WHEEL_MAX_ENCODER_DELTA 200


#define ACTION_TYPE_ACCELERATION 0
#define ACTION_TYPE_ROTATION_1_ROUE 0

#define MOVE_NUMBER_OF_ENCODER 2
#define MOVE_MASTER 1
#define MOVE_SLAVE 0
#define MOVE_DERIVATIVE_ADJUSTEMENT_FACTOR 1.0
#define MOVE_INTEGRATIVE_ADJUSTEMENT_FACTOR 0.5
#define MOVE_DELAY 50
    
//Prototypes de fonctions
float fGetSlaveSpeed(int iMotorID);
void vMoveStraight(float speed);
void vSetMotorSpeed(int iMotorID, float fSpeed);
void acceleration(float initialSpeed, float finalSpeed, unsigned int time);


#endif