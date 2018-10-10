#ifndef MOVE_H 
#define MOVE_h


/**
 * @brief 
 * 
 * @file move.h
 * @author your name
 * @date 2018-09-25
 */

/**
 * @def Diamètre de la roue en millimetres.
 * 
 */
#define MOVE_WHEEL_DIAMETER 77

/**
 * @brief Distance entre les deux roue en millimetres
 * 
 */
#define MOVE_LARGEUR_ROBOT 190

/**
 * @def Nombre de "Pulse" que l'encodeur génère pour chaque tour de roue complet.
 * 
 */
#define MOVE_PULSE_PER_TURN 3200

/**
 * @def Différence permis en nombre de pulse avant de corriger la trajectoire.
 * 
 */
#define MOVE_WHEEL_MAX_ENCODER_DELTA 100


#define ACTION_TYPE_ACCELERATION 0
#define ACTION_TYPE_ROTATION_1_ROUE 0

#define MOVE_NUMBER_OF_ENCODER 2
#define MOVE_MASTER 1
#define MOVE_SLAVE 0
#define MOVE_DERIVATIVE_ADJUSTEMENT_FACTOR 0.001
#define MOVE_INTEGRATIVE_ADJUSTEMENT_FACTOR 0.001
#define MOVE_DELAY 50
#define MOVE_WAIT 20
#define MOVE_MAX_SPEED 0.7

//Valeur en % où le robot doit ralentir durant son virage
#define MOVE_SLOW_AT_PERCENT 50
//temps sur lequel le robot doit faire la fin du virage.
#define MOVE_TURN_TIME 10


#endif      