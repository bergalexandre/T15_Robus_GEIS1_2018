/**
 * @brief 
 * 
 * @file move.h
 * @author your name
 * @date 2018-09-25
 */

/**
 * @def Diamètre de la roue en centimètre.
 * 
 */
#define MOVE_WHEEL_DIAMETER 50

/**
 * @def Nombre de "Pulse" que l'encodeur génère pour chaque tour de roue complet.
 * 
 */
#define MOVE_PULSE_PER_TURN 250

/**
 * @def Différence permis en nombre de pulse avant de corriger la trajectoire.
 * 
 */
#define MOVE_WHEEL_THERSHOLD 10

#define MOVE_NUMBER_OF_ENCODER 2