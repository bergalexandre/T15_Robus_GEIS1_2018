#ifndef __CAPTEUR_H__
#define __CAPTEUR_H__

#define BAS 0
#define HAUT 1
#define CAPTEUR_IR_DISTANCE_BAS   A0
#define CAPTEUR_IR_DISTANCE_HAUT  A1

#define SERVOMOTEUR_1 A2
#define SERVOMOTEUR_2 A3

//Maybe not needed.
#define CAPTEUR_COULEUR_INT 48

#define CAPTEUR_SUIVEUR_LIGNE_GAUCHE    A13
#define CAPTEUR_SUIVEUR_LIGNE_MILIEU    A14
#define CAPTEUR_SUIVEUR_LIGNE_DROIT     A15


typedef enum
{
    UNKNOW,
    BLEU,
    VERT,
    ROUGE,
    JAUNE,
    NOIR,
    BLANC,
    NOMBRE_DE_COULEUR
} CAPTEUR_Couleur;

#define DEBUG_CAPTEUR false

#endif //__CAPTEUR_H__
