#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define FACE_DROIT 0
#define FACE_GAUCHE 1
#define FACE_AVANT 2
#define FACE_ARRIERE 3

#define DIAMETRE_ROUE_MM 76.2f
#define VITESSE_REFERENCE 0.5f
#define TOURNE_GAUCHE 0
#define TOURNE_DROITE 1

#define DISTANCEENTREROUES_MM 190

//MOTEUR 0 SWITCH FIL MAUVE (DROIT)

extern int vertpin;
extern int rougepin;
extern int pinsifflet;

extern float lab_posx;
extern float lab_posy;

extern int direction;

extern int32_t encodeur_1_precedent;


#endif