#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#define FACE_DROIT 0
#define FACE_GAUCHE 1
#define FACE_AVANT 2
#define FACE_ARRIERE 3

#define LIMIT_OUEST 1
#define LIMIT_EST 2
#define LIMIT_NORD 3
#define LIMIT_SUD 4

#define DIAMETRE_ROUE_MM 76.2f
#define VITESSE_REFERENCE 0.5f
#define TOURNE_GAUCHE 0
#define TOURNE_DROITE 1

#define DISTANCEENTREROUES_MM 190

#define CASE_CM 50
#ifndef RETOUR_ACTIF
#define RETOUR_ACTIF 0 
#endif
#define RAMPE_ARRET_MS 120
#define RAMPE_OBSTACLE_MS 200
#define COEFF_ANTICIPATION_ARRET 1.0f
#define MARGE_ARRET_CM 0.0f

#define CALIBRATION_DEBUG 0
#define IMPULSIONS_PAR_TOUR 3200.0f
#define COEFF_DISTANCE_0 1.0f
#define COEFF_DISTANCE_1 1.0f
#define COEFF_ROTATION_GAUCHE 0.997f
#define COEFF_ROTATION_DROITE 0.997f
#define CM_PAR_IMPULSION_0 (DIAMETRE_ROUE_MM * 0.1f * 3.14159f / IMPULSIONS_PAR_TOUR * COEFF_DISTANCE_0)
#define CM_PAR_IMPULSION_1 (DIAMETRE_ROUE_MM * 0.1f * 3.14159f / IMPULSIONS_PAR_TOUR * COEFF_DISTANCE_1)
#define TOLERANCE_CASE_CM (CASE_CM / 2.0f - 2.0f)


extern int vertpin;
extern int rougepin;
extern int pinsifflet;

extern float lab_posx;
extern float lab_posy;

extern float y_total;
extern float x_total_max;
extern float x_total_min;

extern int direction;

extern int32_t encodeur_1_precedent;


#endif
