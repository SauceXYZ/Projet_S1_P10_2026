#ifndef PID_H
#define PID_H

//Constantes pour le robot
extern const int CLICPARTOUR;
extern const float DIAMETREROUE;
extern const float DISTANCEENTREROUES;
extern const int PULSEATTENDUDROIT;
extern const int PULSEATTENDUGAUCHE;

extern int compteurTotaleDroit;
extern int compteurTotaleGauche;
extern float vitesseDroite;
extern float vitesseGauche;
extern float pulseParSeconde;

void pid(int delai);

#endif