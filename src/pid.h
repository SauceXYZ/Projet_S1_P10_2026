#ifndef PID_H
#define PID_H

//Constantes pour le robot
extern const int CLICPARTOUR;
extern const float DIAMETREROUE;
extern const float DISTANCEENTREROUES;
extern const int PULSEATTENDUDROIT;
extern const int PULSEATTENDUGAUCHE;

extern long compteurTotaleDroit;
extern long compteurTotaleGauche;
extern float vitesseDroite;
extern float vitesseGauche;
extern float pulseParSeconde;

void pid(int delai);
void pidChat(int delai);
#endif