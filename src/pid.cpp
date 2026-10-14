/*
Projet: pid.c
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Position, Integral, Derivé (PID)
Date: 13/10/2026
*/

#include <LibRobus.h>

#include "stdbool.h"
#include "stdio.h"

#include "map.h"
#include "control.h"
#include "pid.h"
#include "sensor.h"

//Constantes moteur & robot
const int CLICPARTOUR = 3200;
const float DIAMETREROUE = 7.62; // en cm
const float DISTANCEENTREROUES = 17.2; // en cm
const int PULSEATTENDUDROIT = 2300;
const int PULSEATTENDUGAUCHE = 2300;

int compteurTotaleDroit = 0;
int compteurTotaleGauche = 0;
float vitesseBase = 0.20;
float vitesseDroite = 0.40;
float vitesseGauche = 0.49;
float pulseParSeconde = 7.0;

void pid(int delai){
    int pulseAttendu = delai * pulseParSeconde * 2.25;
    float kGauche=0;  //Différence
    float kDroite=0;  //Différence
    float differenceK = 0;
    float KP=0.00001; //Correction proportionnelle
    
    int compteurDroit = 0;
    int compteurGauche = 0;

    compteurDroit = ENCODER_ReadReset(RIGHT);
    compteurGauche = ENCODER_ReadReset(LEFT);
    compteurTotaleDroit += compteurDroit;
    compteurTotaleGauche += compteurGauche;

    differenceK = compteurDroit - compteurGauche;

    if(differenceK > 100 || differenceK < -100){
       if(differenceK > 0){
        vitesseGauche += differenceK*KP/2;
        vitesseDroite -= differenceK*KP/2;
      }
      else if(differenceK < 0){
      vitesseDroite += -differenceK*KP/2;
      vitesseGauche -= -differenceK*KP/2;
      }
      } 
      else {
      kDroite = pulseAttendu - compteurDroit;
      vitesseDroite = vitesseBase + kDroite*KP;

      kGauche = pulseAttendu - compteurGauche;
      vitesseGauche = vitesseBase + KP*kGauche;
    }
    
    avance(vitesseDroite, vitesseGauche);
    delay(100);
    
};
