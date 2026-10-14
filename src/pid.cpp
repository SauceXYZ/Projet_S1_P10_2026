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

long compteurTotaleDroit = 0;
long compteurTotaleGauche = 0;
float vitesseBase = 0.40;
float vitesseDroite = 0.40;
float vitesseGauche = 0.49;
float pulseParSeconde = 7.0;

static float clampVitesse(float valeur) {
    if (valeur > 1.0f) return 1.0f;
    if (valeur < 0.0f) return 0.0f;
    return valeur;
}

void pid(int delai){
    const float KP = 0.0005f;
    const float GAUCHE_TRIM = 0.01f;

    const long compteurDroit = ENCODER_ReadReset(1);
    const long compteurGauche = ENCODER_ReadReset(LEFT);
    compteurTotaleDroit += compteurDroit;
    compteurTotaleGauche += compteurGauche;

    const float differenceK = static_cast<float>(compteurDroit) -
                              static_cast<float>(compteurGauche);
    const float correction = differenceK * KP;

    vitesseDroite = clampVitesse(vitesseBase - correction);
    vitesseGauche = clampVitesse(vitesseBase + GAUCHE_TRIM + correction);

    Serial.print("Pulses droite/gauche: ");
    Serial.print(compteurDroit);
    Serial.print("/");
    Serial.print(compteurGauche);
    Serial.print(" | ");
    Serial.print("Vitesse Droite: ");
    Serial.print(vitesseDroite);
    Serial.print(" | Vitesse Gauche: ");
    Serial.print(vitesseGauche);
    Serial.println();

    avance(vitesseDroite, vitesseGauche);
    delay(delai);
}

void pidChat(int delai){
    pid(delai);
}
