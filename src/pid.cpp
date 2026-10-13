#include "pid.h"
#include <LibRobus.h>
#include "main.h"

#define DEBUG_EN 1

const float vitesseReference = 0.5f;
const unsigned long periodeMs = 50;
const float Kp = 0.00015f;
const float Ki = 0.0001f;
const float Kd = 0.000005f;
const float tempsFiltreDerivee = 0.1f;
float erreurPrecedente = 0.0f;
float deriveeFiltree = 0.0f;
bool erreurPrecedenteValide = false;
float correctionIntegrale = 0.0f;
unsigned long tempsPrecedent = 0;
int32_t encodeurGauchePrecedent = 0;
int32_t encodeurDroitPrecedent = 0;
bool initialise = false;


void pid_init(void)
{
    correctionIntegrale = 0.0f;
    erreurPrecedente = 0.0f;
    deriveeFiltree = 0.0f;
    erreurPrecedenteValide = false;
    encodeurGauchePrecedent = ENCODER_Read(0);
    encodeurDroitPrecedent = ENCODER_Read(1);
    tempsPrecedent = millis();
    initialise = true;
    MOTOR_SetSpeed(0, vitesseReference);
    MOTOR_SetSpeed(1, vitesseReference);
}
//source : https://www.teachmemicro.com/arduino-pid-control-tutorial/
void pid_avance(void)
{
    if (!initialise) {
        pid_init();
        return;
    }
    const unsigned long tempsPresent = millis();
    const unsigned long tempsEcoule = tempsPresent - tempsPrecedent;

    //Temps minimum entre calcule de pid
    if (tempsEcoule < periodeMs) return;

    const int32_t encodeurGauche = ENCODER_Read(0);
    const int32_t encodeurDroit = ENCODER_Read(1);

    const float dt = tempsEcoule / 1000.0f;

    //Calcule de la vitesse (en pulse par seconde)
    const float vitesseGauche = (encodeurGauche - encodeurGauchePrecedent) / dt;
    const float vitesseDroite = (encodeurDroit - encodeurDroitPrecedent) / dt;

    const float erreur = vitesseGauche - vitesseDroite;

    // calcule de proportionel seulement car faire une derrive cause un spike qui fait que le robot ne va pas droit au depart
    if (erreurPrecedenteValide) {
        const float derivee = (erreur - erreurPrecedente) / dt;
        const float alpha = dt / (tempsFiltreDerivee + dt);
        deriveeFiltree += alpha * (derivee - deriveeFiltree);
    }
    erreurPrecedente = erreur;
    erreurPrecedenteValide = true;
    const float correctionDerivee = Kd * deriveeFiltree;

    const float integraleCandidate = correctionIntegrale + Ki * erreur * dt;
    const float commandeCandidate = vitesseReference + Kp * erreur + integraleCandidate + correctionDerivee;
    if ((commandeCandidate >= 0.0f && commandeCandidate <= 1.0f) ||
        (commandeCandidate > 1.0f && erreur < 0.0f) ||
        (commandeCandidate < 0.0f && erreur > 0.0f)) {
        correctionIntegrale = integraleCandidate;
    }
    float commandeDroite = vitesseReference + Kp * erreur + correctionIntegrale + correctionDerivee;

    if (commandeDroite > 1.0f) commandeDroite = 1.0f;
    if (commandeDroite < 0.0f) commandeDroite = 0.0f;

    MOTOR_SetSpeed(0, vitesseReference);
    MOTOR_SetSpeed(1, commandeDroite);

    encodeurGauchePrecedent = encodeurGauche;
    encodeurDroitPrecedent = encodeurDroit;
    tempsPrecedent = tempsPresent;

    if(DEBUG_EN == 0) return;

    Serial.print("Vitesse gauche (imp/s): ");
    Serial.print(vitesseGauche);
    Serial.print(" | Vitesse droite (imp/s): ");
    Serial.print(vitesseDroite);
    Serial.print(" | Commande droite: ");
    Serial.print(commandeDroite, 4);
    Serial.print(" | Correction I: ");
    Serial.print(correctionIntegrale, 4);
    Serial.print(" | Correction D: ");
    Serial.println(correctionDerivee, 4);
}


void pid_tourne(bool direction, int angle)
{
    if (!initialise) {
        pid_init();
        return;
    }
    float distance = angle * (DISTANCEENTREROUES_MM * 3.14159f) / 360.0f;
    int nbrPulsesPourTourner = (int)(distance / (DIAMETRE_ROUE_MM * 3.14159f) * 3200.0f);
    Serial.print("Nombre de pulses pour tourner: ");
    Serial.println(nbrPulsesPourTourner);

    if (direction == true) {
        Serial.println("Tourne droite");
        MOTOR_SetSpeed(0, vitesseReference);
        MOTOR_SetSpeed(1, -vitesseReference);
    } else {
        Serial.println("Tourne gauche");
        MOTOR_SetSpeed(0, -vitesseReference);
        MOTOR_SetSpeed(1, vitesseReference);
    }

    while(abs(ENCODER_Read(0)) < nbrPulsesPourTourner && abs(ENCODER_Read(1)) < nbrPulsesPourTourner) {
        delay(10);
    }

    MOTOR_SetSpeed(0, 0.0f);
    MOTOR_SetSpeed(1, 0.0f);

    initialise = false;
}

