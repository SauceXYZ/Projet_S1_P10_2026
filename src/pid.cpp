#include "pid.h"
#include <LibRobus.h>
#include "main.h"

#define DEBUG_EN 0

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
    const float vitesseMin = 0.15f;
    const float vitesseMax = 0.4f;
    const float acceleration = 0.8f; // Commande moteur par seconde.
    const float kpPosition = 0.0008f;
    const float kpSynchronisation = 0.0003f;
    const int32_t tolerancePulses = 8;
    const unsigned long periodeRotationMs = 5;

    if (angle <= 0) return;

    const float distance = angle * (DISTANCEENTREROUES_MM * 3.14159f) / 360.0f;
    const int32_t cible = (int32_t)(distance / (DIAMETRE_ROUE_MM * 3.14159f) * 3200.0f);
    Serial.print("Nombre de pulses pour tourner: ");
    Serial.println(cible);
    if(direction == TOURNE_DROITE) {
        Serial.println("Tourne a droite");
    } else {
        Serial.println("Tourne a gauche");
    }

    const int32_t debut0 = ENCODER_Read(0);
    const int32_t debut1 = ENCODER_Read(1);
    float signe0 = 0.0f;
    if(direction == TOURNE_DROITE)
    {
        signe0 = 1.0f;
    } 
    else {
        signe0 = -1.0f;
    }
    const float signe1 = -signe0;
    const unsigned long debutMs = millis();
    bool termine0 = false;
    bool termine1 = false;

    while (!termine0 || !termine1)
    {
        const int32_t parcours0 = abs(ENCODER_Read(0) - debut0);
        const int32_t parcours1 = abs(ENCODER_Read(1) - debut1);
        const int32_t restant0 = cible - parcours0;
        const int32_t restant1 = cible - parcours1;

        if(!termine0 && restant0 <= tolerancePulses) {
            termine0 = true;
            Serial.println("Roue gauche termine");
        }
        if(!termine1 && restant1 <= tolerancePulses) {
            termine1 = true;
            Serial.println("Roue droite termine");
        }

        // Limiter la montee de puissance au demarrage du pivot du caster.
        float plafond = vitesseMin + acceleration * ((millis() - debutMs) / 1000.0f);
        if (plafond > vitesseMax) plafond = vitesseMax;

        // Ralentir selon la distance restante et freiner la roue en avance.
        const float ecart = parcours0 - parcours1;
        const float correction = (!termine0 && !termine1) ? kpSynchronisation * ecart : 0.0f;
        float commande0 = kpPosition * restant0 - correction;
        float commande1 = kpPosition * restant1 + correction;
        if (commande0 < vitesseMin) {commande0 = vitesseMin;}
        if (commande1 < vitesseMin) {commande1 = vitesseMin;}
        if (commande0 > plafond) {commande0 = plafond;}
        if (commande1 > plafond) {commande1 = plafond;}

        // Une roue arrivee reste arretee pendant que l'autre termine.
        if(!termine0) {
            MOTOR_SetSpeed(0, signe0 * commande0);
        } else {
            MOTOR_SetSpeed(0, 0.0f);
        }
        if(!termine1) {
            MOTOR_SetSpeed(1, signe1 * commande1);
        } else {
            MOTOR_SetSpeed(1, 0.0f);
        }
        if (!termine0 || !termine1) delay(periodeRotationMs);
    }

    MOTOR_SetSpeed(0, 0.0f);
    MOTOR_SetSpeed(1, 0.0f);
    // Laisser le robot se stabiliser avant de reprendre la mesure de position.
    delay(100);
    encodeur_1_precedent = ENCODER_Read(1);
    initialise = false;
    Serial.println("Tourne complete");
}
