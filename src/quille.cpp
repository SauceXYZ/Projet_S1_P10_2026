#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include <math.h>
#include "main.h"
#include "quille.h"

#define ANGLE_CORRECTION 40
#define PAS_SCAN_DEGRES 5

void detectionQuille(uint8_t capteur1);
float CalculDistanceQuille(uint8_t capteur1);
void abatDodo(float distanceCm);

int compteurTounreQuille = 0;
static int32_t encodeursDebutScan[2] = {0, 0};
const float TOLERANCE_RETOUR_DEGRES = 2.5f;
const uint8_t MAX_CORRECTIONS_RETOUR = 3;

static void memoriserDepartScan() {
    encodeursDebutScan[0] = ENCODER_Read(0);
    encodeursDebutScan[1] = ENCODER_Read(1);
}

static float erreurOrientationInitiale() {
    // Signed wheel travel includes scan overshoot, the sensor offset,
    // the U-turn and yaw accumulated during both straight movements.
    const float gaucheCm = (ENCODER_Read(0) - encodeursDebutScan[0]) * CM_PAR_IMPULSION_0;
    const float droiteCm = (ENCODER_Read(1) - encodeursDebutScan[1]) * CM_PAR_IMPULSION_1;
    const float angle = (gaucheCm - droiteCm) * 180.0f /
        (DISTANCEENTREROUES_MM * 0.1f * 3.14159265f);
    float normalise = fmodf(angle + 180.0f, 360.0f);
    if (normalise < 0) normalise += 360.0f;
    return normalise - 180.0f; // positive = robot is clockwise from initial heading
}

static void restaurerOrientationInitiale() {
    for (uint8_t essai = 0; essai < MAX_CORRECTIONS_RETOUR; ++essai) {
        const float erreur = erreurOrientationInitiale();
        Serial.print("Retour quille: erreur_encodeurs_deg="); Serial.println(erreur);
        if (fabsf(erreur) <= TOLERANCE_RETOUR_DEGRES) return;
        const int angle = static_cast<int>(lroundf(fabsf(erreur)));
        const bool sens = erreur > 0 ? TOURNE_GAUCHE : TOURNE_DROITE;
        Serial.print("Correction retour quille: ");
        Serial.print(sens == TOURNE_GAUCHE ? "gauche_deg=" : "droite_deg=");
        Serial.println(angle);
        pid_tourne(sens, angle); // always positive, at most 180 degrees
    }
    Serial.print("Retour quille: erreur_finale_deg=");
    Serial.println(erreurOrientationInitiale());
    if (fabsf(erreurOrientationInitiale()) > TOLERANCE_RETOUR_DEGRES) {
        Serial.println("Retour quille: tolerance non atteinte apres 3 corrections");
    }
}

//Doit etre alimenter
//05 cm = 500-600
//15 cm = 250-300
//30 cm = 150

void detectionQuille(uint8_t capteur1) {
//Fonction qui retourne une valeur double retournant la distance approximative de l'objet
//capteur est la valeur de la pin analogique que le capteur retourne (0-1024)
//Normalement deux capteur, un en avant et un sur le coté gauche (rotation droite 90deg = 180 detection)
    compteurTounreQuille = 0;
    memoriserDepartScan();
    bool detection = false;
    float distance = 100;
    while (detection == false) {
        distance = CalculDistanceQuille(capteur1);
        if (distance < 30 && distance > 5) {
            detection = true;
        }
        else {
            pid_tourne(TOURNE_DROITE, PAS_SCAN_DEGRES);
            compteurTounreQuille++;
        }
    }
};

static int32_t cmEnImpulsions(float cm){
    return (int32_t)(cm / CM_PAR_IMPULSION_1);
};

float CalculDistanceQuille(uint8_t capteur1){
    float distance = (4235.0 / (ROBUS_ReadIR(capteur1) - 47));   //Calcul de la distance approximative en cm
    return distance;
}

void abatDodo(float distanceCm) {
    // Preserve the quille's 40-degree offset and 5 cm approach margin.
    // Use one shared distance target out and back, retaining fractional cm.
    const int32_t impulsionsTrajet = cmEnImpulsions(distanceCm + 5.0f);
    pid_tourne(TOURNE_DROITE, ANGLE_CORRECTION);
    avanceImpulsions(impulsionsTrajet, false, false);
    delay(100);
    pid_tourne(TOURNE_DROITE, 180);
    avanceImpulsions(impulsionsTrajet, false, false);
    delay(100);
    Serial.print("Retour quille: scan_consigne_deg=");
    Serial.println((compteurTounreQuille % (360 / PAS_SCAN_DEGRES)) * PAS_SCAN_DEGRES);
    restaurerOrientationInitiale();
}

void EpreuveQuille() {
    detectionQuille(3);
    float distance = CalculDistanceQuille(3);
    abatDodo(distance);
    delay(1000);
};