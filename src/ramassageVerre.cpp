#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include <math.h>
#include "main.h"
#include "ramassageVerre.h"
#include "servo.h"

#define ANGLE_CORRECTION 45
#define PAS_SCAN_DEGRES 5

const uint8_t NB_LECTURES_VERRE = 7;
const uint8_t MIN_LECTURES_VALIDES = 5;
const unsigned long INTERVALLE_MESURE_VERRE_MS = 40;
const float DISTANCE_MIN_VERRE_CM = 5.0f;
const float DISTANCE_MAX_VERRE_CM = 25.0f;
const float DISPERSION_MAX_VERRE_CM = 2.0f;
const float MARGE_PINCE_CM = 2.0f;
const float VITESSE_APPROCHE_VERRE = 0.18f;
static float distanceVerreConfirmee = NAN;


int compteurTounreVerre = 0;
static int32_t encodeursDebutScan[2] = {0, 0};
const float TOLERANCE_RETOUR_DEGRES = 2.5f;
const uint8_t MAX_CORRECTIONS_RETOUR = 3;

static void memoriserDepartScan() {
    encodeursDebutScan[0] = ENCODER_Read(0);
    encodeursDebutScan[1] = ENCODER_Read(1);
}

static float erreurOrientationInitiale() {
    // Signed wheel travel includes scan overshoot, the gripper offset,
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
        Serial.print("Retour initial: erreur_encodeurs_deg="); Serial.println(erreur);
        if (fabsf(erreur) <= TOLERANCE_RETOUR_DEGRES) return;
        const int angle = static_cast<int>(lroundf(fabsf(erreur)));
        const bool sens = erreur > 0 ? TOURNE_GAUCHE : TOURNE_DROITE;
        Serial.print("Correction retour: ");
        Serial.print(sens == TOURNE_GAUCHE ? "gauche_deg=" : "droite_deg=");
        Serial.println(angle);
        pid_tourne(sens, angle); // always positive, at most 180 degrees
    }
    Serial.print("Retour initial: erreur_finale_deg=");
    Serial.println(erreurOrientationInitiale());
    if (fabsf(erreurOrientationInitiale()) > TOLERANCE_RETOUR_DEGRES) {
        Serial.println("Retour initial: tolerance non atteinte apres 3 corrections");
    }
}
//Doit etre alimenter
//05 cm = 500-600
//15 cm = 250-300
//30 cm = 150

float CalculDistanceVerre(uint8_t capteur1){
    const int brut = ROBUS_ReadIR(capteur1);
    if (brut <= 47) return NAN; // invalid denominator, not a usable distance
    return 4235.0f / (brut - 47);
}

static bool mesurerDistanceVerre(uint8_t capteur, float &distanceCm) {
    float lectures[NB_LECTURES_VERRE];
    uint8_t nombre = 0;
    for (uint8_t i = 0; i < NB_LECTURES_VERRE; ++i) {
        const float d = CalculDistanceVerre(capteur);
        if (isfinite(d) && d > DISTANCE_MIN_VERRE_CM && d < DISTANCE_MAX_VERRE_CM) {
            // Insertion sort of the small window, excluding invalid/out-of-range readings.
            uint8_t j = nombre;
            while (j > 0 && lectures[j - 1] > d) {
                lectures[j] = lectures[j - 1];
                --j;
            }
            lectures[j] = d;
            ++nombre;
        }
        if (i + 1 < NB_LECTURES_VERRE) delay(INTERVALLE_MESURE_VERRE_MS);
    }
    if (nombre < MIN_LECTURES_VALIDES) return false;
    // Trim one value at each end: isolated spikes cannot change the target.
    if (lectures[nombre - 2] - lectures[1] > DISPERSION_MAX_VERRE_CM) return false;
    distanceCm = nombre % 2 ? lectures[nombre / 2] :
        (lectures[nombre / 2 - 1] + lectures[nombre / 2]) * 0.5f;
    Serial.print("[VERRE] distance_mediane_cm="); Serial.print(distanceCm);
    Serial.print(" valides="); Serial.print(nombre);
    Serial.print(" dispersion_cm="); Serial.println(lectures[nombre - 2] - lectures[1]);
    return true;
}

void detectionVerre(uint8_t capteur1) {
//Fonction qui retourne une valeur double retournant la distance approximative de l'objet
//capteur est la valeur de la pin analogique que le capteur retourne (0-1024)
//Normalement deux capteur, un en avant et un sur le coté gauche (rotation droite 90deg = 180 detection)
    compteurTounreVerre = 0;
    memoriserDepartScan(); // Preserve the pre-scan heading, without resetting encoders.
    distanceVerreConfirmee = NAN;
    while (true) {
        // pid_tourne() returns with the motors stopped. Sample in that same pose.
        if (mesurerDistanceVerre(capteur1, distanceVerreConfirmee)) return;
        pid_tourne(TOURNE_DROITE, PAS_SCAN_DEGRES);
        compteurTounreVerre++;
    }
};

static int32_t cmEnImpulsions(float cm){
    return (int32_t)(cm / CM_PAR_IMPULSION_1);
};

static void avancerVerre(int32_t impulsions) {
    const int32_t debutG = ENCODER_Read(0), debutD = ENCODER_Read(1);
    avanceImpulsions(impulsions, false, false, VITESSE_APPROCHE_VERRE);
    Serial.print("[VERRE] cible_cm="); Serial.print(impulsions * CM_PAR_IMPULSION_1);
    Serial.print(" mesure_G_cm="); Serial.print((ENCODER_Read(0) - debutG) * CM_PAR_IMPULSION_0);
    Serial.print(" mesure_D_cm="); Serial.println((ENCODER_Read(1) - debutD) * CM_PAR_IMPULSION_1);
}


void grabDodo(float distanceCm) {
    // Align the gripper with the glass, keeping the sensor/gripper offset.
    // Use exactly the same target distance out and back, in centimetres.
    const int32_t impulsionsTrajet = cmEnImpulsions(distanceCm - MARGE_PINCE_CM);
    pid_tourne(TOURNE_DROITE, ANGLE_CORRECTION);
    avancerVerre(impulsionsTrajet);
    delay(100);
    grabServo();
    pid_tourne(TOURNE_DROITE, 180);
    avancerVerre(impulsionsTrajet);
    delay(100);

    // Counter * 5 is only a requested angle: each small rotation can overshoot.
    Serial.print("Retour initial: scan_consigne_deg=");
    Serial.println((compteurTounreVerre % (360 / PAS_SCAN_DEGRES)) * PAS_SCAN_DEGRES);
    restaurerOrientationInitiale();
}

void EpreuveVerre() {
    initServo();
    detectionVerre(3);
    const float distance = distanceVerreConfirmee; // reuse the validated window, no new noisy sample
    dropServo();
    grabDodo(distance);
    delay(1000);
};