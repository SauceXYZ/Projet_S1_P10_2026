#include "pid.h"
#include <LibRobus.h>
#include "main.h"

const float vitesseReference = 0.5f;
const unsigned long periodeMs = 50;
const float accelerationAvance = 0.6f;
float commandeGauchePrecedente = 0.0f;
float commandeDroitePrecedente = 0.0f;
const float Kp = 0.00015f;
const float Ki = 0.00012f;
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
static float vitesseDroiteCmS = 0.0f;

void pid_init(void)
{
    commandeGauchePrecedente = 0.0f;
    commandeDroitePrecedente = 0.0f;
    vitesseDroiteCmS = 0.0f;
    erreurPrecedente = 0.0f;
    deriveeFiltree = 0.0f;
    erreurPrecedenteValide = false;
    encodeurGauchePrecedent = ENCODER_Read(0);
    encodeurDroitPrecedent = ENCODER_Read(1);
    tempsPrecedent = millis();
    initialise = true;
}
//source : https://www.teachmemicro.com/arduino-pid-control-tutorial/
void pid_avance(float vitesseCible)
{
    if (!initialise) {
        pid_init();
        return;
    }
    const unsigned long tempsPresent = millis();
    const unsigned long tempsEcoule = tempsPresent - tempsPrecedent;

    if (tempsEcoule < periodeMs) return;

    const int32_t encodeurGauche = ENCODER_Read(0);
    const int32_t encodeurDroit = ENCODER_Read(1);

    const float dt = tempsEcoule / 1000.0f;
    float dtRampe = dt;
    if (dtRampe > 0.1f) dtRampe = 0.1f;
    const float variationMax = accelerationAvance * dtRampe;
    if (vitesseCible < 0.0f) vitesseCible = 0.0f;
    if (vitesseCible > vitesseReference) vitesseCible = vitesseReference;
    float commandeGauche = vitesseCible;

    const float vitesseGauche = (encodeurGauche - encodeurGauchePrecedent) * COEFF_DISTANCE_0 / dt;
    const float vitesseDroite = (encodeurDroit - encodeurDroitPrecedent) * COEFF_DISTANCE_1 / dt;

    vitesseDroiteCmS = (encodeurDroit - encodeurDroitPrecedent) * CM_PAR_IMPULSION_1 / dt;
    const float erreur = vitesseGauche - vitesseDroite;

    if (erreurPrecedenteValide) {
        const float derivee = (erreur - erreurPrecedente) / dt;
        const float alpha = dt / (tempsFiltreDerivee + dt);
        deriveeFiltree += alpha * (derivee - deriveeFiltree);
    }
    erreurPrecedente = erreur;
    erreurPrecedenteValide = true;
    const float correctionDerivee = Kd * deriveeFiltree;

    const float integraleCandidate = correctionIntegrale + Ki * erreur * dt;
    const float commandeCandidate = commandeGauche + Kp * erreur + integraleCandidate + correctionDerivee;
    if (vitesseCible > 0.0f &&
        ((commandeCandidate >= 0.0f && commandeCandidate <= 1.0f) ||
         (commandeCandidate > 1.0f && erreur < 0.0f) ||
         (commandeCandidate < 0.0f && erreur > 0.0f))) {
        correctionIntegrale = integraleCandidate;
    }
    float commandeDroite = commandeGauche + Kp * erreur + correctionIntegrale + correctionDerivee;

    if (commandeDroite > 1.0f) commandeDroite = 1.0f;
    if (commandeDroite < 0.0f) commandeDroite = 0.0f;

    if (vitesseCible == 0.0f) commandeDroite = 0.0f;


    const float changement0 = commandeGauche - commandeGauchePrecedente;
    const float changement1 = commandeDroite - commandeDroitePrecedente;
    float amplitude0;
    if (changement0 < 0.0f)
    {
        amplitude0 = -changement0;
    }
    else
    {
        amplitude0 = changement0;
    }
    float amplitude1;
    if (changement1 < 0.0f)
    {
        amplitude1 = -changement1;
    }
    else
    {
        amplitude1 = changement1;
    }
    float amplitudeMax = amplitude0;
    if (amplitude1 > amplitudeMax) amplitudeMax = amplitude1;
    float facteurRampe = 1.0f;
    if (amplitudeMax > variationMax) facteurRampe = variationMax / amplitudeMax;
    commandeGauche = commandeGauchePrecedente + changement0 * facteurRampe;
    commandeDroite = commandeDroitePrecedente + changement1 * facteurRampe;
    MOTOR_SetSpeed(0, commandeGauche);
    MOTOR_SetSpeed(1, commandeDroite);
    commandeGauchePrecedente = commandeGauche;
    commandeDroitePrecedente = commandeDroite;

    encodeurGauchePrecedent = encodeurGauche;
    encodeurDroitPrecedent = encodeurDroit;
    tempsPrecedent = tempsPresent;

}


float pid_distanceArret(void)
{
    if (!initialise || vitesseDroiteCmS <= 0.0f ||
        (commandeGauchePrecedente <= 0.0f && commandeDroitePrecedente <= 0.0f))
        return 0.0f;
    unsigned long duree = RAMPE_ARRET_MS;
    if (!digitalRead(vertpin) || !digitalRead(rougepin)) duree = RAMPE_OBSTACLE_MS;
    return vitesseDroiteCmS * duree / 2000.0f * COEFF_ANTICIPATION_ARRET + MARGE_ARRET_CM;
}


void pid_arret(void)
{
    float depart0 = commandeGauchePrecedente;
    float depart1 = commandeDroitePrecedente;
    bool obstacleDetecte = !digitalRead(vertpin) || !digitalRead(rougepin);
    unsigned long duree = RAMPE_ARRET_MS;
    if (obstacleDetecte) duree = RAMPE_OBSTACLE_MS;
    unsigned long debutRampe = millis();

    while (commandeGauchePrecedente > 0.0f || commandeDroitePrecedente > 0.0f)
    {
        if (!obstacleDetecte && (!digitalRead(vertpin) || !digitalRead(rougepin)))
        {
            depart0 = commandeGauchePrecedente;
            depart1 = commandeDroitePrecedente;
            debutRampe = millis();
            duree = RAMPE_OBSTACLE_MS;
            obstacleDetecte = true;
        }
        float facteur = 0.0f;
        if (duree > 0 && millis() - debutRampe < duree)
            facteur = 1.0f - (millis() - debutRampe) / (float)duree;
        commandeGauchePrecedente = depart0 * facteur;
        commandeDroitePrecedente = depart1 * facteur;
        MOTOR_SetSpeed(0, commandeGauchePrecedente);
        MOTOR_SetSpeed(1, commandeDroitePrecedente);
        if (facteur > 0.0f) delay(10);
    }
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    initialise = false;
}


void pid_tourne(bool direction, int angle)
{
    const float vitesseMin = 0.15f;
    const float vitesseMax = 0.25f;
    const float acceleration = 0.4f;
    const float kpPosition = 0.0008f;
    const float kpSynchronisation = 0.0003f;
    const int32_t tolerancePulses = 8;
    const unsigned long periodeRotationMs = 5;

    if (angle <= 0) return;

    float coefficient = COEFF_ROTATION_GAUCHE;
    if (direction == TOURNE_DROITE) coefficient = COEFF_ROTATION_DROITE;
    const float distance = angle * (DISTANCEENTREROUES_MM * 3.14159f) / 360.0f * coefficient;
    const int32_t cible0 = (int32_t)(distance * 0.1f / CM_PAR_IMPULSION_0 + 0.5f);
    const int32_t cible1 = (int32_t)(distance * 0.1f / CM_PAR_IMPULSION_1 + 0.5f);
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
        const int32_t restant0 = cible0 - parcours0;
        const int32_t restant1 = cible1 - parcours1;

        if(!termine0 && restant0 <= tolerancePulses) {
            termine0 = true;

        }
        if(!termine1 && restant1 <= tolerancePulses) {
            termine1 = true;

        }

        float plafond = vitesseMin + acceleration * ((millis() - debutMs) / 1000.0f);
        if (plafond > vitesseMax) plafond = vitesseMax;

        const float ecart = parcours0 * COEFF_DISTANCE_0 - parcours1 * COEFF_DISTANCE_1;
        float correction;
        if (!termine0 && !termine1)
        {
            correction = kpSynchronisation * ecart;
        }
        else
        {
            correction = 0.0f;
        }
        float commande0 = kpPosition * restant0 - correction;
        float commande1 = kpPosition * restant1 + correction;
        if (commande0 < vitesseMin) {commande0 = vitesseMin;}
        if (commande1 < vitesseMin) {commande1 = vitesseMin;}
        if (commande0 > plafond) {commande0 = plafond;}
        if (commande1 > plafond) {commande1 = plafond;}

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
    delay(100);
    encodeur_1_precedent = ENCODER_Read(1);
    initialise = false;
    if (CALIBRATION_DEBUG)
    {
        const int32_t parcours0 = abs(ENCODER_Read(0) - debut0);
        const int32_t parcours1 = abs(ENCODER_Read(1) - debut1);
        const float angleEncodeurs = (parcours0 * CM_PAR_IMPULSION_0 + parcours1 * CM_PAR_IMPULSION_1) *
            10.0f / DISTANCEENTREROUES_MM * 180.0f / 3.14159f;
        Serial.print("CAL ROT sens="); Serial.print((int)direction);
        Serial.print(" angle_demande="); Serial.print(angle);
        Serial.print(" cible0="); Serial.print(cible0);
        Serial.print(" cible1="); Serial.print(cible1);
        Serial.print(" pulses0="); Serial.print(parcours0);
        Serial.print(" pulses1="); Serial.print(parcours1);
        Serial.print(" erreur0="); Serial.print(parcours0 - cible0);
        Serial.print(" erreur1="); Serial.print(parcours1 - cible1);
        Serial.print(" angle_encodeurs="); Serial.println(angleEncodeurs, 2);
    }
    Serial.println("Tourne complete");
}
