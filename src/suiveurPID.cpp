#include <LibRobus.h>
#include <math.h>
#include "main.h"
#include "pid.h"
#include "suiveurPID.h"

namespace {
// Starting values: calibrate each sensor on the actual floor and tape.
const int SOL[3] = {160, 160, 160}; // left, centre, right
const int NOIR[3] = {750, 750, 750};
const int SEUIL_CAPTEUR = 450;
const int HYSTERESIS = 15;
const unsigned long PERIODE_MS = 5;
const unsigned long PONT_MS = 120;
const unsigned long RECHERCHE_MS = 1800;
const float KP = 0.20f;
const float KD = 0.0f; // error per second
const float VITESSE = 0.23f;
const float VITESSE_LENTE = 0.12f;
const float CORRECTION_MAX = 0.25f;
// Positive error (right sensor) must make the left wheel faster.
// Change to -1 only if the physical sensor/motor mapping is reversed.
const float SENS_DIRECTION = -1.0f;
enum Etat { SUIVI, PONT, RECHERCHE, ARRET };
enum Trou { INACTIF, APPROCHE, ALIGNEMENT, TRAVERSEE, TERMINE };
Etat etat = PONT;
Trou modeTrou = INACTIF;
bool initialise = false;
bool detecte[3] = {false, false, false};
float filtre[3] = {0, 0, 0};
float erreurPrecedente = 0;
float derivee = 0;
float correction = 0;
float gauche = 0;
float droite = 0;
int dernierCote = 0;
bool deriveeValide = false;
unsigned long dernierTick = 0;
unsigned long derniereDetection = 0;
unsigned long debutTrou = 0;
unsigned long debutMotif = 0;
bool motifStable = false;

float borner(float x, float minimum, float maximum)
{
    return fmaxf(minimum, fminf(maximum, x));
}

void commander(float vitesse, float direction, float dt)
{
    const float cibleG = borner(vitesse + SENS_DIRECTION * direction, -0.25f, 0.4f);
    const float cibleD = borner(vitesse - SENS_DIRECTION * direction, -0.25f, 0.4f);
    // Decelerate immediately; ramp increases in forward speed only.
    gauche = cibleG > gauche ? fminf(cibleG, gauche + 1.5f * dt) : cibleG;
    droite = cibleD > droite ? fminf(cibleD, droite + 1.5f * dt) : cibleD;
    MOTOR_SetSpeed(0, gauche);
    MOTOR_SetSpeed(1, droite);
}

bool confirmerMotif(bool present, unsigned long maintenant)
{
    if (!present) { motifStable = false; return false; }
    if (!motifStable) { debutMotif = maintenant; motifStable = true; }
    return maintenant - debutMotif >= 20;
}
}

void reinitialiserSuiveur(void)
{
    initialise = false;
    etat = PONT;
    modeTrou = INACTIF;
    erreurPrecedente = derivee = correction = 0;
    gauche = droite = 0;
    dernierCote = 0;
    deriveeValide = false;
    motifStable = false;
    for (int i = 0; i < 3; ++i) detecte[i] = false;
}

void arreterSuiveur(void)
{
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    gauche = droite = 0;
    etat = ARRET;
    deriveeValide = false;
}

void suivreLaLigne(void)
{
    const unsigned long maintenant = millis();
    if (etat == ARRET) return;
    const bool premier = !initialise;
    const unsigned long ecoule = maintenant - dernierTick;
    if (!premier && ecoule < PERIODE_MS) return;
    const bool interruption = !premier && ecoule > 50;
    const float dt = premier ? 0.005f : fminf(ecoule * 0.001f, 0.05f);
    dernierTick = maintenant;
    if (premier) { initialise = true; derniereDetection = maintenant; }
    if (interruption) deriveeValide = false;

    const int pins[3] = {pinSuiveurGauche, pinSuiveurMilieu, pinSuiveurDroit};
    float force[3];
    int nombre = 0;
    for (int i = 0; i < 3; ++i) {
        const int brut = analogRead(pins[i]);
        // Fast smoothing, independent of call frequency.
        filtre[i] = premier ? brut : filtre[i] + dt / (0.008f + dt) * (brut - filtre[i]);
        if (filtre[i] > SEUIL_CAPTEUR + HYSTERESIS) detecte[i] = true;
        else if (filtre[i] < SEUIL_CAPTEUR - HYSTERESIS) detecte[i] = false;
        // Sub-threshold reflectivity is background, not lateral line evidence.
        const float fond = fmaxf(SOL[i], SEUIL_CAPTEUR - HYSTERESIS);
        force[i] = borner((filtre[i] - fond) / fmaxf(NOIR[i] - fond, 1.0f), 0, 1);
        if (detecte[i]) ++nombre;
    }

    // The hole marker is deliberately separate from normal line estimation.
    if (modeTrou != INACTIF && modeTrou != TERMINE) {
        if (maintenant - debutTrou > 8000) { arreterSuiveur(); return; }
        if (modeTrou == APPROCHE && confirmerMotif(nombre == 3, maintenant)) {
            modeTrou = ALIGNEMENT; motifStable = false;
        }
        if (modeTrou == ALIGNEMENT) {
            // Cross the marker slowly, keeping the side sensor strengths balanced.
            commander(0.10f, borner(0.12f * (force[2] - force[0]), -0.08f, 0.08f), dt);
            if (confirmerMotif(!detecte[0] && !detecte[2], maintenant)) {
                modeTrou = TRAVERSEE; motifStable = false;
                pid_init();
            }
            return;
        }
        if (modeTrou == TRAVERSEE) {
            // Encoder PID owns the motors during the deliberate gap crossing.
            pid_avance(0.2f);
            if (confirmerMotif(detecte[1] && (detecte[0] || detecte[2]), maintenant)) {
                pid_arret();
                reinitialiserSuiveur();
                modeTrou = TERMINE;
            }
            return;
        }
    }

    // Only comparable outside signals are ambiguous. A strong tape signal
    // must outweigh a second sensor barely crossing its threshold.
    const bool ambigu = detecte[0] && detecte[2] &&
        fabsf(force[0] - force[2]) < 0.25f;
    if (nombre && !ambigu) {
        const bool continu = etat == SUIVI && deriveeValide && !interruption;
        const float total = force[0] + force[1] + force[2];
        // A side card may be as dark as tape. While the centre still
        // detects the line, a single outside detection must not pull us away.
        // Keep both-outside handling separate for markers/intersections.
        const bool centrePrioritaire = detecte[1] && !(detecte[0] && detecte[2]);
        const float erreur = centrePrioritaire ? 0.0f :
            (force[2] - force[0]) / fmaxf(total, 0.05f);
        const float taux = continu ? (erreur - erreurPrecedente) / dt : 0;
        derivee = continu ? derivee + dt / (0.035f + dt) * (taux - derivee) : 0;
        correction = borner(KP * erreur + KD * derivee, -CORRECTION_MAX, CORRECTION_MAX);
        erreurPrecedente = erreur;
        deriveeValide = true;
        if (erreur > 0.12f) dernierCote = 1;
        else if (erreur < -0.12f) dernierCote = -1;
        etat = SUIVI;
        derniereDetection = maintenant;
        const float vitesse = VITESSE - (VITESSE - VITESSE_LENTE) *
            borner(fabsf(correction) / CORRECTION_MAX, 0, 1);
        commander(vitesse, correction, dt);
        return;
    }

    deriveeValide = false;
    if (ambigu) {
        // Preserve heading briefly, then stop if the ambiguous patch persists.
        if (maintenant - derniereDetection > 500) { arreterSuiveur(); return; }
        commander(0.10f, correction * 0.5f, dt);
        return;
    }
    const unsigned long absence = maintenant - derniereDetection;
    if (absence < PONT_MS) {
        etat = PONT;
        // Keep the last measured lateral error, without a stale D term.
        commander(0.10f, borner(KP * erreurPrecedente, -0.16f, 0.16f), dt);
    } else if (absence < PONT_MS + RECHERCHE_MS) {
        etat = RECHERCHE;
        // Search toward the last known side, then sweep across the other side.
        // With no direction evidence, stop rather than choose arbitrarily.
        if (!dernierCote) { arreterSuiveur(); return; }
        const unsigned long recherche = absence - PONT_MS;
        const int sens = recherche < 600 ? dernierCote : -dernierCote;
        commander(0, sens * 0.16f, dt);
    } else {
        arreterSuiveur();
    }
}

void trou(void)
{
    reinitialiserSuiveur();
    modeTrou = APPROCHE;
    debutTrou = millis();
}

bool trouTermine(void) { return modeTrou == TERMINE; }

void testSuiveur(void)
{
    static unsigned long precedent = 0;
    if (millis() - precedent < 100) return;
    precedent = millis();
    Serial.print(analogRead(pinSuiveurGauche)); Serial.print(' ');
    Serial.print(analogRead(pinSuiveurMilieu)); Serial.print(' ');
    Serial.print(analogRead(pinSuiveurDroit)); Serial.print(' ');
    Serial.print(static_cast<int>(etat)); Serial.print(' ');
    Serial.println(erreurPrecedente, 3);
}
