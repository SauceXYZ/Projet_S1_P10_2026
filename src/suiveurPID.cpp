#include <LibRobus.h>
#include <math.h>
#include <stdio.h>
#include "main.h"
#include "pid.h"
#include "suiveurPID.h"
#include "trou_config.h"

#ifndef DEBUG_RECHERCHE
#define DEBUG_RECHERCHE 1
#endif

#ifndef DEBUG_TROU
#define DEBUG_TROU 1
#endif
#if DEBUG_TROU
#define TRACE_TROU(message) debugTrou(F(message))
#else
#define TRACE_TROU(message) do {} while (false)
#endif

namespace {
// Starting values: calibrate each sensor on the actual floor and tape.
const int SOL[3] = {160, 160, 160}; // left, centre, right
const int NOIR[3] = {750, 750, 750};
const int SEUIL_CAPTEUR[3] = {300, 300, 500}; // left, centre, right
const int HYSTERESIS = 15;
const unsigned long PERIODE_MS = 5;
const unsigned long PONT_MS = 120;
// Four widening sweeps: 500, 1000, 1500 and 2000 ms (5 s total).
const unsigned long BALAYAGE_BASE_MS = 500;
const unsigned long RECHERCHE_MS = 10 * BALAYAGE_BASE_MS;
const unsigned long REACQUISITION_MS = 30;
const float VITESSE_BALAYAGE = 0.14f;
const float ACCELERATION_BALAYAGE = 0.8f;
const float KP = 0.25f;
const float KD = 0.006f; // error per second
const float VITESSE = 0.23f;
const float VITESSE_LENTE = 0.12f;
const float CORRECTION_MAX = 0.25f;
// Positive error (right sensor) must make the left wheel faster.
// Change to -1 only if the physical sensor/motor mapping is reversed.
const float SENS_DIRECTION = -1.0f;
enum Etat { SUIVI, PONT, RECHERCHE, ARRET };
Etat etat = PONT;
bool initialise = false;
bool detecte[3] = {false, false, false};
float filtre[3] = {0, 0, 0};
float erreurPrecedente = 0, derivee = 0, correction = 0;
float gauche = 0, droite = 0;
int dernierCote = 0;
bool deriveeValide = false;
unsigned long dernierTick = 0, derniereDetection = 0;
int sensInitialRecherche = -1;
int dernierBalayage = -1;
bool ligneCandidate = false;
unsigned long debutLigneCandidate = 0;

enum class PhaseTrou { INACTIF, RECUL, ALIGNEMENT, TRAVERSEE, ARRIVEE, TERMINE, ECHEC };
struct EtatTrou {
    PhaseTrou phase = PhaseTrou::INACTIF;
    unsigned long debutEtape = 0;
    bool pause = false;
    bool tFranchi = false;
    int premier = -1;
    bool premierConfirme = false;
    bool extremiteT[2] = {false, false};
    bool perteExtremite = false;
    unsigned long debutPerte = 0;
    bool confirmation = false;
    unsigned long debutConfirmation = 0;
    bool videConfirme = false;
    int32_t encodeursVide[2] = {0, 0};
    unsigned long debutTraversee = 0;
    float distanceCm = 0;
    unsigned long dernierDebug = 0;
} trouEtat;

bool capteurVoitT(int capteur)
{
    return trouEtat.extremiteT[capteur == 0 ? 0 : 1];
}

bool roueGaucheAlignement()
{
    return (trouEtat.premier == 0) == (ReglageTrou::SENS_ALIGNEMENT > 0);
}

#if DEBUG_TROU
const __FlashStringHelper* nomPhase(PhaseTrou phase)
{
    switch (phase) {
        case PhaseTrou::INACTIF: return F("INACTIF");
        case PhaseTrou::RECUL: return F("1_RECUL");
        case PhaseTrou::ALIGNEMENT: return F("2_ALIGNEMENT");
        case PhaseTrou::TRAVERSEE: return F("3_TRAVERSEE_PID");
        case PhaseTrou::ARRIVEE: return F("4_ARRIVEE");
        case PhaseTrou::TERMINE: return F("TERMINE");
        case PhaseTrou::ECHEC: return F("ECHEC");
    }
    return F("?");
}
void debugTrou(const __FlashStringHelper* message)
{
    Serial.print(F("[TROU ")); Serial.print(millis()); Serial.print(F("] "));
    Serial.print(nomPhase(trouEtat.phase)); Serial.print(' '); Serial.println(message);
}
// These snapshots are emitted only while both motors are stopped.
void debugPivot(const __FlashStringHelper* evenement)
{
    Serial.print(F("[ALIGNEMENT] ")); Serial.print(evenement);
    Serial.print(F(" premier=")); Serial.print(trouEtat.premier);
    Serial.print(F(" roue="));
    Serial.print(roueGaucheAlignement() ? F("GAUCHE(0)") : F("DROITE(1)"));
    Serial.print(F(" GMD="));
    for (int i = 0; i < 3; ++i) { Serial.print(static_cast<int>(filtre[i])); Serial.print('/'); }
    Serial.print(F(" noir="));
    for (int i = 0; i < 3; ++i) Serial.print(detecte[i] ? '1' : '0');
    Serial.print(F(" barreGD="));
    Serial.print(trouEtat.extremiteT[0] ? '1' : '0');
    Serial.println(trouEtat.extremiteT[1] ? '1' : '0');
}
void debugMesures(unsigned long maintenant)
{
    if (maintenant - trouEtat.dernierDebug < 250) return;
    char ligne[64];
    const int taille = snprintf(ligne, sizeof(ligne),
        "[T%u] GMD=%d/%d/%d n=%u%u%u b=%u%u T=%u p=%d d=%dcm\n",
        static_cast<unsigned>(trouEtat.phase), static_cast<int>(filtre[0]),
        static_cast<int>(filtre[1]), static_cast<int>(filtre[2]),
        static_cast<unsigned>(detecte[0]), static_cast<unsigned>(detecte[1]),
        static_cast<unsigned>(detecte[2]), static_cast<unsigned>(trouEtat.extremiteT[0]),
        static_cast<unsigned>(trouEtat.extremiteT[1]), static_cast<unsigned>(trouEtat.tFranchi),
        trouEtat.premier, static_cast<int>(trouEtat.distanceCm));
    if (taille > 0 && taille < static_cast<int>(sizeof(ligne)) &&
        Serial.availableForWrite() >= taille) {
        Serial.print(ligne);
        trouEtat.dernierDebug = maintenant;
    }
}
#endif

float borner(float x, float minimum, float maximum)
{
    return fmaxf(minimum, fminf(maximum, x));
}
void immobiliserT()
{
    gauche = droite = 0;
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
}
float rampeTrou(float actuelle, float cible, float dt)
{
    if (actuelle * cible < 0) actuelle = 0;
    if (fabsf(cible) <= fabsf(actuelle)) return cible;
    const float amplitude = fminf(fabsf(cible), fabsf(actuelle) + ReglageTrou::ACCELERATION * dt);
    return cible < 0 ? -amplitude : amplitude;
}
void rouesTrou(float cibleG, float cibleD, float dt)
{
    gauche = rampeTrou(gauche, cibleG, dt);
    droite = rampeTrou(droite, cibleD, dt);
    MOTOR_SetSpeed(0, gauche);
    MOTOR_SetSpeed(1, droite);
}
void commander(float vitesse, float direction, float dt)
{
    const float cibleG = borner(vitesse + SENS_DIRECTION * direction, -0.25f, 0.4f);
    const float cibleD = borner(vitesse - SENS_DIRECTION * direction, -0.25f, 0.4f);
    gauche = cibleG > gauche ? fminf(cibleG, gauche + 1.5f * dt) : cibleG;
    droite = cibleD > droite ? fminf(cibleD, droite + 1.5f * dt) : cibleD;
    MOTOR_SetSpeed(0, gauche);
    MOTOR_SetSpeed(1, droite);
}
// Pivot in place, alternating farther across the last known heading.
// Elapsed time stays tied to the original loss, even if a spurious signal appears.
void balayerLigne(unsigned long recherche)
{
    unsigned long debutBalayage = 0;
    unsigned long duree = BALAYAGE_BASE_MS;
    int passage = 0;
    while (passage < 3 && recherche - debutBalayage >= duree) {
        debutBalayage += duree;
        ++passage;
        duree = BALAYAGE_BASE_MS * (passage + 1);
    }
    const int sens = passage % 2 == 0 ? sensInitialRecherche : -sensInitialRecherche;
    const float amplitude = fminf(VITESSE_BALAYAGE,
        ACCELERATION_BALAYAGE * (recherche - debutBalayage) * 0.001f);
    gauche = SENS_DIRECTION * sens * amplitude;
    droite = -gauche;
    MOTOR_SetSpeed(0, gauche);
    MOTOR_SetSpeed(1, droite);
    if (passage != dernierBalayage) {
        dernierBalayage = passage;
#if DEBUG_RECHERCHE
        // Keep motion diagnostics short and skip if the serial buffer is busy.
        if (Serial.availableForWrite() >= 48) {
            if (sens > 0) Serial.println(F("[SUIVEUR] recherche cote droit"));
            else Serial.println(F("[SUIVEUR] recherche cote gauche"));
        }
#endif
    }
}

bool confirmerTrou(bool present, unsigned long maintenant, unsigned long duree)
{
    if (!present) { trouEtat.confirmation = false; return false; }
    if (!trouEtat.confirmation) {
        trouEtat.confirmation = true;
        trouEtat.debutConfirmation = maintenant;
    }
    return maintenant - trouEtat.debutConfirmation >= duree;
}
void entrerTrou(PhaseTrou phase, unsigned long maintenant)
{
    immobiliserT();
    trouEtat.phase = phase;
    trouEtat.debutEtape = maintenant;
    trouEtat.pause = ReglageTrou::PAUSE_MS > 0;
    trouEtat.confirmation = false;
    if (phase == PhaseTrou::TRAVERSEE && !trouEtat.pause) {
        pid_init();
        trouEtat.debutTraversee = maintenant;
    }
    TRACE_TROU("ETAPE, moteurs arretes pendant la pause");
}
void echouerTrou()
{
    immobiliserT();
    trouEtat.phase = PhaseTrou::ECHEC;
    trouEtat.pause = false;
    etat = ARRET;
    TRACE_TROU("ECHEC VERROUILLE: reinitialisation requise");
}
float distanceDepuisVide()
{
    const float g = fmaxf(0, (ENCODER_Read(0) - trouEtat.encodeursVide[0]) * CM_PAR_IMPULSION_0);
    const float d = fmaxf(0, (ENCODER_Read(1) - trouEtat.encodeursVide[1]) * CM_PAR_IMPULSION_1);
    return fminf(g, d);
}
// Four sequential steps. A failure never starts a previous step automatically.
void executerTrou(unsigned long maintenant, float dt)
{
    using namespace ReglageTrou;
    if (trouEtat.phase == PhaseTrou::ECHEC) return;
    if (trouEtat.pause) {
        if (maintenant - trouEtat.debutEtape < PAUSE_MS) return;
        trouEtat.pause = false;
        trouEtat.debutEtape = maintenant;
        if (trouEtat.phase == PhaseTrou::TRAVERSEE) {
            pid_init();
            trouEtat.debutTraversee = maintenant;
        }
        TRACE_TROU("DEBUT MOUVEMENT");
    }
#if DEBUG_TROU
    debugMesures(maintenant);
#endif
    const unsigned long ecoule = maintenant - trouEtat.debutEtape;
    switch (trouEtat.phase) {
        case PhaseTrou::RECUL:
            if (ecoule > RECUL_MAX_MS) {
                immobiliserT();
                if (!trouEtat.tFranchi) { TRACE_TROU("TIMEOUT: T non vu pendant le recul"); }
                else { TRACE_TROU("TIMEOUT: centre seul non retrouve derriere le T"); }
                echouerTrou(); return;
            }
            rouesTrou(-VITESSE_MANOEUVRE, -VITESSE_MANOEUVRE, dt);
            if (!trouEtat.tFranchi) {
                if (confirmerTrou(capteurVoitT(0) && capteurVoitT(2), maintenant, T_CONFIRMATION_MS)) {
                    trouEtat.tFranchi = true;
                    trouEtat.confirmation = false;
                }
            } else if (confirmerTrou(detecte[1] && !detecte[0] && !detecte[2],
                                     maintenant, CENTRE_CONFIRMATION_MS)) {
                entrerTrou(PhaseTrou::ALIGNEMENT, maintenant);
            }
            return;
        case PhaseTrou::ALIGNEMENT:
            if (ecoule > ALIGNEMENT_MAX_MS) {
                immobiliserT(); TRACE_TROU("TIMEOUT: alignement du T incomplet");
                echouerTrou(); return;
            }
            if (!trouEtat.premierConfirme) {
                if (!capteurVoitT(0) && !capteurVoitT(2)) {
                    trouEtat.premier = -1;
                    trouEtat.confirmation = false;
                    rouesTrou(VITESSE_MANOEUVRE, VITESSE_MANOEUVRE, dt);
                } else {
                    immobiliserT();
                    if (trouEtat.premier < 0 || !capteurVoitT(trouEtat.premier)) {
                        trouEtat.premier = capteurVoitT(0) ? 0 : 2;
                        trouEtat.confirmation = false;
                    }
                    if (confirmerTrou(capteurVoitT(trouEtat.premier), maintenant, PREMIERE_EXTREMITE_MS)) {
                        trouEtat.premierConfirme = true;
                        trouEtat.confirmation = false;
                        if (trouEtat.premier == 0) { TRACE_TROU("GAUCHE CONFIRMEE 100ms"); }
                        else { TRACE_TROU("DROITE CONFIRMEE 100ms"); }
#if DEBUG_TROU
                        debugPivot(F("DEBUT PIVOT"));
#endif
                    }
                }
                return;
            }
            if (!capteurVoitT(trouEtat.premier)) {
                immobiliserT();
                trouEtat.confirmation = false;
                if (!trouEtat.perteExtremite) {
                    trouEtat.perteExtremite = true;
                    trouEtat.debutPerte = maintenant;
                    TRACE_TROU("EXTREMITE PERDUE: moteurs arretes");
#if DEBUG_TROU
                    Serial.print(F("[ALIGNEMENT] attente_max_ms="));
                    Serial.println(PERTE_EXTREMITE_MAX_MS);
#endif
                }
                if (maintenant - trouEtat.debutPerte >= PERTE_EXTREMITE_MAX_MS) {
                    TRACE_TROU("ECHEC: premiere extremite absente au-dela du delai");
                    echouerTrou();
                }
                return;
            }
            if (trouEtat.perteExtremite) {
                trouEtat.perteExtremite = false;
                TRACE_TROU("EXTREMITE RETROUVEE: reprise avec rampe");
            }
            if (capteurVoitT(0) && capteurVoitT(2)) {
                immobiliserT();
                if (confirmerTrou(true, maintenant, T_CONFIRMATION_MS)) {
#if DEBUG_TROU
                    debugPivot(F("VALIDATION T"));
#endif
                    entrerTrou(PhaseTrou::TRAVERSEE, maintenant);
                }
            } else {
                trouEtat.confirmation = false;
                // The side that has not reached the transverse bar catches up.
                const bool gaucheActive = roueGaucheAlignement();
                rouesTrou(gaucheActive ? VITESSE_MANOEUVRE : 0,
                          gaucheActive ? 0 : VITESSE_MANOEUVRE, dt);
            }
            return;
        case PhaseTrou::TRAVERSEE:
        case PhaseTrou::ARRIVEE:
            if (maintenant - trouEtat.debutTraversee > TRAVERSEE_MAX_MS) {
                immobiliserT();
                if (!trouEtat.videConfirme) { TRACE_TROU("TIMEOUT: T de depart non quitte"); }
                else { TRACE_TROU("TIMEOUT: arrivee non confirmee"); }
                pid_init();
                echouerTrou(); return;
            }
            // PID exclusively owns the motors through departure, gap and arrival.
            pid_avance(VITESSE_TRAVERSEE);
            if (trouEtat.phase == PhaseTrou::TRAVERSEE) {
                if (!trouEtat.videConfirme) {
                    if (confirmerTrou(!detecte[0] && !detecte[1] && !detecte[2],
                                      maintenant, VIDE_CONFIRMATION_MS)) {
                        trouEtat.videConfirme = true;
                        trouEtat.encodeursVide[0] = ENCODER_Read(0);
                        trouEtat.encodeursVide[1] = ENCODER_Read(1);
                        trouEtat.confirmation = false;
                        TRACE_TROU("VIDE CONFIRME: origine distance");
                    }
                } else {
                    trouEtat.distanceCm = distanceDepuisVide();
                    if (trouEtat.distanceCm >= DISTANCE_MIN_ARRIVEE_CM) {
                        // Detection gate only: continue PID without another stop.
                        trouEtat.phase = PhaseTrou::ARRIVEE;
                        trouEtat.confirmation = false;
                        TRACE_TROU("DISTANCE MIN ATTEINTE: ARRIVEE AUTORISEE");
                    }
                }
                return;
            }
            trouEtat.distanceCm = distanceDepuisVide();
            if (confirmerTrou(filtre[0] >= SEUIL_ARRIVEE[0] || filtre[1] >= SEUIL_ARRIVEE[1] ||
                              filtre[2] >= SEUIL_ARRIVEE[2], maintenant, ARRIVEE_CONFIRMATION_MS)) {
                immobiliserT();
                pid_init();
                TRACE_TROU("ARRIVEE CONFIRMEE: retour au suivi normal");
                reinitialiserSuiveur();
                trouEtat.phase = PhaseTrou::TERMINE;
            }
            return;
        default: return;
    }
}
}

void reinitialiserSuiveur(void)
{
    initialise = false;
    etat = PONT;
    trouEtat = EtatTrou{};
    erreurPrecedente = derivee = correction = 0;
    gauche = droite = 0;
    dernierCote = 0;
    sensInitialRecherche = -1;
    dernierBalayage = -1;
    ligneCandidate = false;
    deriveeValide = false;
    for (int i = 0; i < 3; ++i) detecte[i] = false;
}
void arreterSuiveur(void)
{
    immobiliserT();
    etat = ARRET;
    deriveeValide = false;
    if (trouEtat.phase != PhaseTrou::INACTIF && trouEtat.phase != PhaseTrou::TERMINE)
        trouEtat.phase = PhaseTrou::ECHEC;
    TRACE_TROU("ARRET VERROUILLE");
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
        if (filtre[i] > SEUIL_CAPTEUR[i] + HYSTERESIS) detecte[i] = true;
        else if (filtre[i] < SEUIL_CAPTEUR[i] - HYSTERESIS) detecte[i] = false;
        if (i != 1) {
            const int cote = i == 0 ? 0 : 1;
            const int seuilT = ReglageTrou::SEUIL_T_EXTREMITE[cote];
            if (filtre[i] > seuilT + HYSTERESIS) trouEtat.extremiteT[cote] = true;
            else if (filtre[i] < seuilT - HYSTERESIS) trouEtat.extremiteT[cote] = false;
        }
        // Sub-threshold reflectivity is background, not lateral line evidence.
        const float fond = fmaxf(SOL[i], SEUIL_CAPTEUR[i] - HYSTERESIS);
        force[i] = borner((filtre[i] - fond) / fmaxf(NOIR[i] - fond, 1.0f), 0, 1);
        if (detecte[i]) ++nombre;
    }

    if (trouEtat.phase != PhaseTrou::INACTIF && trouEtat.phase != PhaseTrou::TERMINE) {
        executerTrou(maintenant, dt);
        return;
    }

    // Only comparable outside signals are ambiguous. A strong tape signal
    // must outweigh a second sensor barely crossing its threshold.
    const bool ambigu = detecte[0] && detecte[2] &&
        fabsf(force[0] - force[2]) < 0.25f;
    if (etat == RECHERCHE) {
        const unsigned long absence = maintenant - derniereDetection;
        if (absence >= PONT_MS + RECHERCHE_MS) {
            arreterSuiveur();
            return;
        }
        if (nombre && !ambigu) {
            // Stop the sweep at the first reading; resume following only after
            // a stable detection, without resetting the overall search timeout.
            immobiliserT();
            if (!ligneCandidate) {
                ligneCandidate = true;
                debutLigneCandidate = maintenant;
            }
            if (maintenant - debutLigneCandidate < REACQUISITION_MS) return;
            ligneCandidate = false;
            deriveeValide = false;
#if DEBUG_RECHERCHE
            if (Serial.availableForWrite() >= 48)
                Serial.println(F("[SUIVEUR] ligne retrouvee"));
#endif
        } else {
            ligneCandidate = false;
            balayerLigne(absence - PONT_MS);
            return;
        }
    }
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
        if (maintenant - derniereDetection > 500) { TRACE_TROU("ARRET: capteurs exterieurs ambigus >500ms"); arreterSuiveur(); return; }
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
        sensInitialRecherche = dernierCote != 0 ? dernierCote : -1;
        dernierBalayage = -1;
        ligneCandidate = false;
        balayerLigne(absence - PONT_MS);
    } else {
        TRACE_TROU("ARRET: recherche ligne expiree");
        arreterSuiveur();
    }
}

void trou(void)
{
    reinitialiserSuiveur();
    entrerTrou(PhaseTrou::RECUL, millis());
#if DEBUG_TROU
    Serial.print(F("[TROU] pause_ms=")); Serial.print(ReglageTrou::PAUSE_MS);
    Serial.print(F(" min_cm(PROVISOIRE)=")); Serial.println(ReglageTrou::DISTANCE_MIN_ARRIVEE_CM);
    Serial.print(F("[TROU] seuils_T_GD(PROVISOIRES)="));
    Serial.print(ReglageTrou::SEUIL_T_EXTREMITE[0]); Serial.print('/');
    Serial.println(ReglageTrou::SEUIL_T_EXTREMITE[1]);
    Serial.print(F("[TROU] seuils_arrivee(PROVISOIRES)="));
    for (int i = 0; i < 3; ++i) { Serial.print(ReglageTrou::SEUIL_ARRIVEE[i]); Serial.print('/'); }
    Serial.println();
#endif
}
bool trouTermine(void) { return trouEtat.phase == PhaseTrou::TERMINE; }
bool trouEchoue(void) { return trouEtat.phase == PhaseTrou::ECHEC; }

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
