#include <cassert>
#include <cmath>
#include <iostream>
#include "LibRobus.h"
unsigned long temps = 0;
int valeurs[3] = {200, 750, 200};
float moteurs[2] = {0, 0};
int pinSuiveurGauche = 0, pinSuiveurMilieu = 1, pinSuiveurDroit = 2;
SerialStub Serial;
int32_t encodeursTest[2] = {10000, 10000};
int32_t ENCODER_Read(int moteur) { return encodeursTest[moteur]; }
unsigned long millis() { return temps; }
int analogRead(int pin) { return valeurs[pin]; }
void MOTOR_SetSpeed(int moteur, float vitesse) {
    assert(std::isfinite(vitesse));
    assert(vitesse >= -0.25f && vitesse <= 0.4f);
    moteurs[moteur] = vitesse;
}
void pid_init() {}
void pid_arret() { moteurs[0] = moteurs[1] = 0; }
void pid_avance(float vitesse) { moteurs[0] = moteurs[1] = vitesse; }
#define DEBUG_TROU 0
#define DEBUG_RECHERCHE 0
void delay(unsigned long) {}
#include "../../src/suiveurPID.cpp"
void avancer(int g, int m, int d, int duree) {
    valeurs[0] = g; valeurs[1] = m; valeurs[2] = d;
    for (int i = 0; i < duree; i += 5) { temps += 5; suivreLaLigne(); }
}
#include "scenarios_trou.h"
#include "scenarios_recherche.h"
int main() {
    reinitialiserSuiveur();
    avancer(200, 750, 200, 300);
    assert(moteurs[0] > 0 && fabs(moteurs[0] - moteurs[1]) < 0.001f);
    avancer(200, 200, 200, 60); // middle -> sensor gap
    assert(moteurs[0] > 0 && moteurs[1] > 0);
    avancer(200, 200, 750, 100);
    assert((moteurs[0] - moteurs[1]) * SENS_DIRECTION > 0);
    avancer(200, 200, 200, 180); // recover toward last side
    assert(moteurs[0] * SENS_DIRECTION > 0 && moteurs[1] * SENS_DIRECTION < 0);
    avancer(200, 750, 200, 400); // reacquire
    assert(moteurs[0] > 0 && fabs(moteurs[0] - moteurs[1]) < 0.01f);
    avancer(750, 200, 200, 100);
    assert((moteurs[1] - moteurs[0]) * SENS_DIRECTION > 0);
    avancer(200, 200, 200, PONT_MS + RECHERCHE_MS + 500);
    assert(moteurs[0] == 0 && moteurs[1] == 0);
    avancer(200, 750, 200, 100); // stop remains latched
    assert(moteurs[0] == 0 && moteurs[1] == 0);
    reinitialiserSuiveur();
    avancer(750, 750, 750, 600); // ambiguous marker timeout
    assert(moteurs[0] == 0 && moteurs[1] == 0);
    // Regression from the robot log: centre tape with bright right background.
    reinitialiserSuiveur();
    avancer(287, 733, 438, 400);
    assert(fabs(erreurPrecedente) < 0.05f);
    // A weak second outside detection must not trigger the ambiguity timeout.
    avancer(717, 395, 489, 1000);
    assert(etat == SUIVI);
    avancer(663, 406, 469, 600);
    assert(etat == SUIVI);
    avancer(160, 700, 160, 400);
    assert(etat == SUIVI && fabs(erreurPrecedente) < 0.01f);
    // A dark card beside the tape must not steer away from an active centre.
    avancer(178, 512, 750, 500);
    assert(etat == SUIVI && fabs(erreurPrecedente) < 0.001f);
    assert(fabs(moteurs[0] - moteurs[1]) < 0.01f);
    avancer(750, 512, 178, 500); // same rule on the opposite side
    assert(etat == SUIVI && fabs(erreurPrecedente) < 0.001f);
    // Once the centre loses tape, real sharp turns must still be followed.
    avancer(25, 35, 644, 200);
    assert(etat == SUIVI && erreurPrecedente > 0.9f);
    assert((moteurs[0] - moteurs[1]) * SENS_DIRECTION > 0);
    testerTrou();
    testerRecherche();
    std::cout << "Line follower and hole scenarios passed\n";
}
