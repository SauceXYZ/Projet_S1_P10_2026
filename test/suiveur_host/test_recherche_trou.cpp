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
int main() {
    testerTrou();
    std::cout << "Hole scenarios passed\n";
}
