#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>

const uint8_t SERVO_J6 = 0;
const uint8_t SERVO_J7 = 1;
const uint8_t J6_SERRE = 110;
const uint8_t J7_LEVE = 160;

bool lireDistanceIR(float &distanceCm);

bool initialiserServos();
bool reglerCible(uint8_t id, uint8_t angle);
void mettreAJourServos(unsigned long maintenant);
bool servoTermine(uint8_t idServo);
void executerRamassageVerre();

#endif
