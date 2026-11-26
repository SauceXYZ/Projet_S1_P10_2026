#ifndef POMPE_H
#define POMPE_H

#include <Arduino.h>

#define PIN_POMPE_DROITE 49
#define PIN_POMPE_GAUCHE 47


void init_pompe();
void actionner_pompe_Gauche();
void actionner_pompe_Droite();

#endif 