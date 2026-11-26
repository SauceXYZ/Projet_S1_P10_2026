#include "Pompe.h"
#include "../config.h"

void initPompe(void)
{
   pinMode(PIN_POMPE_GAUCHE, OUTPUT);
   pinMode(PIN_POMPE_DROITE, OUTPUT);
}

void actionner_pompe_Gauche()
{
   digitalWrite(PIN_POMPE_GAUCHE, HIGH);
   delay(1000);
   digitalWrite(PIN_POMPE_GAUCHE, LOW);
}

void actionner_pompe_Droite()
{
   digitalWrite(PIN_POMPE_DROITE, HIGH);
   delay(1000);
   digitalWrite(PIN_POMPE_DROITE, LOW);
}