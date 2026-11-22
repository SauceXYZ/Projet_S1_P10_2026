#include "Pompe.h"

void init_pompe()
{
    pinMode(PIN_POMPE, LOW);
}

void actionner_pompe()
{
    digitalWrite(PIN_POMPE, HIGH);
    delay(1000);
    digitalWrite(PIN_POMPE, LOW);
}