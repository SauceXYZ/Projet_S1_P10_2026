#include "Pompe.h"



void actionner_pompe()
{
    digitalWrite(PIN_POMPE, HIGH);
    delay(1000);
    digitalWrite(PIN_POMPE, LOW);
}