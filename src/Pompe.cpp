#include "Pompe.h"

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