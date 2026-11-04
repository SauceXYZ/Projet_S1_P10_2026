#include <Arduino.h>
#include <LibRobus.h>
#include "robotVerre.h"

bool lireDistanceIR(float &distanceCm)
{
    // Le seul capteur utilise est branche sur l'entree IR3.
    const uint16_t valeurBrute = ROBUS_ReadIR(3);
    const float tension = valeurBrute * (5.0 / 1023.0);

    if (tension <= 0.42)
    {
        return false;
    }

    distanceCm = 27.86 / (tension - 0.42);
    return distanceCm >= 10 && distanceCm <= 80;
}
