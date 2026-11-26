#include "deplacement.h"
#include <LibRobus.h>
#include "../config.h"

float clamp(float max, float min, float valeur)
{
    if(valeur > max)
    {
        valeur = max;
    }
    else if(valeur < min)
    {
        valeur = min;
    }
    return valeur;
}

void avance(float offset)
{
    offset = clamp(1, -1, offset / MAX_DEPLACEMENT_OFFSET);
    float vitesseDroit = MaxSpeed * (1.0f -max(0.0f, offset));
    float vitesseGauche = MaxSpeed * (1.0f -max(0.0f, -offset));

    MOTOR_SetSpeed(RIGHT, vitesseDroit);
    MOTOR_SetSpeed(LEFT, vitesseGauche);
}