#include <Arduino.h>
#include "robotVerre.h"

const unsigned long INTERVALLE_CAPTEUR_MS = 200;
const float DISTANCE_VERRE_CM = 18.0;
const byte LECTURES_A_CONFIRMER = 3;

void executerRamassageVerre()
{
    byte nombreLectures = 0;
    unsigned long derniereLecture = 0;
    bool j6Ferme = false;
    bool commandeJ7Envoyee = false;

    while (true)
    {
        const unsigned long maintenant = millis();
        mettreAJourServos(maintenant);

        if (!j6Ferme && maintenant - derniereLecture >= INTERVALLE_CAPTEUR_MS)
        {
            derniereLecture = maintenant;

            float distance = 0;
            if (lireDistanceIR(distance) && distance <= DISTANCE_VERRE_CM)
            {
                nombreLectures++;
            }
            else
            {
                nombreLectures = 0;
            }

            if (nombreLectures >= LECTURES_A_CONFIRMER)
            {
                j6Ferme = reglerCible(SERVO_J6, J6_SERRE);
            }
        }

        if (j6Ferme && servoTermine(SERVO_J6) && !commandeJ7Envoyee)
        {
            commandeJ7Envoyee = reglerCible(SERVO_J7, J7_LEVE);
        }

        if (commandeJ7Envoyee && servoTermine(SERVO_J7))
        {
            break;
        }
    }
}
