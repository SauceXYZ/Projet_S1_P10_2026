#include <LibRobus.h>
#include "contourne_mur.h"
#include "main.h"
#include "mouvement.h"

static int32_t cmEnImpulsions(float cm)
{
    return (int32_t)(cm / CM_PAR_IMPULSION_1);
}

static bool murDevant(void)
{
    // LOW = obstacle
    return !digitalRead(rougepin) || !digitalRead(vertpin);
}

bool contourneMur(void)
{
    const int MAX_DECALAGES = 4;
    int32_t decalageImpulsions = 0;
    int decalages = 0;

    while (decalages < MAX_DECALAGES)
    {
        tourne(TOURNE_DROITE);
        delay(100);

        int32_t avant = ENCODER_Read(1);
        bool coteeLibre = avanceImpulsions(cmEnImpulsions(50), false, false);
        decalageImpulsions += ENCODER_Read(1) - avant;
        decalages++;
        if (!coteeLibre) return false;      // obstacle sur le cote

        tourne(TOURNE_GAUCHE);      // refait face a la direction initiale
        delay(100);

        if (!murDevant()) break;    // passage libre
    }
    if (murDevant()) return false;  // trop de tentatives

    avanceImpulsions(cmEnImpulsions(20), false, false);   // depasse le mur (a calibrer)
    tourne(TOURNE_GAUCHE);                                // vers la ligne
    delay(100);
    avanceImpulsions(decalageImpulsions, false, false);   // retour du decalage total
    tourne(TOURNE_DROITE);                                // reprend la direction initiale
    return true;
}