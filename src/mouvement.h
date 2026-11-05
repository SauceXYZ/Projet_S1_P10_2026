#ifndef MOUVEMENT_H
#define MOUVEMENT_H

#include <stdint.h>

void avanceV3(void);
bool avanceImpulsions(int32_t distance, bool versCentre, bool finDeCase);
// Optional forward-target cap; the three-argument version keeps its old behavior.
bool avanceImpulsions(int32_t distance, bool versCentre, bool finDeCase, float vitesseMax);
void tourne(int directionDemande);

#endif