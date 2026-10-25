#ifndef MOUVEMENT_H
#define MOUVEMENT_H

#include <stdint.h>

void avanceV3(void);
bool avanceImpulsions(int32_t distance, bool versCentre, bool finDeCase);
void tourne(int directionDemande);

#endif