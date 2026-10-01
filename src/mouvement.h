#ifndef MOUVEMENT_H
#define MOUVEMENT_H


#define TOURNE_DROIT 0
#define TOURNE_GAUCHE 1

int avance(void);
int testavance(int distance);
int detectObstacle(void);

void tourne(char direction);
void tourner_gauche(void);
void tourner_droite(void);
void scan(void);

#endif