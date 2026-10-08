#ifndef MOUVEMENT_H
#define MOUVEMENT_H



int avance(void);
int testavance(int distance);
int detectObstacle(void);

void tourne(char direction);
void tourner_gauche(void);
void tourner_droite(void);
void scan(void);

#endif