#ifndef MOUVEMENT_H
#define MOUVEMENT_H



int avance(void);
int testavance(int distance);
int detectObstacle(void);

void tourne(int direction);
void tourner_gauche(void);
void tourner_droite(void);
void scan(void);
void faitFace(int orientation);
void avanceV2(void);
void avanceV3(void);
int deplaceCase(void);



#endif