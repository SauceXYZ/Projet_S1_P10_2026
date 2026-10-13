#ifndef MAP_H
#define MAP_H

extern int possibilite[4];
extern int clicParTour;
extern float diametreRoue;

void possibiliteTape(int x, int y);
void tournerGauche();
void tournerDroit();
void arret();
void tournerSelonAngle(int angle);
char tournerDansDirection(char directionActuelle, char directionVoulue);

#endif