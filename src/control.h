#ifndef CONTROL_H
#define CONTROL_H

extern const float pi;
extern char direction;
extern int etat;
extern int pos_y;
extern int pos_x;

void avance(float vitessedroit, float vitessegauche);
void recule(float vitessedroit, float vitessegauche);
void tournerGauche();
void tournerDroit();
void arret();

void tournerSelonAngle(int angle);
char tournerDansDirection(char directionActuelle, char directionVoulue);

void avancerDistance(float distance);
bool verifierMur();
void ajusterPositionRobot(char direction);
void directionAller();


#endif