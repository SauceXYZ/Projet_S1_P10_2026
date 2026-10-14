#ifndef MAP_H
#define MAP_H

extern int possibilite[4];
extern int carre[3][6];

void possibiliteTape(int x, int y);
void enleverAncienneCase(int posX, int posY, int AncX, int AncY);
void remettreAncienneCase(int posX, int posY, int AncX, int Anc);

#endif