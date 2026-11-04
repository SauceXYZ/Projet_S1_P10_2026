#ifndef CAPTEUR_COULEUR_H
#define CAPTEUR_COULEUR_H

#include <Arduino.h>

bool initialiser();
void lireRGB(int &rouge, int &vert, int &bleu, int &luminosite);
bool estjaune(int rouge, int vert, int bleu);
bool estvert(int rouge, int vert, int bleu);
bool estbleu(int rouge, int vert, int bleu);
bool estrose(int rouge, int vert, int bleu);
bool estorange(int rouge, int vert, int bleu);
void detecterCouleur(bool &jaune, bool &vert, bool &bleu, bool &rose, bool &orange);

#endif
