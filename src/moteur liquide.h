// distributeur.h
#ifndef MOTEUR_LIQUIDE_H
#define MOTEUR_LIQUIDE_H
 
#include <Arduino.h>
 
// Prépare les broches des pompes (à appeler une fois dans setup())
void moteur_liquide_init();
 
// Sert un volume en ml avec la pompe 0 ou 1.
// Retourne true si servi, false si refusé (pompe/volume invalide ou réservoir insuffisant).
// C'est cette fonction que l'interface appellera.
bool servir_si_possible(int index, float volume_ml);
 
// À appeler après avoir rempli le réservoir "index" (remet le niveau à 125 ml)
void remplir_reservoir(int index);
 
// Niveau estimé (ml) du réservoir "index"
float niveau_reservoir(int index);
 
#endif
 