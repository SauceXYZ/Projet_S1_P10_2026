/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/*
Inclure les librairies de functions que vous voulez utiliser
*/
#include <LibRobus.h>
#include "mouvement.h"
#include "main.h"
#include "sifflet.cpp"

/*
Variables globales et defines
 -> defines...
 -> L'ensemble des fonctions y ont acces
*/

int vertpin = 48;
int rougepin = 49;
int pinsifflet = 47;

float lab_posx = 0;
float lab_posy = 0;

int direction = FACE_AVANT;

/*
Vos propres fonctions sont creees ici
*/

/*
Fonctions d'initialisation (setup)
 -> Se fait appeler au debut du programme
 -> Se fait appeler seulement un fois
 -> Generalement on y initilise les varibbles globales
*/
void setup(){
  BoardInit();
  
  //initialisation
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  ENCODER_Reset(1);
  ENCODER_Reset(0);
  Serial.println("Initialisation complete\n");
  attendSifflet();
  delay(100);
}

/*
Fonctions de boucle infini
 -> Se fait appeler perpetuellement suite au "setup"
*/
void loop()
{
  avance();
if (avance() == 1)
{ 
  scan(); 
}

}
