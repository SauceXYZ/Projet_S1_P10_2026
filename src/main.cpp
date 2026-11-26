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
#include "robusSerial.h"
#include "../config.h"
#include "cupDetection.h"
#include "Pompe.h"

/*
Variables globales et defines
 -> defines...
 -> L'ensemble des fonctions y ont acces
*/


/*
Fonctions d'initialisation (setup)
 -> Se fait appeler au debut du programme
 -> Se fait appeler seulement un fois
 -> Generalement on y initilise les varibbles globales
*/
void setup(){
  BoardInit();
  Serial.begin(BAUD);
  serialInit3_3V(BAUD);
  init_pompe();
  delay(100);
  Serial.println("Initialisation complete");
}

/*
Fonctions de boucle infini
 -> Se fait appeler perpetuellement suite au "setup"
*/
void loop() 
{
  int rx = Serial1RX();
  if(rx >= 0)
  {
    Serial.println(rx);
    goToCup(rx);
  }
  else if(rx == -1)
  {
    Serial.println("STOP");
    stopCup();
  }

  if (ROBUS_IsBumper(LEFT)) 
    {
      stopCup();
       actionner_pompe_Gauche();
     }
 
    if (ROBUS_IsBumper(RIGHT)) 
    {
      stopCup();
       actionner_pompe_Droite();
     }

  
}
