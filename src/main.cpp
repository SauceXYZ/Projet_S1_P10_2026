/*
Projet: main.c
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Script d'execution du parcours pour le robot
Date: 08/10/2026
*/

#include "LibRobus.h"

#include "stdbool.h"
#include "stdio.h"

#include "map.h"
#include "control.h"
#include "pid.h"
#include "sensor.h"

//Fct Avance dans control.cpp
//Fct Recule dans control.cpp
//Fct avancerDistance dans control.cpp
//Fct ajusterPositionRobot dans control.cpp
//Fct directionAller dans control.cpp

//Fct pid fans pid.cpp

void setup(){
  BoardInit();
  
  //initialisation
  pinMode(vertPin, INPUT);
  pinMode(rougePin, INPUT);
  pinMode(siffletPin, INPUT);
  delay(100);
  beep(3);
}

/*
Fonctions de boucle infini
 -> Se fait appeler perpetuellement suite au "setup"
*/
void loop() {
  bumperArr = ROBUS_IsBumper(3);
  bumperAva = ROBUS_IsBumper(2);
  bumperGch = ROBUS_IsBumper(1);
  bumperDrt = ROBUS_IsBumper(0);

  if (bumperAva){
    //tournerDansDirection();
  }
  if (bumperArr){
    //tournerSelonAngle(180);
    if(etat == 0){
      etat = 1;
    } else {
      etat = 0;
    }
  } 
  if(bumperDrt){
    tournerSelonAngle(-90);
  }
  if(bumperGch){
    tournerSelonAngle(90);
  } 
  if(etat == 1){
    directionAller();
  } else {
    arret();
  }
}


