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

#include "main.h"
#include "map.h"
#include "control.h"
#include "pid.h"

//Initialisation des bumpers
bool bumperArr=false;
bool bumperAva=false;
bool bumperGch=false;
bool bumperDrt=false;

//Initialisation des capteurs
bool vert = false;
bool rouge = false;
bool sifflet = false;


//Fct Avance dans control.cpp
//Fct Recule dans control.cpp
//Fct avancerDistance dans control.cpp
//Fct ajusterPositionRobot dans control.cpp
//Fct directionAller dans control.cpp

//Fct pid fans pid.cpp


void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}

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
    tournerSelonAngle(-180);
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


