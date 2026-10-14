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

void setup(){
  BoardInit();
  Serial.begin(9600);
  
  //initialisation des SENSORS
  pinMode(vertPin, INPUT);
  pinMode(rougePin, INPUT);
  pinMode(siffletPin, INPUT);

  pos_x = 1;
  pos_y = 5;

  //Fin de l'initialisation
  delay(100);
  beep(3);
}



void loop() {
  //Assigner les valeurs au BUMPERS
  bumperArr = ROBUS_IsBumper(3);
  bumperAva = ROBUS_IsBumper(2);
  bumperDrt = ROBUS_IsBumper(1);
  bumperGch = ROBUS_IsBumper(0);
  if(digitalRead(siffletPin)){
    etat = 1;
  }
  if(bumperArr){
    Serial.println("Etat: Reculer");
    etat = 2;
  }

  if(etat == 1){
    tournerDroit();
  } else if(etat == 2){
    directionAller();
  }
  
  //Si (verifieMur && Direction = NORD && LastDirection != EST)
    //Verifie limite
      //Sinon GO OUEST (Break)
    //Tourne EST
    //LastDirection == EST
    //Avance 50cm
    //Redirection NORD

  //Sinon si (verifieMur && Direction = NORD && LastDirection != OUEST)
    //Verifie limite
      //Sinon GO EST
    //Tourne OUEST
    //LastDirection == OUEST
    //Avance 50cm
    //Redirection NORD
  
  //Sinon
    //Avance 50cm
    //LastDirection == NORD
  delay(50);
}

