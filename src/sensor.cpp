/*
Projet: sensor.c
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Pour tout les sensor du micro-controlleur
Date: 13/10/2026
*/

#include <LibRobus.h>

#include "stdbool.h"
#include "stdio.h"

#include "map.h"
#include "control.h"
#include "pid.h"
#include "sensor.h"

//Initialisation des bumpers
bool bumperArr=false;
bool bumperAva=false;
bool bumperGch=false;
bool bumperDrt=false;

//Initialisation des capteurs
bool proxVert = false;
bool proxRouge = false;
bool sifflet = false;


void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}

bool verifierMur(){
    proxVert = digitalRead(vertPin);
    proxRouge= digitalRead(rougePin);
    if(!proxVert && !proxRouge){
        return true;
    } else {
        return false;
    }

}

