/*
Projet: control.c
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Controle les deplacement du robot
Date: 13/10/2026
*/

#include <LibRobus.h>

#include "stdbool.h"
#include "stdio.h"

#include "map.h"
#include "control.h"
#include "pid.h"
#include "sensor.h"

//Constante Math
const float pi = 3.141593;
int etat = 0;           // 0=avancer, 1=reculer, 2=tourner, 3=stop
char direction = 'N';   // N = Nord, E = Est, S = Sud, O = Ouest
int pos_y = 0;
int pos_x = 0;

void avance(float vitessedroit, float vitessegauche){
    etat = 0;
  MOTOR_SetSpeed(RIGHT,vitessedroit);
  MOTOR_SetSpeed(LEFT, vitessegauche);
};

void recule(float vitessedroit, float vitessegauche){
    etat = 1;
  MOTOR_SetSpeed(RIGHT, -vitessedroit);
  MOTOR_SetSpeed(LEFT, -vitessegauche);
}

void tournerGauche(){
    etat = 2;
  MOTOR_SetSpeed(RIGHT, -0.5*vitesseDroite);
  MOTOR_SetSpeed(LEFT, 0.5*vitesseGauche);
}

void tournerDroit(){
    etat = 2;
  MOTOR_SetSpeed(RIGHT, 0.5*vitesseDroite);
  MOTOR_SetSpeed(LEFT, -0.5*vitesseGauche);
}

void arret(){
    etat = 3;
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
};

void tournerSelonAngle(int angle){
    arret();
    etat = 2;
    delay(300);
    ENCODER_ReadReset(RIGHT);
  float distanceParDegree = (DISTANCEENTREROUES * pi) /360;
  float distance = distanceParDegree * abs(angle);
  float pulses_requis = (distance * CLICPARTOUR) / (pi * DIAMETREROUE);
  float pulses_deplace = 0;
  while(pulses_deplace < pulses_requis){
    if(angle > 0){
        tournerDroit();
    } else {
        tournerGauche();
    }
    delay(50);
    pulses_deplace = abs(ENCODER_Read(RIGHT));
  }
  pulses_deplace = 0;
  arret();
}
void tournerSelonAngleChat(int angle) {
arret();
    delay(200);

    int angleAbs = abs(angle) * 1800;

    // Temps pour tourner l'angle demandé
    int tempsRotation = (angleAbs) / 360;

    if (angle > 0) {
        // Tourner à droite
        tournerDroit();
    } else {
        // Tourner à gauche
        tournerGauche();
    }

    delay(tempsRotation);

    arret();
}




char tournerDansDirection(char directionActuelle, char directionVoulue){
    printf("Direction actuelle : %c",directionActuelle);
    switch(directionActuelle){
        case 'N':
            switch(directionVoulue){
                case 'N':
                    return 'N';
                case 'E':
                {
                    tournerSelonAngle(-90);
                    return 'E';
                }
                case 'S':
                {
                    tournerSelonAngle(180);
                    return 'S';
                }
                case 'O':
                {
                    tournerSelonAngle(90);
                    return 'O';
                }
            }
        case 'E':
            switch(directionVoulue){
                case 'N':
                {
                    tournerSelonAngle(90);
                    return 'N';
                }
                case 'E':
                    return 'E';
                case 'S':
                {
                    tournerSelonAngle(-90);
                    return 'S';
                }
                case 'O':
                {
                    tournerSelonAngle(180);
                    return 'O';
                }
            }
        case 'S':
            switch(directionVoulue){
                case 'N':
                {
                    tournerSelonAngle(180);
                    return 'N';
                }
                case 'E':
                {
                    tournerSelonAngle(90);
                    return 'E';
                }
                case 'S':
                    return 'S';
                case 'O':
                {
                    tournerSelonAngle(-90);
                    return 'O';
                }
            }
        case 'O':
            switch(directionVoulue){
                case 'N':
                {
                    tournerSelonAngle(-90);
                    return 'N';
                }
                case 'E':
                {
                    tournerSelonAngle(180);
                    return 'E';
                }
                case 'S':
                {
                    tournerSelonAngle(90);
                    return 'S';
                }
                case 'O':
                    return 'O';
            }
    }
    return directionActuelle;
}

void avancerDistance(float distance){
    int pulseParTour = 2700;
    
    // On utilise "long" pour les pulses car ce sont de grands nombres entiers
    float pulses_requis =  ((distance * pulseParTour) / (pi * DIAMETREROUE));
    float pulses_deplace = 0;

    while(pulses_deplace < pulses_requis){
        //avancerDroitPID();
        pid(50);
        delay(50);
        pulses_deplace += ENCODER_Read(RIGHT);

    }
    
    arret();
}

void ajusterPositionRobot(char direction){
    switch(direction){
        case 'N':
            pos_y--;
            break;
        case 'E':
            pos_x++;
            break;
        case 'S':
            pos_y++;
            break;
        case 'O':
            pos_x--;
            break;
    }
}

void directionAller(){
    possibiliteTape(pos_x, pos_y);
    if(pos_y==0){
      arret();
    }
    char tabDirection[4] = {'N', 'E', 'S', 'O'};
    Serial.print("Possibilite: ");
          Serial.print(possibilite[0]);
          Serial.print(possibilite[1]);
          Serial.print(possibilite[2]);
          Serial.println(possibilite[3]);
    for(int i = 0; i < 4; i++){
        if(possibilite[i] == 1){
          
        direction = tournerDansDirection(direction, tabDirection[i]);
        Serial.print("Direction apres tournerDansDirection: ");
        Serial.println(direction);
                if(verifierMur()){
            avancerDistance(50);
            ajusterPositionRobot(direction);
            i=4; // on Sort de la boucle
        } else {
            possibilite[i] = 0;
        }
        }
        
    }
}
