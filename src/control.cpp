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
int anciennePosition[2] = {1,5};

void avance(float vitessedroit, float vitessegauche){
  MOTOR_SetSpeed(RIGHT,vitessedroit);
  MOTOR_SetSpeed(LEFT, vitessegauche);
};

void recule(float vitessedroit, float vitessegauche){
  MOTOR_SetSpeed(RIGHT, -vitessedroit);
  MOTOR_SetSpeed(LEFT, -vitessegauche);
}

void tournerGauche(){
  MOTOR_SetSpeed(RIGHT, -0.5*vitesseDroite);
  MOTOR_SetSpeed(LEFT, 0.5*vitesseGauche);
}

void tournerDroit(){
  MOTOR_SetSpeed(RIGHT, 0.5*vitesseDroite);
  MOTOR_SetSpeed(LEFT, -0.5*vitesseGauche);
}

void arret(){
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
};

void tournerSelonAngle(int angle){
    arret();
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
    const int pulseParTour = 3000;
    const float pulses_requis = (distance * pulseParTour) / (pi * DIAMETREROUE);
    float pulses_deplace = 0.0f;

    ENCODER_Reset(1);
    ENCODER_Reset(LEFT);

    while (pulses_deplace < pulses_requis) {
        Serial.print("Pulses requis: ");
        Serial.print(pulses_requis);
        Serial.print(" | Pulses deplace: ");
        Serial.println(pulses_deplace);
        pid(50);
        pulses_deplace += ENCODER_Read(1);
    }

    arret();
}

void ajusterPositionRobot(char direction){
    anciennePosition[0] = pos_x;
    anciennePosition[1] = pos_y;
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
    Serial.print("Position actuelle : (");
    Serial.print(pos_x);
    possibiliteTape(pos_x, pos_y);
    enleverAncienneCase(pos_x, pos_y, anciennePosition[0], anciennePosition[1]);
    if(possibilite[0] == 0 && possibilite[1] == 0 && possibilite[2] == 0 && possibilite[3] == 0){
        remettreAncienneCase(pos_x, pos_y, anciennePosition[0], anciennePosition[1]);
    } 
    if(pos_y==0){
      arret();
    } else {
        char tabDirection[4] = {'N', 'E', 'S', 'O'};
        for(int i = 0; i < 4; i++){
            if(possibilite[i] == 1){
          
                direction = tournerDansDirection(direction, tabDirection[i]);
                if(!verifierMur()){
                    avancerDistance(50);
                    ajusterPositionRobot(direction);
                    i=4; // on Sort de la boucle
                } else {
                    possibilite[i] = 0;
                }
            }
        
        }
    }
    
}