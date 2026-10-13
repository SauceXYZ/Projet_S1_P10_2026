#include <LibRobus.h>
#include "map.h"

//Le robot va avoir un x et y
// Le robot va avoir un sens de déplacement 0 1 2 3 ou qqch


// binaire pour mur
//H D B G


// 1001 = 9 
//

int carre[3][6] = {
    {9, 1, 1, 1, 1, 3},
    {8, 0, 0, 0, 0, 2},
    {12, 4, 4, 4, 4, 6}
};

//8 bloquer en haut
//4 bloquer a droite
//2 bloquer en bas
//1 bloquer a gauche
int possibilite[4] = {0, 0, 0, 0};

void possibiliteTape(int x, int y){
    Serial.print(carre[1][5]&2);
    
    possibilite[0] = 0;
    possibilite[1] = 0;
    possibilite[2] = 0;
    possibilite[3] = 0;
    if((carre[x][y] & 8) == 0){
        // avancer
        possibilite[0] = 1;
    }
    if((carre[x][y] & 4) == 0){
        // droite
        possibilite[1] = 1;
    }
    if((carre[x][y] & 2) == 0){
        // reculer
        possibilite[2] = 1;
    }
    if((carre[x][y] & 1) == 0){
        // gauche
        possibilite[3] = 1;
        
    }
}

float vitesse = 0.50;
int clicParTour = 3200;

float diametreRoue = 7.62; // en cm
float distanceEntreRoues = 17.2; // en cm

void tournerGauche(){
  MOTOR_SetSpeed(RIGHT, -0.5*vitesse);
  MOTOR_SetSpeed(LEFT, 0.5*vitesse);
}

void tournerDroit(){
  MOTOR_SetSpeed(RIGHT, 0.5*vitesse);
  MOTOR_SetSpeed(LEFT, -0.5*vitesse);
}

void arret(){
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
};

void tournerSelonAngle(int angle){
    ENCODER_ReadReset(RIGHT);
  float pi = 3.1415926535;
  float distanceParDegree = (distanceEntreRoues * pi) /360;
  float distance = distanceParDegree * abs(angle);
  float pulses_requis = (distance * clicParTour) / (pi * diametreRoue);
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
    Serial.print("Direction actuelle : ");
    Serial.println(directionActuelle);
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





