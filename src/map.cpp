/*
Projet: map.c
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Mappage du labyrinthe
Date: 13/10/2026
*/

#include <LibRobus.h>

#include "stdbool.h"
#include "stdio.h"

#include "map.h"
#include "control.h"
#include "pid.h"
#include "sensor.h"

//Le robot va avoir un x et y
// Le robot va avoir un sens de déplacement 0 1 2 3 ou qqch
// binaire pour mur
//H D B G
// 1001 = 9 

//8 bloquer en haut
//4 bloquer a droite
//2 bloquer en bas
//1 bloquer a gauche
int possibilite[4] = {0, 0, 0, 0};

int grilleMap[3][10] = {
    {13, 1, 5,1, 5,1, 5, 1, 5, 7},
    {13, 0, 5,0, 5,0, 5, 0, 5, 2},
    {13, 4, 5,4, 5,4, 5, 4, 5, 6}
};

void possibiliteTape(int x, int y){
    //Serial.print(carre[1][5]&2);
    
    possibilite[0] = 0;
    possibilite[1] = 0;
    possibilite[2] = 0;
    possibilite[3] = 0;
    if((grilleMap[x][y] & 8) == 0){
        // avancer
        possibilite[0] = 1;
    }
    if((grilleMap[x][y] & 4) == 0){
        // droite
        possibilite[1] = 1;
    }
    if((grilleMap[x][y] & 2) == 0){
        // reculer
        possibilite[2] = 1;
    }
    if((grilleMap[x][y] & 1) == 0){
        // gauche
        possibilite[3] = 1;
        
    }
}



