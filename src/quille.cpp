#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h
#include "quille.h"

int detection(int capteur){
//Fonction qui retourne une valeur double retournant la distance approximative de l'objet
//capteur est la valeur de la pin analogique que le capteur retourne (0-1024)
//Normalement deux capteur, un en avant et un sur le coté gauche (rotation droite 90deg = 180 detection)

int detection = 0;       //Permet de valider si le capteur a detecter la quille
int distance = 0;       //(cm)Valeur approximative de la distance qui sépare le capteur de la quille
int valMaxCapt = 0;     //Valeur en cm ou le capteur donne 5v (1024 analog)


while (detection != 1){
    tourne(TOURNE_DROITE);
    if(capteur > (1024/4)){         //A calibrer
        distance = (1024/valMaxCapt) * capteur;
        //Quand detecte objet face capteur, arrete les moteur pour etre aligné
        MOTOR_SetSpeed(0, 0);
        MOTOR_SetSpeed(1, 0);
        detection = 1;
    }
    delay(1000)
    return distance;
};

static int32_t cmEnImpulsions(float cm){
    return (int32_t)(distance / CM_PAR_IMPULSION_1);
}

void abatDodo(int distance) {
//Fonction qui fais avancer le robot pour faire tomber la quille et qui retourne a position ini
//Distance est la distance en mm que le robot doit parcourir

int distanceAParc = distance + 5;  //Afin qu'il rentre bien dans la quille
    //Avance dans quille
    avanceImpulsions(cmEnImpulsions(distanceAParc), false, false);
    delay(50);
    //180 deg
    tourne(TOURNE_DROITE);
    delay(100);
    tourne(TOURNE_DROITE);
    delay(100);
    //Retourne pos initial
    avanceImpulsions(cmEnImpulsions(distanceAParc), false, false);
    delay(50);
    //180 deg
    tourne(TOURNE_DROITE);
    delay(100);
    tourne(TOURNE_DROITE);
    delay(100);
}