#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"
#include "quille.h"

//Doit etre alimenter
//05 cm = 500-600
//15 cm = 250-300
//30 cm = 150

void detection(uint8_t capteur1) {
//Fonction qui retourne une valeur double retournant la distance approximative de l'objet
//capteur est la valeur de la pin analogique que le capteur retourne (0-1024)
//Normalement deux capteur, un en avant et un sur le coté gauche (rotation droite 90deg = 180 detection)
    bool detection = false;
    float distance = 100;
    while (detection == false) {
        distance = CalculDistance(capteur1); 
        if (distance < 30 && distance > 5) {
            detection = true;
        }
        else {
            pid_tourne(TOURNE_DROITE, 5);
        }
    }
};

static int32_t cmEnImpulsions(float cm){
    return (int32_t)(cm / CM_PAR_IMPULSION_1);
};

float CalculDistance(uint8_t capteur1){
    float distance = (4235.0 / (ROBUS_ReadIR(capteur1) - 47));   //Calcul de la distance approximative en cm
    return distance;
}

void abatDodo(int distance) {
    //Fonction qui fais avancer le robot pour faire tomber la quille et qui retourne a position ini
    //Le robot doit avoir une ligne de vue droite avec la quille
    //Distance est la distance en mm que le robot doit parcourir
    avanceImpulsions(cmEnImpulsions(distance + 5), false, false);
    delay(100);
    pid_tourne(TOURNE_DROITE, 180);
    avanceImpulsions(cmEnImpulsions(distance + 5), false, false);
    delay(100);
    pid_tourne(TOURNE_DROITE, 180);

};

void EpreuveQuille() {
    detection(3);
    float distance = CalculDistance(3);
    abatDodo(distance);
    delay(1000);
};