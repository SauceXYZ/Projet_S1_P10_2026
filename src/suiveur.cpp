#include <libRobus.h>
#include "main.h"

const float vitesse_initiale = 0.5f;
float vitesseSuiveurGauche = 0.1f;
float vitesseSuiveurDroit = 0.1f;
float ajoutCorrection = 0.1f; 
float ajoutLongTerme = 0.03f; 
int etatPrecedent = 0; //0=milieu, 1=gauche, 2=droite
int nbAugmentation = 0;


void avancerDroit()
{
    etatPrecedent = 0;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);   
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);
    Serial.println("Avance en ligne droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);  
}

void correctionVersGauche()
{
    etatPrecedent = 1;
    nbAugmentation++;
    vitesseSuiveurDroit += ajoutCorrection;
    if (vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    MOTOR_SetSpeed(0, vitesse_initiale); 
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Fait une correction vers la gauche");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
}

void correctionVersDroite()
{
    etatPrecedent = 2;
    nbAugmentation++;
    vitesseSuiveurGauche += ajoutCorrection;
    if(vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesse_initiale);

    Serial.println("Fait une correction vers la droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
}

void tournerGauche()
{
    etatPrecedent = 1;
    nbAugmentation++;
    vitesseSuiveurDroit += ajoutCorrection * 1.5f;
    if(vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    MOTOR_SetSpeed(0, vitesse_initiale * 0.2f); 
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Tourne à gauche");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void tournerDroite()
{
    etatPrecedent = 2;
    nbAugmentation++;
    vitesseSuiveurGauche += ajoutCorrection * 1.5f;
    if(vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesse_initiale * 0.2f);

    Serial.println("Tourne à droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void suivreLaLigne()
{
    bool milieu = digitalRead(pinSuiveurMilieu);
    bool gauche = digitalRead(pinSuiveurGauche);
    bool droite = digitalRead(pinSuiveurDroit);

    if(milieu && !gauche && !droite){
        if(etatPrecedent == 1) {
            vitesseSuiveurDroit -= ajoutCorrection * nbAugmentation;
        }
        else if(etatPrecedent == 2) {
            vitesseSuiveurGauche -= ajoutCorrection * nbAugmentation;
        }
        
        // Sécurité pour ne pas descendre sous la vitesse de base
        if(vitesseSuiveurDroit < vitesse_initiale) vitesseSuiveurDroit = vitesse_initiale;
        if(vitesseSuiveurGauche < vitesse_initiale) vitesseSuiveurGauche = vitesse_initiale;

        nbAugmentation = 0;
        avancerDroit();
    }
    else if(milieu && gauche){
        correctionVersGauche();
    }
    else if(milieu && droite){
        correctionVersDroite();
    }
    else if(!milieu && gauche && !droite){
        tournerGauche();
    }
    else if(!milieu && !gauche && droite){
        tournerDroite();
    }
}