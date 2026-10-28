#include <libRobus.h>
#include "main.h"

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
    vitesseSuiveurDroit += ajoutCorrection;
    nbAugmentation++;
    
    if (vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);
    Serial.println("Fait une correction vers la gauche");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
}

void correctionVersDroite()
{
    etatPrecedent = 2;
    vitesseSuiveurGauche += ajoutCorrection;
    nbAugmentation++;
    
    if (vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    Serial.println("Fait une correction vers la droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
}

void tournerGauche()
{
    vitesseSuiveurDroit += ajoutCorrection;
    nbAugmentation++;
    
    if (vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    
    // On force aussi le moteur gauche à ralentir pour pivoter de force
    MOTOR_SetSpeed(0, vitesseSuiveurGauche * 0.5f); 
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);
    etatPrecedent = 1;
    Serial.println("Tourne à gauche");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void tournerDroite()
{
    vitesseSuiveurGauche += ajoutCorrection;
    nbAugmentation++;
    
    if (vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesseSuiveurDroit * 0.5f);
    etatPrecedent = 2;
    Serial.println("Tourne à droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void suivreLaLigne()
{
    
    bool milieu = digitalRead(pinSuiveurMilieu);
    bool gauche = digitalRead(pinSuiveurGauche);
    bool droite = digitalRead(pinSuiveurDroit);

    if (milieu && !gauche && !droite) {
        if (etatPrecedent == 1)
        {
            vitesseSuiveurDroit -= ajoutCorrection * nbAugmentation;
            vitesseSuiveurDroit += ajoutLongTerme;
            vitesseSuiveurGauche -= ajoutLongTerme;
        }
        else if (etatPrecedent == 2)
        {
            vitesseSuiveurGauche -= ajoutCorrection * nbAugmentation;
            vitesseSuiveurGauche += ajoutLongTerme;
            vitesseSuiveurDroit -= ajoutLongTerme;
        }
        
        if (vitesseSuiveurDroit > 0.65f)  vitesseSuiveurDroit = 0.65f;
        if (vitesseSuiveurDroit < 0.35f)  vitesseSuiveurDroit = 0.35f;
        if (vitesseSuiveurGauche > 0.65f) vitesseSuiveurGauche = 0.65f;
        if (vitesseSuiveurGauche < 0.35f) vitesseSuiveurGauche = 0.35f;

        nbAugmentation = 0;
        avancerDroit();
    }
    else if (milieu && gauche) {
        correctionVersGauche();
    }
    else if (milieu && droite) {
        correctionVersDroite();
    }
    else if (!milieu && gauche && !droite) {
        tournerGauche();
    }
    else if (!milieu && !gauche && droite) {
        tournerDroite();
    }

}