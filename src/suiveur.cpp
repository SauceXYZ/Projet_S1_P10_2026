#include <libRobus.h>
#include "main.h"
#include "pid.h"

#define SEUIL_CAPTEUR 650
#define VITESSE_PID 0.2

const float vitesse_initiale = 0.3f;
float vitesseSuiveurGauche = 0.1f;
float vitesseSuiveurDroit = 0.1f;
float ajoutCorrection = 0.02f;
float correctionTournant = 0.2f;
int etatPrecedent = 0; //0=milieu, 1=gauche, 2=droite
int nbAugmentation = 0;


void avancerDroit()
{
    etatPrecedent = 0;
    MOTOR_SetSpeed(0, vitesse_initiale);//vitesseSuiveurGauche);   
    MOTOR_SetSpeed(1, vitesse_initiale);
    Serial.println("Avance en ligne droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);  
}

void arreter()
{
    etatPrecedent = 0;
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    Serial.println("Arrete le robot");
}

void correctionVersGauche()
{
    etatPrecedent = 1;
    //nbAugmentation++;
    vitesseSuiveurDroit += ajoutCorrection;
    if (vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche); 
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Fait une correction vers la gauche");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
}

void correctionVersDroite()
{
    etatPrecedent = 2;
    //nbAugmentation++;
    vitesseSuiveurGauche += ajoutCorrection;
    if(vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Fait une correction vers la droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
}

void tournerGauche()
{
    etatPrecedent = 1;
    //nbAugmentation++;
    vitesseSuiveurDroit += correctionTournant;
    vitesseSuiveurGauche -= correctionTournant;
    if(vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    if(vitesseSuiveurGauche < 0.0f) vitesseSuiveurGauche = 0.0f;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche); 
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Tourne à gauche");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void tournerDroite()
{
    etatPrecedent = 2;
    //nbAugmentation++;
    vitesseSuiveurGauche += correctionTournant;
    vitesseSuiveurDroit -= correctionTournant;
    if(vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    if(vitesseSuiveurDroit < 0.0f) vitesseSuiveurDroit = 0.0f;  
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Tourne à droite");
    Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void suivreLaLigne()
{
    bool milieu = analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR;
    bool gauche = analogRead(pinSuiveurGauche)>SEUIL_CAPTEUR;
    bool droite = analogRead(pinSuiveurDroit)>SEUIL_CAPTEUR;

    float testmilieu = analogRead(pinSuiveurMilieu);
    float testgauche = analogRead(pinSuiveurGauche);
    float testdroite = analogRead(pinSuiveurDroit);

    Serial.print("Milieu:");
    Serial.println(milieu);
    Serial.print("Gauche:");
    Serial.println(gauche);
    Serial.print("Droit:");
    Serial.println(droite);
/*
    Serial.print("Test milieu:");
    Serial.println(testmilieu);
    Serial.print("Test gauche:");
    Serial.println(testgauche);
    Serial.print("Test droite:");
    Serial.println(testdroite);
*/
    //delay(250);

    //return ;
    if(vitesseSuiveurDroit>0.5) vitesseSuiveurDroit = 0.5;
    if(vitesseSuiveurDroit<0.1) vitesseSuiveurDroit = 0.1;
    if(vitesseSuiveurGauche>0.5) vitesseSuiveurGauche = 0.5;
    if(vitesseSuiveurGauche<0.1) vitesseSuiveurGauche = 0.1;

    // Fait accélérer le robot pour ne pas qu'il reste trop bas
    //if(vitesseSuiveurGauche < 0.3 && vitesseSuiveurDroit < 0.3)
    //{
    //    vitesseSuiveurGauche *= 1.15;
    //    vitesseSuiveurDroit *= 1.15;
    //}
    

    if(milieu && !gauche && !droite){
        if(etatPrecedent == 1) {
            vitesseSuiveurDroit = vitesse_initiale;//-= ajoutCorrection * nbAugmentation;
        }
        else if(etatPrecedent == 2) {
            vitesseSuiveurGauche = vitesse_initiale;//-= ajoutCorrection //* nbAugmentation;
        }

        nbAugmentation = 0;
        avancerDroit();
    }
    else if(milieu && gauche && !droite){
        correctionVersDroite();
    }
    else if(milieu && droite && !gauche){
        correctionVersGauche();
    }
    else if(!milieu && gauche && !droite){
        tournerDroite();
    }
    else if(!milieu && !gauche && droite){
        tournerGauche();
        
    }
    else if(!milieu && !gauche && !droite){
        //arreter();
    }
    else if(milieu && gauche && droite){
        while(analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR && analogRead(pinSuiveurDroit)>SEUIL_CAPTEUR 
        && analogRead(pinSuiveurGauche)>SEUIL_CAPTEUR)
        {
            MOTOR_SetSpeed(1,vitesse_initiale);
            MOTOR_SetSpeed(0,vitesse_initiale);
        }
        bool atteindfin = false;
        while(!atteindfin)
        {
            /*
            if(!analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR && !analogRead(pinSuiveurDroit)<SEUIL_CAPTEUR 
            && !analogRead(pinSuiveurGauche)<SEUIL_CAPTEUR)
            {
                delay(50);
                if(!analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR && !analogRead(pinSuiveurDroit)<SEUIL_CAPTEUR 
                && !analogRead(pinSuiveurGauche)<SEUIL_CAPTEUR)
                {
                    atteindfin = true;
                }

            }
                */
            if(analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR && analogRead(pinSuiveurDroit)>SEUIL_CAPTEUR 
            && analogRead(pinSuiveurGauche)>SEUIL_CAPTEUR)
            {
                atteindfin = true;
            }
            pid_avance(VITESSE_PID);
        }
        while(analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR && analogRead(pinSuiveurDroit)>SEUIL_CAPTEUR 
        && analogRead(pinSuiveurGauche)>SEUIL_CAPTEUR)
        {
            MOTOR_SetSpeed(1,vitesse_initiale);
            MOTOR_SetSpeed(0,vitesse_initiale);
        }

    }
}

void testSuiveur()
{
    int milieu = analogRead(pinSuiveurMilieu);
    int gauche = analogRead(pinSuiveurGauche);
    int droite = analogRead(pinSuiveurDroit);

    Serial.print("Milieu:");
    Serial.println(milieu);
    Serial.print("Gauche:");
    Serial.println(gauche);
    Serial.print("Droit:");
    Serial.println(droite);
    
    delay(50);
}
