#include <libRobus.h>
#include "main.h"

float vitesseSuiveurGauche = 0.5f;
float vitesseSuiveurDroit = 0.5f;
float ajoutCorrection = 0.1f; //Valeur pour faire retourner le robot vers la ligne
float ajoutLongTerme = 0.03f; //Valeur pour que le robot suive la ligne sur le long terme
int etatPrecedent = 0; //0=milieu, 1=gauche, 2=droite
int nbAugmentation = 0;

void suivreLaLigne()
{
    bool milieu = digitalRead(pinSuiveurMilieu);
    bool gauche = digitalRead(pinSuiveurGauche);
    bool droite = digitalRead(pinSuiveurDroit);

    

    if(milieu && !gauche && !droite){
        if(etatPrecedent == 1)
        {
            vitesseSuiveurDroit -= ajoutCorrection * nbAugmentation;
            vitesseSuiveurDroit += ajoutLongTerme;
        }
        else if(etatPrecedent == 2)
        {
            vitesseSuiveurGauche -= ajoutCorrection * nbAugmentation;
            vitesseSuiveurGauche += ajoutLongTerme;
        }
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

void avancerDroit()
{
    etatPrecedent = 0;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);   
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);    
}

void correctionVersGauche()
{
    etatPrecedent = 1;
    vitesseSuiveurDroit += ajoutCorrection;
    nbAugmentation++;
}

void correctionVersDroite()
{
    etatPrecedent = 2;
    vitesseSuiveurGauche += ajoutCorrection;
    nbAugmentation++;
}


void tournerGauche()
{
    //Si on tournait, on tourne encore plus
    if(etatPrecedent == 1)
    {
        vitesseSuiveurDroit += ajoutCorrection;
        nbAugmentation++;
        MOTOR_SetSpeed(1, vitesseSuiveurDroit);
    }
}

void tournerDroite()
{
    if(etatPrecedent == 2)
    {
        vitesseSuiveurGauche += ajoutCorrection;
        nbAugmentation ++;
        MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    }
}