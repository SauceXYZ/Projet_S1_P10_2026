#include <LibRobus.h>
#include "main.h"
#include "pid.h"

#define SEUIL_CAPTEUR 300
#define VITESSE_PID 0.2
#define MILIEU 0
#define GAUCHE 1
#define DROITE 2
#define MIN_DELTA_T_DETECTION 50
#define CORRECTION_POSITIVE 0.02
#define CORRECTION_NEGATIVE -0.02
#define TEMPS_CONFIRMATION 75

const float vitesse_initiale = 0.3f;
float vitesseSuiveurGauche = 0.1f;
float vitesseSuiveurDroit = 0.1f;
float ajoutCorrection = 0.02f;
float correctionTournant = 0.02f;
int etatPrecedent = 0; //0=milieu, 1=gauche, 2=droite
int nbAugmentation = 0;
unsigned long tempsSuiveur[] = {0,0,0};
bool capteurs[3] = {false,false,false};
bool passerTrou = false;


void waitInput()
{
    while(!Serial.read() == '\n')
    {
        delay(10);
    }
}

void clampVitesse()
{
    if(vitesseSuiveurDroit>1.0f) vitesseSuiveurDroit = 1.0f;
    if(vitesseSuiveurDroit<0.0f) vitesseSuiveurDroit = 0.0f;
    if(vitesseSuiveurGauche>1.0f) vitesseSuiveurGauche = 1.0f;
    if(vitesseSuiveurGauche<0.0f) vitesseSuiveurGauche = 0.0f;
}

void verificationLigne()
{
    bool resultat[] = {
        analogRead(pinSuiveurMilieu)>SEUIL_CAPTEUR,
        analogRead(pinSuiveurGauche)>SEUIL_CAPTEUR,
        analogRead(pinSuiveurDroit)>SEUIL_CAPTEUR
    };

    // Serial.print("CApteurs");
    // Serial.println(analogRead(pinSuiveurMilieu));
    // Serial.println(analogRead(pinSuiveurGauche));
    // Serial.println(analogRead(pinSuiveurDroit));
    static bool ancienResultat[3];
    /*Serial.print("temps:");
    Serial.println(tempsSuiveur[0]);
    Serial.println(tempsSuiveur[1]);
    Serial.println(tempsSuiveur[2]);*/

    for(int i=0; i<3; i++)
    {
        if(resultat[i] == ancienResultat[i])
        {
            if(millis() - tempsSuiveur[i] > TEMPS_CONFIRMATION)
            {
                ancienResultat[i] = resultat[i];
                capteurs[i] = ancienResultat[i];
            }
        }
        else
        {
            tempsSuiveur[i] = millis();
            ancienResultat[i] = resultat[i];
        }
    }

    // Serial.print("Capteurs: ");
    // Serial.print(ancienResultat[0]);
    // Serial.print(" ");
    // Serial.print(ancienResultat[1]);
    // Serial.print(" ");
    // Serial.println(ancienResultat[2]);

    
}


void avancerDroit()
{
    etatPrecedent = 0;
    vitesseSuiveurDroit = vitesse_initiale;
    vitesseSuiveurGauche = vitesse_initiale;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);
    Serial.println("Avance en ligne droite");
    // Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    // Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);  
}

void arreter()
{
    pid_arret();
}


void placerPourPIB()
{  
    unsigned long tempsDetection[3] = {0,0,0};
    bool capteursFoundLine[3] = {false,false,false};
    bool done = false;
    float correctionDroite = 0.0;
    float correctionGauche = 0.0;
    while(!done)
    {
        while(capteurs[GAUCHE] || capteurs[DROITE])
        {
            MOTOR_SetSpeed(0, 0.1f+correctionGauche);
            MOTOR_SetSpeed(1, 0.1f+correctionDroite);
            verificationLigne();
        }
        pid_arret();
        unsigned long tempsDebut = millis();
        while(!capteursFoundLine[GAUCHE] || !capteursFoundLine[DROITE])
        {
            MOTOR_SetSpeed(0, -0.1f);
            MOTOR_SetSpeed(1, -0.1f);
            verificationLigne();
            for(int i=1; i<3; i++)
            {
                if(capteurs[i])
                {
                    if(!capteursFoundLine[i])
                    {
                        capteursFoundLine[i] = true;
                        tempsDetection[i] = millis() - tempsDebut;
                    }
                }
            }
        }

        pid_arret();
        for(int i =1; i<3; i++)
        {
            Serial.print("Temps detection capteur ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(tempsDetection[i]);
        }
        //check biggest delta t between detection times
        float maxDeltaT = 0;
        for(int i=1; i<3; i++)
        {
            for(int j=i+1; j<3; j++)
            {
                float deltaT = abs(tempsDetection[i] - tempsDetection[j]);
                if(deltaT > maxDeltaT)
                {
                    maxDeltaT = deltaT;
                }
            }
        }
        Serial.print("Max delta t: ");
        Serial.println(maxDeltaT);
        if(maxDeltaT < MIN_DELTA_T_DETECTION)
        {
            while(capteurs[GAUCHE] || capteurs[DROITE])
            {
                MOTOR_SetSpeed(0, 0.1f);
                MOTOR_SetSpeed(1, 0.1f);
                verificationLigne();
            }
            pid_arret();
            done = true;
        }
        else
        {
            if(tempsDetection[GAUCHE] > tempsDetection[DROITE])
            {
                correctionGauche = CORRECTION_POSITIVE;
                correctionDroite = CORRECTION_NEGATIVE;
            }
            else
            {
                correctionDroite = CORRECTION_POSITIVE;
                correctionGauche = CORRECTION_NEGATIVE;
            }
            for(int i=1; i<3; i++)
            {
                capteursFoundLine[i] = false;
                tempsDetection[i] = 0;
            }
        }

    }
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
    //Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    //Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
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
    //Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    //Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit); 
}

void tournerGauche()
{
    etatPrecedent = 1;
    //nbAugmentation++;
    vitesseSuiveurDroit += correctionTournant;
    vitesseSuiveurGauche -= correctionTournant*2.5;
    if(vitesseSuiveurDroit > 1.0f) vitesseSuiveurDroit = 1.0f;
    if(vitesseSuiveurGauche < 0.0f) vitesseSuiveurGauche = 0.0f;
    MOTOR_SetSpeed(0, vitesseSuiveurGauche); 
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Tourne à gauche");
    //Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    //Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void tournerDroite()
{
    etatPrecedent = 2;
    //nbAugmentation++;
    vitesseSuiveurGauche += correctionTournant;
    vitesseSuiveurDroit -= correctionTournant*2.5;
    if(vitesseSuiveurGauche > 1.0f) vitesseSuiveurGauche = 1.0f;
    if(vitesseSuiveurDroit < 0.0f) vitesseSuiveurDroit = 0.0f;  
    MOTOR_SetSpeed(0, vitesseSuiveurGauche);
    MOTOR_SetSpeed(1, vitesseSuiveurDroit);

    Serial.println("Tourne à droite");
    //Serial.print("Vitesse gauche: "); Serial.println(vitesseSuiveurGauche);
    //Serial.print("Vitesse droite: "); Serial.println(vitesseSuiveurDroit);
}

void suivreLaLigne()
{
    verificationLigne();

    // Serial.print("Capteurs: ");
    // Serial.print(capteurs[MILIEU]);
    // Serial.print(" ");
    // Serial.print(capteurs[GAUCHE]);
    // Serial.print(" ");
    // Serial.println(capteurs[DROITE]);

    float testmilieu = analogRead(pinSuiveurMilieu);
    float testgauche = analogRead(pinSuiveurGauche);
    float testdroite = analogRead(pinSuiveurDroit);
/*
    Serial.print("Test milieu:");
    Serial.println(testmilieu);
    Serial.print("Test gauche:");
    Serial.println(testgauche);
    Serial.print("Test droite:");
    Serial.println(testdroite);
*/

    //return ;
    
    clampVitesse();
    if((capteurs[MILIEU] && !capteurs[GAUCHE] && !capteurs[DROITE]) || (etatPrecedent == 0 && !capteurs[GAUCHE]&& !capteurs[DROITE]&& !capteurs[MILIEU])){
        //Serial.println("Avance en ligne droite");
        if(etatPrecedent == 1) {
            vitesseSuiveurDroit = vitesse_initiale;//-= ajoutCorrection * nbAugmentation;
        }
        else if(etatPrecedent == 2) {
            vitesseSuiveurGauche = vitesse_initiale;//-= ajoutCorrection //* nbAugmentation;
        }

        nbAugmentation = 0;
        clampVitesse();
        avancerDroit();
    }
    // else if(capteurs[MILIEU] && capteurs[GAUCHE] && !capteurs[DROITE]){
    //     Serial.println("Correction vers la gauche");
    //     correctionVersDroite();
    // }
    // else if(capteurs[MILIEU] && capteurs[GAUCHE] && !capteurs[DROITE]){
    //     Serial.println("Correction vers la droite");
    //     correctionVersGauche();
    //}
    else if(!capteurs[MILIEU] && capteurs[GAUCHE] && !capteurs[DROITE]){
        Serial.println("Tourne à gauche");
        tournerDroite();
    }
    else if(!capteurs[MILIEU] && !capteurs[GAUCHE] && capteurs[DROITE]){
        Serial.println("Tourne à droite");
        tournerGauche();
        
    }
    else if(!capteurs[MILIEU] && !capteurs[GAUCHE] && !capteurs[DROITE]){
        arreter();
    }
    else if(capteurs[MILIEU] && capteurs[GAUCHE] && capteurs[DROITE]){
        placerPourPIB();

        bool atteindfin = false;
        while(!atteindfin)
        {
            if(capteurs[MILIEU] && (capteurs[DROITE] || capteurs[GAUCHE]))
            {
                atteindfin = true;
            }
            pid_avance(VITESSE_PID);
            verificationLigne();
        }
        while(capteurs[MILIEU] && capteurs[GAUCHE] && capteurs[DROITE] )
        {
            pid_avance(vitesse_initiale);
            verificationLigne();
            
        }
        passerTrou = true;
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

void trou()
{
    while(!passerTrou)
    {
        suivreLaLigne();
    }
}
