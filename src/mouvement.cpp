#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"

#define SCAN_DEBUG 0

int32_t encodeur_1_precedent = 0;

static void attendSerial(void)
{
    // Each pause requires new input; discard anything already buffered.
    while (Serial.available() > 0)
    {
        Serial.read();
    }

    // Consume the input and ignore the Serial Monitor's CR/LF line endings.
    int caractere;
    do
    {
        while (Serial.available() == 0) {}
        caractere = Serial.read();
    } while (caractere == '\r' || caractere == '\n');
}

int avance(void)
{
    float distance_enc_cm;
    int32_t compte=0;
    int vert;
    int rouge;
    float y_total = 500.0;
    float x_total_max = 50.0;
    float x_total_min = -50.0;
    compte = ENCODER_Read(1);  // Implementation for advancing
    distance_enc_cm = (compte - encodeur_1_precedent) * (23.94/3200);
    //Serial.println(distance_enc_cm, DEC);
    vert = digitalRead(vertpin);
    rouge = digitalRead(rougepin);
    //Serial.println("A");
    // Serial.print("DEL vert: ");
    // Serial.print(vert);
    // Serial.print(" | DEL rouge: ");
    // Serial.println(rouge);
    if (vert && rouge)
    {
        //Serial.println("B");
        if (lab_posy < y_total && lab_posx < x_total_max && lab_posx > x_total_min)
        {
            //Serial.print("C");
            pid_avance();
            if(distance_enc_cm >= 1)
            {
                if(direction == FACE_AVANT)
                {
                    lab_posy=lab_posy + distance_enc_cm;
                }
                else if(direction == FACE_ARRIERE)
                {
                    lab_posy=lab_posy - distance_enc_cm;
                }
                else if (direction == FACE_DROIT)
                {
                    lab_posx=lab_posx + distance_enc_cm;
                }
                else if (direction == FACE_GAUCHE)
                {
                    lab_posx=lab_posx - distance_enc_cm;
                }
                encodeur_1_precedent = compte;

            }

        }
        else
        {
            //Serial.println("D");
            if (FACE_GAUCHE == direction && lab_posx <= x_total_max)
            
            {
                if (lab_posx < x_total_min)
                {
                    lab_posx = x_total_min+0.1;
                }

                faitFace(FACE_AVANT);
                return 0;
            }
            
            else if (FACE_DROIT == direction && lab_posx >= x_total_max)
            {
                if (lab_posx > x_total_max)
                {
                    lab_posx = x_total_max-0.1;
                }

                faitFace(FACE_GAUCHE);
                return 0;
            }
            else if (FACE_AVANT == direction && lab_posy >= y_total)
            {
                if (lab_posy > y_total)
                {
                    lab_posy = y_total-0.1;
                }
                
                faitFace(FACE_ARRIERE);
                return 0;
            }
           
        }
    }
    else
    {
        Serial.println("E");
        MOTOR_SetSpeed(0,0);
        MOTOR_SetSpeed(1,0);
        return 1;
     
    }
return 0;
} 


// void tourner_gauche(void)
// {
//     pid_tourne(TOURNE_GAUCHE, 90);
//     if (direction == FACE_AVANT)
//     {
//         direction = FACE_GAUCHE;
//     }
//     else if (direction == FACE_GAUCHE)
//     {
//         direction = FACE_ARRIERE;
//     }
//     else if (direction == FACE_DROIT)
//     {
//         direction = FACE_AVANT;
//     }
//     else if (direction == FACE_ARRIERE)
//     {
//         direction = FACE_DROIT;
//     }
//     ENCODER_Reset(1);
//     ENCODER_Reset(0);
//     return;
// }

// void tourner_droite(void)
// {
//     pid_tourne(TOURNE_DROITE, 90);
//     MOTOR_SetSpeed(1,0);
//     if (direction == FACE_AVANT)
//     {
//         direction = FACE_DROIT;
//     }
//     else if (direction == FACE_DROIT)
//     {
//         direction = FACE_ARRIERE;
//     }
//     else if (direction == FACE_ARRIERE)
//     {
//         direction = FACE_GAUCHE;
//     }
//     else if (direction == FACE_GAUCHE)
//     {
//         direction = FACE_AVANT;
//     }
//     ENCODER_Reset(1);
//     ENCODER_Reset(0);
//     return;
// }

// Ordre des orientations pour une rotation vers la droite.
static const int orientations[] = {FACE_AVANT, FACE_DROIT, FACE_ARRIERE, FACE_GAUCHE};

static int indiceOrientation(int orientation)
{
    for (int i = 0; i < 4; ++i)
    {
        if (orientations[i] == orientation) return i;
    }
    return -1;
}

void faitFace(int orientation)
{
    const int depart = indiceOrientation(direction);
    const int arrivee = indiceOrientation(orientation);
    if (depart < 0 || arrivee < 0 || depart == arrivee) return;

    const int quartsDeTour = (arrivee - depart + 4) % 4;
    if (quartsDeTour == 3)
    {
        pid_tourne(TOURNE_GAUCHE, 90);
    }
    else
    {
        pid_tourne(TOURNE_DROITE, quartsDeTour * 90);
    }
    direction = orientation;
}

void tourne(int directionDemande)
{
    const int depart = indiceOrientation(direction);
    if (depart < 0) return;

    if (directionDemande == TOURNE_DROITE)
    {
        faitFace(orientations[(depart + 1) % 4]);
    }
    else if (directionDemande == TOURNE_GAUCHE)
    {
        faitFace(orientations[(depart + 3) % 4]);
    }
}

void scan(void)
{
    static bool scan_droit;
    static bool scan_gauche;
    static bool scan_avant;

    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    if(SCAN_DEBUG){Serial.println("Moteur stop");}
    //delay(1000);
    if(SCAN_DEBUG){
        Serial.println("Appuyer sur le bouton pour continuer le scan");
        attendSerial();
    }

    if(SCAN_DEBUG){Serial.println("Scan avant");}
    if(SCAN_DEBUG){Serial.print("LED vert: "); Serial.println(digitalRead(vertpin));}
    if(SCAN_DEBUG){Serial.print("LED rouge: "); Serial.println(digitalRead(rougepin));}
    if(!digitalRead(rougepin) || !digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("objet detecte avant");}
        scan_avant = true;

        if(SCAN_DEBUG){
            Serial.println("Objet detecte avant, appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte avant");}
        scan_avant = false;
        
        if(SCAN_DEBUG){
            Serial.println("Pas d'objet detecte avant, appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
    }

    if(scan_avant == false)
    {
        if(SCAN_DEBUG){Serial.println("Choix: avancer");}
        
        if(SCAN_DEBUG){
            Serial.println("Fin scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
        return;
    }


    tourne(TOURNE_GAUCHE);
    if(SCAN_DEBUG){
        Serial.println("Fini de tourner a gauche, appuyer sur le bouton pour continuer le scan");
        attendSerial();
    }
    if(SCAN_DEBUG){Serial.println("Scan gauche");}
    delay(1000);
    if(digitalRead(rougepin) || digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte a gauche");}
        
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_gauche = false;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("objet detecte a gauche");}
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_gauche = true;
    }

    tourne(TOURNE_DROITE);
    tourne(TOURNE_DROITE);
    delay(1000);
    if(SCAN_DEBUG){Serial.println("Scan droit");}
    if(digitalRead(rougepin) || digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte a droite");}
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_droit = false;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("objet detecte a droite");}
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_droit = true;
    }

    tourne(TOURNE_GAUCHE);
    delay(500);

    if(!scan_gauche)
    {
        if(SCAN_DEBUG){Serial.println("Choix: tourner gauche");}
        tourne(TOURNE_GAUCHE);
        
        if(SCAN_DEBUG){
            Serial.println("fin du scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
    }
    else if(!scan_droit)
    {
        if(SCAN_DEBUG){Serial.println("Choix: tourner droit");}
        tourne(TOURNE_DROITE);
        if(SCAN_DEBUG){
            Serial.println("fin du scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
    }
    else{
        if(SCAN_DEBUG){Serial.println("Default face arriere");}
        tourne(TOURNE_GAUCHE);
        tourne(TOURNE_GAUCHE);
        if(SCAN_DEBUG){
            Serial.println("fin du scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
    }
}
