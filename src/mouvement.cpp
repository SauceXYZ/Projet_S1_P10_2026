#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"
#include "pid.h"

#define SCAN_DEBUG 1

static int32_t distance_enc_cm_precedent = 0;

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
    distance_enc_cm = (compte - distance_enc_cm_precedent) * (23.94/3200);
    //Serial.println(distance_enc_cm, DEC);
    vert = digitalRead(vertpin);
    rouge = digitalRead(rougepin);
    //Serial.println("A");

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
                distance_enc_cm_precedent = compte;

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

                faitFace(FACE_DROIT);
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
        //Serial.println("E");
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

void faitFace(int orientation)
{
    if (orientation == direction)
    {return;}

    switch (direction)
    {
    case FACE_AVANT:
        switch (orientation)
        {
        case FACE_GAUCHE:
            pid_tourne(TOURNE_GAUCHE,90);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_GAUCHE;
            break;
        
        case FACE_DROIT:
            pid_tourne(TOURNE_DROITE,90);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_DROIT;
        
        case FACE_ARRIERE:
            pid_tourne(TOURNE_DROITE, 180);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_ARRIERE;
        }
        break;
    case FACE_DROIT:
        switch (orientation)
        {
        case FACE_GAUCHE:
            pid_tourne(TOURNE_GAUCHE,180);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_GAUCHE;
            break;
    
        case FACE_AVANT:
            pid_tourne(TOURNE_GAUCHE,90);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_DROIT;
        
        case FACE_ARRIERE:
            pid_tourne(TOURNE_DROITE, 90);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_ARRIERE;
        }
        break;
    case FACE_GAUCHE:
        switch (orientation)
        {
        case FACE_DROIT:
            pid_tourne(TOURNE_DROITE,180);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_DROIT;
            break;
    
        case FACE_AVANT:
            pid_tourne(TOURNE_DROITE,90);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_DROIT;
        
        case FACE_ARRIERE:
            pid_tourne(TOURNE_GAUCHE, 90);
            distance_enc_cm_precedent = ENCODER_Read(1);
            direction = FACE_ARRIERE;
        }
        break;
    default:
        break;
    }
}

void tourne(int directionDemande)
{
    if(direction == directionDemande)
    {return;}
    switch (directionDemande)
    {
    case TOURNE_DROITE:
        switch (direction)
        {
        case FACE_AVANT:
            faitFace(FACE_DROIT);
            break;

        case FACE_GAUCHE:
            faitFace(FACE_AVANT);
            break;
        case FACE_ARRIERE:
            faitFace(FACE_GAUCHE);
            break;
        
        }
        break;
    
    case TOURNE_GAUCHE:
        switch (direction)
        {
        case FACE_AVANT:
            faitFace(FACE_GAUCHE);
            break;

        case FACE_DROIT:
            faitFace(FACE_AVANT);
            break;
        case FACE_ARRIERE:
            faitFace(FACE_DROIT);
            break;
        }
        break;
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
    delay(1000);

    if(SCAN_DEBUG){Serial.println("Scan avant");}
    if(SCAN_DEBUG){Serial.print("LED vert: "); Serial.println(digitalRead(vertpin));}
    if(SCAN_DEBUG){Serial.print("LED rouge: "); Serial.println(digitalRead(rougepin));}
    if(!digitalRead(rougepin) || !digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("objet detecte avant");}
        scan_avant = true;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte avant");}
        scan_avant = false;
    }

    if(scan_avant == false)
    {
        if(SCAN_DEBUG){Serial.println("Choix: avancer");}
        return;
    }


    tourne(TOURNE_GAUCHE);
    if(SCAN_DEBUG){Serial.println("Scan gauche");}
    delay(1000);
    if(digitalRead(rougepin) || digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte a gauche");}
        scan_gauche = false;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("objet detecte a gauche");}
        scan_gauche = true;
    }

    tourne(TOURNE_DROITE);
    tourne(TOURNE_DROITE);
    delay(1000);
    if(SCAN_DEBUG){Serial.println("Scan droit");}
    if(digitalRead(rougepin) || digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte a droite");}
        scan_droit = false;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("objet detecte a droite");}
        scan_droit = true;
    }

    tourne(TOURNE_GAUCHE);
    delay(500);

    if(!scan_gauche)
    {
        if(SCAN_DEBUG){Serial.println("Choix: tourner gauche");}
        tourne(TOURNE_GAUCHE);
    }
    else if(!scan_droit)
    {
        if(SCAN_DEBUG){Serial.println("Choix: tourner droit");}
        tourne(TOURNE_DROITE);
    }
    else{
        if(SCAN_DEBUG){Serial.println("Default face arriere");}
        tourne(TOURNE_GAUCHE);
        tourne(TOURNE_GAUCHE);
    }
}