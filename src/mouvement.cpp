#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"

void lock(void)
{
    while(1)
    {delay(100);}
}

int avance(void)
{
    float distance_enc_cm;
    int compte=0;
    int vert;
    int rouge;
    float y_total = 500.0;
    float x_total_max = 50.0;
    float x_total_min = -50.0;
    compte = ENCODER_Read(1);  // Implementation for advancing
    distance_enc_cm = (compte * (23.94/3200));
    //Serial.println(distance_enc_cm, DEC);
    vert = digitalRead(vertpin);
    rouge = digitalRead(rougepin);
    

    if (vert && rouge)
    {
        if (lab_posy < y_total && lab_posx < x_total_max && lab_posx > x_total_min)
        {
            MOTOR_SetSpeed(0,0.48);
            MOTOR_SetSpeed(1,0.5);
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
                ENCODER_Reset(1);
                ENCODER_Reset(0);
            }
        }
        else
        {
            // Serial.print("lab_posy: ");
            // Serial.print(lab_posy, DEC);
            // Serial.print(" distance_enc_cm: ");
            // Serial.print(distance_enc_cm, DEC);
            // Serial.print("\r");
            MOTOR_SetSpeed(0,0);
            MOTOR_SetSpeed(1,0);
        }
    }
    else
    {
        MOTOR_SetSpeed(0,0);
        MOTOR_SetSpeed(1,0);
        return 1;
     
    }
return 0;
} 

void tourner_gauche(void)
{
    MOTOR_SetSpeed(0,-0.2);
    MOTOR_SetSpeed(1,0.2);
    delay(390);
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0);
    if (direction == FACE_AVANT)
    {
        direction = FACE_GAUCHE;
    }
    else if (direction == FACE_GAUCHE)
    {
        direction = FACE_ARRIERE;
    }
    else if (direction == FACE_DROIT)
    {
        direction = FACE_AVANT;
    }
    else if (direction == FACE_ARRIERE)
    {
        direction = FACE_DROIT;
    }
    return;
}

void tourner_droite(void)
{
    MOTOR_SetSpeed(0,0.5);
    MOTOR_SetSpeed(1,-0.5);
    delay(390);
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0);
    if (direction == FACE_AVANT)
    {
        direction = FACE_DROIT;
    }
    else if (direction == FACE_DROIT)
    {
        direction = FACE_ARRIERE;
    }
    else if (direction == FACE_ARRIERE)
    {
        direction = FACE_GAUCHE;
    }
    else if (direction == FACE_GAUCHE)
    {
        direction = FACE_AVANT;
    }
    return;
}

void scan(void)
{
    static bool scan_droit;
    static bool scan_gauche;
    static bool scan_avant;

    if(direction == FACE_AVANT)
    {
        Serial.println("Scan avant");
        if(digitalRead(rougepin) || digitalRead(vertpin))
        {
            Serial.println("objet detecte avant");
            scan_avant = true;
        }
        else
        {
            Serial.println("pas d'objet detecte avant");
            scan_avant = false;
        }

        if(scan_avant == false)
        {
            Serial.println("Choix: avancer");
            return;
        }


        tourner_gauche();
        Serial.println("Scan gauche");
        delay(1000);
        if(digitalRead(rougepin) || digitalRead(vertpin))
        {
            Serial.println("pas d'objet detecte a gauche");
            scan_gauche = false;
        }
        else
        {
            Serial.println("objet detecte a gauche");
            scan_gauche = true;
        }

        tourner_droite();
        tourner_droite();
        delay(1000);
        Serial.println("Scan droit");
        if(digitalRead(rougepin) || digitalRead(vertpin))
        {
            Serial.println("pas d'objet detecte a droite");
            scan_droit = false;
        }
        else
        {
            Serial.println("objet detecte a droite");
            scan_droit = true;
        }
        
        tourner_gauche();
        delay(500);

        if(!scan_gauche)
        {
            Serial.println("Choix: tourner gauche");
            tourner_gauche();
        }
        else if(!scan_droit)
        {
            Serial.println("Choix: tourner droit");
            tourner_droite();
        }
    }
}