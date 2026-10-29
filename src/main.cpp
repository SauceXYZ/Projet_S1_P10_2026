#include <LibRobus.h>
#include "main.h"
#include "pid.h"
#include "mouvement.h"
#include "contourne_mur.h"
#include "quille.h"

int vertpin = 48;
int rougepin = 49;
int pinsifflet = 47;

float lab_posx = 0;
float lab_posy = 0;

int direction = FACE_AVANT;

void setup()
{
    BoardInit();
    pid_init();
    Serial.begin(115200);

    pinMode(vertpin, INPUT);
    pinMode(rougepin, INPUT);
    ENCODER_Reset(0);
    ENCODER_Reset(1);
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    Serial.println("Initialisation complete");
}

void loop()
{
    static bool fait = false;
    if (fait) return;

    delay(3000);                 // temps de poser le robot devant le mur
    bool ok = contourneMur();
    Serial.print("Contournement reussi: ");
    Serial.println(ok);
    fait = true;                 // un seul essai
}