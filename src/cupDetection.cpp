#include <LibRobus.h>
#include "cupDetection.h"
#include "robusSerial.h"
#include "../config.h"
#include "deplacement.h"

static float integrale = 0;
static float prev_erreur = 0;
static float dt = 0;
static unsigned long timefin = 0;

float pidStep(int mesure);

int getCupPosition(void)
{
    int data = Serial1RX();
    if(data > 0)
    {
        return data;
    }
    return -1;
}

int goToCup(int posx)
{
    if (posx < 0)
    {return -1;} 
    float pid_out = pidStep(posx);
    avance(pid_out);
    return 0;
}

void stopCup(void)
{
    MOTOR_SetSpeed(RIGHT, 0);
    MOTOR_SetSpeed(LEFT,0);
    integrale = 0;
    prev_erreur = 0;
    dt = 0;
    timefin = 0;
}

//https://www.steppeschool.com/blog/pid-implementation-in-c
float pidStep(int mesure)
{
    dt = millis() - timefin;
    int erreur = VID_CENTER - mesure;

    int P = V_KP * erreur;

    integrale += erreur*dt;

    float I = V_KI * integrale;

    float D =0;

    if(dt > 0)
    {
        D = V_KD * (erreur - prev_erreur) / dt;
    }

    prev_erreur = erreur;

    float output = P + I + D;

    if(output > V_CLAMP_MAX)
    {
        output = V_CLAMP_MAX;
        integrale -= erreur * dt;
    }
    else if (output < V_CLAMP_MIN)
    {
        output = V_CLAMP_MIN;
        integrale -= erreur * dt;
    }
    timefin = millis();

    return output;

}