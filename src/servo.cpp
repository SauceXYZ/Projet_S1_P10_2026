#include "servo.h"
#include <LibRobus.h>


#define SERVO_PINCE 0 //J6 0
#define SERVO_PIVOT 1 //J7 1

#define ANGLE_PINCE_OUVERT 160
#define ANGLE_PINCE_GRAB 100
#define ANGLE_PIVOT_BAS 178
#define ANGLE_PIVOT_HAUT 160


void initServo()
{
    Serial.println("Initialisation des servos");
    SERVO_Enable(SERVO_PINCE);
    SERVO_Enable(SERVO_PIVOT);
    SERVO_SetAngle(SERVO_PINCE, ANGLE_PINCE_GRAB);
    SERVO_SetAngle(SERVO_PIVOT, ANGLE_PIVOT_BAS);
    delay(800);
    SERVO_Disable(SERVO_PINCE);
    SERVO_Disable(SERVO_PIVOT);
}

void grabServo()
{
    Serial.println("Activation des servos");
    SERVO_Enable(SERVO_PINCE);
    SERVO_Enable(SERVO_PIVOT);
    SERVO_SetAngle(SERVO_PINCE, ANGLE_PINCE_GRAB);
    delay(800);
    SERVO_SetAngle(SERVO_PIVOT, ANGLE_PIVOT_HAUT);
    delay(800);
    //SERVO_Disable(SERVO_PINCE);
    SERVO_Disable(SERVO_PIVOT);
}

void dropServo()
{
    Serial.println("Activation des servos");
    SERVO_Enable(SERVO_PINCE);
    SERVO_Enable(SERVO_PIVOT);
    SERVO_SetAngle(SERVO_PIVOT, ANGLE_PIVOT_BAS);
    delay(800);
    SERVO_SetAngle(SERVO_PINCE, ANGLE_PINCE_OUVERT);
    delay(800);
    SERVO_Disable(SERVO_PINCE);
    SERVO_Disable(SERVO_PIVOT);
}