#include <Arduino.h>
#include <LibRobus.h>
#include "robotVerre.h"

const uint8_t J6_DEPART = 160;
const uint8_t J7_DEPART = 180;

const uint8_t J6_MIN = 40;
const uint8_t J6_MAX = 180;
const uint8_t J7_MIN = 90;
const uint8_t J7_MAX = 180;
const unsigned long DELAI_MOUVEMENT_MS = 20;

struct ServoVerre
{
    uint8_t angle;
    uint8_t cible;
    unsigned long dernierPas;
};

ServoVerre j6 = {J6_DEPART, J6_DEPART, 0};
ServoVerre j7 = {J7_DEPART, J7_DEPART, 0};

ServoVerre *choisirServo(uint8_t id)
{
    if (id == SERVO_J6)
    {
        return &j6;
    }

    if (id == SERVO_J7)
    {
        return &j7;
    }

    return 0;
}

bool reglerCible(uint8_t id, uint8_t angle)
{
    ServoVerre *servo = choisirServo(id);
    if (servo == 0)
    {
        return false;
    }

    const uint8_t minimum = id == SERVO_J6 ? J6_MIN : J7_MIN;
    const uint8_t maximum = id == SERVO_J6 ? J6_MAX : J7_MAX;

    if (angle < minimum || angle > maximum)
    {
        Serial.println("Angle refuse: limite atteinte.");
        return false;
    }

    servo->cible = angle;
    servo->dernierPas = millis();
    return true;
}

void bougerUnPas(uint8_t id, ServoVerre &servo, unsigned long maintenant)
{
    if (servo.angle == servo.cible ||
        maintenant - servo.dernierPas < DELAI_MOUVEMENT_MS)
    {
        return;
    }

    // Le servo avance d'un degre a la fois pour bouger doucement.
    if (servo.angle < servo.cible)
    {
        servo.angle++;
    }
    else
    {
        servo.angle--;
    }

    SERVO_SetAngle(id, servo.angle);
    servo.dernierPas = maintenant;
}

bool initialiserServos()
{
    SERVO_Enable(SERVO_J6);
    SERVO_Enable(SERVO_J7);
    SERVO_SetAngle(SERVO_J6, J6_DEPART);
    SERVO_SetAngle(SERVO_J7, J7_DEPART);
    return true;
}

void mettreAJourServos(unsigned long maintenant)
{
    bougerUnPas(SERVO_J6, j6, maintenant);
    bougerUnPas(SERVO_J7, j7, maintenant);
}

bool servoTermine(uint8_t idServo)
{
    ServoVerre *servo = choisirServo(idServo);
    return servo != 0 && servo->angle == servo->cible;
}
