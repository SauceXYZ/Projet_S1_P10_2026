#include <LibRobus.h>
#include "main.h"

void setup()
{
  BoardInit();
  Serial.begin(115200);
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  pinMode(PIN_POMPE, OUTPUT);

  Serial.println("Initialisation complete");

}

void loop()
{
    if (ROBUS_IsBumper(LEFT)) 
    {
       actionner_pompe();
     }
} 
