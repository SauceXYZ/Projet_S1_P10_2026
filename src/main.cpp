#include <LibRobus.h>
#include "main.h"

void setup()
{
  BoardInit();
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  init_pompe(); 

  Serial.println("Initialisation complete");

}

void loop()
{
    if (ROBUS_IsBumper(FRONT)) 
    {
        actionner_pompe();
    }
} 
