#include "led.h"
#include <LibRobus.h>

int ledBleuDelai = 0;
int ledJauneDelai = 0;
int ledVertDelai = 0;
int ledRoseDelai = 0;
int ledOrangeDelai = 0;

void eteindreLed()
{
  if(ledRoseDelai != 0)
  {
    if(millis() - ledRoseDelai >= 10000)
    {
      digitalWrite(LED_ROSE, LOW);
      ledRoseDelai = 0;
    }
  }
  if(ledBleuDelai != 0)
  {
    if(millis() - ledBleuDelai >= 10000)
    {
      digitalWrite(LED_BLEU, LOW);
      ledBleuDelai = 0;
    }
  }
  if(ledJauneDelai != 0)
  {
    if(millis() - ledJauneDelai >= 10000)
    {
      digitalWrite(LED_JAUNE, LOW);
      ledJauneDelai = 0;
    }
  }
  if(ledVertDelai != 0)
  {
    if(millis() - ledVertDelai >= 10000)
    {
      digitalWrite(LED_VERT, LOW);
      ledVertDelai = 0;
    }
  }
  if(ledOrangeDelai != 0)
  {
    if(millis() - ledOrangeDelai >= 10000)
    {
      digitalWrite(LED_ORANGE, LOW);
      ledOrangeDelai = 0;
    }
  }
}