#include "led.h"
#include <LibRobus.h>

// Match millis(): AVR int overflows after 32.767 s.
unsigned long ledBleuDelai = 0;
unsigned long ledJauneDelai = 0;
unsigned long ledVertDelai = 0;
unsigned long ledRoseDelai = 0;
unsigned long ledOrangeDelai = 0;

void eteindreLed()
{
  if(ledRoseDelai != 0)
  {
    if(millis() - ledRoseDelai >= 10000)
    {
      digitalWrite(LED_ROSE, HIGH);
      ledRoseDelai = 0;
    }
  }
  if(ledBleuDelai != 0)
  {
    if(millis() - ledBleuDelai >= 10000)
    {
      digitalWrite(LED_BLEU, HIGH);
      ledBleuDelai = 0;
    }
  }
  if(ledJauneDelai != 0)
  {
    if(millis() - ledJauneDelai >= 10000)
    {
      digitalWrite(LED_JAUNE, HIGH);
      ledJauneDelai = 0;
    }
  }
  if(ledVertDelai != 0)
  {
    if(millis() - ledVertDelai >= 10000)
    {
      digitalWrite(LED_VERT, HIGH);
      ledVertDelai = 0;
    }
  }
  if(ledOrangeDelai != 0)
  {
    if(millis() - ledOrangeDelai >= 10000)
    {
      digitalWrite(LED_ORANGE, HIGH);
      ledOrangeDelai = 0;
    }
  }
}