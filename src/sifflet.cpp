#include "LibRobus.h"
#include "sifflet.h"
#include "main.h"

bool debut_lock = 0;

void attendSifflet(void)
{
while(1)
  {
    if (digitalRead(pinsifflet) == 1)
    {
      debut_lock = 1;
      break;
    }
    else
    {
      debut_lock = 0;
    }
  }
}