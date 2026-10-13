#include "LibRobus.h"
#include "sifflet.h"
#include "main.h"

bool debut_lock = 0;

void attendSifflet(void)
{
while(!ROBUS_IsBumper(3))//!digitalRead(pinsifflet))
  {
    delay(100);
  }
}