#include <LibRobus.h>
#include "sifflet.h"
#include "main.h"

void attendSifflet(void)
{
    while (!digitalRead(pinsifflet))
    {
        delay(100);
    }
}
