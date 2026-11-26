#include <LibRobus.h>
#include "robusSerial.h"
#include "../config.h"

int serialInit3_3V(unsigned long baud)
{
    Serial1.begin(baud);
    return 0;
    
}

int Serial1RX(void)
{
    // A line can arrive across several calls to Serial1RX().
    static int data = 0;
    static bool reception = false;
    static bool estNegatif = false;
    while(Serial1.available() > 0)
    {
        char c = Serial1.read();
        Serial.print(c);

        if(c == '-')
        {
            estNegatif = true;
        }
        else if(c >= '0' && c <= '9')
        {
            data = data * 10 + (c - '0');
            reception = true;
        }
        else if(reception && (c == '\n' || c == '\r'))
        {
            int completedData = data;
            data = 0;
            reception = false;
            if(estNegatif)
            {
                completedData = completedData * -1;
                estNegatif = false;
            }
            return completedData;
        }
    }

    return -2;
}
