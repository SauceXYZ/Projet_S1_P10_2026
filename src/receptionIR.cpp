#include "receptionIR.h"

#include <Arduino.h>

#define DECODE_NEC  

#include <IRremote.hpp>


int receptionIR(void)
{
    if (IrReceiver.decode()) {
 
      bool isRepeat = IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT;
      bool valide = !isRepeat &&
                    IrReceiver.decodedIRData.protocol == NEC &&
                    IrReceiver.decodedIRData.address == IR_ADDRESS;
      uint8_t commande = IrReceiver.decodedIRData.command;
 
      IrReceiver.resume();   
 
      if (valide) {
        if (commande == CMD_ON)  return START_IR;
        if (commande == CMD_OFF) return CONTINUE_IR;
      }
    }
    return RIEN_IR;
}
