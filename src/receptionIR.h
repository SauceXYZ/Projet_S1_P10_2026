#ifndef RECEPTIONIR_H
#define RECEPTIONIR_H

#define RIEN_IR 0
#define START_IR 1 
#define CONTINUE_IR 2

#define CMD_ON     0x01
#define CMD_OFF    0x02

#define IR_RECEIVE_PIN 37
 
#define IR_ADDRESS 0x00

int receptionIR(void);

#endif