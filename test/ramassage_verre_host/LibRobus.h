#pragma once
#include <stdint.h>
uint16_t ROBUS_ReadIR(uint8_t capteur);
void delay(unsigned long ms);
int32_t ENCODER_Read(uint8_t moteur);
struct SerialStub {
    template <typename T> void print(T) {}
    template <typename T> void println(T) {}
};
extern SerialStub Serial;
