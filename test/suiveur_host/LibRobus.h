#pragma once
#include <stdint.h>
unsigned long millis();
int analogRead(int);
int32_t ENCODER_Read(int);
void MOTOR_SetSpeed(int, float);
struct SerialStub {
    template<typename T> void print(T) {}
    template<typename T> void println(T) {}
    void println(float, int) {}
};
extern SerialStub Serial;
