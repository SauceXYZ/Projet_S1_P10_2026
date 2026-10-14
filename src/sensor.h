#ifndef SENSOR_H
#define SENSOR_H

extern bool bumperArr;
extern bool bumperAva;
extern bool bumperGch;
extern bool bumperDrt;

//Pins capteur sur Arduino
#define vertPin 49
#define rougePin 48
#define siffletPin 52

extern bool proxVert;
extern bool proxRouge;
extern bool sifflet;


void beep(int count);
bool verifierMur();

#endif