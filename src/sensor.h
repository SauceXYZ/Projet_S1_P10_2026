#ifndef SENSOR_H
#define SENSOR_H

//Numero des bumpers
#define DROIT 1
#define AVANT 2
#define ARRIERE 3
#define GAUCHE 4

extern bool bumperArr;
extern bool bumperAva;
extern bool bumperGch;
extern bool bumperDrt;

//Pins capteur sur Arduino
#define vertPin 2
#define rougePin 3
#define siffletPin 4

extern bool vert;
extern bool rouge;
extern bool sifflet;


void beep(int count);

#endif