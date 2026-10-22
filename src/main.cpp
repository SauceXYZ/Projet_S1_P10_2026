#include <LibRobus.h>
#include "mouvement.h"
#include "main.h"
#include "sifflet.h"
#include "pid.h"

int vertpin = 48;
int rougepin = 49;
int pinsifflet = 47;

float lab_posx = 0;
float lab_posy = 0;

int direction = FACE_AVANT;

void setup(){
  BoardInit();
  pid_init();
  Serial.begin(115200);

  //initialisation
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  ENCODER_Reset(1);
  ENCODER_Reset(0);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  Serial.println("Initialisation complete\n");

  attendSifflet();
  Serial.println("Sifflet detecter");
}

void loop() {
  avanceV3();
}
