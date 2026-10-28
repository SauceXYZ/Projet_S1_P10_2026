#include <LibRobus.h>
#include "main.h"
#include "pid.h"
#include "mouvement.h"
#include "contourne_mur.h"
#include "suiveur.h"

int vertpin = 48;
int rougepin = 49;
int pinsifflet = 47;

int pinSuiveurGauche = 31; //À changer pour la bonne pin
int pinSuiveurMilieu = 33; //À changer pour la bonne pin
int pinSuiveurDroit = 35; //À changer pour la bonne pin

float lab_posx = 0;
float lab_posy = 0;

int direction = FACE_AVANT;

void setup()
{
  BoardInit();
  pid_init();
  Serial.begin(115200);

  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  pinMode(pinSuiveurGauche, INPUT);
  pinMode(pinSuiveurMilieu, INPUT);
  pinMode(pinSuiveurDroit, INPUT);
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  Serial.println("Initialisation complete");
}

void loop()
{
  Serial.println("Debut du suivi de ligne");
  suivreLaLigne();
}