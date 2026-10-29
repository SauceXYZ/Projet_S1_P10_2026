#include <LibRobus.h>
#include "main.h"
#include "pid.h"
#include "mouvement.h"
#include "contourne_mur.h"
#include "suiveur.h"

int vertpin = 48;
int rougepin = 49;
int pinsifflet = 47;

int pinSuiveurGauche = A8; //À changer pour la bonne pin
int pinSuiveurMilieu = A9; //À changer pour la bonne pin
int pinSuiveurDroit = A10; //À changer pour la bonne pin
// les capteurs retourne entre 500 et 800 environ pour des tapes noirs à environ
//  4 mm (Va falloir tester avec le robot)

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
  delay(300);
}