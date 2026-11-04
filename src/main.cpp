#include <LibRobus.h>
#include "main.h"
#include "pid.h"
#include "mouvement.h"
#include "contourne_mur.h"
#include "suiveur.h"
#include "quille.h"
#include "capteurCouleur.h"

int LED_JAUNE = 31;
int LED_VERT = 33;
int LED_BLEU = 35;
int LED_ROSE = 41;
int LED_ORANGE = 39;
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

int etat = 0;
short int compteurEpreuve = 0;

void setup()
{
  BoardInit();
  pid_init();
  Serial.begin(115200);

  pinMode(LED_JAUNE, OUTPUT);
  pinMode(LED_VERT, OUTPUT);
  pinMode(LED_BLEU, OUTPUT);
  pinMode(LED_ROSE, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);

  digitalWrite(LED_JAUNE, LOW);
  digitalWrite(LED_VERT, LOW);
  digitalWrite(LED_BLEU, LOW);
  digitalWrite(LED_ROSE, LOW);
  digitalWrite(LED_ORANGE, LOW);

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

void allumerLED(int brocheLED)
{
  pid_arret();
    digitalWrite(brocheLED, HIGH);
    delay(10000);
    digitalWrite(brocheLED, LOW);
}

void loop()
{
  int rouge;
  int vert;
  int bleu;
  int luminosite;
  bool jaune, vertclair, bleufonce, rose, orange;

  while(etat == 0)
  {
    
  }
  while(etat == 1)
  {
    lireRGB(rouge, vert, bleu, luminosite);
    detecterCouleur(jaune, vertclair, bleufonce, rose, orange);
    if(jaune)
    {
      allumerLED(LED_JAUNE);
      //Mur
      contourneMur();
      compteurEpreuve++;
    }
    else if(vertclair)
    {
      allumerLED(LED_VERT);
      //Trou
      trou();
      compteurEpreuve++;
      
    }
    else if(bleufonce)
    {
      allumerLED(LED_BLEU);
      //attente
      etat = 0;
      compteurEpreuve++;
      break;
    }
    else if(rose)
    {
      allumerLED(LED_ROSE);
      //quille
      EpreuveQuille();
      compteurEpreuve++;
    }
    else if(orange)
    {
      allumerLED(LED_ORANGE);
      //Verre
      
      compteurEpreuve++;
    }
    if(compteurEpreuve >= 5)
    {
      pid_arret();
      //déposer le verre

      etat = 2;
      break;
    }
    suivreLaLigne();

  }
  Serial.println("Debut du suivi de ligne");
} 
