#include <LibRobus.h>
#include "main.h"
#include "pid.h"
#include "mouvement.h"
#include "contourne_mur.h"
#include "suiveur.h"
#include "quille.h"
#include "capteurCouleur.h"
#include "receptionIR.h"
#include "led.h"


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
bool verreEpreuve = false;
bool murEpreuve = false;
bool stopEpreuve = false;
bool quilleEpreuve = false;
bool trouEpreuve = false;

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
    // delay(10000);
    // digitalWrite(brocheLED, LOW);
}



void beep(int nb)
{
  for(int i = 0; i< nb; i++)
  {
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(25);
  }
}

void loop()
{
  int rouge;
  int vert;
  int bleu;
  int luminosite;
  bool jaune, vertclair, bleufonce, rose, orange;

  eteindreLed();

  while(etat == 0)
  {
    if(receptionIR() == START_IR)
    {
      etat = 1;
      beep(2);
      break;
    }
    else if(receptionIR() == CONTINUE_IR)
    {
      beep(3);
    }
  }
  while(etat == 2)
  {
    if(receptionIR() == CONTINUE_IR)
    {
      etat = 1;
      beep(3);
      break;
    }
    else if(receptionIR() == START_IR)
    {
      beep(2);
    }
  }
  while(etat == 1)
  {
    lireRGB(rouge, vert, bleu, luminosite);
    detecterCouleur(jaune, vertclair, bleufonce, rose, orange);
    if(jaune && !murEpreuve)
    {
        ledJauneDelai = millis();
        allumerLED(LED_JAUNE);
        //Mur
        contourneMur();
        murEpreuve = true;
      
    }
    else if(vertclair && !trouEpreuve)
    {
        ledVertDelai = millis();
        allumerLED(LED_VERT);
        //Trou
        trou();
        trouEpreuve = true;
      
    }
    else if(bleufonce && !stopEpreuve)
    {
        ledBleuDelai = millis();
        allumerLED(LED_BLEU);
        //attente
        etat = 2;
        stopEpreuve = true;
        break;
      
    }
    else if(rose && !quilleEpreuve)
    {
        ledRoseDelai = millis();
        allumerLED(LED_ROSE);
        //quille
        EpreuveQuille();
        quilleEpreuve = true;
      
    }
    else if(orange && !verreEpreuve)
    {
        ledOrangeDelai = millis();
        allumerLED(LED_ORANGE);
        //Verre

        verreEpreuve = true;
      
    }
    else if(verreEpreuve && murEpreuve && stopEpreuve && quilleEpreuve && trouEpreuve)
    {
      pid_arret();
      //déposer le verre

      etat = 3;
      break;
    }
    else
    {
      suivreLaLigne();
    }

  }
  Serial.println("Debut du suivi de ligne");
} 
