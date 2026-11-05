#include <LibRobus.h>
#include "main.h"
#include "pid.h"
#include "mouvement.h"
#include "contourne_mur.h"
#include "suiveurPID.h"
#include "quille.h"
#include "capteurCouleur.h"
#include "receptionIR.h"
#include "led.h"
#include "robotVerre.h"

int vertpin = 48;
int rougepin = 49;
int pinsifflet = 47;

#define TEMPS_DETECTION_COULEUR 500

#define ACTIVE_EPREUVE_MURE false
#define ACTIVE_EPREUVE_TROUE false
#define ACTIVE_EPREUVE_STOP false
#define ACTIVE_EPREUVE_QUILLE false
#define ACTIVE_EPREUVE_VERRE false
#define ACTIVE_SUIVREUR true


int pinSuiveurGauche = A8; //À changer pour la bonne pin
int pinSuiveurMilieu = A9; //À changer pour la bonne pin
int pinSuiveurDroit = A10; //À changer pour la bonne pin
// les capteurs retourne entre 500 et 800 environ pour des tapes noirs à environ
//  4 mm (Va falloir tester avec le robot)

float lab_posx = 0;
float lab_posy = 0;

int direction = FACE_AVANT;

int etat = 1;
short int compteurEpreuve = 0;
bool verreEpreuve = false;
bool murEpreuve = false;
bool stopEpreuve = false;
bool quilleEpreuve = false;
bool trouEpreuve = false;
bool trouEnCours = false;
bool couleurInitialisee = false;

void setup()
{
  BoardInit();
  pid_init();
  Serial.begin(9600);
  if (ACTIVE_EPREUVE_MURE || ACTIVE_EPREUVE_TROUE || ACTIVE_EPREUVE_STOP ||
      ACTIVE_EPREUVE_QUILLE || ACTIVE_EPREUVE_VERRE) {
    couleurInitialisee = initialiser();
  }
  reinitialiserSuiveur();

  pinMode(LED_JAUNE, OUTPUT);
  pinMode(LED_VERT, OUTPUT);
  pinMode(LED_BLEU, OUTPUT);
  pinMode(LED_ROSE, OUTPUT);
  pinMode(LED_ORANGE, OUTPUT);

  digitalWrite(LED_JAUNE, HIGH);
  digitalWrite(LED_VERT, HIGH);
  digitalWrite(LED_BLEU, HIGH);
  digitalWrite(LED_ROSE, HIGH);
  digitalWrite(LED_ORANGE, HIGH);

  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  pinMode(pinSuiveurGauche, INPUT);
  pinMode(pinSuiveurMilieu, INPUT);
  pinMode(pinSuiveurDroit, INPUT);
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  //initialiserServos();

  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  Serial.println("Initialisation complete");
}

void allumerLED(int brocheLED)
{
  arreterSuiveur();
  pid_arret();
    digitalWrite(brocheLED, LOW);
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
  bool jaune = false;
  bool vertclair = false;
  bool bleufonce = false;
  bool rose = false;
  bool orange = false;
  static unsigned long oldTempsCouleur = 0;

  eteindreLed();

  while(etat == 0)
  {
    if(receptionIR() == START_IR)
    {
      reinitialiserSuiveur();
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
      reinitialiserSuiveur();
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
    if (trouEnCours) {
      suivreLaLigne();
      if (trouTermine()) { trouEnCours = false; trouEpreuve = true; }
      continue;
    }
    // Read completed color integrations without waiting inside the driver.
    if(couleurInitialisee && millis() - oldTempsCouleur > TEMPS_DETECTION_COULEUR)
    {
      oldTempsCouleur = millis();
      detecterCouleur(jaune, vertclair, bleufonce, rose, orange);
    }
    // if(jaune || vertclair || bleufonce || rose || orange) Serial.print("Couleur detectee: ");
    // if(jaune) {Serial.println("Jaune"); jaune = false;}
    // else if(vertclair) {Serial.println("Vert"); vertclair = false;}
    // else if(bleufonce) {Serial.println("Bleu"); bleufonce = false;}
    // else if(rose) {Serial.println("Rose"); rose = false;}
    // else if(orange) {Serial.println("Orange"); orange = false;}
    //continue;
    if(jaune && !murEpreuve && ACTIVE_EPREUVE_MURE)
    {
        ledJauneDelai = millis();
        allumerLED(LED_JAUNE);
        //Mur
        arreterSuiveur();
        contourneMur();
        reinitialiserSuiveur();
        murEpreuve = true;
      
    }
    else if(vertclair && !trouEpreuve && ACTIVE_EPREUVE_TROUE)
    {
        ledVertDelai = millis();
        allumerLED(LED_VERT);
        //Trou
        trou();
        trouEnCours = true;
      
    }
    else if(bleufonce && !stopEpreuve && ACTIVE_EPREUVE_STOP)
    {
        ledBleuDelai = millis();
        allumerLED(LED_BLEU);
        //attente
        etat = 2;
        stopEpreuve = true;
        break;
      
    }
    else if(rose && !quilleEpreuve && ACTIVE_EPREUVE_QUILLE)
    {
        ledRoseDelai = millis();
        allumerLED(LED_ROSE);
        //quille
        arreterSuiveur();
        EpreuveQuille();
        reinitialiserSuiveur();
        quilleEpreuve = true;
      
    }
    else if(orange && !verreEpreuve && ACTIVE_EPREUVE_VERRE)
    {
        ledOrangeDelai = millis();
        allumerLED(LED_ORANGE);
        arreterSuiveur();
        executerRamassageVerre();
        reinitialiserSuiveur();

        verreEpreuve = true;
      
    }
    else if(verreEpreuve && murEpreuve && stopEpreuve && quilleEpreuve && trouEpreuve)
    {
      arreterSuiveur();
      pid_arret();
      //déposer le verre

      etat = 3;
      break;
    }
    else
    {
        if(ACTIVE_SUIVREUR)
        {
            suivreLaLigne();
            testSuiveur();
        }
    }

  }
  Serial.println("Debut du suivi de ligne");
} 
