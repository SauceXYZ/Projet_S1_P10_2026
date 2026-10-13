//3200 pulse par tour
//pulse directionnel
//compteur ++ ou compteur --

//int32_t ENCODER_Read(uint8_t id)
//int32_t ENCODER_ReadReset(uint8_t id)

//-0.15 a 0,15 nn fonctionnel
//-1 a 1 (signe = direction)

/*
Projet: Labyrinthe
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Script d'execution du parcours pour le robot
Date: 08/10/2026
*/

/*
Inclure les librairies de functions que vous voulez utiliser
*/
#include <LibRobus.h>
#include "map.h"

/*
Variables globales et defines
 -> defines...
 -> L'ensemble des fonctions y ont acces
*/
bool bumperArr=false;
bool bumperAv=false;
int vertpin = 48;
int rougepin = 49;
bool vert = false;
bool rouge = false;
int etat = 0; // = 0 arrêt 1 = avance 2 = recule 3 = TourneDroit 4 = TourneGauche
int etatPast = 0;

int x = 0;
int y = 0;
char direction = 'N'; // N = Nord, E = Est, S = Sud, O = Ouest

int compteurTotaleDroit = 0;
int compteurTotaleGauche = 0;
const int PULSEATTENDUDROIT = 2300;
const int PULSEATTENDUGAUCHE = 2300;
float vitesseDroite = 0.50;
float vitesseGauche = 0.59;
float pulseParSeconde = 7.0;

/*
Vos propres fonctions sont creees ici
*/

void beep(int count){
  for(int i=0;i<count;i++){
    AX_BuzzerON();
    delay(100);
    AX_BuzzerOFF();
    delay(100);  
  }
  delay(400);
}



void avance(float vitessedroit, float vitessegauche){
  MOTOR_SetSpeed(LEFT, vitessegauche);
  MOTOR_SetSpeed(RIGHT,vitessedroit);
};

void recule(float vitessedroit, float vitessegauche){
  MOTOR_SetSpeed(RIGHT, -vitessedroit);
  MOTOR_SetSpeed(LEFT, -vitessegauche);
};

void pid(int delai){
    int pulseAttendu = delai * pulseParSeconde * 2.25;
    float kGauche=0;  //Différence
    float kDroite=0;  //Différence
    float differenceK = 0;
    float KP=0.0001; //Correction proportionnelle
    
    int compteurDroit = 0;
    int compteurGauche = 0;
    Serial.print("compteurDroit: ");
    Serial.println(ENCODER_Read(RIGHT));
    Serial.print("compteurGauche: ");
    Serial.println(ENCODER_Read(LEFT));

    compteurDroit = ENCODER_ReadReset(RIGHT);
    compteurGauche = ENCODER_ReadReset(LEFT);
    compteurTotaleDroit += compteurDroit;
    compteurTotaleGauche += compteurGauche;

    differenceK = compteurDroit - compteurGauche;

    if(differenceK > 100){
       if(differenceK > 0){
      vitesseGauche += differenceK*KP/2;
      vitesseDroite -= differenceK*KP/2;
    }
    else if(differenceK < 0){
      vitesseDroite += -differenceK*KP/2;
      vitesseGauche -= -differenceK*KP/2;
    }
    } else {
      kDroite = pulseAttendu - compteurDroit;
      vitesseDroite += kDroite*KP;

      kGauche = pulseAttendu - compteurGauche;
      vitesseGauche += kGauche*KP;
    }
    Serial.print("vitesseDroite: ");
    Serial.println(vitesseDroite);
    Serial.print("vitesseGauche: ");
    Serial.println(vitesseGauche);
    
    avance(vitesseDroite, vitesseGauche);
};


void avancerDistance(float distance){
    float pi = 3.14159;
    int pulseParTour = 2700;
    
    // On utilise "long" pour les pulses car ce sont de grands nombres entiers
    float pulses_requis =  ((distance * pulseParTour) / (pi * diametreRoue));
    float pulses_deplace = 0;

    while(pulses_deplace < pulses_requis){
        //avancerDroitPID();
        pid(50);
        delay(50);
        pulses_deplace += ENCODER_Read(RIGHT);

    }
    
    arret();
}


bool verifierMur(){
    vert = digitalRead(vertpin);
    rouge = digitalRead(rougepin);
    if(vert && rouge){
        return true;
    } else {
        return false;
    }
}

void ajusterPositionRobot(char direction){
    switch(direction){
        case 'N':
            y--;
            break;
        case 'E':
            x++;
            break;
        case 'S':
            y++;
            break;
        case 'O':
            x--;
            break;
    }
}

void directionAAller(){
    possibiliteTape(x, y);
    char tabDirection[4] = {'N', 'E', 'S', 'O'};
    Serial.print("Possibilite: ");
          Serial.print(possibilite[0]);
          Serial.print(possibilite[1]);
          Serial.print(possibilite[2]);
          Serial.println(possibilite[3]);
    for(int i = 0; i < 4; i++){
        if(possibilite[i] == 1){
          
        direction = tournerDansDirection(direction, tabDirection[i]);
        Serial.print("Direction apres tournerDansDirection: ");
        Serial.println(direction);

        if(verifierMur()){
            avancerDistance(50);
            ajusterPositionRobot(direction);
            i=4; // on Sort de la boucle
        } else {
            possibilite[i] = 0;
        }
        }
        
    }
    
}

/*
Fonctions d'initialisation (setup)
 -> Se fait appeler au debut du programme
 -> Se fait appeler seulement un fois
 -> Generalement on y initilise les varibbles globales
*/
void setup(){
  BoardInit();

  x = 1;
  y = 5;
  direction = 'N';
  
  //initialisation
  pinMode(vertpin, INPUT);
  pinMode(rougepin, INPUT);
  delay(100);
  beep(3);
}


/*
Fonctions de boucle infini
 -> Se fait appeler perpetuellement suite au "setup"
*/
void loop() {
  bumperArr = ROBUS_IsBumper(3);
  bumperAv = ROBUS_IsBumper(2);

  if (bumperAv){
    etat = 0;
  }
  
  // remplacer par détection de sifflet
  if (bumperArr){
    //etat = 1;
    directionAAller();
  } 
  if(etat == 1){
    directionAAller();
  }
  /*if(etat == 1){
    delay(200);
    pid();
  } else if(etat == 0){
    arret();
    delay(500);
  }*/
  
}





