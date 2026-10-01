//3200 pulse par tour
//pulse directionnel
//compteur ++ ou compteur --

//int32_t ENCODER_Read(uint8_t id)
//int32_t ENCODER_ReadReset(uint8_t id)

//-0.15 a 0,15 nn fonctionnel
//-1 a 1 (signe = direction)

/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
Date: Derniere date de modification
*/

/*
Inclure les librairies de functions que vous voulez utiliser
*/
#include <LibRobus.h>

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

float vitesse = 0.50;
int clicParTour = 3200;
float diametreRoue = 7.62; // en cm


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

void arret(){
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
};

void avance(float vitessedroit, float vitessegauche){
  MOTOR_SetSpeed(RIGHT,vitessedroit);
  MOTOR_SetSpeed(LEFT, vitessegauche);
};

void recule(float vitesse){
  MOTOR_SetSpeed(RIGHT, -vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
};

void tourneDroit(float vitesse){
  MOTOR_SetSpeed(RIGHT, 0.5*vitesse);
  MOTOR_SetSpeed(LEFT, -0.5*vitesse);
};


int compteurTotaleDroit = 0;
int compteurTotaleGauche = 0;
const int PULSEATTENDUDROIT = 2300;
const int PULSEATTENDUGAUCHE = 2300;
float vitesseDroite = 0.50;
float vitesseGauche = 0.59;

void pid(){
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
    }

   

    kDroite = PULSEATTENDUDROIT - compteurDroit;
    vitesseDroite += kDroite*KP;


    kGauche = PULSEATTENDUGAUCHE - compteurGauche;
    vitesseGauche += kGauche*KP;

    Serial.print("vitesseDroite: ");
    Serial.println(vitesseDroite);
    Serial.print("vitesseGauche: ");
    Serial.println(vitesseGauche);
    

    avance(vitesseDroite, vitesseGauche);

    
};

void tourneGauche(){
  MOTOR_SetSpeed(RIGHT, -0.5*vitesse);
  MOTOR_SetSpeed(LEFT, 0.5*vitesse);
  delay(820);
  arret();
}

void tournerDroit(){
  MOTOR_SetSpeed(RIGHT, 0.5*vitesse);
  MOTOR_SetSpeed(LEFT, -0.5*vitesse);
  delay(820);
  arret();
}





/*
Fonctions d'initialisation (setup)
 -> Se fait appeler au debut du programme
 -> Se fait appeler seulement un fois
 -> Generalement on y initilise les varibbles globales
*/
void setup(){
  BoardInit();
  
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
    tournerDroit();
  }
  
  if (bumperArr){
    tourneGauche();
    /*if(etat == 0){
      etat = 1;
    }
    else {
        Serial.print("bumperArr: ");
        Serial.println(bumperArr);
        etat = 0;
    }*/
  } 
  /*if(etat == 1){
    delay(200);
    pid();
  } else if(etat == 0){
    arret();
    delay(500);
  }*/
  
}


