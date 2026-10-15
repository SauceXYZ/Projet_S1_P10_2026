#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"

#define SCAN_DEBUG 0

// Tableau carte du parcours
// chaque bit represente une directuion possible (0 = libre, 1 = obstacle) (N, E, S, O)

//8 bloquer en haut
//4 bloquer a droite
//2 bloquer en bas
//1 bloquer a gauche

char grilleMap[10][3] =
{
    {13, 8, 13},
    {1, 0, 4},
    {5, 5, 5},
    {1, 0, 4},
    {5, 5, 5},
    {1, 0, 4},
    {5, 5, 5},
    {1, 0, 4},
    {5, 5, 5},
    {3, 2, 6}
};

bool positionRobot[10][3] =
{
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, false, false},
    {false, true, false}
};

int32_t encodeur_1_precedent = 0;

static void attendSerial(void)
{
    // Each pause requires new input; discard anything already buffered.
    while (Serial.available() > 0)
    {
        Serial.read();
    }

    // Consume the input and ignore the Serial Monitor's CR/LF line endings.
    int caractere;
    do
    {
        while (Serial.available() == 0) {}
        caractere = Serial.read();
    } while (caractere == '\r' || caractere == '\n');
}

int avance(void)
{
    float distance_enc_cm;
    int32_t compte=0;
    int vert;
    int rouge;
    compte = ENCODER_Read(1);  // Implementation for advancing
    distance_enc_cm = (compte - encodeur_1_precedent) * (23.94/3200);
    //Serial.println(distance_enc_cm, DEC);
    vert = digitalRead(vertpin);
    rouge = digitalRead(rougepin);
    //Serial.println("A");
    // Serial.print("DEL vert: ");
    // Serial.print(vert);
    // Serial.print(" | DEL rouge: ");
    // Serial.println(rouge);
    if (vert && rouge)
    {
        //Serial.println("B");
        if (lab_posy < y_total && lab_posx < x_total_max && lab_posx > x_total_min)
        {
            //Serial.print("C");
            pid_avance();
            if(distance_enc_cm >= 1)
            {
                if(direction == FACE_AVANT)
                {
                    lab_posy=lab_posy + distance_enc_cm;
                }
                else if(direction == FACE_ARRIERE)
                {
                    lab_posy=lab_posy - distance_enc_cm;
                }
                else if (direction == FACE_DROIT)
                {
                    lab_posx=lab_posx + distance_enc_cm;
                }
                else if (direction == FACE_GAUCHE)
                {
                    lab_posx=lab_posx - distance_enc_cm;
                }
                encodeur_1_precedent = compte;

            }

        }
        else
        {
            //Serial.println("D");
            if (FACE_GAUCHE == direction && lab_posx <= x_total_max)
            
            {
                if (lab_posx < x_total_min)
                {
                    lab_posx = x_total_min+0.1;
                }

                faitFace(FACE_AVANT);
                return 0;
            }
            
            else if (FACE_DROIT == direction && lab_posx >= x_total_max)
            {
                if (lab_posx > x_total_max)
                {
                    lab_posx = x_total_max-0.1;
                }

                faitFace(FACE_GAUCHE);
                return 0;
            }
            else if (FACE_AVANT == direction && lab_posy >= y_total)
            {
                if (lab_posy > y_total)
                {
                    lab_posy = y_total-0.1;
                }
                
                faitFace(FACE_ARRIERE);
                return 0;
            }
           
        }
    }
    else
    {
        Serial.println("E");
        MOTOR_SetSpeed(0,0);
        MOTOR_SetSpeed(1,0);
        return 1;
     
    }
return 0;
} 

// return 0 si ok
// return 1 si trop proche de limite ouest
// return 2 si trop proche de limite est
// return 3 si trop proche de limite nord
// return 4 si trop proche de limite sud

int checkBoundary(void)
{
    if (lab_posx < x_total_min)
    {
        return LIMIT_OUEST;
    }
    else if (lab_posx > x_total_max)
    {
        return LIMIT_EST;
    }
    else if (lab_posy > y_total)
    {
        return LIMIT_NORD;
    }
    else if (lab_posy < 0)
    {
        return LIMIT_SUD;
    }
    return 0;
}

static void actualisePosition(int32_t compte)
{
    const float distance_cm = (compte - encodeur_1_precedent) * (23.94f / 3200.0f);
    if (direction == FACE_AVANT) lab_posy += distance_cm;
    else if (direction == FACE_ARRIERE) lab_posy -= distance_cm;
    else if (direction == FACE_DROIT) lab_posx += distance_cm;
    else if (direction == FACE_GAUCHE) lab_posx -= distance_cm;
    encodeur_1_precedent = compte;
}

// Version interne en cm decimaux, pour revenir apres un trajet interrompu.
static bool deplaceMesure(float cm)
{
    if (cm <= 0.0f) return true;
    const int32_t distance = (int32_t)(cm * (3200.0f / 23.94f) + 0.5f);
    const int32_t debut = ENCODER_Read(1);
    int32_t compte;

    while (true)
    {
        compte = ENCODER_Read(1);
        actualisePosition(compte);
        if (compte - debut >= distance || !digitalRead(vertpin) ||
            !digitalRead(rougepin) || checkBoundary() != 0)
        {
            break;
        }
        pid_avance();
        delay(1);
    }
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    // Inclure le mouvement residuel avant une eventuelle rotation.
    delay(100);
    actualisePosition(ENCODER_Read(1));
    return compte - debut >= distance;
}

void deplace(int cm)
{
    deplaceMesure(cm);
}

/*
pseudo code

si pas d'obstacle
    avance d'une case
sinon
    tourne vers cote plus loing de limite x
    si pas d'obstacle
        avance d'une case
        fait face avant
    sinon
        tourne autre direction
        si pas d'obstacle
            avance d'une case
            fait face avant
        sinon
            arrete

 */

 /*
    N
O       E
    S
 */

void avanceV2(void)
{
    int vert = digitalRead(vertpin);
    int rouge = digitalRead(rougepin);

    if(checkBoundary() == LIMIT_NORD)
    {
        faitFace(FACE_ARRIERE);
        vert = digitalRead(vertpin);
        rouge = digitalRead(rougepin);
    }

    if(checkBoundary() == LIMIT_OUEST)
    {
        faitFace(FACE_DROIT);
        vert = digitalRead(vertpin);
        rouge = digitalRead(rougepin);
    }
    else if(checkBoundary() == LIMIT_EST)
    {
        faitFace(FACE_GAUCHE);
        vert = digitalRead(vertpin);
        rouge = digitalRead(rougepin);
    }

    if(vert && rouge)
    {
        deplace(case_cm);
        //delay(1000);
    }
    else
    {
        if (lab_posx < 0)
        {
            tourne(TOURNE_DROITE);
            vert = digitalRead(vertpin);
            rouge = digitalRead(rougepin);
            if(vert && rouge)
            {
                deplace(case_cm);
                faitFace(FACE_AVANT);
            }
            else
            {
                tourne(TOURNE_GAUCHE);
                tourne(TOURNE_GAUCHE);
                vert = digitalRead(vertpin);
                rouge = digitalRead(rougepin);
                if(vert && rouge)
                {
                    deplace(case_cm);
                    faitFace(FACE_AVANT);
                }
                else
                {
                    MOTOR_SetSpeed(0, 0);
                    MOTOR_SetSpeed(1, 0);
                }
            }
        }
        else
        {
            tourne(TOURNE_GAUCHE);
            vert = digitalRead(vertpin);
            rouge = digitalRead(rougepin);
            if(vert && rouge)
            {
                deplace(case_cm);
                faitFace(FACE_AVANT);
            }
            else
            {
                tourne(TOURNE_DROITE);
                tourne(TOURNE_DROITE);
                vert = digitalRead(vertpin);
                rouge = digitalRead(rougepin);
                if(vert && rouge)
                {
                    deplace(case_cm);
                    faitFace(FACE_AVANT);
                }
                else
                {
                    MOTOR_SetSpeed(0, 0);
                    MOTOR_SetSpeed(1, 0);
                }
            }
        }
    }

    
}

// Grille du schema : 3 colonnes de 50 cm, 10 rangees, nord en haut.
static const int tailleCaseV3 = 50;
static const int8_t deltaLigneV3[4] = {-1, 0, 1, 0};
static const int8_t deltaColonneV3[4] = {0, 1, 0, -1};
static const uint8_t bitMurV3[4] = {8, 4, 2, 1};
static const int orientationV3[4] = {FACE_AVANT, FACE_DROIT, FACE_ARRIERE, FACE_GAUCHE};
static uint8_t mursDetectesV3[10][3] = {};

static void arreteV3(const char *message)
{
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    Serial.println(message);
}

static bool passagePossibleV3(int ligne, int colonne, int sens)
{
    const int voisinLigne = ligne + deltaLigneV3[sens];
    const int voisinColonne = colonne + deltaColonneV3[sens];
    if (voisinLigne < 0 || voisinLigne >= 10 || voisinColonne < 0 || voisinColonne >= 3)
        return false;
    const int oppose = (sens + 2) % 4;
    // Respecter les murs des DEUX cases, meme si la carte est asymetrique.
    return !((grilleMap[ligne][colonne] | mursDetectesV3[ligne][colonne]) & bitMurV3[sens]) &&
           !((grilleMap[voisinLigne][voisinColonne] | mursDetectesV3[voisinLigne][voisinColonne]) & bitMurV3[oppose]);
}

static void memoriseMurV3(int ligne, int colonne, int sens)
{
    mursDetectesV3[ligne][colonne] |= bitMurV3[sens];
    mursDetectesV3[ligne + deltaLigneV3[sens]][colonne + deltaColonneV3[sens]] |= bitMurV3[(sens + 2) % 4];
}

// Chercher le prochain pas du plus court chemin vers une case Fin.
// Les passages inconnus sont essayes, puis retires si les capteurs voient un mur.
static int prochainSensV3(int ligne, int colonne)
{
    int8_t parent[30];
    uint8_t file[30];
    for (int i = 0; i < 30; ++i) parent[i] = -2;
    const int depart = ligne * 3 + colonne;
    parent[depart] = -1;
    int lecture = 0, ecriture = 0, fin = -1;
    file[ecriture++] = depart;

    while (lecture < ecriture)
    {
        const int cellule = file[lecture++];
        const int l = cellule / 3, c = cellule % 3;
        if (l == 0) { fin = cellule; break; }
        // A distance egale, avancer au nord puis revenir vers le centre.
        const int ordre[4] = {0, c < 1 ? 1 : 3, c < 1 ? 3 : 1, 2};
        for (int i = 0; i < 4; ++i)
        {
            const int sens = ordre[i];
            if (!passagePossibleV3(l, c, sens)) continue;
            const int voisin = (l + deltaLigneV3[sens]) * 3 + c + deltaColonneV3[sens];
            if (parent[voisin] != -2) continue;
            parent[voisin] = cellule;
            file[ecriture++] = voisin;
        }
    }
    if (fin < 0 || fin == depart) return -1;
    int prochain = fin;
    while (parent[prochain] != depart) prochain = parent[prochain];
    for (int sens = 0; sens < 4; ++sens)
        if ((ligne + deltaLigneV3[sens]) * 3 + colonne + deltaColonneV3[sens] == prochain)
            return sens;
    return -1;
}

void avanceV3(void)
{
    static bool initialiseV3 = false;
    static bool termineV3 = false;
    static int ligne = 9, colonne = 1;
    if (termineV3) return;

    if (!initialiseV3)
    {
        // Le robot doit etre place au centre de la case Depart, face au nord.
        int positions = 0;
        for (int l = 0; l < 10; ++l)
            for (int c = 0; c < 3; ++c)
                if (positionRobot[l][c]) { ligne = l; colonne = c; ++positions; }
        if (positions != 1 || direction != FACE_AVANT)
        {
            arreteV3("V3: position initiale invalide");
            termineV3 = true;
            return;
        }
        lab_posx = (colonne - 1) * tailleCaseV3;
        lab_posy = (9 - ligne) * tailleCaseV3 + tailleCaseV3 / 2.0f;
        encodeur_1_precedent = ENCODER_Read(1);
        initialiseV3 = true;
    }
    if (ligne == 0)
    {
        arreteV3("V3: arrivee atteinte");
        termineV3 = true;
        return;
    }

    const int sens = prochainSensV3(ligne, colonne);
    if (sens < 0)
    {
        arreteV3("V3: aucun chemin vers la fin");
        termineV3 = true;
        return;
    }
    faitFace(orientationV3[sens]);
    delay(100); // Laisser les capteurs se stabiliser apres la rotation.
    if (!digitalRead(vertpin) || !digitalRead(rougepin))
    {
        memoriseMurV3(ligne, colonne, sens);
        return; // Recalculer le chemin au prochain appel.
    }

    pid_init();
    const int32_t debut = ENCODER_Read(1);
    if (!deplaceMesure(tailleCaseV3))
    {
        const float parcouru = (ENCODER_Read(1) - debut) * (23.94f / 3200.0f);
        memoriseMurV3(ligne, colonne, sens);
        // Ne pas changer de case si le capteur a arrete le robot en chemin.
        if (parcouru > 0.0f)
        {
            faitFace(orientationV3[(sens + 2) % 4]);
            delay(100);
            pid_init();
            if (!deplaceMesure(parcouru))
            {
                arreteV3("V3: retour au centre impossible");
                termineV3 = true;
            }
        }
        return;
    }

    positionRobot[ligne][colonne] = false;
    ligne += deltaLigneV3[sens];
    colonne += deltaColonneV3[sens];
    positionRobot[ligne][colonne] = true;
    if (ligne == 0)
    {
        arreteV3("V3: arrivee atteinte");
        termineV3 = true;
    }
}

// void tourner_gauche(void)
// {
//     pid_tourne(TOURNE_GAUCHE, 90);
//     if (direction == FACE_AVANT)
//     {
//         direction = FACE_GAUCHE;
//     }
//     else if (direction == FACE_GAUCHE)
//     {
//         direction = FACE_ARRIERE;
//     }
//     else if (direction == FACE_DROIT)
//     {
//         direction = FACE_AVANT;
//     }
//     else if (direction == FACE_ARRIERE)
//     {
//         direction = FACE_DROIT;
//     }
//     ENCODER_Reset(1);
//     ENCODER_Reset(0);
//     return;
// }

// void tourner_droite(void)
// {
//     pid_tourne(TOURNE_DROITE, 90);
//     MOTOR_SetSpeed(1,0);
//     if (direction == FACE_AVANT)
//     {
//         direction = FACE_DROIT;
//     }
//     else if (direction == FACE_DROIT)
//     {
//         direction = FACE_ARRIERE;
//     }
//     else if (direction == FACE_ARRIERE)
//     {
//         direction = FACE_GAUCHE;
//     }
//     else if (direction == FACE_GAUCHE)
//     {
//         direction = FACE_AVANT;
//     }
//     ENCODER_Reset(1);
//     ENCODER_Reset(0);
//     return;
// }

// Ordre des orientations pour une rotation vers la droite.
static const int orientations[] = {FACE_AVANT, FACE_DROIT, FACE_ARRIERE, FACE_GAUCHE};

static int indiceOrientation(int orientation)
{
    for (int i = 0; i < 4; ++i)
    {
        if (orientations[i] == orientation) return i;
    }
    return -1;
}

void faitFace(int orientation)
{
    const int depart = indiceOrientation(direction);
    const int arrivee = indiceOrientation(orientation);
    if (depart < 0 || arrivee < 0 || depart == arrivee) return;

    const int quartsDeTour = (arrivee - depart + 4) % 4;
    if (quartsDeTour == 3)
    {
        pid_tourne(TOURNE_GAUCHE, 90);
    }
    else
    {
        pid_tourne(TOURNE_DROITE, quartsDeTour * 90);
    }
    direction = orientation;
}

void tourne(int directionDemande)
{
    const int depart = indiceOrientation(direction);
    if (depart < 0) return;

    if (directionDemande == TOURNE_DROITE)
    {
        faitFace(orientations[(depart + 1) % 4]);
    }
    else if (directionDemande == TOURNE_GAUCHE)
    {
        faitFace(orientations[(depart + 3) % 4]);
    }
}

void scan(void)
{
    static bool scan_droit;
    static bool scan_gauche;
    static bool scan_avant;

    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    if(SCAN_DEBUG){Serial.println("Moteur stop");}
    //delay(1000);
    if(SCAN_DEBUG){
        Serial.println("Appuyer sur le bouton pour continuer le scan");
        attendSerial();
    }

    if(SCAN_DEBUG){Serial.println("Scan avant");}
    if(SCAN_DEBUG){Serial.print("LED vert: "); Serial.println(digitalRead(vertpin));}
    if(SCAN_DEBUG){Serial.print("LED rouge: "); Serial.println(digitalRead(rougepin));}
    if(!digitalRead(rougepin) || !digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("objet detecte avant");}
        scan_avant = true;

        if(SCAN_DEBUG){
            Serial.println("Objet detecte avant, appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte avant");}
        scan_avant = false;
        
        if(SCAN_DEBUG){
            Serial.println("Pas d'objet detecte avant, appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
    }

    if(scan_avant == false)
    {
        if(SCAN_DEBUG){Serial.println("Choix: avancer");}
        
        if(SCAN_DEBUG){
            Serial.println("Fin scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
        return;
    }


    tourne(TOURNE_GAUCHE);
    if(SCAN_DEBUG){
        Serial.println("Fini de tourner a gauche, appuyer sur le bouton pour continuer le scan");
        attendSerial();
    }
    if(SCAN_DEBUG){Serial.println("Scan gauche");}
    delay(1000);
    if(digitalRead(rougepin) || digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte a gauche");}
        
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_gauche = false;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("objet detecte a gauche");}
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_gauche = true;
    }

    tourne(TOURNE_DROITE);
    tourne(TOURNE_DROITE);
    delay(1000);
    if(SCAN_DEBUG){Serial.println("Scan droit");}
    if(digitalRead(rougepin) || digitalRead(vertpin))
    {
        if(SCAN_DEBUG){Serial.println("pas d'objet detecte a droite");}
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_droit = false;
    }
    else
    {
        if(SCAN_DEBUG){Serial.println("objet detecte a droite");}
        if(SCAN_DEBUG){
            Serial.println("appuyer sur le bouton pour continuer le scan");
            attendSerial();
        }
        scan_droit = true;
    }

    tourne(TOURNE_GAUCHE);
    delay(500);

    if(!scan_gauche)
    {
        if(SCAN_DEBUG){Serial.println("Choix: tourner gauche");}
        tourne(TOURNE_GAUCHE);
        
        if(SCAN_DEBUG){
            Serial.println("fin du scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
    }
    else if(!scan_droit)
    {
        if(SCAN_DEBUG){Serial.println("Choix: tourner droit");}
        tourne(TOURNE_DROITE);
        if(SCAN_DEBUG){
            Serial.println("fin du scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
    }
    else{
        if(SCAN_DEBUG){Serial.println("Default face arriere");}
        tourne(TOURNE_GAUCHE);
        tourne(TOURNE_GAUCHE);
        if(SCAN_DEBUG){
            Serial.println("fin du scan, appuyer sur le bouton pour continuer");
            attendSerial();
        }
    }
}
