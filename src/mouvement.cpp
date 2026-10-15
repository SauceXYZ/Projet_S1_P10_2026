#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"

#define SCAN_DEBUG 0
#ifndef AVANCE_V3_DEBUG
#define AVANCE_V3_DEBUG 0
#endif

static int ligne = 9;
static int colomne = 1;

// Tableau carte du parcours
// chaque bit represente une directuion possible (0 = libre, 1 = obstacle) (N, E, S, O)

//8 bloquer en haut
//4 bloquer a droite
//2 bloquer en bas
//1 bloquer a gauche

char grilleMap[10][3] =
{
    {13, 15, 13},
    {1, 10, 4},
    {5, 15, 5},
    {1, 8, 4},
    {5, 5, 5},
    {1, 2, 4},
    {5, 15, 5},
    {1, 10, 4},
    {5, 15, 5},
    {3, 10, 6}
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
    while (Serial.available() > 0)
    {
        Serial.read();
    }

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
    compte = ENCODER_Read(1); 
    distance_enc_cm = (compte - encodeur_1_precedent) * CM_PAR_IMPULSION_1;
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

static bool avanceImpulsions(int32_t distance, bool versCentre = false)
{
    const int32_t debut = ENCODER_Read(1);
    if (AVANCE_V3_DEBUG)
    {
        Serial.print("Case: ligne "); Serial.print(ligne);
        Serial.print(" | colonne "); Serial.println(colomne);
        Serial.print("Direction: "); Serial.println(direction);
        Serial.print("Encodeur 1 debut: "); Serial.println(debut);
        Serial.print("Encodeur 1 precedent: "); Serial.println(encodeur_1_precedent);
        Serial.print("Cible en impulsions: "); Serial.println(distance);
        Serial.println("Envoyer un caractere pour commencer le deplacement");
        attendSerial();
    }
    const int32_t debut0 = ENCODER_Read(0);
    pid_init();
    bool obstacleFinCase = false;
    bool freinageAnticipe = false;
    float anticipation_cm = 0.0f;
    while (ENCODER_Read(1) - debut < distance)
    {
        float restant_cm = (distance - (ENCODER_Read(1) - debut)) * CM_PAR_IMPULSION_1;
        if (!digitalRead(vertpin) || !digitalRead(rougepin))
        {
            if (!versCentre || restant_cm > TOLERANCE_CASE_CM) break;
            obstacleFinCase = true;
        }
        anticipation_cm = pid_distanceArret();
        float maximumAnticipation = distance * CM_PAR_IMPULSION_1 / 4.0f;
        if (anticipation_cm > maximumAnticipation) anticipation_cm = maximumAnticipation;
        if (restant_cm <= anticipation_cm)
        {
            freinageAnticipe = true;
            break;
        }
        float vitesseCible = VITESSE_REFERENCE;
        if (restant_cm < 15.0f)
        {
            vitesseCible *= restant_cm / 15.0f;
            if (vitesseCible < 0.15f) vitesseCible = 0.15f;
        }
        if (obstacleFinCase) vitesseCible = 0.15f;
        pid_avance(vitesseCible);
        delay(1);
    }
    const int32_t avantArret = ENCODER_Read(1);
    bool cibleAtteinteAvantArret = avantArret - debut >= distance;
    bool complete = cibleAtteinteAvantArret || freinageAnticipe;
    pid_arret();
    delay(100);
    encodeur_1_precedent = ENCODER_Read(1);
    const float parcouru_cm = (encodeur_1_precedent - debut) * CM_PAR_IMPULSION_1;
    if (direction == FACE_AVANT) lab_posy += parcouru_cm;
    else if (direction == FACE_ARRIERE) lab_posy -= parcouru_cm;
    else if (direction == FACE_DROIT) lab_posx += parcouru_cm;
    else if (direction == FACE_GAUCHE) lab_posx -= parcouru_cm;
    if (AVANCE_V3_DEBUG)
    {
        Serial.print("Encodeur 1 fin: "); Serial.println(encodeur_1_precedent);
        Serial.print("Impulsions parcourues: "); Serial.println(encodeur_1_precedent - debut);
        Serial.print("Distance mesuree (cm): ");
        Serial.println((encodeur_1_precedent - debut) * CM_PAR_IMPULSION_1);
        Serial.print("Capteur vert: "); Serial.println(digitalRead(vertpin));
        Serial.print("Capteur rouge: "); Serial.println(digitalRead(rougepin));
        Serial.print("Distance complete: "); Serial.println(complete);
        Serial.print("Obstacle detecte pendant la fin de case: "); Serial.println(obstacleFinCase);
    }
    if (CALIBRATION_DEBUG)
    {
        const int32_t pulses0 = ENCODER_Read(0) - debut0;
        const int32_t pulses1 = encodeur_1_precedent - debut;
        const float distance0 = pulses0 * CM_PAR_IMPULSION_0;
        const float distance1 = pulses1 * CM_PAR_IMPULSION_1;
        Serial.print("CAL AVANCE ligne="); Serial.print(ligne);
        Serial.print(" colonne="); Serial.print(colomne);
        Serial.print(" direction="); Serial.print(direction);
        Serial.print(" cible_cm="); Serial.print(distance * CM_PAR_IMPULSION_1, 3);
        Serial.print(" pulses0="); Serial.print(pulses0);
        Serial.print(" pulses1="); Serial.print(pulses1);
        Serial.print(" distance0_cm="); Serial.print(distance0, 3);
        Serial.print(" distance1_cm="); Serial.print(distance1, 3);
        Serial.print(" ecart_roues_cm="); Serial.print(distance0 - distance1, 3);
        Serial.print(" freinage_et_inertie_cm="); Serial.print((encodeur_1_precedent - avantArret) * CM_PAR_IMPULSION_1, 3);
        Serial.print(" erreur_cible_cm="); Serial.print(distance1 - distance * CM_PAR_IMPULSION_1, 3);
        Serial.print(" obstacle_fin_case="); Serial.print(obstacleFinCase);
        Serial.print(" anticipation_cm="); Serial.print(anticipation_cm, 3);
        Serial.print(" freinage_anticipe="); Serial.print(freinageAnticipe);
        Serial.print(" cible_atteinte_avant_arret="); Serial.println(cibleAtteinteAvantArret);
    }
    return complete;
}

int deplaceCase(void)
{
    const float centreX = (colomne - 1) * CASE_CM;
    const float centreY = (9 - ligne) * CASE_CM + CASE_CM / 2.0f;
    float distance_cm = CASE_CM;
    if (direction == FACE_AVANT) distance_cm = centreY + CASE_CM - lab_posy;
    else if (direction == FACE_ARRIERE) distance_cm = lab_posy - (centreY - CASE_CM);
    else if (direction == FACE_DROIT) distance_cm = centreX + CASE_CM - lab_posx;
    else if (direction == FACE_GAUCHE) distance_cm = lab_posx - (centreX - CASE_CM);
    if (distance_cm <= 0.0f) return -1;

    const int32_t distance = (int32_t)(distance_cm * (1.0f / CM_PAR_IMPULSION_1));
    const int32_t debut = ENCODER_Read(1);
    bool complete = avanceImpulsions(distance, true);
    const int32_t parcouru = ENCODER_Read(1) - debut;
    if (!complete)
    {
        if (AVANCE_V3_DEBUG) Serial.println("Deplacement interrompu: retour a la case precedente");
        if (parcouru > 0)
        {
            tourne(TOURNE_DROITE);
            tourne(TOURNE_DROITE);
            delay(100);
            if (!avanceImpulsions(parcouru))
            {
                return -1;
            }
        }
        return 0;
    }

    positionRobot[ligne][colomne] = false;
    if (direction == FACE_AVANT)
    {
        ligne -= 1;
    }
    else if (direction == FACE_ARRIERE)
    {
        ligne += 1;
    }
    else if (direction == FACE_DROIT)
    {
        colomne += 1;
    }
    else if (direction == FACE_GAUCHE)
    {
        colomne -= 1;
    }
    positionRobot[ligne][colomne] = true;
    if (AVANCE_V3_DEBUG)
    {
        Serial.print("Position mesuree X (cm): "); Serial.print(lab_posx);
        Serial.print(" | Y (cm): "); Serial.println(lab_posy);
    }
    return 1;
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

// void avanceV2(void)
// {
//     int vert = digitalRead(vertpin);
//     int rouge = digitalRead(rougepin);

//     if(checkBoundary() == LIMIT_NORD)
//     {
//         faitFace(FACE_ARRIERE);
//         vert = digitalRead(vertpin);
//         rouge = digitalRead(rougepin);
//     }

//     if(checkBoundary() == LIMIT_OUEST)
//     {
//         faitFace(FACE_DROIT);
//         vert = digitalRead(vertpin);
//         rouge = digitalRead(rougepin);
//     }
//     else if(checkBoundary() == LIMIT_EST)
//     {
//         faitFace(FACE_GAUCHE);
//         vert = digitalRead(vertpin);
//         rouge = digitalRead(rougepin);
//     }

//     if(vert && rouge)
//     {
//         deplace(case_cm);
//         //delay(1000);
//     }
//     else
//     {
//         if (lab_posx < 0)
//         {
//             tourne(TOURNE_DROITE);
//             vert = digitalRead(vertpin);
//             rouge = digitalRead(rougepin);
//             if(vert && rouge)
//             {
//                 deplace(case_cm);
//                 faitFace(FACE_AVANT);
//             }
//             else
//             {
//                 tourne(TOURNE_GAUCHE);
//                 tourne(TOURNE_GAUCHE);
//                 vert = digitalRead(vertpin);
//                 rouge = digitalRead(rougepin);
//                 if(vert && rouge)
//                 {
//                     deplace(case_cm);
//                     faitFace(FACE_AVANT);
//                 }
//                 else
//                 {
//                     MOTOR_SetSpeed(0, 0);
//                     MOTOR_SetSpeed(1, 0);
//                 }
//             }
//         }
//         else
//         {
//             tourne(TOURNE_GAUCHE);
//             vert = digitalRead(vertpin);
//             rouge = digitalRead(rougepin);
//             if(vert && rouge)
//             {
//                 deplace(case_cm);
//                 faitFace(FACE_AVANT);
//             }
//             else
//             {
//                 tourne(TOURNE_DROITE);
//                 tourne(TOURNE_DROITE);
//                 vert = digitalRead(vertpin);
//                 rouge = digitalRead(rougepin);
//                 if(vert && rouge)
//                 {
//                     deplace(case_cm);
//                     faitFace(FACE_AVANT);
//                 }
//                 else
//                 {
//                     MOTOR_SetSpeed(0, 0);
//                     MOTOR_SetSpeed(1, 0);
//                 }
//             }
//         }
//     }

    
// }


/*
Avance dans le parcours en regardant les possibilites dans le tableau de la carte
pseudo code

regarde la position du robot dans le tableau de la carte
regarde les possibilites de deplacement dans le tableau de la carte
si possible de deplacer vers l'avant
    avance d'une case
sinon si possible de deplacer vers la gauche
    tourne a gauche
    avance d'une case
    garde cette orientation
sinon si possible de deplacer vers la droite
    tourne a droite
    avance d'une case
    garde cette orientation
sinon si objet en avant et impossible gauche droit
    face arriere 
    avance une case

*/
void avanceV3(void)
{
    static unsigned int visites[10][3] = {};
    static bool impasses[10][3] = {};
    static bool debut = true;
    static bool termine = false;
    static bool retour = false;
    static uint8_t chemin[30]; // Une case = ligne * 3 + colonne.
    static int longueurChemin = 0;
    if (termine) return;

    if (debut)
    {
        visites[ligne][colomne] = 1;
        chemin[0] = ligne * 3 + colomne;
        longueurChemin = 1;
        lab_posx = (colomne - 1) * CASE_CM;
        lab_posy = (9 - ligne) * CASE_CM + CASE_CM / 2.0f;
        encodeur_1_precedent = ENCODER_Read(1);
        debut = false;
    }
    if (retour)
    {
        if (longueurChemin <= 1)
        {
            MOTOR_SetSpeed(0, 0);
            MOTOR_SetSpeed(1, 0);
            if (AVANCE_V3_DEBUG) Serial.println("Retour au depart termine");
            termine = true;
            return;
        }
        int prochaineLigne = chemin[longueurChemin - 2] / 3;
        int prochaineColomne = chemin[longueurChemin - 2] % 3;
        if (AVANCE_V3_DEBUG)
        {
            Serial.print("Retour vers ligne "); Serial.print(prochaineLigne);
            Serial.print(" | colonne "); Serial.println(prochaineColomne);
        }
        if (prochaineLigne < ligne) faitFace(FACE_AVANT);
        else if (prochaineLigne > ligne) faitFace(FACE_ARRIERE);
        else if (prochaineColomne < colomne) faitFace(FACE_GAUCHE);
        else faitFace(FACE_DROIT);
        delay(100);

        if (deplaceCase() == 1)
        {
            longueurChemin -= 1;
        }
        else
        {
            MOTOR_SetSpeed(0, 0);
            MOTOR_SetSpeed(1, 0);
            if (AVANCE_V3_DEBUG) Serial.println("Retour bloque sur le chemin memorise");
            termine = true;
        }
        return;
    }
    if (ligne == 0)
    {
        MOTOR_SetSpeed(0, 0);
        MOTOR_SetSpeed(1, 0);
        if (AVANCE_V3_DEBUG) Serial.println("Arrivee atteinte");
        if (RETOUR_ACTIF)
        {
            retour = true;
        }
        else
        {
            termine = true;
        }
        return;
    }

    if (AVANCE_V3_DEBUG)
    {
        Serial.println("--- Nouvelle decision V3 ---");
        Serial.print("Ligne: "); Serial.print(ligne);
        Serial.print(" | colonne: "); Serial.println(colomne);
        Serial.print("Murs carte (8=N, 4=E, 2=S, 1=O): ");
        Serial.println((int)grilleMap[ligne][colomne]);
    }

    const int faces[4] = {FACE_AVANT, FACE_GAUCHE, FACE_DROIT, FACE_ARRIERE};
    const int bits[4] = {8, 1, 4, 2};
    const int bitsOpposes[4] = {2, 4, 1, 8};
    const int changementLigne[4] = {-1, 0, 0, 1};
    const int changementColomne[4] = {0, -1, 1, 0};
    int choix = -1;
    bool possibles[4] = {false, false, false, false};
    int sorties = 0;

    // Lire la carte sans tourner : seules les directions utiles seront scannees.
    for (int i = 0; i < 4; i++)
    {
        int prochaineLigne = ligne + changementLigne[i];
        int prochaineColomne = colomne + changementColomne[i];
        if (prochaineLigne < 0 || prochaineLigne >= 10 ||
            prochaineColomne < 0 || prochaineColomne >= 3)
        {
            continue;
        }
        if ((grilleMap[ligne][colomne] & bits[i]) ||
            (grilleMap[prochaineLigne][prochaineColomne] & bitsOpposes[i]) ||
            impasses[prochaineLigne][prochaineColomne])
        {
            continue;
        }
        possibles[i] = true;
        sorties += 1;
    }

    // Nord, cote le moins visite, autre cote, puis sud.
    int ordre[4] = {0, 1, 2, 3};
    if (possibles[1] && possibles[2] &&
        visites[ligne][colomne + 1] < visites[ligne][colomne - 1])
    {
        ordre[1] = 2;
        ordre[2] = 1;
    }
    for (int scan = 0; scan < 4; scan++)
    {
        int i = ordre[scan];
        if (!possibles[i]) continue;
        int prochaineLigne = ligne + changementLigne[i];
        int prochaineColomne = colomne + changementColomne[i];
        faitFace(faces[i]);
        delay(100);
        int vert = digitalRead(vertpin);
        int rouge = digitalRead(rougepin);
        if (AVANCE_V3_DEBUG)
        {
            Serial.print("Scan direction "); Serial.print(faces[i]);
            Serial.print(" | vert: "); Serial.print(vert);
            Serial.print(" | rouge: "); Serial.println(rouge);
        }
        if (vert && rouge)
        {
            choix = i;
            break; // Ne pas tourner vers les autres passages si celui-ci convient.
        }
        grilleMap[ligne][colomne] |= bits[i];
        grilleMap[prochaineLigne][prochaineColomne] |= bitsOpposes[i];
        sorties -= 1;
    }

    if (choix == -1)
    {
        MOTOR_SetSpeed(0, 0);
        MOTOR_SetSpeed(1, 0);
        if (AVANCE_V3_DEBUG) Serial.println("Aucun passage disponible");
        termine = true;
        return;
    }

    // Une case avec une seule sortie deja visitee est une impasse.
    bool impasse = sorties == 1 &&
        visites[ligne + changementLigne[choix]][colomne + changementColomne[choix]] > 0;

    if (AVANCE_V3_DEBUG)
    {
        Serial.print("Choix (0=E, 1=O, 2=N, 3=S): "); Serial.println(faces[choix]);
    }
    int ancienneLigne = ligne;
    int ancienneColomne = colomne;
    int resultat = deplaceCase();
    if (AVANCE_V3_DEBUG)
    {
        Serial.print("Resultat deplaceCase (1=atteinte, 0=retour, -1=echec): ");
        Serial.println(resultat);
        Serial.print("Position apres mouvement: ligne "); Serial.print(ligne);
        Serial.print(" | colonne "); Serial.println(colomne);
    }
    if (resultat == 1)
    {
        if (impasse)
        {
            impasses[ancienneLigne][ancienneColomne] = true;
        }
        visites[ligne][colomne] += 1;
        // Si une case est revisitee, enlever la boucle et les detours du chemin.
        int caseActuelle = ligne * 3 + colomne;
        int indice = 0;
        while (indice < longueurChemin && chemin[indice] != caseActuelle)
        {
            indice += 1;
        }
        chemin[indice] = caseActuelle;
        longueurChemin = indice + 1;
    }
    else if (resultat == 0)
    {
        grilleMap[ancienneLigne][ancienneColomne] |= bits[choix];
        grilleMap[ancienneLigne + changementLigne[choix]][ancienneColomne + changementColomne[choix]] |= bitsOpposes[choix];
    }
    else
    {
        if (AVANCE_V3_DEBUG) Serial.println("Retour a la case impossible");
        termine = true;
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
