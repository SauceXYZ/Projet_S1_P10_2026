#include "mouvement.h"
#include "pid.h"
#include <LibRobus.h>
#include "main.h"

#ifndef AVANCE_V3_DEBUG
#define AVANCE_V3_DEBUG 1
#endif

static int ligne = 10;
static int colomne = 1;

// Tableau carte du parcours
// chaque bit represente une directuion possible (0 = libre, 1 = obstacle) (N, E, S, O)

//8 bloquer en haut
//4 bloquer a droite
//2 bloquer en bas
//1 bloquer a gauche

char grilleMap[11][3] =
{
    {9, 15, 12},
    {5, 15, 5},
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

int32_t encodeur_1_precedent = 0;

static void faitFace(int orientation);
static void tourne(int directionDemande);

static bool avanceImpulsions(int32_t distance, bool versCentre, bool finDeCase)
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
    }
    const int32_t debut0 = ENCODER_Read(0);
    pid_init();
    bool obstacleFinCase = finDeCase;
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

static int deplaceCase(void)
{
    const float centreX = (colomne - 1) * CASE_CM;
    const float centreY = (10 - ligne) * CASE_CM + CASE_CM / 2.0f;
    float distance_cm = CASE_CM;
    if (direction == FACE_AVANT) distance_cm = centreY + CASE_CM - lab_posy;
    else if (direction == FACE_ARRIERE) distance_cm = lab_posy - (centreY - CASE_CM);
    else if (direction == FACE_DROIT) distance_cm = centreX + CASE_CM - lab_posx;
    else if (direction == FACE_GAUCHE) distance_cm = lab_posx - (centreX - CASE_CM);
    if (distance_cm <= 0.0f) return -1;

    const int32_t distance = (int32_t)(distance_cm * (1.0f / CM_PAR_IMPULSION_1));
    const int32_t debut = ENCODER_Read(1);
    bool complete = avanceImpulsions(distance, true, false);
    int32_t parcouru = ENCODER_Read(1) - debut;
    float restant_cm = (distance - parcouru) * CM_PAR_IMPULSION_1;
    if (!complete && restant_cm <= TOLERANCE_CASE_CM)
    {
        if (AVANCE_V3_DEBUG || CALIBRATION_DEBUG)
        {
            Serial.print("Reprise apres freinage vers le centre, reste_cm=");
            Serial.println(restant_cm, 3);
        }
        if (distance > parcouru)
        {
            complete = avanceImpulsions(distance - parcouru, true, true);
        }
        else
        {
            complete = true;
        }
        parcouru = ENCODER_Read(1) - debut;
    }
    if (!complete)
    {
        if (AVANCE_V3_DEBUG) Serial.println("Deplacement interrompu: retour a la case precedente");
        if (parcouru > 0)
        {
            tourne(TOURNE_DROITE);
            tourne(TOURNE_DROITE);
            delay(100);
            if (!avanceImpulsions(parcouru, false, false))
            {
                return -1;
            }
        }
        return 0;
    }

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
    if (AVANCE_V3_DEBUG)
    {
        Serial.print("Position mesuree X (cm): "); Serial.print(lab_posx);
        Serial.print(" | Y (cm): "); Serial.println(lab_posy);
    }
    return 1;
}

void avanceV3(void)
{
    static unsigned int visites[11][3] = {};
    static bool impasses[11][3] = {};
    static bool debut = true;
    static bool termine = false;
    if (termine) return;

    if (debut)
    {
        visites[ligne][colomne] = 1;
        lab_posx = (colomne - 1) * CASE_CM;
        lab_posy = (10 - ligne) * CASE_CM + CASE_CM / 2.0f;
        encodeur_1_precedent = ENCODER_Read(1);
        debut = false;
    }
    if (ligne == 0)
    {
        if (colomne != 1)
        {
            if (colomne == 0) faitFace(FACE_DROIT);
            else faitFace(FACE_GAUCHE);
            delay(100);
            if (deplaceCase() != 1)
            {
                if (AVANCE_V3_DEBUG) Serial.println("Acces a la case (0,1) bloque");
                termine = true;
            }
            return;
        }
        MOTOR_SetSpeed(0, 0);
        MOTOR_SetSpeed(1, 0);
        if (AVANCE_V3_DEBUG) Serial.println("Arrivee atteinte");
        termine = true;
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

    for (int i = 0; i < 4; i++)
    {
        int prochaineLigne = ligne + changementLigne[i];
        int prochaineColomne = colomne + changementColomne[i];
        if (prochaineLigne < 0 || prochaineLigne >= 11 ||
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
            break;
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

static const int orientations[] = {FACE_AVANT, FACE_DROIT, FACE_ARRIERE, FACE_GAUCHE};

static int indiceOrientation(int orientation)
{
    for (int i = 0; i < 4; ++i)
    {
        if (orientations[i] == orientation) return i;
    }
    return -1;
}

static void faitFace(int orientation)
{
    const int depart = indiceOrientation(direction);
    const int arrivee = indiceOrientation(orientation);
    if (depart < 0 || arrivee < 0 || depart == arrivee) return;
    delay(200);
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
    delay(450);
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
