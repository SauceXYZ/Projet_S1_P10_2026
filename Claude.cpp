/*
Projet: Sortie de labyrinthe
Equipe: Equipe P10
Auteurs: Philippe, Nehla, Adam, Etienne
Description: Fichier unique, syntaxe C uniquement (pas de classes, de references,
             ni de Serial). Le robot calcule le chemin dans la carte puis l'execute
             case par case avec un PID. Demarrage : taper le bumper arriere.
*/

#include <LibRobus.h>
#include <stdbool.h>
#include <math.h>

/* =====================================================================
 *                        CONSTANTES
 * ===================================================================== */

/* --- Physique du robot --- */
#define CLICS_PAR_TOUR   3200.0
#define DIAMETRE_ROUE    7.62      /* cm */
#define EMPATTEMENT      18.5      /* cm, distance entre les roues (A CALIBRER) */
#define PI_VAL           3.14159
#define TAILLE_CASE_CM   50.0      /* une case = 0,5 m */

#define BUMPER_AVANT     2
#define BUMPER_ARRIERE   3

/* --- Profils de vitesse --- */
#define VIT_LIGNE_MAX        0.50
#define VIT_LIGNE_MIN        0.20   /* sous cette valeur les moteurs peuvent caler */
#define VIT_ROT_MAX          0.30
#define VIT_ROT_MIN          0.18
#define ACCEL_PAR_CYCLE      0.03   /* rampe d'acceleration par cycle de 20 ms */
#define ZONE_DECEL_LIGNE_CM  12.0   /* on ralentit sur les 12 derniers cm */
#define ZONE_DECEL_ROT       0.30   /* on ralentit sur les 30 % finaux du virage */

#define PERIODE_MS       20
#define DT               0.02       /* PERIODE_MS en secondes */

/* --- Labyrinthe (1,5 m x 5 m, cases de 0,5 m) --- */
#define LABY_LIGNES      10
#define LABY_COLONNES    3
#define MAX_PAS          30         /* LABY_LIGNES * LABY_COLONNES */

#define DEPART_LIGNE     9          /* bas du schema */
#define DEPART_COLONNE   1          /* colonne du milieu */

/* Bits des murs : 8 = haut (N), 4 = droite (E), 2 = bas (S), 1 = gauche (O) */
#define MUR_HAUT    8
#define MUR_DROITE  4
#define MUR_BAS     2
#define MUR_GAUCHE  1

/* Directions absolues (le nord = le haut du schema) */
#define NORD   0
#define EST    1
#define SUD    2
#define OUEST  3

/* =====================================================================
 *                        CARTOGRAPHIE
 * ===================================================================== */

/*
 * Chaque case = somme des murs qui l'entourent.
 * Ligne 0 = haut du schema (cases "Fin"), ligne 9 = bas ("Depart").
 * Les traits pleins ET pointilles du schema sont tous traites comme des murs.
 */
const int labyrinthe[LABY_LIGNES][LABY_COLONNES] = {
    {13, 15, 13},  /* 0  Fin   Fin   Fin */
    { 5,  9,  6},  /* 1 */
    { 7,  5, 13},  /* 2 */
    { 9,  6,  5},  /* 3 */
    { 5, 15,  5},  /* 4 */
    { 3, 12,  5},  /* 5 */
    {15,  5,  5},  /* 6 */
    { 9,  6,  5},  /* 7 */
    { 5, 15,  7},  /* 8 */
    { 3, 10, 14}   /* 9        Depart (colonne 1) */
};

/* Index : 0 = NORD, 1 = EST, 2 = SUD, 3 = OUEST */
const int DEPLACEMENT_LIGNE[4]   = {-1, 0, 1,  0};
const int DEPLACEMENT_COLONNE[4] = { 0, 1, 0, -1};
const int MASQUE_MUR[4]          = {MUR_HAUT, MUR_DROITE, MUR_BAS, MUR_GAUCHE};

bool estSortie(int ligne){
    return ligne == 0;
}

bool murPresent(int ligne, int colonne, int direction){
    return (labyrinthe[ligne][colonne] & MASQUE_MUR[direction]) != 0;
}

/*
 * Plus court chemin (BFS) du depart jusqu'a une case "Fin".
 * chemin[] recoit une direction par case a traverser, *nbPas le nombre de cases.
 * Retourne false si aucun chemin n'existe.
 */
bool trouverChemin(int ligneDepart, int colonneDepart, int chemin[], int *nbPas){
    int dirParent[LABY_LIGNES][LABY_COLONNES];
    bool visite[LABY_LIGNES][LABY_COLONNES];
    int file[MAX_PAS];
    int inverse[MAX_PAS];
    int tete = 0;
    int queue = 0;
    int sortieL = -1;
    int sortieC = -1;
    int l, c, d, n, i;

    for(l = 0; l < LABY_LIGNES; l++){
        for(c = 0; c < LABY_COLONNES; c++){
            dirParent[l][c] = -1;
            visite[l][c] = false;
        }
    }

    file[queue++] = ligneDepart * LABY_COLONNES + colonneDepart;
    visite[ligneDepart][colonneDepart] = true;

    while(tete < queue){
        int courant = file[tete++];
        l = courant / LABY_COLONNES;
        c = courant % LABY_COLONNES;

        if(estSortie(l)){
            sortieL = l;
            sortieC = c;
            break;
        }

        for(d = 0; d < 4; d++){
            int nl, nc;

            if(murPresent(l, c, d)) continue;

            nl = l + DEPLACEMENT_LIGNE[d];
            nc = c + DEPLACEMENT_COLONNE[d];

            if(nl < 0 || nl >= LABY_LIGNES || nc < 0 || nc >= LABY_COLONNES) continue;
            if(visite[nl][nc]) continue;

            visite[nl][nc] = true;
            dirParent[nl][nc] = d;
            file[queue++] = nl * LABY_COLONNES + nc;
        }
    }

    if(sortieL < 0) return false;

    /* Reconstruction a l'envers, puis inversion */
    n = 0;
    l = sortieL;
    c = sortieC;
    while(dirParent[l][c] != -1){
        d = dirParent[l][c];
        inverse[n++] = d;
        l -= DEPLACEMENT_LIGNE[d];
        c -= DEPLACEMENT_COLONNE[d];
    }
    for(i = 0; i < n; i++){
        chemin[i] = inverse[n - 1 - i];
    }
    *nbPas = n;
    return true;
}

/* Angle a tourner : +90 = droite, -90 = gauche, 180 = demi-tour, 0 = rien */
int angleVersDirection(int directionActuelle, int directionVoulue){
    int delta = (directionVoulue - directionActuelle + 4) % 4;
    if(delta == 1) return 90;
    if(delta == 2) return 180;
    if(delta == 3) return -90;
    return 0;
}

/* =====================================================================
 *                              PID
 * ===================================================================== */

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrale;
    float erreurPrecedente;
    float limiteIntegrale;   /* anti-windup */
} PID;

/* Gains de depart : a regler sur le robot */
PID pidLigne    = {0.0002, 0.0001, 0.00005, 0, 0, 500};  /* synchro des roues en ligne droite */
PID pidRotation = {0.0003, 0.0001, 0.00005, 0, 0, 500};  /* synchro des roues en rotation */

void pidReset(PID *p){
    p->integrale = 0;
    p->erreurPrecedente = 0;
}

float pidCalcul(PID *p, float erreur){
    float derivee;

    p->integrale += erreur * DT;
    if(p->integrale >  p->limiteIntegrale) p->integrale =  p->limiteIntegrale;
    if(p->integrale < -p->limiteIntegrale) p->integrale = -p->limiteIntegrale;

    derivee = (erreur - p->erreurPrecedente) / DT;
    p->erreurPrecedente = erreur;

    return p->kp * erreur + p->ki * p->integrale + p->kd * derivee;
}

/* =====================================================================
 *                        FONCTIONS UTILITAIRES
 * ===================================================================== */

float borner(float v, float vmin, float vmax){
    if(v < vmin) return vmin;
    if(v > vmax) return vmax;
    return v;
}

long cmEnClics(float cm){
    return (long)(cm * CLICS_PAR_TOUR / (PI_VAL * DIAMETRE_ROUE));
}

void beep(int count){
    int i;
    for(i = 0; i < count; i++){
        AX_BuzzerON();
        delay(100);
        AX_BuzzerOFF();
        delay(100);
    }
    delay(400);
}

void arret(void){
    MOTOR_SetSpeed(RIGHT, 0);
    MOTOR_SetSpeed(LEFT, 0);
}

void avance(float vitessedroit, float vitessegauche){
    MOTOR_SetSpeed(RIGHT, vitessedroit);
    MOTOR_SetSpeed(LEFT, vitessegauche);
}

/* Rampe d'acceleration + deceleration progressive a l'approche de la cible */
float profilVitesse(float actuelle, long restant, long zone, float vMin, float vMax){
    float cible = vMax;

    if(restant < zone){
        cible = vMin + (vMax - vMin) * ((float)restant / (float)zone);
    }
    if(actuelle < cible){
        actuelle += ACCEL_PAR_CYCLE;
        if(actuelle > cible) actuelle = cible;
    } else {
        actuelle = cible;
    }
    return actuelle;
}

/* =====================================================================
 *                        FONCTIONS DE MOUVEMENT
 * ===================================================================== */

/* Avance en ligne droite. Retourne true si la distance est atteinte,
   false si le bumper avant a touche un mur. */
bool avancerDistance(float distanceCm){
    long cible = cmEnClics(distanceCm);
    long zone  = cmEnClics(ZONE_DECEL_LIGNE_CM);
    float vitesse = VIT_LIGNE_MIN;
    long g, d, restant;
    float correction;

    ENCODER_ReadReset(LEFT);
    ENCODER_ReadReset(RIGHT);
    pidReset(&pidLigne);

    while(true){
        g = ENCODER_Read(LEFT);
        d = ENCODER_Read(RIGHT);
        restant = cible - (g + d) / 2;

        if(restant <= 0) break;

        if(ROBUS_IsBumper(BUMPER_AVANT)){
            arret();
            return false;
        }

        vitesse = profilVitesse(vitesse, restant, zone, VIT_LIGNE_MIN, VIT_LIGNE_MAX);

        /* erreur > 0 : la roue droite est en avance -> on la ralentit */
        correction = pidCalcul(&pidLigne, (float)(d - g));

        avance(borner(vitesse - correction, 0.0, 1.0),
               borner(vitesse + correction, 0.0, 1.0));

        delay(PERIODE_MS);
    }

    arret();
    return true;
}

/* Tourne sur place. angle > 0 : droite (horaire), angle < 0 : gauche. */
void tournerAngle(float angle){
    float sens;
    long cible, zone, g, d, restant;
    float vitesse, correction, vG, vD;

    if(angle == 0) return;

    sens = (angle > 0) ? 1.0 : -1.0;

    /* Arc par roue = PI * EMPATTEMENT * angle / 360 (le PI se simplifie) */
    cible = (long)(EMPATTEMENT * fabs(angle) / 360.0 * CLICS_PAR_TOUR / DIAMETRE_ROUE);
    zone  = (long)(cible * ZONE_DECEL_ROT);
    vitesse = VIT_ROT_MIN;

    ENCODER_ReadReset(LEFT);
    ENCODER_ReadReset(RIGHT);
    pidReset(&pidRotation);

    while(true){
        g = labs(ENCODER_Read(LEFT));
        d = labs(ENCODER_Read(RIGHT));
        restant = cible - (g + d) / 2;

        if(restant <= 0) break;

        vitesse = profilVitesse(vitesse, restant, zone, VIT_ROT_MIN, VIT_ROT_MAX);

        /* erreur > 0 : la roue gauche est en avance -> on la ralentit */
        correction = pidCalcul(&pidRotation, (float)(g - d));
        vG = borner(vitesse - correction, 0.0, 1.0);
        vD = borner(vitesse + correction, 0.0, 1.0);

        MOTOR_SetSpeed(LEFT,   sens * vG);
        MOTOR_SetSpeed(RIGHT, -sens * vD);

        delay(PERIODE_MS);
    }

    arret();
}

/* Oriente le robot vers directionVoulue et retourne la nouvelle direction */
int tournerVers(int directionActuelle, int directionVoulue){
    int angle = angleVersDirection(directionActuelle, directionVoulue);

    if(angle != 0){
        tournerAngle(angle);
        delay(200);   /* laisse le robot se stabiliser */
    }
    return directionVoulue;
}

/* =====================================================================
 *                        EXECUTION DU PARCOURS
 * ===================================================================== */

bool executerParcours(void){
    int chemin[MAX_PAS];
    int nbPas = 0;
    int direction = NORD;   /* le robot demarre face au nord (vers les cases "Fin") */
    int i;

    if(!trouverChemin(DEPART_LIGNE, DEPART_COLONNE, chemin, &nbPas)){
        beep(5);   /* aucun chemin dans la carte */
        return false;
    }

    for(i = 0; i < nbPas; i++){
        direction = tournerVers(direction, chemin[i]);

        if(!avancerDistance(TAILLE_CASE_CM)){
            beep(5);   /* collision : arret */
            return false;
        }

        delay(200);
    }

    beep(2);   /* sortie du labyrinthe */
    return true;
}

/* =====================================================================
 *                           SETUP / LOOP
 * ===================================================================== */

void setup(){
    BoardInit();
    delay(100);
    beep(3);   /* le robot est pret */
}

void loop(){
    /* Un coup sur le bumper arriere lance le parcours */
    if(ROBUS_IsBumper(BUMPER_ARRIERE)){
        while(ROBUS_IsBumper(BUMPER_ARRIERE)) delay(10);   /* attend le relachement */
        delay(1000);                                        /* temps de retirer la main */
        executerParcours();
    }
    delay(10);
}
