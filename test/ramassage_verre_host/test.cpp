#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
#include "LibRobus.h"
#include "../../src/main.h"

SerialStub Serial;
struct Rotation { bool sens; int angle; };
std::vector<Rotation> rotations;
std::vector<int32_t> trajets;
std::vector<float> vitesses;
std::vector<uint16_t> lectures;
unsigned int indexLecture = 0;
int prises = 0;
double angleRobot = 0;
double posX = 0, posY = 0;
double encodeurs[2] = {0, 0};
bool depassementReel = false;
const double PI = 3.141592653589793;

int32_t ENCODER_Read(uint8_t moteur) { return static_cast<int32_t>(lround(encodeurs[moteur])); }
uint16_t ROBUS_ReadIR(uint8_t) {
    assert(indexLecture < lectures.size());
    return lectures[indexLecture++];
}
void delay(unsigned long) {}
void initServo() {}
void dropServo() {}
void grabServo() { ++prises; }
void pid_tourne(bool sens, int angle) {
    assert(angle > 0 && angle <= 180);
    rotations.push_back({sens, angle});
    double realise = angle;
    if (depassementReel) {
        if (angle == 5) realise = 255.14 / 39;
        else if (angle == 43) realise += 1.89;
        else if (angle == 180) realise += 2.03;
        else realise += .95;
    }
    const double signe = sens == TOURNE_DROITE ? 1 : -1;
    const double arcCm = realise * (DISTANCEENTREROUES_MM * .1) * PI / 360;
    encodeurs[0] += signe * arcCm / CM_PAR_IMPULSION_0;
    encodeurs[1] -= signe * arcCm / CM_PAR_IMPULSION_1;
    angleRobot += signe * realise;
}
bool avanceImpulsions(int32_t impulsions, bool, bool, float vitesseMax) {
    vitesses.push_back(vitesseMax);
    assert(impulsions > 0);
    trajets.push_back(impulsions);
    const double cible = impulsions * CM_PAR_IMPULSION_1;
    // Include the two observed straight-line overshoots and wheel mismatch.
    const bool aller = trajets.size() == 1;
    const double droite = cible + (depassementReel ? (aller ? .830 : .666) : 0);
    const double gauche = droite + (depassementReel ? (aller ? .576 : .180) : 0);
    const double lacet = (gauche - droite) * 180 / (DISTANCEENTREROUES_MM * .1 * PI);
    const double capMilieu = (angleRobot + lacet / 2) * PI / 180;
    posX += (gauche + droite) / 2 * cos(capMilieu);
    posY += (gauche + droite) / 2 * sin(capMilieu);
    angleRobot += lacet;
    encodeurs[0] += gauche / CM_PAR_IMPULSION_0;
    encodeurs[1] += droite / CM_PAR_IMPULSION_1;
    return true;
}
#include "../../src/ramassageVerre.cpp"

bool memeOrientation(double a, double b, double tolerance = .1) {
    return fabs(remainder(a - b, 360.0)) <= tolerance;
}
void resetPose(double cap) {
    rotations.clear(); trajets.clear(); vitesses.clear(); prises = 0;
    posX = 12.3; posY = -45.6; angleRobot = cap;
    encodeurs[0] = 10000; encodeurs[1] = -20000;
}
void simulerScan(int pas) {
    memoriserDepartScan();
    compteurTounreVerre = pas;
    for (int i = 0; i < pas; ++i) pid_tourne(TOURNE_DROITE, 5);
    rotations.clear();
}
int main() {
    for (int pas = 0; pas <= 144; ++pas) {
        resetPose(37);
        simulerScan(pas);
        grabDodo(17.75f);
        assert(memeOrientation(angleRobot, 37, TOLERANCE_RETOUR_DEGRES + .1));
        assert(hypot(posX - 12.3, posY + 45.6) < .00001);
        assert(prises == 1 && trajets.size() == 2 && trajets[0] == trajets[1]);
        assert(trajets[0] == cmEnImpulsions(15.75f));
        assert(vitesses.size() == 2 && vitesses[0] == VITESSE_APPROCHE_VERRE && vitesses[1] == VITESSE_APPROCHE_VERRE);
        assert(rotations[0].sens == TOURNE_DROITE && rotations[0].angle == 43);
        assert(rotations[1].sens == TOURNE_DROITE && rotations[1].angle == 180);
        if (pas == 38) assert(rotations[2].sens == TOURNE_GAUCHE && rotations[2].angle == 53);
        if (pas == 70) assert(rotations[2].sens == TOURNE_DROITE && rotations[2].angle == 147);
    }
    // Regression from the log: requested scan 195 degrees, measured 255.14.
    depassementReel = true;
    resetPose(37);
    simulerScan(39);
    grabDodo(18.99f);
    assert(memeOrientation(angleRobot, 37, TOLERANCE_RETOUR_DEGRES + .1));
    assert(rotations[2].sens == TOURNE_GAUCHE && rotations[2].angle > 115 && rotations[2].angle < 135);
    assert(rotations.size() <= 2 + MAX_CORRECTIONS_RETOUR);
    assert(hypot(posX - 12.3, posY + 45.6) < 2); // residual from U-turn/linear errors remains

    for (int pas : {38, 0, 70}) {
        resetPose(37);
        lectures.assign(pas * NB_LECTURES_VERRE, 100);
        lectures.insert(lectures.end(), NB_LECTURES_VERRE, 400);
        indexLecture = 0;
        EpreuveVerre();
        assert(compteurTounreVerre == pas);
        assert(memeOrientation(angleRobot, 37, TOLERANCE_RETOUR_DEGRES + .1));
        assert(indexLecture == lectures.size());
    }
    float mesure = 0;
    // Stable median despite two extreme readings on opposite sides.
    lectures = {400, 402, 398, 401, 399, 700, 250};
    indexLecture = 0;
    assert(mesurerDistanceVerre(3, mesure));
    assert(fabs(mesure - 4235.0f / 353) < .001f);
    // Invalid denominator, out-of-range samples and inconsistent windows are rejected.
    lectures = {0, 47, 40, 100, 400, 400, 65535};
    indexLecture = 0;
    assert(!mesurerDistanceVerre(3, mesure));
    lectures = {230, 240, 250, 400, 450, 550, 650};
    indexLecture = 0;
    assert(!mesurerDistanceVerre(3, mesure));
    std::cout << "Glass return: ideal paths, logged scan/turn/linear overshoot and repeated scans passed\n";
}
