#ifndef TROU_CONFIG_H
#define TROU_CONFIG_H

// Hole-only tuning. Normal line-following thresholds stay in suiveurPID.cpp.
namespace ReglageTrou {
const unsigned long PAUSE_MS = 500;
const float VITESSE_MANOEUVRE = 0.10f;
const float ACCELERATION = 0.20f; // 0 -> 0.10 in 0.5 s, forward and reverse
const float VITESSE_TRAVERSEE = 0.20f;
const unsigned long RECUL_MAX_MS = 4000;
const unsigned long ALIGNEMENT_MAX_MS = 4000;
const unsigned long TRAVERSEE_MAX_MS = 10000; // includes arrival search
const unsigned long T_CONFIRMATION_MS = 20;
const unsigned long PREMIERE_EXTREMITE_MS = 100;
// Provisional departure T thresholds: real bar was about 634 left / 802 right,
// while the false right detections were around 515--570. Same hysteresis as tracking.
const int SEUIL_T_EXTREMITE[2] = {500, 600}; // left, right
const unsigned long PERTE_EXTREMITE_MAX_MS = 500; // pause motors during a short dropout
const unsigned long CENTRE_CONFIRMATION_MS = 100;
const unsigned long VIDE_CONFIRMATION_MS = 50;
const unsigned long ARRIVEE_CONFIRMATION_MS = 50;

// PROVISIONAL: user estimates gap as 2.5 shoe lengths; no exact measurement yet.
// Measured from the confirmed gap, using the lesser of both encoder distances.
const float DISTANCE_MIN_ARRIVEE_CM = 40.0f;
// Any ONE sensor reaching its own threshold can confirm arrival.
// 550 rejects the reported false arrival at 337; real arrival tape is unmeasured.
const int SEUIL_ARRIVEE[3] = {550, 550, 550}; // left, centre, right

// Trial direction: move the wheel on the same named side as the first sensor.
// Flip to -1 to move the opposite wheel.
const int SENS_ALIGNEMENT = 1;
}
#endif
