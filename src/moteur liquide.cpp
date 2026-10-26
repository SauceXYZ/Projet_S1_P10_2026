#include "moteur liquide.h"
#include <Arduino.h>

// ----------- Réglages matériels -----------
// Broches PWM du Mega : 2 à 13 et 44 à 46 (PAS 22, 24...)
static const int BROCHE_POMPE[2] = {9, 10};
static const char* NOM_BOISSON[2] = {"Boisson A", "Boisson B"};

// ----------- Réglages de dosage (à calibrer) -----------
static const float DEBIT_MAX_ML_MIN = 200.0;     // débit à pleine vitesse (PWM 255), à mesurer
static const float VOLUME_RESERVOIR_ML = 125.0;

// ----------- Réglages des rampes -----------
static const int DUTY_MIN = 80;                       // PWM minimal où la pompe démarre (à tester)
static const unsigned long RAMPE_MONTEE_MS = 1000;    // durée de l'accélération
static const float RAMPE_DESCENTE_S = 1.0;            // durée approximative de la décélération
static const unsigned long PAS_MS = 10;               // pas de calcul

// ----------- Niveau estimé de chaque réservoir -----------
// Remis à 125 ml à chaque redémarrage de l'Arduino
static float niveau_ml[2] = {VOLUME_RESERVOIR_ML, VOLUME_RESERVOIR_ML};

void moteur_liquide_init() {
  for (int i = 0; i < 2; i++) {
    pinMode(BROCHE_POMPE[i], OUTPUT);
    analogWrite(BROCHE_POMPE[i], 0);   // pompes arrêtées au démarrage
  }
}

// Sert un volume (ml) avec démarrage et arrêt progressifs
static void servir(int index, float volume_ml) {
  int broche = BROCHE_POMPE[index];
  float debit_ml_s = DEBIT_MAX_ML_MIN / 60.0;

  // Volume versé pendant la décélération (rampe linéaire -> moitié du débit max)
  float volume_descente = debit_ml_s * RAMPE_DESCENTE_S * 0.5;

  float servi = 0.0;
  unsigned long debut = millis();

  while (servi < volume_ml) {
    unsigned long t = millis() - debut;

    // 0 -> 1 pendant la montée
    float montee = min(1.0f, (float)t / RAMPE_MONTEE_MS);
    // 1 -> 0 quand il reste peu de liquide à verser
    float descente = min(1.0f, (volume_ml - servi) / volume_descente);

    float facteur = min(montee, descente);
    int duty = max(DUTY_MIN, (int)(255 * facteur));
    analogWrite(broche, duty);

    // Volume pompé pendant ce pas (débit proportionnel au PWM)
    servi += debit_ml_s * (duty / 255.0f) * (PAS_MS / 1000.0f);
    delay(PAS_MS);
  }

  analogWrite(broche, 0);   // arrêt
}

bool servir_si_possible(int index, float volume_ml) {
  if (index < 0 || index > 1) {
    Serial.println("Erreur : pompe invalide");
    return false;
  }
  if (volume_ml <= 0 || volume_ml > VOLUME_RESERVOIR_ML) {
    Serial.println("Erreur : volume invalide");
    return false;
  }
  if (volume_ml > niveau_ml[index]) {
    Serial.print("Reservoir ");
    Serial.print(NOM_BOISSON[index]);
    Serial.println(" insuffisant");
    return false;
  }

  Serial.print("Service de ");
  Serial.print(volume_ml);
  Serial.print(" ml : ");
  Serial.println(NOM_BOISSON[index]);

  servir(index, volume_ml);
  niveau_ml[index] -= volume_ml;

  Serial.print("Termine. Reste environ ");
  Serial.print(niveau_ml[index]);
  Serial.println(" ml");
  return true;
}

void remplir_reservoir(int index) {
  if (index == 0 || index == 1) {
    niveau_ml[index] = VOLUME_RESERVOIR_ML;
  }
}

float niveau_reservoir(int index) {
  if (index == 0 || index == 1) return niveau_ml[index];
  return 0.0;
}