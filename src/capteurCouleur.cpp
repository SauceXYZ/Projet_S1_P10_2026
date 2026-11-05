#include "capteurCouleur.h"
#include <Adafruit_TCS34725.h>

#define DEBUG_CAPTEUR_COULEUR true

Adafruit_TCS34725 capteur(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);

bool initialiser()
{
    return capteur.begin();
}


void lireRGB(int &rouge,int &vert,int &bleu, int &luminosite)
{

    uint16_t r;
    uint16_t v;
    uint16_t b;
    uint16_t c;

    capteur.getRawData(&r, &v, &b, &c);

    rouge = r;
    vert = v;
    bleu = b;
    luminosite = c;
    if(DEBUG_CAPTEUR_COULEUR)
    {
        Serial.print("R: "); Serial.print(rouge);
        Serial.print(" G: "); Serial.print(vert);  
        Serial.print(" B: "); Serial.print(bleu);
        Serial.print(" C: "); Serial.println(luminosite);
    }

}

void detecterCouleur(bool &jaune, bool &vertclair, bool &bleufonce, bool &rose, bool &orange)
{
    // Called after initialiser(), at intervals longer than the 101 ms integration.
    // Read the completed sample directly: getRawData() adds a blocking delay.
    const int rouge = capteur.read16(TCS34725_RDATAL);
    const int vert = capteur.read16(TCS34725_GDATAL);
    const int bleu = capteur.read16(TCS34725_BDATAL);
    static int precedente = 0;
    int couleur = 0;
    
    if (estjaune(rouge, vert, bleu)) couleur = 1;
    else if (estvert(rouge, vert, bleu)) couleur = 2;
    else if (estbleu(rouge, vert, bleu)) couleur = 3;
    else if (estrose(rouge, vert, bleu)) couleur = 4;
    else if (estorange(rouge, vert, bleu)) couleur = 5;
    // Confirmation uses two separate scheduled samples.
    jaune = couleur == 1 && precedente == couleur;
    vertclair = couleur == 2 && precedente == couleur;
    bleufonce = couleur == 3 && precedente == couleur;
    rose = couleur == 4 && precedente == couleur;
    orange = couleur == 5 && precedente == couleur;
    precedente = couleur;
}

bool estjaune(int rouge,int vert,int bleu)
{
    bool estjaune = false;
    if (rouge >= 190 && rouge <= 225 && vert  >= 225 && vert  <= 255
        && bleu  >= 130 && bleu  <= 152)
    {
        estjaune = true;
    }
    return estjaune;
    
}
bool estvert(int rouge,int vert,int bleu)
{
    bool estvert = false;
    if (rouge >= 155 && rouge <= 173 && vert  >= 210 && vert  <= 290
        && bleu  >= 165 && bleu  <= 222)
    {
        estvert = true;
    }
    return estvert;

}
  bool estbleu(int rouge,int vert,int bleu)
{
    bool estbleu = false;
    if (rouge >= 110 && rouge <= 124 && vert  >= 175 && vert  <= 189
        && bleu  >= 155 && bleu  <= 169)
    {
        estbleu = true;
    }
    return estbleu;
    
}

  bool estrose(int rouge,int vert,int bleu)
{
    bool estrose = false;
    if (rouge >= 210 && rouge <= 248 && vert  >= 180 && vert  <= 207
        && bleu  >= 148 && bleu  <= 169)
    {
        estrose = true;
    }
    return estrose;
    
}

  bool estorange(int rouge,int vert,int bleu)
{
    bool estorange = false;
    if (rouge >= 195 && rouge <= 211 && vert  >= 180 && vert  <= 200
        && bleu  >= 120 && bleu  <= 135)
    {
        estorange = true;
    }
    return estorange;
    
} 

