#include "capteurCouleur.h"
#include <Adafruit_TCS34725.h>

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

    // Serial.print("R: "); Serial.print(rouge);
    // Serial.print(" G: "); Serial.print(vert);  
    // Serial.print(" B: "); Serial.print(bleu);
    // Serial.print(" C: "); Serial.println(luminosite);

}

void detecterCouleur(bool &jaune, bool &vertclair, bool &bleufonce, bool &rose, bool &orange)
{
    int rouge, vert, bleu, luminosite;

    int compteurjaune = 0;
    int compteurvertclair = 0;
    int compteurbleufonce = 0;
    int compteurrose = 0;
    int compteurorange = 0;

    jaune = false;
    vertclair = false;
    bleufonce = false;
    rose = false;
    orange = false;

    for (int i = 0; i < 2; i++)
    {
        lireRGB(rouge, vert, bleu, luminosite);
        Serial.println("Lecture ");
        Serial.println(rouge);
        Serial.println(vert);
        Serial.println(bleu);

        if (estjaune(rouge, vert, bleu))
        {
            compteurjaune++;
        }
        else if (estvert(rouge, vert, bleu))
        {
            compteurvertclair++;
        }
        else if (estbleu(rouge, vert, bleu))
        {
            compteurbleufonce++;
        }
        else if (estrose(rouge, vert, bleu))
        {
            compteurrose++;
        }
        else if (estorange(rouge, vert, bleu))
        {
            compteurorange++;
        }
        else
        {
            //aucune couleur détectée
            break;
        }
    }

    jaune = (compteurjaune == 2);
    vertclair = (compteurvertclair == 2);
    bleufonce = (compteurbleufonce == 2);
    rose = (compteurrose == 2);
    orange = (compteurorange == 2);

    // if(jaune) Serial.println("Jaune");
    // else if(vertclair) Serial.println("Vert");
    // else if(bleufonce) Serial.println("Bleu");
    // else if(rose) Serial.println("Rose");
    // else if(orange) Serial.println("Orange");
}

bool estjaune(int rouge,int vert,int bleu)
{
    bool estjaune = false;
    if (rouge >= 213 && rouge <= 219 && vert  >= 241 && vert  <= 248
        && bleu  >= 142 && bleu  <= 146)
    {
        estjaune = true;
    }
    return estjaune;
    
}
bool estvert(int rouge,int vert,int bleu)
{
    bool estvert = false;
    if (rouge >= 160 && rouge <= 173 && vert  >= 220 && vert  <= 290
        && bleu  >= 170 && bleu  <= 222)
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
    if (rouge >= 234 && rouge <= 248 && vert  >= 194 && vert  <= 207
        && bleu  >= 157 && bleu  <= 169)
    {
        estrose = true;
    }
    return estrose;
    
}

  bool estorange(int rouge,int vert,int bleu)
{
    bool estorange = false;
    if (rouge >= 201 && rouge <= 211 && vert  >= 186 && vert  <= 200
        && bleu  >= 122 && bleu  <= 135)
    {
        estorange = true;
    }
    return estorange;
    
} 

