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

    for (int i = 0; i < 3; i++)
    {
        lireRGB(rouge, vert, bleu, luminosite);

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
    }

    jaune = (compteurjaune == 3);
    vertclair = (compteurvertclair == 3);
    bleufonce = (compteurbleufonce == 3);
    rose = (compteurrose == 3);
    orange = (compteurorange == 3);
}

bool estjaune(int rouge,int vert,int bleu)
{
    bool estjaune = false;
    if (rouge >= 232 && rouge <= 246 && vert  >= 259 && vert  <= 274
        && bleu  >= 146 && bleu  <= 156)
    {
        estjaune = true;
    }
    return estjaune;
    
}
bool estvert(int rouge,int vert,int bleu)
{
    bool estvert = false;
    if (rouge >= 200 && rouge <= 204 && vert  >= 286 && vert  <= 290
        && bleu  >= 218 && bleu  <= 222)
    {
        estvert = true;
    }
    return estvert;

}
  bool estbleu(int rouge,int vert,int bleu)
{
    bool estbleu = false;
    if (rouge >= 117 && rouge <= 129 && vert  >= 189 && vert  <= 200
        && bleu  >= 174 && bleu  <= 190)
    {
        estbleu = true;
    }
    return estbleu;
    
}

  bool estrose(int rouge,int vert,int bleu)
{
    bool estrose = false;
    if (rouge >= 256 && rouge <= 278 && vert  >= 203 && vert  <= 220
        && bleu  >= 168 && bleu  <= 183)
    {
        estrose = true;
    }
    return estrose;
    
}

  bool estorange(int rouge,int vert,int bleu)
{
    bool estorange = false;
    if (rouge >= 220 && rouge <= 250 && vert  >= 195 && vert  <= 220
        && bleu  >= 125 && bleu  <= 141)
    {
        estorange = true;
    }
    return estorange;
    
} 

