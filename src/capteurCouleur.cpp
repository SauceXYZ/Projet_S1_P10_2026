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
    if (rouge >= 164 && rouge <= 173 && vert  >= 225 && vert  <= 290
        && bleu  >= 175 && bleu  <= 222)
    {
        estvert = true;
    }
    return estvert;

}
  bool estbleu(int rouge,int vert,int bleu)
{
    bool estbleu = false;
    if (rouge >= 115 && rouge <= 124 && vert  >= 175 && vert  <= 189
        && bleu  >= 159 && bleu  <= 169)
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
    if (rouge >= 201 && rouge <= 211 && vert  >= 186 && vert  <= 196
        && bleu  >= 122 && bleu  <= 131)
    {
        estorange = true;
    }
    return estorange;
    
} 

