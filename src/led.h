#ifndef LED_H
#define LED_H   

#define LED_JAUNE 31
#define LED_VERT 33
#define LED_BLEU 35
#define LED_ROSE 41
#define LED_ORANGE 39

void eteindreLed();

extern int ledBleuDelai;
extern int ledJauneDelai;
extern int ledVertDelai;
extern int ledRoseDelai;
extern int ledOrangeDelai;


#endif