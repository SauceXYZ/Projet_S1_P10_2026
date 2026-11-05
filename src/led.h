#ifndef LED_H
#define LED_H   

#define LED_JAUNE 32
#define LED_VERT 33
#define LED_BLEU 43
#define LED_ROSE 41
#define LED_ORANGE 39

void eteindreLed();

extern unsigned long ledBleuDelai;
extern unsigned long ledJauneDelai;
extern unsigned long ledVertDelai;
extern unsigned long ledRoseDelai;
extern unsigned long ledOrangeDelai;


#endif