#include <cassert>
#include <iostream>
#include <limits>
#include <type_traits>
#include "LibRobus.h"
unsigned long temps = 0;
int niveaux[64] = {};
unsigned long millis() { return temps; }
void digitalWrite(int pin, int valeur) { niveaux[pin] = valeur; }
#include "../../src/led.cpp"
static_assert(std::is_unsigned<decltype(ledBleuDelai)>::value,
              "LED timestamps must use unsigned millis arithmetic on AVR");
int main() {
    unsigned long* delais[] = {&ledBleuDelai, &ledJauneDelai, &ledVertDelai, &ledRoseDelai, &ledOrangeDelai};
    const int broches[] = {LED_BLEU, LED_JAUNE, LED_VERT, LED_ROSE, LED_ORANGE};
    for (unsigned long debut : {1000UL, 32768UL, 40000UL, 70000UL,
                                std::numeric_limits<unsigned long>::max() - 5000UL}) {
        for (int i = 0; i < 5; ++i) {
            temps = debut;
            *delais[i] = debut;
            niveaux[broches[i]] = LOW;
            eteindreLed();
            assert(niveaux[broches[i]] == LOW);
            temps = debut + 9999UL;
            eteindreLed();
            assert(niveaux[broches[i]] == LOW);
            temps = debut + 10000UL;
            eteindreLed();
            assert(niveaux[broches[i]] == HIGH && *delais[i] == 0);
        }
    }
    std::cout << "LED timers: early/late triggers, full 10 seconds and unsigned wrap passed\n";
}
