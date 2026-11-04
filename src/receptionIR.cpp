#include "receptionIR.h"

#include <LibRobus.h>

static uint8_t inverserBits(uint8_t valeur)
{
    uint8_t resultat = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        resultat = (resultat << 1) | (valeur & 1);
        valeur >>= 1;
    }
    return resultat;
}

int receptionIR(void)
{
    static IRrecv mannette(IR_RECEIVE_PIN);
    decode_results data;
    if (mannette.decode(&data)) 
    {
      mannette.resume();
      if (data.decode_type != NEC || data.bits != 32 || data.value == REPEAT) 
      {
          return RIEN_IR;
      }

      uint8_t adresseBasse = inverserBits(data.value >> 24);
      uint8_t adresseHaute = inverserBits(data.value >> 16);
      uint16_t adresse = adresseBasse;
      if (adresseBasse != static_cast<uint8_t>(~adresseHaute)) 
      {
          adresse |= static_cast<uint16_t>(adresseHaute) << 8;
      }
      uint8_t commande = inverserBits(data.value >> 8);
      uint8_t commandeInverse = inverserBits(data.value);
      bool valide = adresse == IR_ADDRESS &&
                    commande == static_cast<uint8_t>(~commandeInverse);
 
      if (valide) {
        if (commande == CMD_ON)  return START_IR;
        if (commande == CMD_OFF) return CONTINUE_IR;
      }
    }
    return RIEN_IR;
}
