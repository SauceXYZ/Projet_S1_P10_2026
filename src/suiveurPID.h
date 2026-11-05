#ifndef SUIVEURPID_H
#define SUIVEURPID_H

// Call regularly; each call returns without waiting for the next sample.
void suivreLaLigne(void);
void reinitialiserSuiveur(void);
void arreterSuiveur(void);
void testSuiveur(void);
// Starts the hole sequence; progression happens in suivreLaLigne().
void trou(void);
bool trouTermine(void);

#endif
