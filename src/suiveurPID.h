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
// Latched failure; the main loop must stop the challenge instead of retrying.
bool trouEchoue(void);

#endif
