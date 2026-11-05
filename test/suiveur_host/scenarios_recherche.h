void commencerRecherche(int cote) {
    reinitialiserSuiveur();
    avancer(30, 700, 30, 300);
    if (cote < 0) avancer(750, 30, 30, 120);
    if (cote > 0) avancer(30, 30, 750, 120);
    for (int i = 0; i < 100 && etat != RECHERCHE; ++i) avancer(30, 30, 30, 5);
    assert(etat == RECHERCHE);
}
void aTempsRecherche(unsigned long cible) {
    const unsigned long ecoule = temps - derniereDetection - PONT_MS;
    assert(cible >= ecoule);
    avancer(30, 30, 30, cible - ecoule);
}
void testerRecherche() {
    for (int cote = -1; cote <= 1; ++cote) {
        commencerRecherche(cote);
        const int premier = cote == 0 ? -1 : cote;
        aTempsRecherche(200);
        assert((moteurs[0] * SENS_DIRECTION * premier) > 0);
        assert(fabs(moteurs[0] + moteurs[1]) < .00001f);
        aTempsRecherche(BALAYAGE_BASE_MS + 200); // second sweep, opposite side
        assert((moteurs[0] * SENS_DIRECTION * premier) < 0);
        aTempsRecherche(3 * BALAYAGE_BASE_MS + 200); // third sweep, original side
        assert((moteurs[0] * SENS_DIRECTION * premier) > 0);
        aTempsRecherche(6 * BALAYAGE_BASE_MS + 200); // fourth sweep, opposite side
        assert((moteurs[0] * SENS_DIRECTION * premier) < 0);
        aTempsRecherche(RECHERCHE_MS + 10);
        assert(etat == ARRET && moteurs[0] == 0 && moteurs[1] == 0);
    }
    commencerRecherche(0);
    aTempsRecherche(200);
    avancer(30, 1000, 30, 5); // short pulse: stop but do not accept it as reacquired
    assert(ligneCandidate && moteurs[0] == 0 && moteurs[1] == 0);
    avancer(30, 30, 30, 50);
    assert(etat == RECHERCHE && !ligneCandidate);
    avancer(30, 700, 30, 20);
    assert(etat == RECHERCHE);
    avancer(30, 700, 30, 400);
    assert(etat == SUIVI && moteurs[0] > 0 && moteurs[1] > 0);
    assert(fabs(moteurs[0] - moteurs[1]) < .001f);
    // In the gap the PID must retain control, without a scan or normal stop.
    preparerTraversee(0);
    avancer(30, 30, 30, RECHERCHE_MS + 300);
    assert(trouEtat.phase == PhaseTrou::TRAVERSEE && !trouEchoue());
    assert(moteurs[0] > 0 && moteurs[1] > 0);
    reinitialiserSuiveur();
}
