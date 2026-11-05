// Sensor/encoder scenarios; no claim to simulate robot traction or dynamics.
void finirPause(int g, int m, int d) {
    assert(trouEtat.pause);
    const unsigned long restant = ReglageTrou::PAUSE_MS - (temps - trouEtat.debutEtape);
    if (restant > 50) avancer(g, m, d, restant - 50);
    assert(trouEtat.pause && moteurs[0] == 0 && moteurs[1] == 0);
    avancer(g, m, d, 60);
    assert(!trouEtat.pause);
}
void preparerAlignement() {
    trou();
    finirPause(750, 700, 750);
    avancer(750, 700, 750, 80);
    assert(trouEtat.tFranchi);
    avancer(30, 700, 420, 180); // card is background to the right
    assert(trouEtat.phase == PhaseTrou::ALIGNEMENT && trouEtat.pause);
    finirPause(30, 700, 420);
}
void preparerTraversee(int premier) {
    preparerAlignement();
    avancer(premier == 0 ? 750 : 30, 700, premier == 2 ? 750 : 420, 160);
    assert(trouEtat.premierConfirme && trouEtat.premier == premier);
    assert((premier == 0 ? moteurs[0] : moteurs[1]) > 0);
    assert((premier == 0 ? moteurs[1] : moteurs[0]) == 0);
    avancer(750, 700, 750, 80);
    assert(trouEtat.phase == PhaseTrou::TRAVERSEE && trouEtat.pause);
    finirPause(215, 700, 390); // departure still visible under the middle
}
void imposerDistance(float g, float d) {
    encodeursTest[0] = trouEtat.encodeursVide[0] + static_cast<int32_t>(ceilf(g / CM_PAR_IMPULSION_0));
    encodeursTest[1] = trouEtat.encodeursVide[1] + static_cast<int32_t>(ceilf(d / CM_PAR_IMPULSION_1));
}
void testerTrou() {
    using namespace ReglageTrou;
    assert(fabs(rampeTrou(0, .1f, .005f) - .001f) < .00001f);
    assert(fabs(rampeTrou(0, -.1f, .005f) + .001f) < .00001f);
    assert(rampeTrou(.1f, 0, .005f) == 0);
    assert(rampeTrou(-.1f, 0, .005f) == 0);

    // No centre-only exit until the transverse bar is seen while reversing.
    trou();
    finirPause(213, 286, 346);
    avancer(188, 366, 445, 500);
    assert(trouEtat.phase == PhaseTrou::RECUL && !trouEtat.tFranchi);
    avancer(750, 700, 750, 80);
    avancer(30, 700, 420, 60);
    assert(trouEtat.phase == PhaseTrou::RECUL);
    avancer(30, 700, 420, 100);
    assert(trouEtat.phase == PhaseTrou::ALIGNEMENT);

    for (int capteur = 0; capteur < 3; ++capteur) {
        preparerTraversee(capteur == 0 ? 0 : 2); // both physical pivot directions
        avancer(215, 330, 390, 300);
        assert(!trouEtat.videConfirme && !trouTermine());
        avancer(30, 30, 30, 120);
        assert(trouEtat.videConfirme);
        // Even a strong early signal cannot finish before the distance gate.
        avancer(750, 750, 750, 100);
        assert(trouEtat.phase == PhaseTrou::TRAVERSEE && !trouTermine());
        avancer(30, 30, 30, 120);
        imposerDistance(DISTANCE_MIN_ARRIVEE_CM + 1, DISTANCE_MIN_ARRIVEE_CM - 1);
        avancer(30, 30, 30, 20);
        assert(trouEtat.phase == PhaseTrou::TRAVERSEE); // both wheels must travel
        imposerDistance(DISTANCE_MIN_ARRIVEE_CM + 1, DISTANCE_MIN_ARRIVEE_CM + 1);
        avancer(30, 30, 30, 20);
        assert(trouEtat.phase == PhaseTrou::ARRIVEE && moteurs[0] > 0 && moteurs[1] > 0);
        // Exact false arrival from the log (centre 337) must now be ignored.
        avancer(204, 337, 413, 300);
        assert(!trouTermine());
        avancer(capteur == 0 ? 750 : 30, capteur == 1 ? 750 : 30,
                capteur == 2 ? 750 : 30, 30);
        assert(!trouTermine());
        for (int i = 0; i < 20 && !trouTermine(); ++i)
            avancer(capteur == 0 ? 750 : 30, capteur == 1 ? 750 : 30,
                    capteur == 2 ? 750 : 30, 5);
        assert(trouTermine() && !trouEchoue());
        assert(moteurs[0] == 0 && moteurs[1] == 0);
        avancer(30, 700, 30, 300);
        assert(etat == SUIVI); // normal following resumes
    }
    // Logged false edge: right 515 is active for tracking, not for the T.
    preparerAlignement();
    avancer(250, 735, 528, 60); // tracking activates before settling to 515
    avancer(250, 735, 515, 350);
    assert(detecte[2] && !capteurVoitT(2));
    assert(!trouEtat.premierConfirme && moteurs[0] > 0 && moteurs[1] > 0);
    avancer(280, 735, 570, 300);
    assert(!trouEtat.premierConfirme); // formerly also confirmed around 569
    avancer(250, 735, 802, 180);
    assert(trouEtat.premierConfirme && trouEtat.premier == 2);
    avancer(337, 735, 802, 150); // left background must not validate the bar
    assert(trouEtat.phase == PhaseTrou::ALIGNEMENT && !trouEtat.pause);
    avancer(634, 735, 802, 100);
    assert(trouEtat.phase == PhaseTrou::TRAVERSEE);
    // A 20--80 ms spike cannot start a pivot anymore.
    preparerAlignement();
    avancer(30, 700, 750, 60);
    assert(!trouEtat.premierConfirme && moteurs[0] == 0 && moteurs[1] == 0);
    avancer(30, 700, 420, 80);
    assert(!trouEtat.premierConfirme && !trouEchoue());
    // Short dropouts pause the wheel; a returning edge can resume the pivot.
    avancer(30, 700, 750, 160);
    assert(trouEtat.premierConfirme);
    avancer(30, 200, 440, 250);
    assert(!trouEchoue() && trouEtat.perteExtremite);
    assert(moteurs[0] == 0 && moteurs[1] == 0);
    avancer(30, 700, 750, 60);
    assert(!trouEchoue() && !trouEtat.perteExtremite && moteurs[1] > 0);
    // A sustained loss still fails without looping back to approach.
    avancer(30, 200, 440, PERTE_EXTREMITE_MAX_MS + 100);
    assert(trouEchoue() && !trouTermine() && moteurs[0] == 0 && moteurs[1] == 0);
    avancer(750, 750, 750, 300);
    assert(trouEchoue() && moteurs[0] == 0 && moteurs[1] == 0);
    // Missing bar, missing second end, and stalled encoders all have bounds.
    trou();
    finirPause(30, 700, 420);
    avancer(30, 700, 420, RECUL_MAX_MS + 50);
    assert(trouEchoue());
    preparerAlignement();
    avancer(750, 700, 420, ALIGNEMENT_MAX_MS + 50);
    assert(trouEchoue());
    preparerTraversee(0);
    avancer(30, 30, 30, TRAVERSEE_MAX_MS + 50);
    assert(trouEchoue() && !trouTermine() && moteurs[0] == 0 && moteurs[1] == 0);
    reinitialiserSuiveur();
    assert(!trouEchoue() && !trouTermine());
}
