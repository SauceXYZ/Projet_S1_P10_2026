#ifndef PID_H
#define PID_H

void pid_init(void);

void pid_avance(void);
void pid_avance(float vitesseCible);
void pid_arret(void);
float pid_distanceArret(void);

void pid_tourne(bool direction, int angle);


#endif