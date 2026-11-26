#ifndef CONFIG_H
#define CONFIG_H

#define BAUD 115200

#define VID_WIDTH 640
#define VID_CENTER VID_WIDTH/2

#define MaxSpeed 0.2

#define MAX_DEPLACEMENT_OFFSET 100
#define MIN_DEPLACEMNT_OFFSET -100

//PID deplacement vers verre
#define V_KP 0.5
#define V_KI 0
#define V_KD 0
#define V_CLAMP_MAX MAX_DEPLACEMENT_OFFSET
#define V_CLAMP_MIN MIN_DEPLACEMNT_OFFSET



#endif