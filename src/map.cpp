#include <LibRobus.h>

//Le robot va avoir un x et y
// Le robot va avoir un sens de déplacement 0 1 2 3 ou qqch


// binaire pour mur
//H D B G


// 1001 = 9 
//

int map[3][6] = {
    {9, 1, 1, 1, 1, 3},
    {8, 0, 0, 0, 0, 2},
    {12, 4, 4, 4, 4, 6}
};

//8 bloquer en haut
//4 bloquer a droite
//2 bloquer en bas
//1 bloquer a gauche

void progresser(int x, int y, int sens){
    int possibilite[4] = {0, 0, 0, 0};

    if(map[x][y] & 8 == 0){
        // avancer
        possibilite[0] = 1;
    }
    if(map[x][y] & 4 == 0){
        // droite
        possibilite[1] = 1;
    }
    if(map[x][y] & 2 == 0){
        // reculer
        possibilite[2] = 1;
    }
    if(map[x][y] & 1 == 0){
        // gauche
        possibilite[3] = 1;
        
    }

    int vert = 0; //digitalRead(vertpin);
    int rouge = 0; //digitalRead(rougepin); // changer juste pour écrire

    if(vert && rouge){
        possibilite[0] = 0;
    }
    
}

void avancer50cm(){
    //5092,95 ticks pour 50cm
    ENCODER_ReadReset(RIGHT);
    while(ENCODER_Read(RIGHT) < 5092.95){
        MOTOR_SetSpeed(RIGHT, 0.5);//MOTOR_SetSpeed(RIGHT, vitesse);
        MOTOR_SetSpeed(LEFT, 0.5);
    }
}


