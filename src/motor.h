#include "Servo.h"

//Headers-------------------------------------------
inline void motorInit(void) __attribute__((always_inline));                   //Inicializa Motor
inline void motorSetVel(int velA, int velB ) __attribute__((always_inline));  //seta velocidade motor A e B
inline void motorStop(void) __attribute__((always_inline));                   //Trava motores

//Variáveis-----------------------------------------

Servo escD, escE;

int meio = 1500;

//Funções------------------------------------------
void motorInit() {
escD.attach(PB0);
escE.attach(PA6);
}

inline void motorStop(){
  escD.writeMicroseconds(meio);
  escE.writeMicroseconds(meio);
}

inline void motorSetVel(int velA, int velB) {
  //set_MotorA(vel1);
  if (velA > meio) {
    if (velA > 2000) velA = 2000;
    escD.writeMicroseconds(velA);
  } else  {
    velA = velA;
    if (velA < 1000) velA = 1000;
    escD.writeMicroseconds(velA);
  }
  // set_MotorB(vel2);
  if (velB > meio) {
    if (velB > 2000) velB = 2000;
    escE.writeMicroseconds(velB);
  } else  {
    velB = velB;
    if (velB < 1000) velB = 1000;
    escE.writeMicroseconds(velB);
  }
}