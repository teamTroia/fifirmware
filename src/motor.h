/*  Motor para TB6612FNG
 *  Por: Fidelis Melo Junior
*/

//#Define-------------------------------------------
// Pinos dos motores A e B -> PWM IN1 IN2, STBY
#define MAPWM PA3
#define MAIN1 PA4
#define MAIN2 PA5
#define MBPWM PB1
#define MBIN1 PB0
#define MBIN2 PA7
#define MSTBY PA6

//Headers-------------------------------------------
inline void motorInit(void) __attribute__((always_inline));                   //Inicializa Motor
inline void motorSetVel(int velA, int velB ) __attribute__((always_inline));  //seta velocidade -255a255 do motoa A e B
inline void motorStop(void) __attribute__((always_inline));                   //Trava motores
inline void motorSTBY(void) __attribute__((always_inline));                   // Desliga motores

//Variáveis-----------------------------------------

//Funções------------------------------------------
void motorInit() {
  pinMode(MAPWM, PWM);
  pinMode(MAIN1, OUTPUT);
  pinMode(MAIN2, OUTPUT);
  pinMode(MBPWM, PWM);
  pinMode(MBIN1, OUTPUT);
  pinMode(MBIN2, OUTPUT);
  pinMode(MSTBY, OUTPUT);
  
  motorSTBY(); 
  Serial.println("Motores inicializados!");
}

inline void motorSTBY(){
  digitalWrite(MAPWM, LOW);
  digitalWrite(MAIN1, LOW);
  digitalWrite(MAIN2, LOW);
  digitalWrite(MBPWM, LOW);
  digitalWrite(MBIN1, LOW);
  digitalWrite(MBIN2, LOW);
  digitalWrite(MSTBY, LOW);
}

inline void motorStop(){
  digitalWrite(MAPWM, HIGH);
  digitalWrite(MAIN1, LOW);
  digitalWrite(MAIN2, LOW);
  
  digitalWrite(MBPWM, HIGH);
  digitalWrite(MBIN1, LOW);
  digitalWrite(MBIN2, LOW);
  
  digitalWrite(MSTBY, HIGH);
}

inline void motorSetVel(int velA, int velB) {
  //set_MotorA(vel1);
  
  if (velA > 0) {
    digitalWrite(MAIN1, LOW);
    digitalWrite(MAIN2, HIGH);
    if (velA > 65535) velA = 65535;
    pwmWrite(MAPWM, velA);
  } else  {
    velA = -velA;
    digitalWrite(MAIN1, HIGH);
    digitalWrite(MAIN2, LOW);
    if (velA > 65535) velA = 65535;
    pwmWrite(MAPWM, velA);
  }
  // set_MotorB(vel2);
  if (velB > 0) {
    digitalWrite(MBIN1, LOW);
    digitalWrite(MBIN2, HIGH);
    if (velB > 65535) velB = 65535;
    pwmWrite(MBPWM, velB);
  } else  {
    velB = -velB;
    digitalWrite(MBIN1, HIGH);
    digitalWrite(MBIN2, LOW);
    if (velB > 65535) velB = 65535;
    pwmWrite(MBPWM, velB);
  }
  digitalWrite(MSTBY, HIGH);
}
