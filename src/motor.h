#include <SimpleFOC.h>

//Headers-------------------------------------------
inline void motorInit(void) __attribute__((always_inline));                   //Inicializa Motor
inline void motorSetVel(int velA, int velB ) __attribute__((always_inline));  //seta velocidade motor A e B
inline void motorStop(void) __attribute__((always_inline));                   //Trava motores

//Variáveis-----------------------------------------
  // init BLDC motor
  BLDCMotor motA = BLDCMotor(7);
  BLDCMotor motB = BLDCMotor(7);
  // init driver
  BLDCDriver3PWM driverA = BLDCDriver3PWM(PB0, PA7, PA6, PB1);
  BLDCDriver3PWM driverB = BLDCDriver3PWM(PA3, PA2, PA1, PA4);

//Funções------------------------------------------
void motorInit() {
  // power supply voltage
  driverA.voltage_power_supply = 8;
  driverB.voltage_power_supply = 8;
  driverA.init();
  driverB.init();
  // link the motor and the driver
  motA.linkDriver(&driverA);
  motB.linkDriver(&driverB);
  // limiting motor movements
  motA.phase_resistance = 8.5; // [Ohm]
  motB.phase_resistance = 8.5; // [Ohm]
  motA.current_limit = 0.5;   // [Amps] - if phase resistance defined
  motB.current_limit = 0.5;
  //motA.voltage_limit = 3;   // [V] - if phase resistance not defined
  //motA.velocity_limit = 5; // [rad/s] cca 50rpm
  //motB.voltage_limit = 3; 
  //motB.velocity_limit = 5;
  // set control loop type to be used

  motA.controller = MotionControlType::velocity_openloop;
  motB.controller = MotionControlType::velocity_openloop;
  // initialize motor
  motA.init();
  motB.init();
}

inline void motorStop(){
  motA.target = 0;
  motB.target = 0;
  motA.move();
  motB.move();
}

inline void motorSetVel(int velA, int velB) {
  //set_MotorA(vel1);
  if (velA > 0) {
    if (velA > 25) velA = 25;
  } else  {
    velA = velA;
    if (velA < -25) velA = -25;
  }
  
  // set_MotorB(vel2);
  if (velB > 0) {
    if (velB > 25) velB = 25;
  } else  {
    velB = velB;
    if (velB < -25) velB = -25;
  }

  motA.target = velA;
  motB.target = velB;
  motA.move();
  motB.move();
}