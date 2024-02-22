#include "SimpleFOC.h"

//Headers-------------------------------------------
inline void motorInit(void) __attribute__((always_inline));                   //Inicializa Motor
inline void motorSetVel(int velA, int velB ) __attribute__((always_inline));  //seta velocidade motor A e B
inline void motorStop(void) __attribute__((always_inline));                   //Desliga motores
inline void motorRun(void) __attribute__((always_inline));                    //Religa motores

//Variáveis-----------------------------------------
  // init BLDC motor
  BLDCMotor motA = BLDCMotor(7,8.5);
  BLDCMotor motB = BLDCMotor(7,8.5);
  // init driver
  BLDCDriver3PWM driverA = BLDCDriver3PWM(PB0, PA7, PA6, PB1);
  BLDCDriver3PWM driverB = BLDCDriver3PWM(PA3, PA2, PA1, PA4);

//Funções------------------------------------------
void motorInit() {
  // power supply voltage
  driverA.voltage_power_supply = 12;
  driverB.voltage_power_supply = 12;
  driverA.init();
  driverB.init();
  // link the motor and the driver
  motA.linkDriver(&driverA);
  motB.linkDriver(&driverB);
  // limiting motor movements
  motA.phase_resistance = 11.25; // [Ohm]
  motB.phase_resistance = 11.25; // [Ohm]
  motA.current_limit = 1;   // [Amps] - if phase resistance defined
  motB.current_limit = 1;
  motA.voltage_limit = 8;   // [V] - if phase resistance not defined
  motA.velocity_limit = 5; // [rad/s] cca 50rpm
  motB.voltage_limit = 8; 
  motB.velocity_limit = 5;
  // set control loop type to be used

  motA.controller = MotionControlType::velocity_openloop;
  motA.torque_controller = TorqueControlType::foc_current;

  motB.controller = MotionControlType::velocity_openloop;
  motB.torque_controller = TorqueControlType::foc_current;
  // initialize motor
  motA.init();
  motB.init();
}

inline void motorRun(){
  motA.enable();
  motB.enable();
}

inline void motorStop(){
  motA.disable();
  motB.disable();
  // motA.move();
  // motB.move();
}


inline void motorSetVel(int velA, int velB) {
  //set_MotorA(vel1);
  if (velA == 0 and velB == 0);
  else{
    if (velA > 0) {
      if (velA > 50) {
        velA = 50;
      }
    } else  {
      velA = velA;
      if (velA < -50) {
        velA = -50;

      }
    }
    
    // set_MotorB(vel2);
    if (velB > 0) {
      if (velB > 50) {
        velB = 50;
      }
    } else  {
      velB = velB;
      if (velB < -50) {
        velB = -50;
        
      }
    }
    motA.target = velA;
    motB.target = -velB;
    motA.move();
    motB.move();
  }
}