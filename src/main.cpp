#include "Arduino.h"

#define BATMULT 3.065
//3.07466135458
//3.066395641193632020863697705803
//3.07466135458
#define LOWBAT_LEVEL 7.45

#define IDPin0 PB5
#define IDPin1 PB4
#define IDPin2 PB3

#define chPin0 PA1
#define chPin1 PA2
#define auxPin PB9

#define I2CPin1 PB10
#define I2CPin2 PB11
int CONTROLEMANUAL = 0;

#define LEDR PB7
#define LEDG PB8
#define LEDB PB6

#define analogBat PA0

#include "nrfFifi.h"
#include "motor.h"
#include "ledStatus.h"

bool novoDado = false;
uint8_t ROBO_ID = 0;

int ROBO_Vd[2], //Velocidade desejada rodas
    ROBO_V[2]; //

float bateriaV;
int lowBat = 0;
boolean LOWBAT = false;

unsigned long int ultimaLidaNRF = 0,
                  ultimoDadoValido = 0,
                  ultimoSinalControle = 0;
void verificaBateria();
void setup() {
  pinMode(IDPin0, INPUT_PULLUP);
  pinMode(IDPin1, INPUT_PULLUP);
  pinMode(IDPin2, INPUT_PULLUP);
  pinMode(chPin0, INPUT_PULLUP);
  pinMode(chPin1, INPUT_PULLUP);
  pinMode(auxPin, INPUT_PULLUP);
  pinMode(I2CPin1, INPUT_PULLDOWN);
  pinMode(I2CPin2, INPUT_PULLDOWN);
  pinMode(LEDR, PWM);
  pinMode(LEDG, PWM);
  pinMode(LEDB, PWM);
  pinMode(PC13, OUTPUT);
  pinMode(analogBat, INPUT_ANALOG);

  digitalWrite(PC13, HIGH);
  pwmWrite(LEDR, 0);
  pwmWrite(LEDG, 0);
  pwmWrite(LEDB, 0);

  Serial.begin(115200);

  //Determina Canal
  ROBO_ID += !digitalRead(IDPin0) + !digitalRead(IDPin1) * 2 + !digitalRead(IDPin2) * 4;
  CANAL = !digitalRead(chPin0) + !digitalRead(chPin1) * 2;

  pwmWrite(LEDR, ID_COLOR[ROBO_ID][0]);
  pwmWrite(LEDG, ID_COLOR[ROBO_ID][1]);
  pwmWrite(LEDB, ID_COLOR[ROBO_ID][2]);

  config_nrf24();
  int DEBUG;
  DEBUG = !digitalRead(auxPin);
  motorInit();
  ultimaLidaNRF = 0;
  if (DEBUG) {
    digitalWrite(PC13, LOW);
    delay(3000);
    digitalWrite(PC13, HIGH);
    Serial.print("Robo ID = "); Serial.println(ROBO_ID);

    verificaNRF();

    motorSetVel(50000, -50000);
    delay(750);
    motorSetVel(-50000, 50000);
    delay(750);
    motorSTBY();
  }
  CONTROLEMANUAL = !digitalRead(chPin1);
  if (CONTROLEMANUAL){
    
  }
}

void loop() {
  if (millis() - ultimaLidaNRF > 10) {
    ultimaLidaNRF  = millis();
    NRF_ACK = recebe_dados();
  }
  if (NRF_ACK) {
    NRF_ACK = 0;
    if (NRF_RECEBE[0] == 'F' && 3 + NRF_RECEBE[2] < NRF_BUFFER && NRF_RECEBE[3 + NRF_RECEBE[2]] == 'I' ) {
      pacote.tipo = NRF_RECEBE[1];
      pacote.n = NRF_RECEBE[2];

      for (int ii = 3; ii < 3 + pacote.n; ii++)
        pacote.dado[ii - 3] = NRF_RECEBE[ii];

      novoDado = true;


    }
  }

  if (novoDado) {
    novoDado = false;
    /* pacote = [ 'F' tipo_do_pacote n [n_bytes] 'I']
          'F' => início do pacote
          tipo_do_pacote =>| 'D' Velocidade da das rodas para x robos, n = 2*x, [n_bytes]= [V11, V12, V21, V22, ... Vx1, Vx2]
                           | 'd' Velocidade da das rodas para 1 robos de ID especifico, n = 3, [n_bytes]= [ID, V1, V2]
                           | 'U' Sinal de controle para x robos, n = 2*x, [n_bytes]= [V1, W1, V2, W2,... Vx, Wx] - (implementação futura)
                           | 'u' Sinal de controle para 1 robos de ID especifico, n = 3, [n_bytes]= [ID V W] - (implementação futura)
                           | 'C' seta configuração (implementação futura)
                           | 'S' Status (implementação futura)
                           | 'K'|'k' Continua comunicando  n = 0
          n => numero de bytes seguintes
          [n_bytes] => n bytes de dados
          'I' => fim do pacote
    */
    switch (pacote.tipo) {
      case 'D':
        if (ROBO_ID * 2 < pacote.n) {
          ROBO_Vd[0] = pacote.dado[ROBO_ID * 2 + 0] - 100;
          ROBO_Vd[1] = pacote.dado[ROBO_ID * 2 + 1] - 100;
          ultimoDadoValido = millis();
        }
        break;
      case 'd':
        if (pacote.dado[0] == ROBO_ID) {
          ROBO_Vd[0] = pacote.dado[1] - 100;
          ROBO_Vd[1] = pacote.dado[2] - 100;
          ultimoDadoValido = millis();
        }
        break;
      case 'K':
      case 'k':
        //none;
        ultimoDadoValido = millis();
        break;
    }
  }
  

  verificaBateria();


  if (millis() - ultimoDadoValido > 1000) {
    ROBO_Vd[0] = ROBO_Vd[1] = 0;
    motorSTBY();
  } else {
    ROBO_V[0] = map(ROBO_Vd[0], -100, 100, -65535, 65535);
    ROBO_V[1] = map(ROBO_Vd[1], -100, 100, -65535, 65535);
    motorSetVel(ROBO_V[0], ROBO_V[1]);
  }

}

/// Bateria
unsigned long int ultimoEnvio = 0;
int batS;
void verificaBateria() {
  static float bateriaVH[10],
         bateriaVHSum = 0 ;
  bateriaVHSum -= bateriaVH[0];
  for (int ii = 0; ii < 9; ii++) {
    bateriaVH[ii] = bateriaVH[ii + 1];
  }
  bateriaVH[9] =  map(analogRead(analogBat), 0, 4095, 0, 3300) * BATMULT / 1000.0;
  bateriaVHSum += bateriaVH[9];
  bateriaV = bateriaVHSum / 10.0;
  if (bateriaV < 2.5 && bateriaV > 2.1) {
    bateriaV = 0;
    LOWBAT = false;
  } else if (bateriaV < LOWBAT_LEVEL) {
    lowBat++;
    if (lowBat > 10) LOWBAT = true;
  } else {
    lowBat = 0;
    LOWBAT = false;
  }
  /// Bateria FIM

  if (LOWBAT) {
    static int indLED = 0;

    indLED++;
    indLED &= 0x03;
    switch (indLED) {
      case 1:
        pwmWrite(LEDR, 10000);
        break;
      case 2:
        pwmWrite(LEDR, 0);
        pwmWrite(LEDG, 10000);
        break;
      case 3:
        pwmWrite(LEDG, 0);
        pwmWrite(LEDB, 10000);
        break;
      default:
        pwmWrite(LEDR, 0);
        pwmWrite(LEDG, 0);
        pwmWrite(LEDB, 0);
    }
  } else {
    pwmWrite(LEDR, ID_COLOR[ROBO_ID][0]);
    pwmWrite(LEDG, ID_COLOR[ROBO_ID][1]);
    pwmWrite(LEDB, ID_COLOR[ROBO_ID][2]);

  }

  if (millis() - ultimoEnvio > 5000) {
    ultimoEnvio = millis();
    NRF_ENVIA[0] = 'F';
    NRF_ENVIA[1] = 'B';
    NRF_ENVIA[2] = 3;
    NRF_ENVIA[3] = ROBO_ID;
    batS = floor(bateriaV * 100);
    NRF_ENVIA[4] = batS >> 8;
    NRF_ENVIA[5] = batS & 0x00FF;
    NRF_ENVIA[6] = 'I';
    NRF_ACK_ENVIA = envia_dados();
    Serial.println(bateriaV);
  }
}