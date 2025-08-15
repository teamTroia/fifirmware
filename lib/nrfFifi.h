#include <SPI.h>
SPIClass SPInf(2);

//#define CANAL    7   // 1 a 126 definido n amain por NRF_CANAL
#define NRF_BUFFER   21  // 1 a 32
//#define ENDTX    13  // 1 a 255 definido n amain por NRF_ENDTX
//#define ENDRX    13  // 1 a 255 definido n amain por NRF_ENDRX

#define CE  PB12
#define CSN PA8
#define IRQ PA15  // INT_EXT1


uint8_t CANAIS[] = {0x55, 0x0B, 0x73, 0x33};   // 0x00-0x7F
uint8_t END[] = {'#', '&', '#', '@'};    // 0x00- 0xFF
static uint8_t CANAL;

static uint8_t RECEBE[NRF_BUFFER];
static uint8_t ENVIA [NRF_BUFFER];

static int16_t       espera_ack;
static bool NRF_ACK, NRF_ACK_ENVIA;
static uint8_t NRF_RECEBE[NRF_BUFFER];
static uint8_t NRF_ENVIA [NRF_BUFFER];

static uint8_t NRF_ENDTX = 20;
static uint8_t NRF_ENDRX = 20;
static uint8_t NRF_CANAL = 120;

unsigned int ACONFIG, ASTATUS, ANRF_CANAL, ADDRX[5], ADDTX[5];

struct PacoteCom {
  uint8_t tipo;
  uint8_t n;
  uint8_t dado[NRF_BUFFER];
} pacote;

//função que configura modulo
void config_nrf24() {
  pinMode(CSN, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(IRQ, INPUT_PULLUP);
  digitalWrite(CE, LOW);
  delay(1);
  digitalWrite(CSN, HIGH);
  digitalWrite(CE, HIGH);

  NRF_CANAL = CANAIS[CANAL & 0x03];
  NRF_ENDTX = END[CANAL+1 & 0x03];
  NRF_ENDRX = END[CANAL & 0x03];

  //configura modulo SPI
  SPInf.begin(); //Initialize the SPI_2 port.
  SPInf.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  SPInf.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPInf.setClockDivider(SPI_CLOCK_DIV32);  // Use a different speed to SPI 1

  delay(15);
  digitalWrite(CE, LOW);

  //RX_ADDR_P0 - configura endereço de recepção PIPE0
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x2A);
  SPInf.transfer(NRF_ENDRX & 0xFF);
  SPInf.transfer('F');
  SPInf.transfer('I');
  SPInf.transfer('F');
  SPInf.transfer('I');
  digitalWrite(CSN, HIGH);

  //TX_ADDR - configura endereço de transmissão
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x30);
  SPInf.transfer(NRF_ENDTX & 0xFF);
  SPInf.transfer('F');
  SPInf.transfer('I');
  SPInf.transfer('F');
  SPInf.transfer('I');
  digitalWrite(CSN, HIGH);

  //EN_AA - habilita autoACK no PIPE0
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x21);
  SPInf.transfer(0x01);
  digitalWrite(CSN, HIGH);

  //EN_RXADDR - ativa o PIPE0
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x22);
  SPInf.transfer(0x01);
  digitalWrite(CSN, HIGH);

  //SETUP_AW - define o endere篠com tamanho de 5 Bytes
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x23);
  SPInf.transfer(0x03);
  digitalWrite(CSN, HIGH);

  //SETUP_RETR - configura para nao retransmitir pacotes
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x24);
  SPInf.transfer(0x00);
  digitalWrite(CSN, HIGH);

  //RF_CH - define o canal do modulo (TX e RX devem ser iguais)
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x25);
  SPInf.transfer(NRF_CANAL & 0x7F);
  digitalWrite(CSN, HIGH);

  //RF_SETUP - ativa LNA, taxa em 250K, e maxima potencia 0dbm
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x26);
  SPInf.transfer(0b00100110);
  digitalWrite(CSN, HIGH);

  //STATUS - reseta o resgistrador STATUS
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x27);
  SPInf.transfer(0x70);
  digitalWrite(CSN, HIGH);

  //RX_PW_P0 - tamanho do buffer PIPE0
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x31);
  SPInf.transfer(NRF_BUFFER);
  digitalWrite(CSN, HIGH);

  //CONFIG - coloca em modo de recepição, e define CRC de 2 Bytes
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x20);
  SPInf.transfer(0x0F);
  digitalWrite(CSN, HIGH);

  //tempo para sair do modo standby entrar em modo de recepoção
  delay(2);
  digitalWrite(CE, HIGH);

  delayMicroseconds(150);

  //configura interrupição externa 1
  //IRQ

}

//função que transmite os dados
bool envia_dados() {

  uint8_t i;
  int8_t status;

  //desabilita interrupição

  digitalWrite(CE, LOW);
  delayMicroseconds(150);
  //STATUS - reseta registrador STATUS
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x27);
  SPInf.transfer(0x70);
  digitalWrite(CSN, HIGH);
  delayMicroseconds(150);
  // W_TX_PAYLOAD - envia os dados para o buffer FIFO TX
  digitalWrite(CSN, LOW);
  SPInf.transfer(0xA0);
  for (i = 0; i < NRF_BUFFER; i++)
    SPInf.transfer(NRF_ENVIA[i]);
  digitalWrite(CSN, HIGH);


  //CONFIG - ativa modo de transmissão
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x20);
  SPInf.transfer(0x0E);
  digitalWrite(CSN, HIGH);
  delayMicroseconds(150);
  //pulso para transmitir os dados
  digitalWrite(CE, HIGH);

  delayMicroseconds(15);
  digitalWrite(CE, LOW);

  espera_ack = 0;

  while (digitalRead(IRQ) == 1 && espera_ack < 500) {
    espera_ack++;
    delayMicroseconds(10);
    //espera até 5ms, pela recepição do pacote ACK

  }

  //STATUS - leitura do registrador
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x07);
  status = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);
  delayMicroseconds(150);
  //STATUS - limpa registrador
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x27);
  SPInf.transfer(0x70);
  digitalWrite(CSN, HIGH);
  delayMicroseconds(150);
  //TX_FLUSH - limpa o buffer FIFO TX
  digitalWrite(CSN, LOW);
  SPInf.transfer(0xE1);
  digitalWrite(CSN, HIGH);
  delayMicroseconds(150);
  //CONFIG - configura para modo de recepição
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x20);
  SPInf.transfer(0x0F);
  digitalWrite(CSN, HIGH);

  digitalWrite(CE, HIGH);

  delayMicroseconds(150);

  //se não recebeu ACK em 5ms retorna 0
  if (espera_ack == 500) {
    //clear_interrupt(INT_EXT1);
    //enable_interrupts(INTR_GLOBAL);
    return (0);
  }
  //se recebeu ACK retorna 1
  else {
    //clear_interrupt(INT_EXT1);
    //enable_interrupts(INTR_GLOBAL);
    return (1);
  }
}

//função que recebe os dados e joga num vetor
bool recebe_dados() {

  int8_t i;
  int8_t status;


  //STATUS - leitura do registrador
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x07);
  status = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

 //STATUS - limpa registrador
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x27);
  SPInf.transfer(0x70);
  digitalWrite(CSN, HIGH);
  
  //verifica o bit de recepção de dados
  if(((status&0x0E) == 0x0E) ) {//
    return (0);
  }
 
  //R_RX_PAYLOAD - recebe os dados do buffer FIFO RX
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x61);
  for (i = 0; i < NRF_BUFFER; i++)
    NRF_RECEBE[i] = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

  return (1);
}

void verificaNRF() {


  Serial.print("IDCHA="); Serial.print(CANAL);
  Serial.print(" CH=0x"); Serial.print(NRF_CANAL, HEX);
  Serial.print(" TX='"); Serial.print(char(NRF_ENDTX));
  Serial.print("' RX='"); Serial.print(char(NRF_ENDRX));
  Serial.println("'");
  digitalWrite(CSN, LOW);
  SPInf.transfer(0x00);
  ACONFIG = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

  digitalWrite(CSN, LOW);
  SPInf.transfer(0x05);
  ANRF_CANAL = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

  digitalWrite(CSN, LOW);
  SPInf.transfer(0x07);
  ASTATUS = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

  digitalWrite(CSN, LOW);
  SPInf.transfer(0x0A);
  ADDRX[0] = SPInf.transfer(0);
  ADDRX[1] = SPInf.transfer(0);
  ADDRX[2] = SPInf.transfer(0);
  ADDRX[3] = SPInf.transfer(0);
  ADDRX[4] = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

  digitalWrite(CSN, LOW);
  SPInf.transfer(0x10);
  ADDTX[0] = SPInf.transfer(0);
  ADDTX[1] = SPInf.transfer(0);
  ADDTX[2] = SPInf.transfer(0);
  ADDTX[3] = SPInf.transfer(0);
  ADDTX[4] = SPInf.transfer(0);
  digitalWrite(CSN, HIGH);

  Serial.print("NRF24L01: CONFIG = "); Serial.print(ACONFIG, HEX);
  Serial.print(", STATUS= "); Serial.print(ASTATUS, HEX);
  Serial.print(", NRF_CH= "); Serial.println(ANRF_CANAL, HEX);

  Serial.print("ADRX= \"");
  Serial.print(char(ADDRX[0])); Serial.print(char(ADDRX[1])); Serial.print(char(ADDRX[2])); Serial.print(char(ADDRX[3])); Serial.print(char(ADDRX[4]));
  Serial.println("\"");
  Serial.print("ADTX= \"");
  Serial.print(char(ADDTX[0])); Serial.print(char(ADDTX[1])); Serial.print(char(ADDTX[2])); Serial.print(char(ADDTX[3])); Serial.print(char(ADDTX[4]));
  Serial.println("\"");
}
