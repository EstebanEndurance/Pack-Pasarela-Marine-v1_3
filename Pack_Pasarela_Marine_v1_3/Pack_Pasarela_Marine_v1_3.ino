/*********************************************************************************************************
   *ENDURANCE MOTIVE 
   *FRONIUS COMUNICATION - PASARELA VERSION PARA ESP32 V0.1 - MODO TORTUGA ERROR LEDS - PASARELA TRNASISTORES
   *DPT DESARROLLO ELECTRÓNICO E I+D
   *16/06/2021
*********************************************************************************************************/


/*
_________________________________________________________ PIN OUT 
                                         +----------------------------------------+
                                         |                                        |
                                         |                                        |
                                         +----------------------------------------+
                                    GND1 | [ ]                                [ ] | GND             
                                    3,3V | [ ]                                [ ] | GPIO23
                                      EN | [ ]                                [ ] | GPIO22-----------CAN0_INT
                                  GPIO36 | [ ]                                [ ] | GPIO1  /TXD
                                  GPIO39 | [ ]                                [ ] | GPIO3  /RXD
                    IN_1----------GPIO34 | [ ]                                [ ] | GPIO21-----------CS_1
                    IN_2----------GPIO35 | [ ]             ESP32              [ ] | NC
             CONTACTOR_1----------GPIO32 | [0]                                [ ] | GPIO19
             CONTACTOR_2----------GPIO33 | [0]                                [ ] | GPIO18
             CONTACTOR_3----------GPIO25 | [0]                                [ ] | GPIO5
                    LED1----------GPIO26 | [ ]                                [ ] | GPIO17-----------LED4
                    LED2----------GPIO27 | [ ]                                [ ] | GPIO16-----------LED3
                                  GPIO14 | [ ]                                [ ] | GPIO4------------CAN1_INT
                                  GPIO12 | [ ]                                [ ] | GPIO0
                                         |     [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]     |  
                                         +------+--+--+--+--+--+--+--+--+--+------+
                                     GND________|  |  |  |  |  |  |  |  |  |________GPIO2
                                  GPIO13___________|  |  |  |  |  |  |  |___________GPIO15-----------CS_2
                                   GPIO9______________|  |  |  |  |  |______________GPIO8
                                  GPIO10_________________|  |  |  |_________________GPIO7
                                  GPIO11____________________|  |____________________GPIO6

*/



//_______________________________________________________INCLUDES
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

//_______________________________________________________DEFINES
#define LED1 26
#define LED2 27
#define LED3 16
#define LED4 17
#define CONTACTOR_1 32
#define CONTACTOR_2 33
#define CONTACTOR_3 25
#define IN1 34
#define IN2 35

//_______________________________________________________ASIGNACION DE PINES DEL CAN0
MCP_CAN CAN0(21);             // Set CS to pin 21
const int CAN0_INT = 22;

//_______________________________________________________ASIGNACION DE PINES DEL CAN1
MCP_CAN CAN1(15);             // Set CS to pin 15
const int CAN1_INT=4;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

//_______________________________________________________VARIABLES DE CARGA CORRIENTE Y VOLTAJE POR BITS
int primer_envio = 0;
bool CAN_status = 0;
byte pinStatus = 0;
byte pinStatusCharger = 0;
long unsigned Icarga = 0;
long unsigned Vcarga = 0;
long unsigned Vbat = 0;
long unsigned pack_current_pos;
byte Cc1 = 0;
byte Cc2 = 0;
byte Cc3 = 0;
byte Vc1 = 0;
byte Vc2 = 0;
byte Vc3 = 0;
byte SOC = 0;
byte SOH = 0;
byte Cm1 = 0;
byte Cm2 = 0;
byte Cm3 = 0;
byte Vm1 = 0;
byte Vm2 = 0;
byte Vm3 = 0;
byte ta = 0;

//_______________________________________________________Configuramos el NMT para el paso a estado Operativo
byte NMT[2] = {0x01, 0x00}; // NMT MODO OPERATIVO
byte HEART[1] = {0x05}; //HEARTH BEAT DEL BMS MODO ON

// Configuramos las RPDO indicando valores de corriente, voltaje, SOC y SOH

//byte RPDO1[8] = {0x8C, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte RPDO1[8] = {SOC, SOH, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte RPDO2[8] = {0xD0, 0x07, 0x00, 0x04, 0x37, 0x01, 0x14, 0x00};
byte RPDO2[8] = {Cm1, Cm2, Cm3, Vm1, Vm2, Vm3, ta, 0x00};
//byte RPDO3[8] = {0xD0, 0x07, 0x00, 0x48, 0x6B, 0x01, 0x11, 0x00};
byte RPDO3[8] = {Cc1, Cc2, Cc3, Vc1, Vc2, Vc3, 0x11, 0x00};

// PROTOCOL VERSION Y NOMINAL VOLTAGE

byte ProVrs[8] = {0x2B, 0x11, 0x20, 0x00, 0x00, 0x02, 0x00, 0x00};
byte NomV[8] = {0x2B, 0x15, 0x20, 0x00, 0x18, 0x00, 0x00, 0x00};

// BMS SERIAL NUMBER

byte SN1[8] = {0x21, 0x14, 0x20, 0x00, 0x10, 0x00, 0x00, 0x00};
byte SN2[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
byte SN3[8] = {0x10, 0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04};
byte SN4[8] = {0x0B, 0x05, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};

// BMS Manufacter ID

byte MFID[8] = {0x2F, 0x10, 0x20, 0x00, 0x07, 0x00, 0x00, 0x00};

  

void setup()
{
//_______________________________________________________PINMODES
pinMode(LED1,OUTPUT); 
pinMode(LED2,OUTPUT); 
pinMode(LED3,OUTPUT); 
pinMode(LED4,OUTPUT); 
pinMode(CONTACTOR_1,OUTPUT); 
pinMode(CONTACTOR_2,OUTPUT); 
pinMode(CONTACTOR_3,OUTPUT); 
pinMode(IN1,INPUT);
pinMode(IN2,INPUT); 


//_______________________________________________________SALIDAS
digitalWrite(LED1,LOW);    // ROJO
digitalWrite(LED2,HIGH);     // VERDE
digitalWrite(LED3,LOW);    // ROJO
digitalWrite(LED4,HIGH);     // VERDE
digitalWrite(CONTACTOR_1,LOW); 
digitalWrite(CONTACTOR_2,LOW); 
digitalWrite(CONTACTOR_3,HIGH);
  
  Serial.begin(115200);

  // Initialize MCP2515 running at 8MHz with a baudrate of 125kb/s and the masks and filters disabled.
  
   if (CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
  }
  CAN0.setMode(MCP_NORMAL); 
  pinMode(CAN0_INT, INPUT); 
  
  CAN0.init_Mask(0,0,0x01FF0000);  
  CAN0.init_Filt(0,0,0x012E0000);
  CAN0.init_Filt(1,0,0x012C0000); 
 // CAN0.init_Filt(1,0,0x012E0000); 
  
  CAN0.init_Mask(1,0,0x00FF0000);
  CAN0.init_Filt(2,0,0x00D20000);
  CAN0.init_Filt(3,0,0x00C80000);
  CAN0.init_Filt(4,0,0x00C90000);  
  CAN0.init_Filt(5,0,0x00C90000); 



  // Initialize MCP2515 (MODULO 2) running at 8MHz with a baudrate of 250kb/s and the masks and filters disabled.
  
  if(CAN1.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) 
    Serial.println("MCP2515 2 Initialized Successfully!");
  else Serial.println("Error 2 Initializing MCP2515...");
  CAN1.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  
}


void loop()
{

  byte sendNMT = CAN0.sendMsgBuf(0x000, 0, 2, NMT);

  byte sendHEART = CAN0.sendMsgBuf(0x73F, 0, 1, HEART);

  lecturaCan();
  AdaptaValores();
  imprimir();

  primer_envio = 0;

  while(pinStatus == B00000011)
  {
    if(primer_envio == 0){
      Can_Open();
      primer_envio = 1;
    }

    byte sendNMT = CAN0.sendMsgBuf(0x000, 0, 2, NMT);

    byte sendHEART = CAN0.sendMsgBuf(0x73F, 0, 1, HEART);

    byte sndStat = CAN0.sendMsgBuf(0x201, 0, 8, RPDO1);
  
    byte sndStat2 = CAN0.sendMsgBuf(0X301, 0, 8, RPDO2);
  
    byte sndStat3 = CAN0.sendMsgBuf(0X401, 0, 8, RPDO3);

    delay(50);

    lecturaCan();
    AdaptaValores();
    imprimir();
    
  }
  
  delay(100);
}

void lecturaCan() {

  if (!digitalRead(CAN0_INT))  {
    CAN_status = 0;
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    Serial.println(rxId,HEX);
    for (byte i = 0; i < len; i++) {

      if (rxId == 0x12C) { 
        pinStatus = byte(rxBuf[0]);
        SOH = word(rxBuf[4],rxBuf[5])/50;
        //Serial.println("AAAAAAAAAAAAAAAAAAAAA");
      }
      if (rxId == 0x0D2) { 
        pinStatus = byte(rxBuf[2]);
        SOH = word(rxBuf[3],rxBuf[4])/50;
        //Serial.println("AAAAAAAAAAAAAAAAAAAAA");
      }

      if (rxId == 0x0C9) 
      { 
        ta = byte(rxBuf[4]);
        Serial.print("Temperatura ");Serial.println(ta);
        pinStatus = byte(rxBuf[7]);
      }

      if (rxId == 0x0C8) { 
        SOC = word(rxBuf[0],rxBuf[1])/50;
        Vbat = word(rxBuf[2],rxBuf[3])*100;
        pack_current_pos= (word(rxBuf[4], rxBuf[5])*65536  + word(rxBuf[6], rxBuf[7])); //32 bits
        
         if (pack_current_pos > 2147483647) {
              pack_current_pos = (4294967295 - pack_current_pos) + 1;
              } else {
                      pack_current_pos = pack_current_pos;  
                     }

         pack_current_pos = pack_current_pos/100;
      }

      if (rxId == 0x12E) { //848 = 350 en decimal
        Icarga = word(rxBuf[0],rxBuf[1])*100;
        Vcarga = word(rxBuf[2],rxBuf[3])*100;
      }
    }
  }
  else
  {
    CAN_status = 1;
  }

  if (!digitalRead(CAN1_INT))  {
    CAN_status = 0;
    
    CAN1.readMsgBuf(&rxId, &len, rxBuf);
    //Serial.println(rxId);
    for (byte i = 0; i < len; i++) {
      if (rxId == 0x010) {
        pinStatusCharger = byte(rxBuf[0]);
        
      }
    }
  }
  else
  {
    CAN_status = 1;
  }
}

void AdaptaValores() {

  Cm1 = pack_current_pos & 0xff;
  Cm2 = pack_current_pos >>8& 0xff;
  Cm3 = pack_current_pos >>16& 0xff;

  Vm1 = Vbat & 0xff;
  Vm2 = Vbat >>8& 0xff;
  Vm3 = Vbat >>16& 0xff;

  Cc1 = Icarga & 0xff;
  Cc2 = Icarga >>8& 0xff;
  Cc3 = Icarga >>16& 0xff;

  Vc1 = Vcarga & 0xff;
  Vc2 = Vcarga >>8& 0xff;
  Vc3 = Vcarga >>16& 0xff;

  RPDO1[0] = SOC;
  RPDO1[1] = SOH;

  RPDO2[0] = Cm1;
  RPDO2[1] = Cm2;
  RPDO2[2] = Cm3;
  RPDO2[3] = Vm1;
  RPDO2[4] = Vm2;
  RPDO2[5] = Vm3;
  RPDO2[6] = ta;
  
  RPDO3[0] = Cc1;
  RPDO3[1] = Cc2;
  RPDO3[2] = Cc3;
  RPDO3[3] = Vc1;
  RPDO3[4] = Vc2;
  RPDO3[5] = Vc3;

}

void Can_Open(){

  delay(200);
  
  byte sendHEART = CAN0.sendMsgBuf(0x73F, 0, 1, HEART);

  delay(20);

  byte sndStat4 = CAN0.sendMsgBuf(0X601, 0, 8, ProVrs);

  delay(20);

  byte sndStat5 = CAN0.sendMsgBuf(0X601, 0, 8, NomV);

  delay(20);
      
  byte sndStat6 = CAN0.sendMsgBuf(0X601, 0, 8, SN1);

  delay(20);
   
  byte sndStat7 = CAN0.sendMsgBuf(0X601, 0, 8, SN2);

  delay(20);
   
  byte sndStat8 = CAN0.sendMsgBuf(0X601, 0, 8, SN3);

  delay(20);
   
  byte sndStat9 = CAN0.sendMsgBuf(0X601, 0, 8, SN4);

  delay(20);
  
  byte sndStat10 = CAN0.sendMsgBuf(0X601, 0, 8, MFID);
 
  delay(50);   // send data per 100ms
  
}

void imprimir() {

//Serial.print("ID350"," ",Cm1," ",Cm2," ",Cm3," ",Vm1," ",Vm2," ",Vm3);
Serial.print("ID350");
Serial.print(" ");
Serial.print(Cm1,HEX);
Serial.print(" ");
Serial.print(Cm2,HEX);
Serial.print(" ");
Serial.print(Cm3,HEX);
Serial.print(" ");
Serial.print(Vm1,HEX);
Serial.print(" ");
Serial.print(Vm2,HEX);
Serial.print(" ");
Serial.println(Vm3,HEX);

Serial.print("ID360");
Serial.print(" ");
Serial.print(Cc1,HEX);
Serial.print(" ");
Serial.print(Cc2,HEX);
Serial.print(" ");
Serial.print(Cc3,HEX);
Serial.print(" ");
Serial.print(Vc1,HEX);
Serial.print(" ");
Serial.print(Vc2,HEX);
Serial.print(" ");
Serial.print(Vc3,HEX);
Serial.print(" ");
Serial.print(ta,HEX);
Serial.print(" ");
Serial.print(SOC,HEX);
Serial.print(" ");
Serial.println(SOH,HEX);

Serial.print("Estatus");
Serial.print(" ");
Serial.println(pinStatus,HEX);

Serial.print("Estatus Cargador");
Serial.print(" ");
Serial.println(pinStatusCharger,HEX);
  
}