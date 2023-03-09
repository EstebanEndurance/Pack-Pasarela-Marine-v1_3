/*********************************************************************************************************
  ENDURANCE MOTIVE S.L
  FRONIUS COMUNICATION - PASARELA VERSION PARA ESP32 V0.1 - MODO TORTUGA ERROR LEDS - PASARELA TRNASISTORES
  DPT DESARROLLO ELECTRÓNICO E I+D
  16/06/2021
*********************************************************************************************************/


/*
_________________________________________________________ PIN OUT 
                                         +----------------------------------------+
                                         |                                        |
                                         |                                        |
                                         +----------------------------------------+
                                    GND1 | [ ]                                [ ] | GND             
                                    3,3V | [ ]                                [ ] | GPIO23
                                      EN | [ ]                                [ ] | GPIO22
                                  GPIO36 | [ ]                                [ ] | GPIO1  /TXD
                                  GPIO39 | [ ]                                [ ] | GPIO3  /RXD
                                  GPIO34 | [ ]                                [ ] | GPIO21
                                  GPIO35 | [ ]             ESP32              [ ] | NC
                                  GPIO32 | [ ]                                [ ] | GPIO19
                                  GPIO33 | [ ]                                [ ] | GPIO18
                                  GPIO25 | [ ]                                [ ] | GPIO5
                    LED1----------GPIO26 | [ ]                                [ ] | GPIO17-----------LED4
                    LED2----------GPIO27 | [ ]                                [ ] | GPIO16-----------LED3
                                  GPIO14 | [ ]                                [ ] | GPIO4
                                  GPIO12 | [ ]                                [ ] | GPIO0
                                         |     [ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]     |  
                                         +------+--+--+--+--+--+--+--+--+--+------+
                                     GND________|  |  |  |  |  |  |  |  |  |________GPIO2
                                  GPIO13___________|  |  |  |  |  |  |  |___________GPIO15
                                  GPIO9 ______________|  |  |  |  |  |______________GPIO8
                                  GPIO10_________________|  |  |  |_________________GPIO7
                                  GPIO11____________________|  |____________________GPIO6

*/



//_______________________________________________________INCLUDES
#include <mcp_can.h>
#include <SPI.h>

//_______________________________________________________DEFINE´S
#define LED1 26
#define LED2 27
#define LED3 16
#define LED4 17

MCP_CAN CAN0(14);     // Set CS to pin 10
#define CAN0_INT 32

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

MCP_CAN CAN1(15);     // Set CS to pin 7
#define CAN1_INT 4

//VARIABLES DE CARGA CORRIENTE Y VOLTAJE POR BITS

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

// Configuramos el NMT para el paso a estado Operativo

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

//_____________________________variables de error______________________________________________________________________[ST1]
int ReleError = 16; //Inicializamos variable del rele externo Tortuga 2
long error_Status = 0; //Variable de los errores del S_BMS
bool error_Status_flag = 0;
unsigned long tiempo_error = 0;


void setup()
{
//_______________________________________________________PINMODES
pinMode(LED1,OUTPUT); 
pinMode(LED2,OUTPUT); 
pinMode(LED3,OUTPUT); 
pinMode(LED4,OUTPUT); 

//_______________________________________________________SALIDAS
digitalWrite(LED1,HIGH); 

  Serial.begin(115200);

  // Initialize MCP2515 running at 8MHz with a baudrate of 125kb/s and the masks and filters disabled.
  
   if (CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
  }
  CAN0.setMode(MCP_NORMAL); 
  pinMode(CAN0_INT, INPUT);
  
  CAN0.init_Mask(0,0,0x07FF0000);  
  CAN0.init_Filt(0,0,0x03500000); 
  CAN0.init_Filt(1,0,0x03600000); 
  
  CAN0.init_Mask(1,0,0x07FF0000);
  CAN0.init_Filt(2,0,0x02030000); 
  CAN0.init_Filt(3,0,0x03500000); 
  CAN0.init_Filt(4,0,0x03500000); 
  CAN0.init_Filt(5,0,0x03500000);
  
  

  // Initialize MCP2515 (MODULO 2) running at 8MHz with a baudrate of 250kb/s and the masks and filters disabled.
  
  if(CAN1.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) 
    Serial.println("MCP2515 2 Initialized Successfully!");
  else Serial.println("Error 2 Initializing MCP2515...");

  CAN1.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  
//_____________________________DECLARAR SALIDA DEL RELE__________________________________________________________________[ST2]
  pinMode(ReleError,OUTPUT); //Declaramos el Tortuga 2 como salida digital 
  
}


void loop()
{
//_____________________________DEFINIR EN SALIDA DE RELEERRO EN 0_________________________________________________________[ERROR]
  digitalWrite(ReleError,LOW); // Mantenemos en modo normal el Rele Tortuga 2 a LOW 

  byte sendNMT = CAN0.sendMsgBuf(0x000, 0, 2, NMT);

  byte sendHEART = CAN0.sendMsgBuf(0x73F, 0, 1, HEART);

  lecturaCan();
  AdaptaValores();
  imprimir();

  primer_envio = 0;

  while(pinStatus == B00000010 || pinStatus == B00000100 || pinStatus == B00100000)
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

    digitalWrite(ReleError,LOW );

    delay(50);

    lecturaCan();
    AdaptaValores();
    imprimir();
    
  }
  
//________________________________ERRORES_____________________________________//

  while(error_Status_flag == 1 &&  (millis()-tiempo_error) >= 5000)   
  {
    digitalWrite(ReleError,HIGH);
    lecturaCan();
    //imprimir();
    //Serial.print("ERROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOR");
  }
  
  delay(100);
  
}

//____________________________FUNCION LECTURA CAN S______________________________//
void lecturaCan() {

  if (!digitalRead(CAN0_INT))  {
    CAN_status = 0;
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    //Serial.println(rxId);
    for (byte i = 0; i < len; i++) {
      if (rxId == 0x350) {
        pack_current_pos = word(rxBuf[0], rxBuf[1])*100;
        Vbat = word(rxBuf[2], rxBuf[3])*100;
        pinStatus = byte(rxBuf[4]);
        
      }

      if (rxId == 0x360) { 
        Icarga = word(rxBuf[0],rxBuf[1])*100;
        Vcarga = word(rxBuf[2],rxBuf[3])*100;
        ta = byte(rxBuf[4]);
        SOC = byte(rxBuf[5]*2);
        SOH = byte(rxBuf[6]*2);
      }
//_____________________________LER ID ERROR__________________________________________________________________[ERROR]
      if (rxId == 0x203) { 
        error_Status = (word(rxBuf[0], rxBuf[1]) * 65536  + word(rxBuf[2], rxBuf[3]));
        funcion_errores();
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

void funcion_errores() {
  
    if (error_Status == 0){
        tiempo_error = millis();
        error_Status_flag = 0;
    }
    else {
      error_Status_flag = 1;
    }
}

void imprimir() 
{

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

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
