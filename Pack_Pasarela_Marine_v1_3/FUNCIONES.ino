
/*********************************************************************************************************************************
 *                                                           FUNCION LECTURA CAN                                                 *
 *********************************************************************************************************************************/
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


/*********************************************************************************************************************************
 *                                                        FUNCION ADAPTAVALORES                                                  *
 *********************************************************************************************************************************/
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


/*********************************************************************************************************************************
 *                                                        FUNCION CANOPEN_1                                                      *
 *********************************************************************************************************************************/
//_______________________________________________________CAN_OPEN CAN_0
void Can_Open_1(){
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


/*********************************************************************************************************************************
 *                                                        FUNCION CANOPEN_2                                                      *
 *********************************************************************************************************************************/
//_______________________________________________________CAN_OPEN CAN_0
void Can_Open_2(){
  delay(200);
  byte sendHEART = CAN1.sendMsgBuf(0x73F, 0, 1, HEART);
  delay(20);
  byte sndStat4 = CAN1.sendMsgBuf(0X601, 0, 8, ProVrs);
  delay(20);
  byte sndStat5 = CAN1.sendMsgBuf(0X601, 0, 8, NomV);
  delay(20);
  byte sndStat6 = CAN1.sendMsgBuf(0X601, 0, 8, SN1);
  delay(20);
  byte sndStat7 = CAN1.sendMsgBuf(0X601, 0, 8, SN2);
  delay(20);
  byte sndStat8 = CAN1.sendMsgBuf(0X601, 0, 8, SN3);
  delay(20);
  byte sndStat9 = CAN1.sendMsgBuf(0X601, 0, 8, SN4);
  delay(20);
  byte sndStat10 = CAN1.sendMsgBuf(0X601, 0, 8, MFID);
  delay(50);   // send data per 100ms
}



/*********************************************************************************************************************************
 *                                                        FUNCION IMPRIMIR                                                      *
 *********************************************************************************************************************************/
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

/*********************************************************************************************************************************
 *                                                        FUNCION IMPRIMIR                                                      *
 *********************************************************************************************************************************/
void ENTRADAS() {
  IN1State = digitalRead(IN1);
  IN2State = digitalRead(IN2);

// check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (IN1State == LOW) {
    // turn LED on:
    digitalWrite(LED1, HIGH);
  } else {
    // turn LED off:
    digitalWrite(LED1, LOW);
  }

  if (IN2State == LOW) {
    // turn LED on:
    digitalWrite(LED2, HIGH);
  } else {
    // turn LED off:
    digitalWrite(LED2, LOW);
  }


}

