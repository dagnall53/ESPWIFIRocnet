
void DebugMsgSend (char* topic, char* payload) { // use with mosquitto_sub -h 127.0.0.1 -i "CMD_Prompt" -t debug -q 0
  char internal[127];
  int cx;
  if ((CV[47] & 0x01) == 0x01)  {//BIT 0
    Serial.println();
    Serial.print("*Debug Message:");
    Serial.print(payload);
    Serial.println();
  }
  #ifdef _LOCO_SERVO_Driven_Port
  cx = sprintf ( internal, "Time %d:%d:%ds Node:%d Loco:%d (%s) Msg:%s", hrs, mins, secs,RocNodeID, CV[1],Nickname, payload);
  #else
  cx = sprintf ( internal, "Time %d:%d:%ds Node:%d (%s) Msg:%s", hrs, mins, secs,RocNodeID, Nickname, payload);
  #endif
  
  if ((CV[47] & 0x02) == 0x02) {  //  BIT 1
    if ((cx <= 100)) {
      client.publish(topic, internal, strlen(internal));
    }
    if ((cx >= 100) && (strlen(payload) <= 100)) {
      sprintf ( internal, "MSG-%s-", payload);
      client.publish(topic, internal, strlen(internal));
    }// print just one line
    if (strlen(payload) >= 101) {
      sprintf ( internal, "Node:%d Loco:%d Time %d:%d:%ds Msg TOO Big to print", RocNodeID, CV[1], hrs, mins, secs);
      client.publish(topic, internal, strlen(internal));
    }
  }
}


int rnSenderAddrFromPacket(unsigned char* rn, int seven) {
  return rn[RN_PACKET_SNDRL] + rn[RN_PACKET_SNDRH] * (seven ? 128 : 256);
}

int rnReceipientAddrFromPacket(unsigned char* rn, int seven) {
  return rn[RN_PACKET_RCPTL] + rn[RN_PACKET_RCPTH] * (seven ? 128 : 256);
}


int getTwoBytesFromMessageHL( uint8_t* msg, uint8_t highloc) {
  return msg[highloc + 1] + msg[highloc] * (256);
}



/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void SetWordIn_msg_loc_value(uint8_t* msg, uint8_t firstbyte, int value) {
  msg[firstbyte + 1] = value % 256; //low byte
  msg[firstbyte] = value / 256; // order is high first then low
}
int IntFromPacket_at_Addr(uint8_t* msg, uint8_t highbyte) { // example IntFromPacket_at_Addr(recMessage,Recipient_Addr))
  return msg[highbyte + 1] + msg[highbyte] * 256;
}

void dump_byte_array(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
  Serial.println("");
}
void dump_byte(uint8_t buffer) {

  Serial.print(buffer < 0x10 ? " 0" : " ");
  Serial.print(buffer, HEX);
  Serial.print("h ");

}


uint8_t lnCalcCheckSumm(uint8_t *cMessage, uint8_t cMesLen) {
  unsigned char ucIdx = 0;
  char cLnCheckSum = 0;

  for (ucIdx = 0; ucIdx < cMesLen - 1; ucIdx++) //check summ is the last byte of the message
  {
    cLnCheckSum ^= cMessage[ucIdx];
  }

  return (~cLnCheckSum);
}



uint16_t AddrFull (uint8_t HI, uint8_t LO) {
  uint16_t FullAddress;
  FullAddress = 128 * (HI & 0x0F) + LO;
  return (FullAddress);
}

boolean Debounce (int i) {  // Tests for inputs having changed,
  boolean SwitchSensed;
  SwitchSensed = false ;
  if ((lastButtonState[i] != digitalRead(NodeMcuPortD[i])) && (millis() >= PortTimingDelay[i])) {
    SwitchSensed = true;
    lastButtonState[i] = digitalRead(NodeMcuPortD[i]);
    //    DebounceCount[i] =0 ;
    PortTimingDelay[i] = millis() + ((NodePortDelay[i] * 10) + 1);
  }


  return (SwitchSensed);
}
/*boolean Debounce (int i) {  // Tests for inputs having changed,
  boolean SwitchSensed;
  SwitchSensed = false ;
   if (lastButtonState[i] != digitalRead(NodeMcuPortD[i])) { DebounceCount[i]= DebounceCount[i]+1; }
   if ((DebounceCount[i] >= DebounceDelayCount )&& (millis()>=PortTimingDelay[i])) {
          SwitchSensed = true;
          lastButtonState[i] = digitalRead(NodeMcuPortD[i]);
          DebounceCount[i] =0 ;
          }


  return (SwitchSensed);
  }
*/


void copyUid (byte *buffOut, byte *buffIn, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    buffOut[i] = buffIn[i];
  }
  if (bufferSize < UID_LEN) {
    for (byte i = bufferSize; i < UID_LEN; i++) {
      buffOut[i] = 0;
    }
  }
}

void calcAddrBytes(uint16_t uiFull, uint8_t *uiLo, uint8_t *uiHi) {
  *uiHi = (((uiFull - 1)  >> 8) & 0x0F) + (((uiFull - 1) & 1) << 5);
  *uiLo = ((uiFull - 1) & 0xFE) / 2;
}

/**
   Function used to compare two RFID UIDs
*/
bool compareUid(byte *buffer1, byte *buffer2, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    if (buffer1[i] != buffer2[i]) {
      return (false);
    }
  }
  return (true);
}


void Show_MSG() {
  if (Message_Length >= 1) {
#if _SERIAL_SUBS_DEBUG
    Serial.println();
    Serial.print(F("SHOW Message :"));
    Serial.print(F(" Len:"));
    Serial.print(Message_Length);

    dump_byte_array(recMessage, Message_Length);

#endif
  }
}

int FlashHL(int state, int port) {
  int value;
  if (state == 0) {
    value = (NodeChanneloffposH[port] * 256) + NodeChanneloffposL[port];
  }
  if (state == 1) {
    value = (NodeChannelonposH[port] * 256) + NodeChannelonposL[port];
  }
  value = ((value / 4)); // set the PWM  range 0-1023
  return (value) ;
}

void FLASHING() {
  boolean set;
  for (uint8_t port = 1; port <= 8; port++) {
    set = !(ButtonState[port]); // get phasing right so "no flash" is same high or low state as the non blink state
    if ((NodePortType[port] & 193) == 192) { set = !set; }// invert action if "invert", "an output" and "port blink" set ?  
    
      if (((NodePortType[port] & 129) == 128)&&( ((NodeChanneloptions[port] & 32) != 32))) { // make sure its an output '0' and has port blink '128' set! and is NOT a Servo
      if ((set) && (millis() >= PortTimingDelay[port]) && (NodePortDelay[port] >= 1)) {
       
        if ((NodeChanneloptions[port] & 128) == 128) {// is  a 'channel' blink set  if so, use PWM outputs
          if (SDemand[port] == FlashHL(1, port)) {
            SDemand[port] = FlashHL(0, port);
          }
          else {
            SDemand[port] = FlashHL(1, port);
          }
          analogWrite(NodeMcuPortD[port], SDemand[port]);
        }
        else {                                        // its a digital output so just invert current state
          digitalWrite(NodeMcuPortD[port], !digitalRead(NodeMcuPortD[port]));
        }
        PortTimingDelay[port] = millis() + (NodePortDelay[port] * 10);
      }
      

    }
  }
}


void DetachServo(int i) {
   switch (i) {
    case -1: {
        Serial.println(" ---- Switching OFF ALL Servos---");
      //  Loco_LOCO_SERVO_Driven_Port.detach();
        myservo1.detach();
        myservo2.detach();
        myservo3.detach();
        myservo4.detach();
        myservo5.detach();
        myservo6.detach();
        myservo7.detach();
        myservo8.detach();
      }
      break;
    case 1: {
        myservo1.detach();
      }
      break;
    case 2: {
        myservo2.detach();
      }
      break;
    case 3: {
        myservo3.detach();
      }
      break;
    case 4: {
        myservo4.detach();
      }
      break;
    case 5: {
        myservo5.detach();
      }
      break;
    case 6: {
        myservo6.detach();
      }
      break;
    case 7: {
        myservo7.detach();
      }
      break;
    case 8: {
        myservo8.detach();
      }
      break;
  }
 
}

void DETACH() {
 int i;
 for (int i = 1 ; i <= 8; i++) { //up to 8 servos.. but not  loco servo!. 
    if ( (millis() >= ServoOffDelay[i])  && ( (NodeChanneloptions[i] & 32) == 32) ) { // double check its actually a servo !!, as this is called from two places{
        ServoOffDelay[i]=millis()+OneDay;  // set one day ahead as a simple way to avoid setting it again next time around... Moving the servo will reset it to millis anyway
      
        Serial.println();
        Serial.print(" Switching OFF Servo:");
        Serial.println(i);
        DetachServo(i);
            
                                                                                   }
                                  }
 }

void SetServo( int i, uint16_t value) { // uses 0-180
  int CurrentLocoSpeed;
  if ((NodeChanneloptions[i] & 32) == 32) { // double check, as this is called from two places
    ServoLastPos[i]=value;
/*
#if _SERVO_DEBUG
    //NOTE Sending serial messages adds about 200ms delay in the loop !.
    cx = sprintf ( internal, "(280) Set Servo[%d] to {%d} ",i,value);
    DebugMsgSend("debug", internal);
#endif
*/
#ifdef  _LOCO_SERVO_Driven_Port
if (i==_LOCO_SERVO_Driven_Port){
  CurrentLocoSpeed=0; //~ MPH?
  if (value>=91){CurrentLocoSpeed=value-90;} //0-90...
  if (value<=89){CurrentLocoSpeed=90-value;}
  ChuffQuarterPeriod= ChuffFactor/(CurrentLocoSpeed);
}

#endif

    switch (i) {
    
      case 1: {
          if (!myservo1.attached()) { 
            myservo1.attach(D1);
          }
          myservo1.write(value);
          }
        break;
      case 2: {
          if (!myservo2.attached()) {
            myservo2.attach(D2);
          }
          myservo2.write(value);
        }
        break;
      case 3: {
          if (!myservo3.attached()) {
            myservo3.attach(D3);
          }
          myservo3.write(value);
        }
        break;
      case 4: {
          if (!myservo4.attached()) {
            myservo4.attach(D4);
          }
          myservo4.write(value);
        }
        break;
      case 5: {
          if (!myservo5.attached()) {
            myservo5.attach(D5);
          }
          myservo5.write(value);
        }
        break;
      case 6: {
          if (!myservo6.attached()) {
            myservo6.attach(D6);
          }
          myservo6.write(value);
        }
        break;
      case 7: {
          if (!myservo7.attached()) {
            myservo7.attach(D7);
          }
          myservo7.write(value);
        }
        break;
      case 8: {
          if (!myservo8.attached()) {
            myservo8.attach(D8);
          }
          myservo8.write(value);
        }
        break;
    }  // switch
   
  
                                   }// double check its a servo 
}// end 

uint16_t servoLR(int state, int port) {
  int value;
  value = 375; // == mid , 90 degrees
  if (state == 0) {
    value = (NodeChanneloffposH[port] * 256) + NodeChanneloffposL[port];
  }
  if (state == 1) {
    value = (NodeChannelonposH[port] * 256) + NodeChannelonposL[port];
  }
  value = ((value - 150) * 2) / 5; // set the servo immediately range 150-600 = 0 to 180
  return   (value);
}

void SERVOS() {              // attaches and detaches servos, accelerates to demanded positions etc..

  int ServoPositionNow;
  int offset;
  int steps;
  int SERVOSET;
  int set;
  uint32_t LocalTimer;
  LocalTimer=millis();
  for (int i = 1 ; i <= 8; i++) { //up to 8 servos..  servo '0' which was the LOCO motor servo ?
    if ((NodeChanneloptions[i] & 32) == 32) { //only if this port is a "servo"... To address a channel instead of a port the port type servo must be set on the interface tab of switches and outputs
      if (millis() >= (NodeChannelLastUpdated[i] + (NodeChanneloptions[i] & 15) * 10)) { // do update only at the required delay update rate
        ServoPositionNow=ServoLastPos[i];
        offset = SDemand[i] - ServoLastPos[i];
 //      #if _SERVO_DEBUG
 //      cx = sprintf ( internal, " ( 375) Servo[%d] Last Pos:%d  Demand:%d Offset:%d ",i,ServoLastPos[i], SDemand[i],offset);
 //           DebugMsgSend("debug", internal); 
//       #endif
        if ((abs(offset) <= 3) && ((NodePortType[i] & 129) == 128)  && (ButtonState[i] == 1) ) { //changes demand if within 3 of demand and reversing...needs inv to operate on buttonstate, ???as it will not switch off if inv..
//          #if _SERVO_DEBUG
//              cx = sprintf ( internal, " ( 372) Using Reversing code");
 //             DebugMsgSend("debug", internal); 
//          #endif
          ServoOffDelay[i] = LocalTimer + 2000; // not sure if this is needed, but we will not be switching off soon! 
          if (SDemand[i] == servoLR(1, i)) {
                         SDemand[i] = servoLR(0, i);
                                            }
               else {
                     SDemand[i] = servoLR(1, i);
                    }
                                   } //if abs offset <=3
     
        if (offset != 0) {
          ServoOffDelay[i] = LocalTimer + 2000; // 2s sec off timer start but only do if the servo is NOT in the final position, when its in final position, it does NOT update
          if (ButtonState[i]) {
            steps = NodeChannelonsteps[i];
          }
          else {
            steps = NodeChanneloffsteps[i];
          }
          if (steps == 0) {
            SERVOSET = SDemand[i];
          #if _SERVO_DEBUG
              cx = sprintf ( internal, " ( 403 ) Set Immediately");
              DebugMsgSend("debug", internal); 
          #endif
          }
          else {
            if (abs(offset) >= steps) { 
//     #if _SERVO_DEBUG
 //             cx = sprintf ( internal, " ( 410 ) OFFSET:%d   STEPS:%d",offset,steps);   
 //             DebugMsgSend("debug", internal); 
//     #endif
              offset = (offset * steps) / abs(offset);  //offset is now either the error or steps, whichever is less
//    #if _SERVO_DEBUG
 ////             cx = sprintf ( internal, " ( 415 ) OFFSET:%d ServoPositionNow:%d  ",offset,ServoPositionNow);
  //            DebugMsgSend("debug", internal); 
  //   #endif 
                                      }
                                      
            SERVOSET = ServoPositionNow + offset;
            if ((SERVOSET) >= 180) {
              SERVOSET = 180;
            }
            if ((SERVOSET) <= 0  ) {
              SERVOSET = 0; // range for servo is 0-180
            }
          }
#if _SERVO_DEBUG
//NOTE Sending serial messages adds about 200ms delay in the loop !.
 //   cx = sprintf ( internal, "( 425 ) Servo[%d] SERVOSET:%d",i,SERVOSET);
 //   DebugMsgSend("debug", internal);
    cx = sprintf ( internal, "Time(%d) Servo[%d] to {%d} by moving:%d steps from:%d towards:%d",LocalTimer,i,SDemand[i],offset,ServoPositionNow,SDemand[i]);
    DebugMsgSend("debug", internal);
#endif
         // NodeChannelLastUpdated[i] = (NodeChannelLastUpdated[i] + (NodeChanneloptions[i] & 15) * 10);
          NodeChannelLastUpdated[i] = LocalTimer ;
          SetServo( i, SERVOSET);

        } // offset != 0
      } // time to do an update
    } //if servo
  } // end i 1..8 loop

}//end servos

void FlashMessage (char* msg, int p, int ON, int off) {
  Serial.println(msg);
  for (int i = 0; i <= p; i++) {
    //  Serial.print("+");
    #ifndef _EPHAudio
    digitalWrite (BlueLed , LOW) ; ///   turn on
    delay(ON);
    digitalWrite (BlueLed , HIGH) ; ///   turn OFF
    delay(off);
    #endif
  }

}
void SetPortPinIndex() {
  // set my indexed port range for indexed use later
  NodeMcuPortD[0] = D0;
  NodeMcuPortD[1] = D1;
  NodeMcuPortD[2] = D2;
  NodeMcuPortD[3] = D3;
  NodeMcuPortD[4] = D4;
  NodeMcuPortD[5] = D5;
  NodeMcuPortD[6] = D6;
  NodeMcuPortD[7] = D7;
  NodeMcuPortD[8] = D8;
  NodeMcuPortD[9] = D9;
  NodeMcuPortD[10] = D10;

}
void PrintTime(String MSG) {
      Serial.println("");
      Serial.print("@");
      Serial.print(hrs);
      Serial.print(":");
      if (mins <= 9){
        Serial.print("0");
      }
      Serial.print(mins);
      Serial.print(":"); 
      if (secs <= 9){
        Serial.print("0");
      }
      Serial.print(secs);
      Serial.print(" ");
      Serial.print(MSG);

}

