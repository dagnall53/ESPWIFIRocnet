// Stuff to work the ports on the ESP


void READ_PORT( int i) {
  boolean STATE;
  uint8_t TEMP;
  uint16_t senderADDR;
  if (((NodePortType[i] & 0x01) == 1) && ((NodeChanneloptions[i] & 32) != 32))  { // only do this if this port is an "INPUT" and not a "SERVO"
    if (Debounce(i)) { // debounce is true if switch changed

      if (((NodePortType[i] & 32) == 32)  && (digitalRead(NodeMcuPortD[i]) == 1)) {
        ButtonState[i] = !ButtonState[i]; // only change on one state..
      }
      if ((NodePortType[i] & 32) == 32)   {
        STATE = ButtonState[i];
      }
      else {
        STATE = digitalRead(NodeMcuPortD[i]);
      }
      if ((NodePortType[i] & 64) == 64)   {
        STATE = !STATE;
      }
#if _SERIAL_DEBUG
      Serial.print ("Change on IO port : ");
      Serial.print(i);
      Serial.print(" State");
      Serial.println(STATE);
#endif
      SendPortChange(RocNodeID, STATE, i);

      if (STATE && (NodePortDelay[i] >= 1)) {
        PortTimingDelay[i] = millis() + (NodePortDelay[i] * 10);
      }

    }
  }
}// end read port



void ReadInputPorts() {

  // Check for port changes +++++++++++++++++++++++++++++++++++++
  if (!bReaderActive) {
    for (int i = 1 ; i <= 8; i++) {
      READ_PORT(i);
    }
  }
  else {
    READ_PORT(1);
    // only port 1  is fully user available.
  }
}

void PortMode(int i) {
  boolean hardset;
  hardset = false;
  Serial.print (" Port :");
  Serial.print (i);
   #ifdef _EPHAudio
                      if (i== DAC_DIN){
                      Serial.print (" used for I2C Audio D IN");
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 0; hardset =true;
                      }
                      if (i== DAC_BCLK){
                      Serial.print (" used for I2C Audio BCLK");
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 0; hardset =true;
                      }
                      if (i== DAC_LRC){
                      Serial.print (" used for I2C Audio LRC");
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 0; hardset =true;
                      }

    #endif
  if (i == 4) {
    #ifndef _EPHAudio
    Serial.println (" is BlueLed and Output only");
    pinMode(BlueLed, OUTPUT);
    NodePortType[i] = NodePortType[i] & 0xFE;
    #endif
    #ifdef _EPHAudio
    if (i== DAC_LRC){
                      Serial.println (" used for I2C Audio LRC");
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 0; hardset =true;
               }
    #endif
  }
  else { 
     
    //in 1 out 0
    if ((NodeChanneloptions[i] & 32) == 32) {
      Serial.print (" is Servo");
      pinMode(NodeMcuPortD[i], OUTPUT);
      NodePortType[i] = bitClear (NodePortType[i], 0 );
    }
    else {
      if ((NodePortType[i] & 0x01) == 1) {
        pinMode(NodeMcuPortD[i], INPUT_PULLUP);
        Serial.print (" is input with pullup");
      }
      if ((NodePortType[i] & 0x01) == 0) {
        pinMode(NodeMcuPortD[i], OUTPUT);
        Serial.print (" is output");
      }
    }
     #ifdef _LOCO_SERVO_Driven_Port
                      if (i== _LOCO_SERVO_Driven_Port){
                      Serial.print (" used as LOCO MOTOR ");
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 42;
                      hardset =true;
                      }
                      if(i==BACKLight){
                      Serial.print (" used for Back Light ");  
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 0; 
                      hardset =true;
                      }
                       if(i==FRONTLight){
                      Serial.print (" used for Front Light ");
                      NodePortType[i] = 0;
                      NodeChanneloptions[i] = 0; 
                      hardset =true;   
                      }
     #endif
   
    
    // TODO would be good to add more explicit identification of what ports are set to with different channel options etc..
    if (hardset) {Serial.println( " (Port settings cannot be changed by Rocnet) ");}
    else {
          Serial.print (" NodePortType :");
          Serial.print (NodePortType[i]);
          Serial.print ("  NodeChanneloptions");
          Serial.println (NodeChanneloptions[i]);
    }
  }
}

void PortSetupReport() {
  int i;

  if (!bReaderActive) {
    #ifdef  _RFID
        Serial.println("            Reader Absent ....all ports available");
        Serial.println("");
    #endif 
    for (int i = 1 ; i <= 8; i++) {   // for now, Ignore port 0 as it has drive limitations

      PortMode(i);
    }
  }
  else {
    // only port 4 is user available.
    PortMode(4);
  }
 // Serial.println("all Ports set");
}

