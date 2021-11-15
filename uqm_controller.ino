// Coda UQM PowerPhase 100 Control
// Copied/adapted from GEVCU project
// https://github.com/collin80/GEVCU/blob/master/CodaMotorController.cpp

#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;

const int PRECHARGE_PIN = 4;
const int CONTACTOR_PIN = 5;

mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

char serialBuffer[256];

unsigned char rx_flag = 0;
unsigned long rx_id = 0;
unsigned char rx_len = 0;
unsigned char rx_buf[8];

uint8_t sequence = 0;
const uint8_t swizzleTable[] = { 0xAA, 0x7F, 0xFE, 0x29, 0x52, 0xA4, 0x9D, 0xEF, 0xB, 0x16, 0x2C, 0x58, 0xB0, 0x60, 0xC0, 1 };

int torqueRequest = 5;
uint16_t torqueCommand = 0;

int torqueActual;
int dcVoltage;
int dcCurrent;
int speedActual;
int invTemp;
int rotorTemp;
int statorTemp;

void setup() {
    pinMode(PRECHARGE_PIN, OUTPUT);
    pinMode(CONTACTOR_PIN, OUTPUT);
    digitalWrite(PRECHARGE_PIN, 0);
    digitalWrite(CONTACTOR_PIN, 0);

    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), MCP2515_ISR, FALLING); // start interrupt
    
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(10);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
}

void MCP2515_ISR() {
    rx_flag = 1;
}
unsigned long preChargeStart = 0;
int state = 0; // 0: off, 1: precharging, 2: on
int loopCounter = 0;
int incomingByte = 0;
void loop() {
  sendTorqueCmdMsg();
  //sendWatchdogMsg();
  
  delay(9);

  if (rx_flag) {
      rx_flag = 0;
      while (CAN_MSGAVAIL == CAN.checkReceive()) {
          CAN.readMsgBufID(&rx_id,&rx_len,rx_buf);
          rxMsg(rx_id,rx_len,rx_buf);
      }
  }

  loopCounter++;
  if(loopCounter >= 50) {
    loopCounter = 0;
    sprintf(serialBuffer, "State: %d \t UQM Torque: %d \t DC Voltage: %d \t DC Current: %d \t RPM: %d \t Inverter Temp: %d \t Stator Temp: %d \t Rotor Temp: %d", state, torqueActual, dcVoltage, dcCurrent, speedActual, invTemp, statorTemp, rotorTemp);
    SERIAL_PORT_MONITOR.println(serialBuffer);

    if (SERIAL_PORT_MONITOR.available() > 0) {
      incomingByte = Serial.read();
      if(incomingByte == 'g') {
        digitalWrite(PRECHARGE_PIN, 1);
        preChargeStart = millis();
        state = 1;
      }
      if(incomingByte == 'n') {
        digitalWrite(PRECHARGE_PIN, 0);
        digitalWrite(CONTACTOR_PIN, 0);
        state = 0;
      }
      if(incomingByte == 'u') {
        torqueRequest++;
      }
      if(incomingByte == 'd') {
        torqueRequest--;
      }
    }

    switch(state) {
      case 0:
      break;
      case 1:
        if(millis() - preChargeStart > 5000) {
          digitalWrite(CONTACTOR_PIN, 1);
          digitalWrite(PRECHARGE_PIN, 0);
          state = 2;
        }
      break;
      case 2:
      break;
    }
  }
}

unsigned char torqueCmdMsg[5] = {0, 0, 0, 0, 0};
void sendTorqueCmdMsg() {
  torqueCmdMsg[0] = 0x00; //First byte is always zero.
  torqueCmdMsg[1] = 0x80; //1000 0000
  torqueCmdMsg[1] |= 0x20; //xx10 0000
  
  sequence+=1; //Increment sequence
  if (sequence==8){sequence=0;} //If we reach 8, go to zero
    torqueCmdMsg[1] |= sequence; //This should retain left four and add sequence count
  
  uint16_t torqueCommand=32128; //Set our zero offset value -torque=0

  torqueCommand += torqueRequest*10;
  
  torqueCmdMsg[3] = (torqueCommand & 0xFF00) >> 8;  //Stow torque command in bytes 2 and 3.
  torqueCmdMsg[2] = (torqueCommand & 0x00FF);
  torqueCmdMsg[4] = genCodaCRC(torqueCmdMsg[1], torqueCmdMsg[2], torqueCmdMsg[3]); //Calculate security byte        
  CAN.sendMsgBuf(0x204, 0, 5, torqueCmdMsg);
}

unsigned char watchdogMsg[8] = {0xa5, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00};
void sendWatchdogMsg() {
  CAN.sendMsgBuf(0x207, 0, 8, watchdogMsg);
}

void rxMsg(unsigned long id, unsigned char len, unsigned char *buf) {
  
  switch (id) 
  {
    case 0x209:  //Accurate Feedback Message  
      torqueActual =  ((((buf[1] * 256) + buf[0])-32128))/10;
      dcVoltage = (((buf[3] * 256) + buf[2])-32128)/10;
      dcCurrent = (((buf[5] * 256) + buf[4])-32128)/10;
      speedActual = abs((((buf[7] * 256) + buf[6])-32128)/2);
      
    break;
    case 0x20E:     //Temperature Feedback Message
      invTemp = (buf[2]-40);
      rotorTemp = (buf[3]-40);
      statorTemp = (buf[4]-40);
      break;
    case 0x20F:    //CAN Watchdog Status Message           
      sendWatchdogMsg(); //If we get a Watchdog status, we need to respond with Watchdog reset
      break;
  }

}


uint8_t genCodaCRC(uint8_t cmd, uint8_t torq_lsb, uint8_t torq_msb) 
{
  int counter;
  uint8_t crc;
  uint16_t temp_torq = torq_lsb + (256 * torq_msb);
  crc = 0x7F; //7F is the answer if bytes 3 and 4 are zero. We build up from there.

  //this can be done a little more efficiently but this is clearer to read
  if (((cmd & 0xA0) == 0xA0) || ((cmd & 0x60) == 0x60)) temp_torq += 1;

  //Not sure why this happens except to obfuscate the result
  if ((temp_torq % 4) == 3) temp_torq += 4;

  //increment over the bits within the torque command
  //and applies a particular XOR for each set bit.
  for (counter = 0; counter < 16; counter++)
  {
    if ((temp_torq & (1 << counter)) == (1 << counter)) crc = (byte)(crc ^ swizzleTable[counter]);
  }
  return (crc);
}