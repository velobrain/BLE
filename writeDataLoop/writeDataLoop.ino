
#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BT = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void setup(void) {
  Serial.begin(9600);
  while(!Serial); 
  randomSeed(analogRead(0));
  BT.begin();
}
int x;
void loop() {
  x = random(20);
  BT.pollACI();
  String s = String(x);
  uint8_t sbuffer[20];
  s.getBytes(sbuffer,20);
  char size = min(20,s.length());
 Serial.println((char *)sbuffer);
 BT.write(sbuffer,size);  
}


