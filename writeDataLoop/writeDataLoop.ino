
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     
#define ADAFRUITBLE_RST 9

#define REPORTING_PERIOD_MS     1000

PulseOximeter pox;
const int REED_PIN = 3; // Pin connected to reed switch
const int LED_PIN = 13; // LED pin - active-high
const float RADIUS = 0.3; // metres
const float PIE = 3.14159;
const int numberOfMagnets = 1;
const int intervalLength = 5; // 10 seconds
const uint32_t period = intervalLength*1000; // 10 seconds
boolean closed = false;
int revs = 0;
float circumference = 2*PIE*RADIUS;
float arcLength = circumference/numberOfMagnets;
int numIntervals = 0;
int numHeartBeats = 1;

uint32_t tsLastReport = 0;

Adafruit_BLE_UART BT = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void onBeatDetected()
{
    Serial.println("Beat!");
    numHeartBeats += 1;
}
void setup(void) {
  Serial.begin(115200);
  while(!Serial); 
  randomSeed(analogRead(0));
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  BT.begin();
  Serial.print("Initializing pulse oximeter..");
  // Initialize the PulseOximeter instance
  // Failures are generally due to an improper I2C wiring, missing power supply
  // or wrong target chip
  if (!pox.begin()) {
      Serial.println("FAILED");
      for(;;);
  } else {
      Serial.println("SUCCESS");
  }
  // The default current for the IR LED is 50mA and it could be changed
  //   by uncommenting the following line. Check MAX30100_Registers.h for all the
  //   available options.
//   pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  // Register a callback for the beat detection
  pox.setOnBeatDetectedCallback(onBeatDetected);
}
int x;
void loop() {

  float distance = 0;
  for( uint32_t tStart = millis();  (millis()-tStart) < period;  ) {
    pox.update();
    
    int proximity = digitalRead(REED_PIN); // Read the state of the switch
    if (proximity == LOW) // If the pin reads low, the switch is closed.
    {
      Serial.println("Switch closed");
      Serial.print("revs: ");
      Serial.println(revs);
      closed = true;
      digitalWrite(LED_PIN, HIGH); // Turn the LED on
    }
    else
    {
      if (closed == true) {
        revs++;
        distance += arcLength;
      }
      closed = false;
      digitalWrite(LED_PIN, LOW); // Turn the LED off
//      Serial.print("distance: ");
//      Serial.println(distance);
    }
    pox.setOnBeatDetectedCallback(onBeatDetected);
  }
  numIntervals++;
  Serial.println("10 seconds elapsed.....................");
  Serial.print("Distance this interval: ");
  Serial.print(distance);
  Serial.println(" m");
  Serial.print("Speed this interval: ");
  Serial.print(distance/intervalLength);
  Serial.println(" m/s");
  Serial.print("Total distance: ");
  Serial.println(revs*arcLength);
  Serial.print("Average speed: ");
  Serial.println((revs*arcLength)/(numIntervals*intervalLength));
  Serial.print("number of heart beats: ");
  Serial.println(numHeartBeats);
  float totalDistance = revs*arcLength; //the total distance so far
  BT.pollACI();
//  String s = String(x);
//  uint8_t dataBuffer[2];
//  dataBuffer[0] = numHeartBeats;
//  dataBuffer[1] = totalDistance;
//  BT.write(dataBuffer, 2);
  // send using different signs, so distance is always a positive number and heartbeats is always negative 
  BT.write(totalDistance+1);
  BT.write(-numHeartBeats); 

}

