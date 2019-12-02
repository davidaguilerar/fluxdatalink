#include "Wire.h"
#include "HardwareSerial.h"
#include <SoftwareSerial.h>


// Constants to deal with success witn NODEMCU BOARD

#define PIN_WIRE_SDA (4)
#define PIN_WIRE_SCL (5)

static const uint8_t portSDA = PIN_WIRE_SDA;
static const uint8_t portSCL = PIN_WIRE_SCL;
static const uint8_t portLED_BUILTIN = 16;

static const uint8_t portD0 = 16;
static const uint8_t portD1 = 5;
static const uint8_t portD2 = 4;
static const uint8_t portD3 = 0;
static const uint8_t portD4 = 2;
static const uint8_t portD5 = 14;
static const uint8_t portD6 = 12;
static const uint8_t portD7 = 13;
static const uint8_t portD8 = 15;
static const uint8_t portRX = 3;
static const uint8_t portTX = 1;
static const uint8_t portSD3 = 10;
static const uint8_t portSD2 = 9;
static const uint8_t portSD1 = 8;
static const uint8_t portCMD = 11;
static const uint8_t portSD0 = 7;
static const uint8_t portCLK = 6;


SoftwareSerial SerialBT(portD7, portD8, false, 128); // for HC06


void setup() {
  // passthru
  Serial.begin(9600);
  Serial1.begin(9600);
  SerialBT.begin(9600);
}

void loop() {
  // 
  if (Serial.available()) {      // If anything comes in Serial (USB),
    SerialBT.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (SerialBT.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(SerialBT.read());   // read it and send it out Serial (USB)
  }
}
