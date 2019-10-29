/*
   LiCor GasAnalyzer Serial to Bluetooth/USB converter with addtitinal sensor input.
   To be used with FluxPuppy App.

   This program is written to be used with an ESP32 OLED development board.
   David Basler, 2018

   Modified by: David Aguilera-Riquelme
   Changes:
    - Support for two ADS1115 analog-to-digital converters, used to measure PAR, Air Temperature and Canopy Temperature
    - Changed to NodeMCU Amica microcontroller, based on ESP8266 + HC06 Bluetooth dongle
*/

/*
   ESP8266

*/
extern "C" {
#include "user_interface.h"
}
#include "Wire.h"
#include "HardwareSerial.h"
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <string.h>
#include <Adafruit_ADS1015.h>
#include <stdio.h>
#include <assert.h> 

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


//#################### Serial connection

//SoftwareSerial SerialLicor(portSD0,portCLK,false,128);
SoftwareSerial SerialBT(portD7, portD8, false, 128); // for HC06


//## TIMER

os_timer_t myTimer;
bool HzInterrupt = false;

void timerCallback(void *pArg) {
  HzInterrupt = true;
} // End of timerCallback

void user_init(void) {
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1000, true);
}

//#################### GLOBAL CONTROL VRIABLES


bool new_data_ready = false;

// Serial buffers
char inbuffer_licor[750];      // For Licor ON Serial1
int sbl = 0;
char FPbuffer[1023]; // Final Sting to be submitted to Flux Puppy
char ExtraBuffer[255];
int len;

// String parser
int data[25];
int datalen[25];
int tags [25];
int taglen[25];

// Current Values
float co2value = NAN; // FROM LiCor
float h2ovalue = NAN; // FROM LiCor
float pressvalue = NAN; // FROM LiCor (Cell Pressure)

float tempvalue = NAN; // FROM 107 temperature probe
float parvalue = NAN; // FROM LI-190R
float tptempvalue = NAN; // FROM SI-111 (thermopile)
float tstempvalue = NAN; // FROM SI-111 (thermistor)
float canopyvalue = NAN; //FROM SI-111 and computed


//#################### 1HZ INTERRUPT
//Ticker timer1;
void itsasecond() {
  HzInterrupt = true;
}

//#################### EXTRA SENSOR DATA , integration in data string for FLUX PUPPY
//
#define ADS_1 0x49
#define ADS_2 0x48

Adafruit_ADS1115 ads1(ADS_1);  // construct an ads1115 at address 0x49
Adafruit_ADS1115 ads2(ADS_2);  // construct an ads1115 at address 0x48

float scaleads1 = 0.0078125; // mV/number
float scaleads2 = 0.125; // mV/number

bool i2cReady(uint8_t adr) {
  uint32_t timeout = millis();
  bool ready = false;
  while ((millis() - timeout < 100) && (!ready)) {
    Wire.beginTransmission(adr);
    ready = (Wire.endTransmission() == 0);
  }
  return ready;
}

float volt107   = NAN;   //107 temperature probe
float voltsi111a = NAN;  //thermopile for SI-111
float voltsi111b = NAN; //thermistor for SI-111
float voltli190r = NAN; //LI-190R PAR radiometer
float refvolt = 3300; // 3.3V applied to ADS1115

//################### CALIBRATION CONSTANTS FOR SPECIFIC SENSORS, OBTAINED FROM ITS FACTORY CALIBRATION SHEET
// THIS CONSTANTS ARE UNIQUE FOR EACH SENSOR. YOU MUST EDIT THIS PART OF CODE
float licor_calib = 245.42; //calibration constant for LI-190R Serial Number Q105952, connected with an 2290 Millivolt Adapter, units: umol/s/m^2 /mV
float m0 = 1313790000 ; float m1 = 7203270; float m2 = 56191.3;     // calibration polynomial coefficients for SI-111 Serial Number 6798
float b0 =   -8736140 ; float b1 =  146949; float b2 =  3329.97;    // calibration polynomial coefficients for SI-111 Serial Number 6798

void get_environ_data() {
  // get enviromental values from sensors

  tempvalue = NAN; // FROM 107 temperature probe
  parvalue = NAN; // FROM LI-190R
  //tptempvalue = NAN; // FROM SI-111 (thermopile)
  tstempvalue = NAN; // FROM SI-111 (thermistor)
  canopyvalue = NAN; // FROM SI-111 and computed

    if (i2cReady(ADS_1) && i2cReady(ADS_2)) {
      // read data from ADC
      int16_t conv0;
      uint16_t conv1, conv2, conv3;
      double Rs;
      double cA1 = 8.27111E-4;
      double cB1 = 2.088020E-4;
      double cC1 = 8.0592E-8;
  
      conv0 = ads1.readADC_Differential_2_3(); // termopila
      conv1 = ads1.readADC_SingleEnded(0); // LI-190R
      conv2 = ads1.readADC_SingleEnded(1); // T-107
      conv3 = ads2.readADC_SingleEnded(0); // termistor
  
      // convert numbers to physical units
      voltsi111a = conv0 * scaleads1;
      voltli190r = conv1 * scaleads1;
      volt107    = conv2 * scaleads1;
      voltsi111b = conv3 * scaleads2;
  
      // 107 temperature probe conversion
      Rs = 1000 * (refvolt/volt107) - 250000;
      tempvalue = (1/(cA1 + cB1*log(Rs) + cC1*pow(log(Rs),3))) - 273.15;
  
      // LI-190R PAR conversion
      parvalue = voltli190r * licor_calib;
  
      // SI-111 Radiometer conversion
      double sM, sB, Rt, argum;
      double dA1 = 1.129241-3;
      double dB1 = 2.341077E-4;
      double dC1 = 8.775468E-8;
  
      Rt = 24900*((refvolt/voltsi111b)-1);
      tstempvalue = (1/(dA1 + dB1*log(Rt) + dC1*pow(log(Rt),3))) - 273.15;
      sM = m0 +  (m1 * tstempvalue) + (m2*pow(tstempvalue,2));
      sB = b0 +  (b1 * tstempvalue) + (b2*pow(tstempvalue,2));
      argum = pow(tstempvalue+273.15,4)+(sM * voltsi111a)+ sB;
      canopyvalue = sqrt(sqrt(argum)) - 273.15;
  
    }
  return;
}

void getExtraStr() {
  char tbuffer[5];
  char parbuffer[5];
  char ctbuffer[5];
  //SerialBT.println("GES");
  double nodata = -999.0;
  // FORMAT STRINGS
  ExtraBuffer[0] = '\0';
  if (isnan(tempvalue)) {
    dtostrf(nodata, 3, 0, tbuffer);
  } else  {
    dtostrf(tempvalue, 3, 1, tbuffer);
  }
  if (isnan(parvalue)) {
    dtostrf(nodata, 3, 0, parbuffer);
  }  else {
    dtostrf(parvalue, 3, 1, parbuffer);
  }
  if (isnan(canopyvalue)) {
    dtostrf(nodata, 3, 0, ctbuffer);
  }  else {
    dtostrf(canopyvalue, 3, 1, ctbuffer);
  }
  sprintf(ExtraBuffer, "<tair>%s</tair><par>%s</par><tveg>%s</tveg>", tbuffer, parbuffer, ctbuffer);
  //sprintf(ExtraBuffer, "hola");
}


//#################### PARSE LICOR DATA

double sctof (char * value) { //scientific notation to double e.g 123.4e2
  char num[10];
  char ep[3];
  int i = 0, j = 0;
  while (value[i] != 'e') {
    num[i] = value[i];
    i++;
  }
  num[i] = '\0';
  i++;
  while (value[i]) {
    ep[j] = value[i];
    i++;
    j++;
  }
  ep[j] = '\0';
  return (atof(num) * pow(10.0, atof(ep)));
}

void getstring(char * data, int start , int len, char * out) {
  for (int i = 0; i < len; i++) out[i] = data[i + start];
  out[len] = '\0';
}

static void insert(const char *to_insert, char *string, int offset)
{
  int slen = strlen(string);
  int ilen = strlen(to_insert);
 
  assert(ilen + offset <= slen);
 
  memmove(&string[offset + ilen], &string[offset], slen + ilen -1);
  memcpy(&string[offset], to_insert, ilen);
}

void piggyback_licor_string() {
  getExtraStr();
  //int sbl; //largo de inbuffer_licor
  int espacio = strlen(ExtraBuffer); //espacio
  int caractercorte = 14;
  strcpy(FPbuffer,inbuffer_licor);
  //SerialBT.println(FPbuffer);
  insert(ExtraBuffer, FPbuffer, caractercorte-1);
  strcat(FPbuffer, "\n");
}

//#################### SETUP

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  SerialBT.begin(9600);
  SerialBT.println("FluxPuppy Mobile System v1.1.0");
  SerialBT.println("Starting...");
  delay(5000);
  //Serial.print("<LI840><CFG><OUTRATE>0.5</OUTRATE></CFG><RS232><CO2ABS>FALSE</CO2ABS><H2OABS>FALSE</H2OABS><RAW>FALSE</RAW></RS232></LI840>\n");
  // ADS1115
  SerialBT.println("Setting analog interface network...");
  Wire.begin(portSD0,portCLK); // start ADS1115 network
  SerialBT.println("Communicating with analog-digital converter...");
  ads1.begin();
  ads2.begin();
  ads1.setGain(GAIN_SIXTEEN); // any other measurement
  ads2.setGain(GAIN_ONE);  // this is for thermistor
  //DEBUG_MSG("WAITINGLICOR\n");
  delay(5000); // a moment to wait LICOR analyzer
  //Timer
  HzInterrupt = false;
  user_init();
  SerialBT.println("Ready");
}

//#################### MAIN LOOP
int data_ready = 0;
String datafeed;
void loop() {
  //DEBUG_MSG("MAINLOOP\n");
  if (HzInterrupt) {
    get_environ_data();
    HzInterrupt = false;
  }
  // CHECK FOR INCOMING SERIAL DATA FROM LICOR
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      inbuffer_licor[sbl] = '\0';
      piggyback_licor_string(); // puts data in FPbuffer
      SerialBT.print(FPbuffer);
      sbl = 0;
      //inbuffer_licor[sbl] = '\0';
    } else {
      inbuffer_licor[sbl] = c;
      sbl++;
    }

  }
  delay(0);
}
