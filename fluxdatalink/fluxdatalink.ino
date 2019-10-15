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

   PINOUT:

   14 RX   ----   TX S8
   15 TX   ----   RX S8


*/

#include "Wire.h"
#include "HardwareSerial.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <Adafruit_ADS1015.h>

//#################### Serial connection

//HardwareSerial Serial1(1); // for LI-840A
HardwareSerial SerialBT(2); // for HC06

//#################### GLOBAL CONTROL VRIABLES

// Loop interrupts
bool HzInterrupt = false;
bool new_data_ready = false;

// Serial buffers
char inbuffer_licor[500];      // For Licor ON Serial1
int sbl = 0;
char FPbuffer [620]; // Final Sting to be submitted to Flux Puppy
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
Ticker timer1;
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
float m0 = 1313790000 ; float m1 = 7203270; float m2 = 56191.3;    // calibration polynomial coefficients for SI-111 Serial Number 6798
float b0 = -8736140 ; float b1 = 146949; float b2 = 3329.97;    // calibration polynomial coefficients for SI-111 Serial Number 6798


////SI111
//  m = 1313790000 +( 7203270*Temperatura_Si111) + (56191.3*Temperatura_Si111^2)  ' Pendiente de calculo de temperatura de objeto
//    b = -8736140 + (146949*Temperatura_Si111) + (3329.97*Temperatura_Si111^2)     ' Interseccion con el eje y para calcular Tbody
//    ' Calculo de temperatura de objeto usando m y b
//    SBT_K = Temperatura_Si111+273.15
//    TT_K = SBT_K^4 + TTmV*m +b
//    TT_K = SQR(SQR(TT_K))
//    Temperatura_Canopia = TT_K-273.15                   ' Temperatura de Canopia en grados Celsius

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
}

void getExtraStr() {
  char tbuffer[7];
  char pbuffer[7];
  char parbuffer[7];
  char ctbuffer[7];

  double nodata = -999.0;
  // FORMAT STRINGS
  ExtraBuffer[0] = '\0';
  if (isnan(tempvalue)) {
    dtostrf(nodata, 3, 0, tbuffer);
  } else  {
    dtostrf(tempvalue, 3, 1, tbuffer);
  }
  if (isnan(pressvalue)) {
    dtostrf(nodata, 3, 0, pbuffer);
  }  else {
    dtostrf(pressvalue, 3, 1, pbuffer);
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
  sprintf(ExtraBuffer, "<tair>%se0</tair><pair>%se0</pair><par>%se0</par><tveg>%se0</tveg>", tbuffer, pbuffer, parbuffer, ctbuffer);
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

void piggyback_licor_string() {
  // Splits xml structures in tags and elements
  char tagstring [30];
  char datastring [30];
  int bl = strlen(inbuffer_licor);
  int tp = 0;
  int x = -1;
  int i, j, dp;
  // split xml String into tags and data (position and length)
  for (i = 0; i < bl; i++) {         // find
    if (inbuffer_licor[i] == '<') {
      if (x >= 0) datalen[x] = tp;   // finalize data
      x++;
      tp = 0;
      tags[x] = i;                   // Start new tag
      tp++;
    } else if (inbuffer_licor[i] == '>') {
      taglen[x] = tp + 1;            // finalize tag
      tp = 0;
      if (i < (bl - 1)) data[x] = i + 1; // Start new data
    } else {
      tp++;
    }
  }
  x++;

  /* //DEBUG
    Serial.println(inbuffer_licor);
    for (i=0;i<x;i++){Serial.print(tags[i]);Serial.print(":");Serial.println(taglen[i]);getstring(inbuffer_licor,tags[i],taglen[i],tagstring);Serial.print(tagstring);Serial.print(" -----> ");Serial.print(data[i]);Serial.print(":");Serial.println(datalen[i]);getstring(inbuffer_licor,data[i],datalen[i],datastring);Serial.print(datastring);}
  */

  // Test input string is valid (cointains <data> tag in as second element)
  getstring(inbuffer_licor, tags[1], taglen[1], tagstring);
  if (strcmp(tagstring, "<data>")) {
    FPbuffer[0] = '\0';
    return;
  }

  getExtraStr();


  /// Now set the values for display
  for (i = 2; i < x; i++) {
    getstring(inbuffer_licor, tags[i], taglen[i], tagstring);
    getstring(inbuffer_licor, data[i], datalen[i], datastring);
    if (!strcmp(tagstring, "<raw>")) { // stop searching when you reach the raw tag
      break;
    } else if (!strcmp(tagstring, "<co2>")) {
      co2value = sctof(datastring);
    } else if (strcmp(tagstring, "<h2o>") == 0) {
      h2ovalue = sctof(datastring);
    } else if (strcmp(tagstring, "<cellpres>") == 0) {
      pressvalue = sctof(datastring);
    }
  }

  // Now put the string back together adding sensor data
  FPbuffer[0] = '\0';
  getstring(inbuffer_licor, tags[0], taglen[0], tagstring);
  strcpy(FPbuffer, tagstring);       // <device> tag
  getstring(inbuffer_licor, tags[1], taglen[1], tagstring);
  strcat(FPbuffer, tagstring);       // <data> tag
  strcat(FPbuffer, ExtraBuffer);
  strcat(FPbuffer, &inbuffer_licor[tags[2]]);
  strcat(FPbuffer, "\n");
}

//#################### SETUP

void setup() {

  //Serial port USB
  Serial.begin(9600);
  //Bluetooth HC06
  SerialBT.begin(9600);
  //LICOR
  Serial1.begin(9600);
  // ADS1115
  ads1.setGain(GAIN_SIXTEEN); // any other measurement
  ads2.setGain(GAIN_ONE);  // this is for thermistor
  ads1.begin();
  ads2.begin();
  //Timer
  timer1.attach(1, itsasecond);

}

//#################### MAIN LOOP
int data_ready = 0;

void loop() {
  if (HzInterrupt) {
    get_environ_data();
    HzInterrupt = false;
  }
  // CHECK FOR INCOMING SERIAL DATA FROM LICOR
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      inbuffer_licor[sbl] = '\0';
      piggyback_licor_string(); // puts data in FPbuffer
      SerialBT.print(FPbuffer);
      Serial.print(FPbuffer);
      //warmup=false;
      sbl = 0;
      inbuffer_licor[sbl] = '\0';
    } else {
      inbuffer_licor[sbl] = c;
      sbl++;
    }
  }
  delay(20);
}
