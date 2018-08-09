/*
 * LiCor GasAnalyzer Serial to Bluetooth/USB converter with addtitinal sensor input. 
 * To be used with FluxPuppy App.
 *
 * This program is written to be used with an ESP32 OLED development board.
 * David Basler, 2018
 */

/*
 * ESP32 OLED
 * 
 * PINOUT:
 * 5 SDA   ----   SDA BME280
 * 4 SCL   ----   SCL BME280
 * 
 * 14 RX   ----   TX S8
 * 15 TX   ----   RX S8
 * 

 */

#define DEVICENAME "--- DaBa FLUX ---"
#define DISPLAY  // comment out if no display is connected

//#################### DISPLAY
#ifdef DISPLAY
  #include "SSD1306Wire.h" // inclues a call to Wire.h
  SSD1306Wire  display(0x3c, 5, 4);
#else
  #include "Wire.h"
#endif
//#################### BLUETOOTH 
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

//####################  LICOR Serial connection

HardwareSerial Serial1(1); // GPIO 14/15 

//#################### BME280 ENVIRON SENSOR
#include "cactus_io_BME280_I2C.h"
#define BME_ADDRESS 0x76
BME280_I2C bme(BME_ADDRESS);  // I2C using address 0x76
bool bmeAvail=false;
//#################### GLOBAL CONTROL VRIABLES

// Loop interrupts
bool HzInterrupt=false;
bool new_data_ready = false;

// Progressbars
bool warmup=false;
int progress=0;
int progressmax;

// Serial buffers
char inbuffer_licor[500];      // For Licor ON Serial1
int sbl=0;
char FPbuffer [600]; // Final Sting to be submittet to Flux Puppy
char ExtraBuffer[255];
int len;

*/
// String parser
int data[25];
int datalen[25];
int tags [25];
int taglen[25];

#ifdef DISPLAY
// Current Values for display
float co2value=NAN;   // FROM LiCor
float h2ovalue=NAN;   // FROM LiCor

float rhvalue=NAN;    // FROM BME280
float tempvalue=NAN;  // FROM BME280
float pressvalue=NAN; // FROM BME280
#endif

//#################### 1HZ INTERRUPT

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  HzInterrupt=true;
  portEXIT_CRITICAL_ISR(&timerMux);
}


//#################### EXTRA SENSOR DATA , integration in data string for FLUX PUPPY

// ################# BME280

bool i2cReady(uint8_t adr){
uint32_t timeout=millis();
bool ready=false;
while((millis()-timeout<100)&&(!ready)){
  Wire.beginTransmission(adr);
  ready=(Wire.endTransmission()==0);
  }
return ready;
}

void get_environ_data() {
  rhvalue=NAN;    
  tempvalue=NAN;
  pressvalue=NAN; 
  if (i2cReady(BME_ADDRESS)){
        bme.readSensor(); 
        tempvalue =  bme.getTemperature_C();// °C
        pressvalue = bme.getPressure_MB();  // mbar"
        rhvalue = bme.getHumidity();       // %
  }
}

void getExtraStr(){  
  char hbuffer[7];
  char tbuffer[7];
  char pbuffer[7];
  double nodata=-999.0;
  // FORMAT STRINGS
  ExtraBuffer[0]='\0';
  if (isnan(rhvalue))  {dtostrf(nodata, 3, 0, hbuffer);}  else { dtostrf(rhvalue, 3, 1, hbuffer);}  
  if (isnan(tempvalue)) {dtostrf(nodata, 3, 0, tbuffer);} else  { dtostrf(tempvalue, 3, 1, tbuffer);}
  if (isnan(pressvalue)){dtostrf(nodata, 3, 0, pbuffer);}  else { dtostrf(pressvalue, 3, 1, pbuffer);}
  sprintf(ExtraBuffer,"<tair>%se0</tair><pair>%se0</pair><hair>%se0</hair>",tbuffer,pbuffer,hbuffer);
  }


//#################### PARSE LICOR DATA

double sctof (char * value){ //scientific notation to double e.g 123.4e2
    char num[10];
    char ep[3];
    int i=0,j=0;
    while (value[i]!='e') {num[i]=value[i]; i++;}
    num[i]='\0';
    i++;
    while (value[i]) {ep[j]=value[i]; i++; j++;}
    ep[j]='\0';
    return (atof(num)*pow(10.0,atof(ep)));
}

void getstring(char * data, int start ,int len, char * out){
  for (int i=0;i<len;i++) out[i]= data[i+start];
  out[len]='\0';
  }

void piggyback_licor_string() {
  // Splits xml structures in tags and elements
  char tagstring [30];
  char datastring [30];
  int bl = strlen(inbuffer_licor);
  int tp = 0;
  int x = -1;
  int i,j,dp;
  // split xml String into tags and data (position and length)
  for (i = 0; i < bl; i++) {         // find
    if (inbuffer_licor[i] == '<') {
      if (x >= 0) datalen[x]= tp;    // finalize data
      x++;
      tp = 0;
      tags[x]= i;                    // Start new tag
      tp++;
    } else if (inbuffer_licor[i] == '>') {
      taglen[x]=tp+1;                // finalize tag
      tp = 0;
      if (i < (bl-1)) data[x]= i+1;  // Start new data
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
  getstring(inbuffer_licor,tags[1],taglen[1],tagstring);
  if (strcmp(tagstring,"<data>")) {FPbuffer[0]='\0';return;}
  
  getExtraStr();  


  /// Now set the values for display
  for (i=2;i<x;i++){
    getstring(inbuffer_licor,tags[i],taglen[i],tagstring);
    getstring(inbuffer_licor,data[i],datalen[i],datastring);
    if (!strcmp(tagstring,"<raw>")){ // stop searching when you reach the raw tag
      break;
    }else if (!strcmp(tagstring,"<co2>")){
      co2value=sctof(datastring);
    }else if (strcmp(tagstring,"<h2o>")==0){
      h2ovalue=sctof(datastring);
    }
  }
  
 // Now put the string back together adding sensor data
  FPbuffer[0]= '\0';
  getstring(inbuffer_licor,tags[0],taglen[0],tagstring);
  strcpy(FPbuffer,tagstring);        // <device> tag
  getstring(inbuffer_licor,tags[1],taglen[1],tagstring);
  strcat(FPbuffer,tagstring);        // <data> tag
  strcat(FPbuffer,ExtraBuffer); 
  strcat(FPbuffer,&inbuffer_licor[tags[2]]);
  strcat(FPbuffer,"\n");
}

#ifdef DISPLAY
//#################### DISPLAY DATA

void show_all(){
 // clear the display
  char sbuffer[5];
  char cbuffer[7];
  char hbuffer[10];
  char tbuffer[10];
  char pbuffer[10];
  char timestr[20];
  display.clear();
  
  if (warmup){
    drawProgressBar("WELCOME");
    display.display();  
    return;
    }
    
  if (isnan(co2value))  { strcpy(cbuffer,"----.-");}  else { dtostrf(co2value,   3, 1, cbuffer);}  //value, mininum width, precision, buffer
  if (isnan(rhvalue))   { strcpy(hbuffer,  "--.-");}  else { dtostrf(rhvalue,    3, 1, hbuffer);strcat (hbuffer," %");}
  if (isnan(tempvalue)) { strcpy(tbuffer,  "--.-");}  else { dtostrf(tempvalue,  3, 1, tbuffer);strcat (tbuffer," °C");}
  if (isnan(pressvalue)){ strcpy(pbuffer,"----.-");}  else { dtostrf(pressvalue, 3, 1, pbuffer);strcat (pbuffer," mb");}
  int shifty=0;
 
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 3,"FLUX PUPPY" );

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(72, 20+shifty,cbuffer);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(74, 26+shifty, "ppm");

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 52,tbuffer);

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(58, 52,hbuffer);

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 52, pbuffer);
  display.display();  
  }

void drawProgressBar(char * title) { 
  double slope = 100.0 / progressmax;
  int cv =  slope * progress;
  display.setTextAlignment(TEXT_ALIGN_CENTER); 
  display.drawString(64, 0,DEVICENAME);
  //drawProgressBar(x, y, width, height, value);
  display.drawProgressBar(10, 32, 100, 10, cv);
  display.drawString(64, 15, String(cv) + "%");
  display.drawString(64, 50, title);  
}

#endif
//#################### SETUP

void setup() {
  
    //Serial port USB
    Serial.begin(9600);
    //Bluetooth  
    SerialBT.begin("DaBaFlux_01"); //Bluetooth device name
    //LICOR
    Serial1.begin(9600,SERIAL_8N1,15, 14);

#ifdef Display
    //Display
    display.init();
    display.setContrast(255);
    display.flipScreenVertically();
#endif
    
    //Timer
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000, true);
    timerAlarmEnable(timer);

    // BME280
    if (bme.begin()) {
      bmeAvail=true;
      bme.setTempCal(-1);
    } //  else {Serial.println("Could not find a valid BME280 sensor, check wiring!");}

# ifdef
    //WARMUP
    warmup=true;
    progressmax=50;
    progress=0;
    for (int i=0;i<50;i++){
      progress++;
      show_all();
      delay(20);
      }
    warmup=false;
}

//#################### MAIN LOOP
int data_ready=0;

void loop() {

  if (HzInterrupt) {
      if (bmeAvail) get_environ_data();
      show_all();
      HzInterrupt=false;
    }
 
  // CHECK FOR INCOMING SERIAL DATA FROM LICOR
   while (Serial1.available()){
      char c = Serial1.read();
      if (c =='\n'){
       inbuffer_licor[sbl]='\0';
       piggyback_licor_string(); // puts data in FPbuffer
       SerialBT.print(FPbuffer);
       Serial.print(FPbuffer);
       //warmup=false;
       show_all();
       sbl=0;
       inbuffer_licor[sbl]='\0';        
      }else {
       inbuffer_licor[sbl]=c;
       sbl++;
      }
    }
    
  delay(20);
}

