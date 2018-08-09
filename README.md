# fluxdatalink

A versatile low-cost serial-to-Bluetooth/USB adapter to link the LI-COR Li820/840A Gas Analyzers to the [FluxPuppy data logging app](https://github.com/bnasr/FluxPuppy) for Android.

The FluxPuppy data-logging app is Android interface to the LICOR LI-820 and LI-840 gas analyzers using Bluetooth/USB and is part of an integrated system to the to measure carbon fluxes (such as respiration) in an ecosystem.

The adapter presented here is based on an ESP32 development board with an OLED display. It connects the serial output of Li-820/840 Gas Analyzer to the FluxPuppy Android application using Bluetooth or USB, while data from additional sensors may be integrated into the data string sent to Flux Puppy.
If used with an ESP32 OLED Module, selected data is displayed during the transmission. 

![alt text](https://raw.githubusercontent.com/dabasler/fluxdatalink/doc/fluxdatalink.png)

## Hardware

The prototype is based on the following hardware:

* Wemos ESP32 OLED Module
* SparkFun Electronics PRT-00449 RS232 SHIFTER
* BME280 Breakout board. (T/RH/P sensor)

If the display is not neeeded, a Wemos ESP32 without OLED will work fine (an option to disable display is provided in code)

```
ESP32 OLED PINOUT:
5 SDA   ----   SDA BME280
4 SCL   ----   SCL BME280
14 RX   ----   TX RS232 SHIFTER from GasAnalyzer 
15 TX   ----   RX RS232 SHIFTER from GasAnalyzer
```

## Software

The code is currently based on the Arduino platform and requires the Arduino environment to be set up to work with ESP32 (see [https://github.com/espressif/arduino-esp32](https://github.com/espressif/arduino-esp32))
Refer to the code for libraries and other dependecies.

