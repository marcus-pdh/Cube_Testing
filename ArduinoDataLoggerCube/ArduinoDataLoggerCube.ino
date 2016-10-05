/*************************************************** 
  This is an example for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <SPI.h>
#include "Adafruit_MAX31855.h"

// Default connection is using software SPI, but comment and uncomment one of
// the two examples below to switch between software SPI and hardware SPI:

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
//#define MAXDO   3
//#define MAXCS   4
// MAXCLK  5

#define CURRENT_SENSOR_INPUT A0
#define CURRENT_SENSOR_IZERO_REF_INPUT A1
#define VOLTAGE_SUPPLY_INPUT A2


//Pin 1 - Brown Current Sens V
//Pin 2 - Orange Current V ref zero offset
//Pin 3 - Bronw Potential Devider

//Pin 13 - Orange CLK
//Pin 12 - Blue MISO
//Pin 10 - White CS

#define ADJ  12

// initialize the Thermocouple
//Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Example creating a thermocouple instance with hardware SPI (Uno/Mega only)
// on a given CS pin.
#define MAXCS1   10
#define MAXCS2   9
Adafruit_MAX31855 thermocoupleHot(MAXCS1);
Adafruit_MAX31855 thermocoupleCold(MAXCS2);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

unsigned long time_ms;


int led = 3;
int brightness = 20;    // how bright the LED is
int fadeAmount = 1;    // how many points to fade the LED by

int sensValue = 0;        // value read from the pot
int refValue = 0;        // value read from the pot
int VoltageValue = 0;        // value read from the pot

void setup() {
  #ifndef ESP8266
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif

 
  Serial.begin(9600);
  // Print Line for CSV Column Headings - this is optional
  //Serial.println("Time, IC Temp, ThermocoupleCold, ThermocoupleHot, Current[A], Voltage[V], Power [W]");


  // wait for MAX chip to stabilize
  delay(500);
}


void loop() {
  // basic readout test, just print the current temp
   //Serial.print("Internal Temp = ");
   time_ms=millis();
   Serial.print((float)time_ms/1000);
   Serial.print(", ");
   // Read MAXIM IC Internal Temp for reference optional
   //Serial.print(thermocoupleHot.readInternal());
   //Serial.print(", ");

    double c = thermocoupleCold.readCelsius();
    double h = thermocoupleHot.readCelsius();
    sensValue = analogRead(CURRENT_SENSOR_INPUT);
    refValue = analogRead(CURRENT_SENSOR_IZERO_REF_INPUT);
    VoltageValue = analogRead(VOLTAGE_SUPPLY_INPUT);

    // ADC TO AMPS
    float amps = 0.08935*sensValue - 45.777;
    float Volt = 0.00491 * VoltageValue; //(0.00491 = 5V/1024res with a bit of correction)

    //Serial.print("Cold ");

      Serial.print(c-ADJ);
      Serial.print(", ");
    //Serial.print("Hot ");
      Serial.print(h-ADJ);
    //Serial.print(", ");
    //Serial.print(amps);
    //Serial.print(", R");
    //Serial.print(refValue);

    //Serial.print(", ");
    //Serial.print(Volt);
    //Serial.print(", ");
    //Serial.print((Volt*amps));
      Serial.println();

 
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFarenheit());
 
   delay(500);
}
