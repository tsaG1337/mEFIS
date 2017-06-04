/*
  mEICAS - my Engine Indicating Crew Alerting System.
  Copyright (C) 2017 Patrick Bihn <pbihn at uni minus bremen dot de>

  You can find the full repo here:
  https://github.com/tsaG1337/mEFIS
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Wire.h> //Include I2C Library
#include <SPI.h>

#define SIZE_OF_AVG 24
#include "TinyGPSpp.h"
#include "mcp3208.h"
#include "HSS.h"
#include "movingAverage.h"
#include <SoftwareSerial.h>

//********** Setup **********//
#define DEBUG                     //uncomment to remove Debug messages on the USBSerial @115200 baud

#define WHO_AM_I 0xB              // Device check ID
double ID = 100000001;            //Serial Number
#define HARDVERSION 122           //Hardware Version
#define SOFTVERSION 1             //Software Version

#define GPSBaudrate 9600          //Baudrate for onboard GPS connection
#define USBBaudrate 115200        //Baudrate for ESP8266 connection

#define AnalogReadSampleRate 50   //Oversampling rate for analog readings
#define AnalogOffset -150            //Analog Offset that has to be calibrated for each chip
#define I2CAdress 0x22            // I2C Adress

#define loop1interval 100         //Loop1 Interval
#define loop2interval 1000         //Loop2 Interval
#define NCPin   A0                //Pin which is not connected

//********** DEBUG **********//

#ifdef DEBUG
#define DEBUG_PRINT(x)  SerialUSB.print (x)
#define DEBUG_PRINTLN(x)  SerialUSB.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif
#define DEBUGSerial SerialUSB      //SerialUSB can be used as Debugging Port       

//********** Class objects **********//

HSS diffPress(15, -60, 60); //Initialize Differential Pressure Sensor with +-60mBar
HSS absPress(16, 0, 1600); //Initialize Differential Pressure Sensor with +-60mBar
MCP3208 engineADC = MCP3208(11, 10); //Initialize the ADC for the Analog measurements (NTC and current sensor)
SoftwareSerial GPSSerial(A5, NCPin); // RX, TX. Only RX is connected to the GPS Module
TinyGPSPlus gps;          //onboard GPS



//********** Defining Pins **********//
#define A6 8ul
#define boardTempPin A1           //Pin to measure the Board Temperature
#define ESPReset 26     //Pin to enable the ESP
#define inputVoltagePin A4        //Pin to measure the Input Voltage
#define batteryVoltagePin A2       //Pin to measure the Battery Voltage (ANALOG PIN!?!)
#define batteryStatusPin 11        //Pin to read the Battery charging status
#define tacho1Pin 2               //Tacho1 Circuit connected to this Pin
#define tacho2Pin 5               //Tacho2 Circuit connected to this Pin
#define led1Pin 6                //LED 1 Pin


//********** Defining Status Variables **********//
uint8_t requestedByte = 0;        //I2C request Byte
uint16_t boardTemp = 0;            //Board Temperature
uint8_t powerTemp = 0;            //Temperature at the Buck-Converter
uint8_t fanLevel  = 0;            //level/speed of the cooling Fan
uint32_t loop1PreMill = 0;        //Last time Loop1 was updated
uint32_t loop2PreMill = 0;        //Last time Loop2 was updated
uint8_t batteryChargingStatus = 0;//Battery Charging Status (HIGH = Charging, LOW = Not charging)
uint32_t tach1Start = 0;
uint32_t tach2start = 0;
uint8_t rounds1 = 0;
uint8_t rounds2 = 0;
uint16_t freq1 = 0;
uint16_t freq2 = 0;



void setup() {
#ifdef DEBUG
  delay(3000);
  DEBUGSerial.begin(115200);        //USB Serial Port for Debugging Purpose

#endif
  DEBUG_PRINTLN("Initizializing mEFIS Main Core");
  DEBUG_PRINTLN ("Booting up!");
  DEBUG_PRINT ("Serial: ");           DEBUG_PRINTLN (ID);
  DEBUG_PRINT ("Hardware Version: "); DEBUG_PRINTLN (HARDVERSION);
  DEBUG_PRINT ("Software Version: "); DEBUG_PRINTLN (SOFTVERSION);
  DEBUG_PRINTLN ("");

  DEBUG_PRINTLN ("Initializing Serial Ports:");
  DEBUG_PRINT ("GPS Port with Baudrate:"); DEBUG_PRINTLN (GPSBaudrate);
  GPSSerial.begin(GPSBaudrate);          //Start Serial connection on the GPS Port


  DEBUG_PRINTLN ();
  // pinPeripheral(10, PIO_SERCOM);  //Configuration for Serialport2 (RS232) TX
  // pinPeripheral(13, PIO_SERCOM);  //Configuration for Serialport2 (RS232) RX

  DEBUG_PRINTLN ("Setting ADC resosution to 12 bit.");
  analogReadResolution(12);       // set ADC resolution to 12 bit

  DEBUG_PRINT ("Joining the I2C Bus with Adress: ");
  DEBUG_PRINTLN (I2CAdress);
  Wire.begin(I2CAdress);          //joining the I2C Bus
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  //**** Configurating Pin Modes ****//
  DEBUG_PRINTLN ("Configurating Pin Modes.");
  pinMode(led1Pin, OUTPUT);
  digitalWrite(led1Pin, HIGH);
  pinMode(boardTempPin, INPUT);
  pinMode(inputVoltagePin, INPUT);
  pinMode(batteryVoltagePin, INPUT);
  pinMode(batteryStatusPin, INPUT);
  pinMode(ESPReset, OUTPUT);
  pinMode(tacho1Pin, INPUT);
  pinMode(tacho2Pin, INPUT);


  //**** Configurating Interrups ****//
  DEBUG_PRINTLN ("Configurating Interrupts.");
  attachInterrupt(tacho1Pin, Tacho1ISR, RISING); //Interupt for Tacho1
  attachInterrupt(tacho2Pin, Tacho2ISR, RISING); //Interupt for Tacho2

  DEBUG_PRINTLN ("Set up SPI to 2MHz.");
  SPI.begin();
  SPI.setClockDivider(48);

  DEBUG_PRINTLN ("Resetting ESP.");
  digitalWrite(ESPReset, LOW);
  delay(10);
  digitalWrite(ESPReset, HIGH);
  DEBUG_PRINTLN ("Bootup complete.");
  digitalWrite(led1Pin, LOW);


}

//********** Beginn of Loop **********//
void loop() {
  uint32_t currentMillis = millis();

  //Loop1
  if (currentMillis - loop1PreMill >= loop1interval) {
    loop1PreMill = currentMillis;
 readBoardTemp();

    engineADC.readADC();
  }

  //Loop2
  if (currentMillis - loop2PreMill >= loop2interval) {
    loop2PreMill = currentMillis;
    DEBUG_PRINT ("Board Temp: ");
    DEBUG_PRINTLN (getBoardTemp());
    
  }
}

//********** Reading Tachometer inputs **********//
void Tacho1ISR() {  //Measure time between 10 impulses
  if (tach1Start ==0){
    tach1Start = millis();
    rounds1++;
  } else if (rounds < 10){
    rounds1++;
  }
  
  if (rounds == 10){
    freq1 = (millis() - tach1Start)/10;
    tach1Start = 0;
    rounds = 0;
  }
}

void Tacho2ISR() {
    if (tach2Start ==0){
    tach2Start = millis();
    rounds2++;
  } else if (rounds < 10){
    rounds2++;
  }
  
  if (rounds == 10){
    freq2 = (millis() - tach2Start)/10;
    tach2Start = 0;
    rounds = 0;
  }
}

void readBatteryChargingStatus() {
  batteryChargingStatus = !digitalRead(batteryStatusPin);
}

uint8_t getBatteryChargingStatus() {
  return batteryChargingStatus;
}

void readBoardTemp() {
  /*MCP9701A Transfer Function: VOUT = TC * TA + V0
      TA  = Ambient Temperature
      VOUT= Sensor Output Voltage
      V0  = Sensor Output Voltage at 0°C (400mV)
      TC  = Temperature Coefficient ( 19.5mV per °C)
     Which results in: TA=(VOUT - VO) / TC
  */
  uint16_t ADCVal = analogRead(boardTempPin);
  boardTemp = (((220 * (ADCVal+AnalogOffset)) / 273) - 400) / 19.5;
}

uint8_t getBoardTemp() {
  return boardTemp;
}

//********** Serial Port **********//
void serialEventRun(void) {
  if (Serial1.available()) gps.encode(Serial1.read());
}

//**********   I2C   **********//
void receiveEvent(int howMany) {
  while (Wire.available())
  {
    // read I2C value
    //requestedByte = howMany;
    requestedByte = Wire.read();
    //digitalWrite(led1Pin, LOW);
  }
}
void requestEvent()
{
  switch (requestedByte) {
    case 1: { //Sends the Device Name
        Wire.write(WHO_AM_I);
        break;
      }
    case 2: { //Sends the Device Hardware ID (Serial)
        I2cDoubleWrite(ID);
        break;
      }
    case 3: { //Sends the Device Hardware Version
        Wire.write(HARDVERSION);
        break;
      }
    case 4: { //Sends the Device Software Version
        Wire.write(SOFTVERSION);
        break;
      }
    case 5: { //Send several status values
        bool r0 = gps.location.isValid();
        bool r1 = gps.date.isValid();
        bool r2 = gps.time.isValid();
        bool r3 = getBatteryChargingStatus();
        bool r4 = 0;
        bool r5 = 0;
        bool r6 = 0;
        bool r7 = 0;
        char statusByte = (r0 << 7) | (r1 << 6) | (r2 << 5) | (r3 << 4) | (r4 << 3) | (r5 << 2) | (r6 << 1) | r7;
        Wire.write(statusByte);
        break;
      }
    case 10: { //Sends the Device Software Version
        I2cDoubleWrite(gps.altitude.feet());
        break;
      }
    case 11: { //Sends the GPS latitude (int32_t)
        I2cDoubleWrite(gps.location.lat());
        break;
      }
    case 12: { //Sends the GPS longitude (int32_t)
        I2cDoubleWrite(gps.location.lng());
        break;
      }
    case 13: { //Sends the GPS speed in knots (int32_t)
        I2cDoubleWrite(gps.speed.knots());
        break;
      }
    case 14: { //Sends the GPS course in degree (int32_t)
        I2cDoubleWrite(gps.course.deg());
        break;
      }
    case 15: { //Sends the GPS year (uint16_t)
        I2cInt16Write(gps.date.year());
        break;
      }
    case 16: { //Sends the GPS month (uint8_t)
        Wire.write(gps.date.month());
        break;
      }
    case 17: { //Sends the GPS day (uint8_t)
        Wire.write(gps.date.day());
        break;
      }
    case 18: { //Sends the GPS hour of the day (uint8_t)
        Wire.write(gps.time.hour());
        break;
      }
    case 19: { //Sends the GPS minute of the day (uint8_t)
        Wire.write(gps.time.minute());
        break;
      }
    case 20: { //Sends the GPS second of the day (uint8_t)
        Wire.write(gps.time.second());
        break;
      }
    case 21: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(0));
        break;
      }
    case 22: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(1));
        break;
      }
    case 23: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(2));
        break;
      }
    case 24: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(3));
        break;
      }
    case 25: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(4));
        break;
      }
    case 26: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(5));
        break;
      }
    case 27: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(6));
        break;
      }
    case 28: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(engineADC.getMovingValue(7));
        break;
      }
  }
}

void I2cInt16Write(uint16_t bigVal) {
  Wire.write((byte *)&bigVal, sizeof(uint16_t));
}
void I2cDoubleWrite(double bigVal) {
  Wire.write((byte *)&bigVal, sizeof(double));
}
