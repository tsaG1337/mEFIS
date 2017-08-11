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

#include <Wire.h>       //I2C Library
#include <SPI.h>        //SPI Library
#include <RTCZero.h>    //Sleepmode Library
#define SIZE_OF_AVG 24
#include "TinyGPSpp.h"
#include "mcp3208.h"
#include "HSS.h"
#include "movingAverage.h"
//#include <SoftwareSerial.h>
#include "wiring_private.h" // pinPeripheral() function
#include "ledFlasher.h"

#define DEBUG_OUTPUT
#define DEBUG "MainThread"
#include "debug.h"


//********** Setup **********//

#define WHO_AM_I 0xB              // Device check ID
#define ID 10001            //Serial Number
#define HARDVERSION 122           //Hardware Version
#define SOFTVERSION 1             //Software Version

#define GPSBaudrate 9600          //Baudrate for onboard GPS connection
#define USBBaudrate 115200        //Baudrate for USB
#define RS232Baudrate 115200      //Baudrate for RS232

#define AnalogReadSampleRate 50   //Oversampling rate for analog readings
#define AnalogOffset 30            //Analog Offset that has to be calibrated for each chip
#define pulseSampleqt1  5
#define pulseSampleqt2 5
#define I2CAdress 0x22            // I2C Adress

#define loop1interval 100         //Loop1 Interval
#define loop2interval 1000        //Loop2 Interval
#define lowVoltageThreshold 8000  //Low input Voltage threshold
#define NCPin   A0                //Pin which is not connected

//********** Defining Pins **********//
#define A6 8ul
#define boardTempPin A1           //Pin to measure the Board Temperature
#define ESPReset 26               //Pin to enable the ESP
#define inputVoltagePin A4        //Pin to measure the Input Voltage
#define batteryVoltagePin A2      //Pin to measure the Battery Voltage
#define batteryStatusPin 11       //Pin to read the Battery charging status
#define tacho1Pin 2               //Tacho1 Circuit connected to this Pin
#define tacho2Pin 5               //Tacho2 Circuit connected to this Pin
#define led1Pin 6                 //LED 1 Pin
#define powerPin 12               //Peripheral Power
#define enLinePin  7              //Signal Pin to enable the device

//********** Class objects **********//

RTCZero rtc;
HSS diffPress(A3, -60, 60);           //Initialize Differential Pressure Sensor with +-60mBar
HSS absPress(8, 0, 1600);             //Initialize Differential Pressure Sensor with +-60mBar
MCP3208 engineADC = MCP3208(3, 10);  //Initialize the ADC for the Analog measurements (NTC and current sensor)
//SoftwareSerial GPSSerial(A5, NCPin);// RX, TX. Only RX is connected to the GPS Module
TinyGPSPlus gps;                      //onboard GPS
ledFlasher LED(led1Pin);              //LED blinking pattern initialized
tMovingAvgFilter frequency[2];        //Moving Average Filter for Frequency
/*
   1= Freq. Input 1
   2= Freq. Input 2
*/
tMovingAvgFilter internalADC[3];      //Moving Average Filter for Internal ADC
/*
   1 = Board Temp             (in °Celsius)
   2 = Battery Input Voltage  (MilliVolts)
   3 = Power Input Voltage    (MilliVolts)
*/


//********** Defining Status Variables **********//
uint8_t requestedByte = 0;        //I2C request Byte
uint16_t boardTemp = 0;            //Board Temperature
uint32_t loop1PreMill = 0;        //Last time Loop1 was updated
uint32_t loop2PreMill = 0;        //Last time Loop2 was updated
bool batteryChargingStatus = 0;   //Battery Charging Status (LOW = Charging, HIGH = Not charging)
uint8_t powerLevel = 0;           //Power Level
/*
   0 = low Power mode (Peripherals off)
   1 = normal Power mode (Peripherals on)
*/
uint8_t lowVoltageInput = 0;
uint8_t enLine = 0;
uint32_t freqCount1Start = 0;
uint32_t freqCount2Start = 0;
uint8_t pulses1 = 0;
uint8_t pulses2 = 0;
uint32_t timeduration1 = 0;
uint32_t timeduration2 = 0;

uint16_t freq1 = 0;
uint16_t freq2 = 0;


Uart Serial2 (&sercom1, 13, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
#ifdef DEBUG
  DEBUGSerial.begin(115200);
#endif
delay(2000);
  LOG("Initizializing mEFIS DSP Core");
  LOG("Booting up!");
  LOG ("Serial No.: ",ID);
  LOG ("Hardware Version: ",HARDVERSION);
  LOG ("Software Version: ",SOFTVERSION);

  LOG ("Initializing Serial Ports:");
  //DEBUG_PRINT ("GPS Port with Baudrate: "); DEBUG_PRINTLN (GPSBaudrate);
  //  GPSSerial.begin(GPSBaudrate);          //Start Serial connection on the GPS Port
  LOG ("RS232 with Baudrate: ",RS232Baudrate);
  Serial2.begin(RS232Baudrate);          //Start Serial connection on the GPS Port
  
  pinPeripheral(10, PIO_SERCOM);  //Configuration for Serialport2 (RS232) TX
  pinPeripheral(13, PIO_SERCOM);  //Configuration for Serialport2 (RS232) RX

  LOG ("Setting ADC resolution to 12 bit.");
  analogReadResolution(12);       // set ADC resolution to 12 bit

  LOG ("Joining the I2C Bus with Adress: ",I2CAdress);
  Wire.begin(I2CAdress);          //joining the I2C Bus
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);

  //**** Configurating Pin Modes ****//
  LOG ("Configurating Pin Modes.");
  pinMode(led1Pin,            OUTPUT);
  pinMode(powerPin,           OUTPUT);
  pinMode(ESPReset,           OUTPUT);
  pinMode(boardTempPin,       INPUT);
  pinMode(inputVoltagePin,    INPUT);
  pinMode(batteryVoltagePin,  INPUT);
  pinMode(batteryStatusPin,   INPUT);
  pinMode(tacho1Pin,          INPUT);
  pinMode(tacho2Pin,          INPUT);
  pinMode(enLinePin,          INPUT);
  digitalWrite(led1Pin,       HIGH);

  //**** Configurating Interrups ****//
  LOG ("Configurating Interrupts.");
  attachInterrupt(tacho1Pin, Tacho1ISR, RISING); //Interupt for Tacho1
  attachInterrupt(tacho2Pin, Tacho2ISR, RISING); //Interupt for Tacho2
  //attachInterrupt(enLinePin, wakeUp, LOW);       //Test Interrupt for waking
  //**** Configurating Hardware ****//
  LOG ("Set up SPI to 2MHz.");
  SPI.begin();
  SPI.setClockDivider(48);
  LOG ("External Enable Line: ");
  readEnLine();
  if (getEnLine()) {
    LOG ("Activated!");
    LOG ("Powering up Peripheral devices.");
    peripheralPower(1);
    LOG ("Waiting for the Main Core to boot.");
    delay(500);
    LOG ("Resetting Main Core.");
    digitalWrite(ESPReset, LOW);
    delay(10);
    digitalWrite(ESPReset, HIGH);
  } else {
    LOG ("Disabled!");
    LOG ("Booting in low Power mode");
    peripheralPower(0);
  }
  LOG ("Initializing Sleep Mode");
  rtc.begin();
  LOG ("Bootup complete.");
  digitalWrite(led1Pin, LOW);

}

//********** Beginn of Loop **********//
void loop() {
  uint32_t currentMillis = millis();

  //Loop1 (high Speed loop)
  if (currentMillis - loop1PreMill >= loop1interval) {
    loop1PreMill = currentMillis;
    if (getPowerLevel()) {   //only if Board is fully powered (otherwise sensors are off and there is nothing to read)

      diffPress.readSensor();
      absPress.readSensor();
      
      readBoardTemp();
      
      engineADC.readADC(7);
     
      readBatteryChargingStatus();
      
    }

    readInputVoltage();
    readBatteryVoltage();
    readEnLine();
  }

  //Loop2 (low Speed loop)
  if (currentMillis - loop2PreMill >= loop2interval) {
    loop2PreMill = currentMillis;
    peripheralPower(getEnLine());
    
//    LOG ("Input Voltage: ",(getInputVoltage() / 1000.0));
//    
//    LOG ("Battery Voltage: ", getBatteryVoltage() / 1000.0);
//  
//    LOG ("INPUT 8", engineADC.getMovingValue(0));
//    SerialUSB.println(engineADC.getMovingValue(0));
//    LOG ("INPUT 7", engineADC.getMovingValue(1));
//    LOG ("INPUT 6", engineADC.getMovingValue(2));
//    LOG ("INPUT 5", engineADC.getMovingValue(3));
//    LOG ("INPUT 4", engineADC.getMovingValue(4));
//    LOG ("INPUT 3", engineADC.getMovingValue(5));
//    LOG ("INPUT 2", engineADC.getMovingValue(6));
//    LOG ("INPUT 1", engineADC.getMovingValue(7));
//    SerialUSB.print("Hallo:");
//    SerialUSB.println(engineADC.getMovingValue(7));
//    //
//    LOG ("Board Temperatur: ", getBoardTemp());

   
  }
  LED.update();
  //__WFI();
}

//********** Reading Tachometer inputs **********//
void Tacho1ISR() {  //Measure time between 10 impulses
  if (pulses1 == 0) {
    freqCount1Start = micros();
    pulses1++;
  } else if (pulses1 != 0 && pulses1 < pulseSampleqt1) {
    pulses1++;
  } else if (pulses1 == pulseSampleqt1) {
    uint32_t timeduration1 = micros() - freqCount1Start;
    freq1 = 60000000 / (timeduration1 / pulseSampleqt1);
    AddToMovingAvg(&frequency[1], freq1);
    pulses1 = 0;
  }
}
uint16_t getFreq1() {
  return GetOutputValue(&frequency[1]);
}

void Tacho2ISR() {
  if (pulses2 == 0) {
    freqCount2Start = micros();
    pulses2++;
  } else if (pulses2 != 0 && pulses2 < pulseSampleqt2) {
    pulses2++;
  } else if (pulses2 == pulseSampleqt2) {
    uint32_t timeduration2 = micros() - freqCount2Start;
    freq2 = 60000000 / (timeduration2 / pulseSampleqt2);
    AddToMovingAvg(&frequency[2], freq2);


    pulses2 = 0;
  }
}
uint16_t getFreq2() {
  return GetOutputValue(&frequency[2]);
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
  uint16_t voltageReading = (ADCVal + AnalogOffset) * (3300.0 / 4095);
  //voltageReading = 2/3*(-400 + voltageReading);
  voltageReading = (2 * (voltageReading - 400)) / 39;
  AddToMovingAvg(&internalADC[1], voltageReading);
}

uint8_t getBoardTemp() {
  return GetOutputValue(&internalADC[1]);
}

void readBatteryVoltage() {
  uint16_t ADCVal = analogRead(batteryVoltagePin);
  uint16_t voltageReading = (ADCVal + AnalogOffset) * (3300.0 / 4095);
  voltageReading = (153 * voltageReading) / 55;
  AddToMovingAvg(&internalADC[2], voltageReading);

}
uint16_t getBatteryVoltage() {
  return GetOutputValue(&internalADC[2]);
}

void readInputVoltage() {
  uint16_t ADCVal = analogRead(A4);
  uint16_t voltageReading = (ADCVal + AnalogOffset) * (3300.0 / 4095);
  voltageReading = (101 * voltageReading) / 10;
  AddToMovingAvg(&internalADC[3], voltageReading);

  if (GetOutputValue(&internalADC[3]) <= lowVoltageThreshold) { // if Input Voltage is below Treshold Voltage, set the lowVoltage Flag.
    lowVoltageInput = 1;
  } else {
    lowVoltageInput = 0;
  }
}
uint16_t getInputVoltage() {
  return GetOutputValue(&internalADC[3]);
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
    requestedByte = Wire.read();
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
    case 29: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(getFreq1());
        break;
      }
    case 30: { //Gets the moving average value of the called ADC channel (MCP3208)
        I2cDoubleWrite(getFreq2());
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

//********* Power Management ****************//
void peripheralPower(bool power) {
  LED.pattern(power); //set the LED mode to the corresponding pattern
  if (powerLevel == power) { //check if current power level equeals the level that should been switched into.

    // do nothing

  } else {
    powerLevel = power;
    if (powerLevel) {
      digitalWrite(powerPin, power);
      delay(500);                   //waiting for everything to power up.
      pinMode(A3, OUTPUT);          // mapping Diff. Pressure SPI Slave Select Pin as Output
      pinMode(8, OUTPUT);           // mapping absolute Pressure SPI Slave Select Pin as Output
      digitalWrite(A3, HIGH);       // disabling Diff. Pressure SPI-Slave
      digitalWrite(8, HIGH);        // disabling absolute Pressure SPI-Slave
    } else if (!powerLevel) { //Power off everything
      //Power off Sensors
      pinMode(A3, INPUT);           // mapping Diff. Pressure SPI Slave Select Pin as Input
      pinMode(8, INPUT);            // mapping absolute Pressure SPI Slave Select Pin as Input
      InitMovingAvg(&internalADC[1], 0); //set Board Temperature to 0
      InitMovingAvg(&frequency[2], 0); //set Frequency1 to 0
      InitMovingAvg(&frequency[2], 0); //set Frequency2 to 0
      batteryChargingStatus = 0;       //set Battery charging Status to 0 (otherwise it would be floating and recognized as charging)
      digitalWrite(powerPin, power);
      
      //rtc.standbyMode();

    }
  }

}

uint8_t getPowerLevel() {
  return powerLevel;
}

void readBatteryState() {
  batteryChargingStatus = digitalRead(batteryStatusPin);
  if (batteryChargingStatus) {
    LED.pattern(2);
  } else {
    LED.pattern(1);
  }
}

void readEnLine() {
  enLine = digitalRead(enLinePin);
}

uint8_t getEnLine() {
  return enLine;
}

void wakeUp() {
  peripheralPower(1);
}
