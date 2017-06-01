//The HSS library by Patrick Bihn is licensed under CC BY 3.0
#include "HSS.h"
#include "Arduino.h"
#include "SPI.h"
//Define Communication Settings
#define baudrate 500000
#define BitFirst MSBFIRST
#define SPIMode SPI_MODE3
#define DEBUG 0



HSS::HSS(uint8_t ChipSelect, int16_t low, int16_t high) {
  lowPressure = low;
  highPressure = high;
  ChipselectPin = ChipSelect;
  pinMode(ChipselectPin, OUTPUT);
  digitalWrite(ChipselectPin, HIGH);

}


int HSS::getTemperature() {
  return temperature;
}


int16_t HSS::getPressure() {
  return pressure;
}


void HSS::readSensor() {
  SPI.beginTransaction(SPISettings(baudrate, BitFirst, SPIMode)); // Set to 800kHz, MSB and MODE1
  SPI.begin();
  digitalWrite(ChipselectPin, LOW);       //pull Chipselect Pin to Low


  inByte_1 = SPI.transfer(0x00);  // Read first Byte of Pressure
  //delayMicroseconds(10);
  inByte_2 = SPI.transfer(0x00);  // Read second Byte of Pressure
  //delayMicroseconds(10);
  inByte_3 = SPI.transfer(0x00);  // Read first Byte of Temperature
  //delayMicroseconds(10);
  inByte_4 = SPI.transfer(0x00);  // Read second Byte of Temperature

  SPI.endTransaction();               //end SPI Transaction
  SPI.end();

  if (DEBUG) {
    Serial.print("Chipselect = "); Serial.print(ChipselectPin, DEC); Serial.print(" ");
    Serial.print("Byte_1 = "); Serial.print(inByte_1, DEC); Serial.print(" ");
    Serial.print("Byte_2 = "); Serial.print(inByte_2, DEC); Serial.print(" ");
    Serial.print("Byte_3 = "); Serial.print(inByte_3, DEC); Serial.print(" ");
    Serial.print("Byte_4 = "); Serial.print(inByte_4, DEC); Serial.print(" ");
  }
  digitalWrite(ChipselectPin, HIGH);      //pull Chipselect Pin to High
  pressureDigital = inByte_1 << 8 | inByte_2;

  //pressure = pressureDigital;
  pressure = (((float)(pressureDigital - 1638) * (highPressure - lowPressure)) / 13104) + lowPressure; //transform digital value into real pressure
  inByte_3 = inByte_3 << 3 | inByte_4; //Shift first Temperature byte 3 left
  temperature = ((float)inByte_3 * 200 / 2047) - 50; //Convert Digital value to °C
  if (DEBUG) {
    Serial.print("Temp[C]= "); Serial.print(temperature); Serial.print(" ");
  }
}


void HSS::oversampleReadSensor() {
  float tempBuffer = 0;
  int32_t pressBuffer = 0;
  for (int i = 0; i < sampleRate; i++) {
    readSensor();
    tempBuffer += temperature;
    pressBuffer += pressure;
  }
  temperature = (tempBuffer / sampleRate);
  pressure = (pressBuffer / sampleRate);
}



uint8_t HSS::getSampleRate() {
  return sampleRate;
}

void HSS::setSampleRate(uint8_t newSampleRate) {
  sampleRate = newSampleRate;
}


