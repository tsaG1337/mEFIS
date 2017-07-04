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
#include "HSS.h"
#include "Arduino.h"
#include "SPI.h"
#include "debugMessenger.h"
#define DEBUG false
//Define Communication Settings
#define baudrate 500000
#define BitFirst MSBFIRST
#define SPIMode SPI_MODE3



HSS::HSS(uint8_t ChipSelect, int16_t low, int16_t high) {
  lowPressure = low;
  highPressure = high;
  ChipselectPin = ChipSelect;


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
    DEBUG_PRINT("Chipselect = "); DEBUG_PRINTLN(ChipselectPin);
    DEBUG_PRINT("Byte_1 = "); DEBUG_PRINTLN(inByte_1);
    DEBUG_PRINT("Byte_2 = "); DEBUG_PRINTLN(inByte_2);
    DEBUG_PRINT("Byte_3 = "); DEBUG_PRINTLN(inByte_3);
    DEBUG_PRINT("Byte_4 = "); DEBUG_PRINTLN(inByte_4);
  }
  digitalWrite(ChipselectPin, HIGH);      //pull Chipselect Pin to High
  pressureDigital = inByte_1 << 8 | inByte_2;

  //pressure = pressureDigital;
  pressure = (((float)(pressureDigital - 1638) * (highPressure - lowPressure)) / 13104) + lowPressure; //transform digital value into real pressure
  inByte_3 = inByte_3 << 3 | inByte_4; //Shift first Temperature byte 3 left
  temperature = ((float)inByte_3 * 200 / 2047) - 50; //Convert Digital value to °C
  if (DEBUG) {
    Serial.print("Temp[C]= "); Serial.print(temperature); Serial.println(" ");
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


