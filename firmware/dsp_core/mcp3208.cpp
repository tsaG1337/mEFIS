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

#include "mcp3208.h"
#include "movingAverage.h"
//#define DEBUG "MCP3208"
#include "debug.h"

//Define Communication Settings
#define baudrate 500000
#define BitFirst MSBFIRST
#define SPIMode SPI_MODE3

uint16_t value[8] = {0,0,0,0,0,0,0,0}; 
MCP3208::MCP3208(char Pin, char oversamplingRate) {
  _csPin = Pin;
  pinMode(_csPin, OUTPUT);    //configure CS Pin as Output
  digitalWrite(_csPin, HIGH); //set CS Pin high
  _oversamplingRate = oversamplingRate;
  for (int i = 0; i < 8; i++) {
    InitMovingAvg(&filteredChannel[i], 0); //init Filter
  }
}

void MCP3208::readADC(char channel)
{
  uint32_t oversampleRead = 0;
  uint16_t intResult = 0;
  // enable ADC SPI
  LOG("Begin Transaction");
  SPI.beginTransaction(SPISettings(baudrate, BitFirst, SPIMode)); // Set to 800kHz, MSB and MODE1
  LOG("SPI Begin");
  SPI.begin();

    //Select chip by pulling down CS
    digitalWrite(_csPin, LOW);

    // compute 16 bit value for channel and shift into position
    int tInt = (channel + 24) << 6;

    // transfer the high byte
    SPI.transfer(highByte(tInt));

    // transfer the low byte and get high 4 bits
    byte hiRes = SPI.transfer(lowByte(tInt));

    // get the low 8 bits
    byte lowRes = SPI.transfer(0);
    hiRes = hiRes & B00001111;

    //Combine to make 12 bit value
    value[channel] +=  (hiRes << 8 | lowRes);

    // disable ADC SPI
    digitalWrite(_csPin, HIGH);
    LOG("End Transaction");
    SPI.endTransaction();               //end SPI Transaction
    LOG("SPI end");
    SPI.end();
  
  LOG("Calculate oversampled result");
  
  AddToMovingAvg(&filteredChannel[channel], intResult);
  LOG("End read channel", channel );
}

void MCP3208::readADC() {
  for (int i = 0; i < 8; i++) {
    readADC(i);
  }
}
uint16_t MCP3208::getMovingValue(char channel) {
  return value[channel];
  //return GetOutputValue(&filteredChannel[channel]);
}

