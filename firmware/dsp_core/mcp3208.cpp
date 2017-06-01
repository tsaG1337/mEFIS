#include "mcp3208.h"
#include "movingAverage.h"

MCP3208::MCP3208(char Pin, char oversamplingRate){
    _csPin = Pin;
  pinMode(_csPin,OUTPUT);     //configure CS Pin as Output
  digitalWrite(_csPin,HIGH);  //set CS Pin high
  _oversamplingRate = oversamplingRate;
  for (int i=0; i<8; i++){
    InitMovingAvg(&filteredChannel[i], 0); //ini6
  }
}

double MCP3208::readADC(char channel)
{
  uint32_t oversampleRead = 0;
  uint16_t intResult = 0;
  // enable ADC SPI
  //Select chip by pulling down CS
  

for (int i=0; i < _oversamplingRate; i++){
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
  oversampleRead +=  (hiRes << 8 | lowRes);

    // disable ADC SPI
  digitalWrite(_csPin, HIGH);
  
}
 
  intResult = oversampleRead / _oversamplingRate ;
  AddToMovingAvg(&filteredChannel[channel], intResult);
  
  return intResult;
}

double MCP3208::readADC(){
  for (int i=0; i<8;i++){
    readADC(i);
  }
}
double MCP3208::getMovingValue(char channel){
  return GetOutputValue(&filteredChannel[channel]);
}

