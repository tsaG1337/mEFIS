#include "movingAverage.h"
#include <Arduino.h>
#include <SPI.h>

class MCP3208
{
  private:

    char _csPin;
    char _oversamplingRate;
    char _movingValRate;

    tMovingAvgFilter filteredChannel[8];




  public:
    MCP3208(char Pin, char oversamplingRate);
    double readADC(char channel);
    double readADC();
    double getMovingValue(char channel);
};

