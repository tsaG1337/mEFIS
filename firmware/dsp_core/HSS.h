//The HSS library by Patrick Bihn is licensed under CC BY 3.0
#include "Arduino.h"
class HSS
{
  private:
    int8_t ChipselectPin = 0;
    float temperature = 0;
    int16_t pressure = 0;
    int16_t lowPressure = 0;
    int16_t highPressure = 0;
    int16_t pressureDigital = 0;
    int inByte_1 = 0;
    int inByte_2 = 0;
    int inByte_3 = 0;
    int inByte_4 = 0;
    uint8_t sampleRate = 5;
  public:
    HSS(uint8_t ChipSelect, int16_t low, int16_t high);
    int getTemperature();
    int16_t getPressure();
    void readSensor();
    void oversampleReadSensor();
    uint8_t getSampleRate();
    void setSampleRate(uint8_t newSampleRate);

};
