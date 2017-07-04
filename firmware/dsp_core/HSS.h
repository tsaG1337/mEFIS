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
    bool enabled = 0;
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
