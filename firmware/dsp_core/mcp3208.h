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
#include "movingAverage.h"
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

