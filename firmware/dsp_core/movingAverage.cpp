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

void InitMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter, 
      tMovingAvgType i_DefaultValue)
{ 
  for (uint8_t i = 0; i < SIZE_OF_AVG; ++i)
  {
    io_pMovingAvgFilter->aData[i] = i_DefaultValue;
  }
  io_pMovingAvgFilter->IndexNextValue = 0;
} 


void AddToMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter,
       tMovingAvgType i_NewValue)
{ 
  // writes the value into the current position
  io_pMovingAvgFilter->aData[io_pMovingAvgFilter->IndexNextValue] =
    i_NewValue;
  // the next value will be written into the next position
  io_pMovingAvgFilter->IndexNextValue++;
  // start from the beginning if the end is reached
  io_pMovingAvgFilter->IndexNextValue %= SIZE_OF_AVG;
}  


tMovingAvgType GetOutputValue(tMovingAvgFilter * io_pMovingAvgFilter)
{
  tTempSumType TempSum = 0;
  // calculate the moving average
  for (uint8_t i = 0; i < SIZE_OF_AVG; ++i)
  {
    TempSum += io_pMovingAvgFilter->aData[i];
  }
   //divide the Sum of samples by the number of averages
  tMovingAvgType o_Result = (tMovingAvgType) (TempSum / SIZE_OF_AVG);
  return o_Result;
}
