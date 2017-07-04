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
#ifndef Moving_AVERAGE_H
#define Moving_AVERAGE_H

#include <inttypes.h>

#ifndef SIZE_OF_AVG
#define SIZE_OF_AVG 12
#endif


typedef uint16_t tMovingAvgType; //typedef for Moving average (maximum number is 65.535)


// will be used internally to gather all the values. Has to be SIZE_OF_AVG bigger than tMovingAvgType.
// In this constellation (using uint16_t) it can hold a maximum of 65446 samples.
typedef uint32_t tTempSumType;


// Die Struktur, in der die Daten zwischengespeichert werden
typedef struct
 {
  tMovingAvgType aData[SIZE_OF_AVG];
  uint8_t IndexNextValue;
 } tMovingAvgFilter;



void InitMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter,
      tMovingAvgType i_DefaultValue); //initializes the filter with a starting value

// Adds a new value to the filter
void AddToMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter,
       tMovingAvgType i_ui16NewValue);

// gets the moving average of the last SIZE_OF_AVG values
tMovingAvgType GetOutputValue(tMovingAvgFilter * io_pMovingAvgFilter);

#endif
