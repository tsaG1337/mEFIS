#include "movingAverage.h"

void InitMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter, 
      tMovingAvgType i_DefaultValue)
{ 
  for (uint8_t i = 0; i < SIZE_OF_AVG; ++i)
  {
    io_pMovingAvgFilter->aData[i] = 0;
  }
  io_pMovingAvgFilter->IndexNextValue = 0;
} 


void AddToMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter,
       tMovingAvgType i_NewValue)
{ 
  // Neuen Wert an die dafuer vorgesehene Position im Buffer schreiben.
  io_pMovingAvgFilter->aData[io_pMovingAvgFilter->IndexNextValue] =
    i_NewValue;
  // Der naechste Wert wird dann an die Position dahinter geschrieben.
  io_pMovingAvgFilter->IndexNextValue++;
  // Wenn man hinten angekommen ist, vorne wieder anfangen.
  io_pMovingAvgFilter->IndexNextValue %= SIZE_OF_AVG;
}  


tMovingAvgType GetOutputValue(tMovingAvgFilter * io_pMovingAvgFilter)
{
  tTempSumType TempSum = 0;
  // Durchschnitt berechnen
  for (uint8_t i = 0; i < SIZE_OF_AVG; ++i)
  {
    TempSum += io_pMovingAvgFilter->aData[i];
  }
  // Der cast is OK, wenn tMovingAvgType und tTempSumType korrekt gewaehlt wurden.
  tMovingAvgType o_Result = (tMovingAvgType) (TempSum / SIZE_OF_AVG);
  return o_Result;
}
