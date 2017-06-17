#ifndef Moving_AVERAGE_H
#define Moving_AVERAGE_H

#include <inttypes.h>

#ifndef SIZE_OF_AVG
#define SIZE_OF_AVG 12
#endif

// Datentyp, ueber den der gleitende Mittelwert berechnet werden soll.
typedef uint16_t tMovingAvgType;
// typedef Moving tMovingAvgType;

// Wird nur intern fuer die Durchschnittsberechnung benutzt.
// Muss Zahlen fassen koennen, die SIZE_OF_AVG mal groesser als tMovingAvgType sind.
typedef uint32_t tTempSumType;
// typedef Moving tTempSumType;

// Die Struktur, in der die Daten zwischengespeichert werden
typedef struct
 {
  tMovingAvgType aData[SIZE_OF_AVG];
  uint8_t IndexNextValue;
 } tMovingAvgFilter;


// Initialisiert das Filter mit einem Startwert.
void InitMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter,
      tMovingAvgType i_DefaultValue);

// Schreibt einen neuen Wert in das Filter.
void AddToMovingAvg(tMovingAvgFilter * io_pMovingAvgFilter,
       tMovingAvgType i_ui16NewValue);

// Berechnet den Durchschnitt aus den letzten SIZE_OF_AVG eingetragenen Werten.
tMovingAvgType GetOutputValue(tMovingAvgFilter * io_pMovingAvgFilter);

#endif
