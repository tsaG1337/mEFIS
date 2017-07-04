#ifdef DEBUG
#define DEBUG_PRINT(x)  SerialUSB.print (x)
#define DEBUG_PRINTLN(x)  SerialUSB.println (x)
#define DEBUGSerial SerialUSB      //SerialUSB can be used as Debugging Port       
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)

  //DEBUGSerial.begin(115200);        //USB Serial Port for Debugging Purpose
  #endif
