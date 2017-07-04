#ifdef DEBUG
#define DEBUG_PRINT(x)  SerialUSB.print (x)
#define DEBUG_PRINTLN(x)  SerialUSB.println (x)
#define DEBUGSerial SerialUSB      //SerialUSB can be used as Debugging Port       
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUGSerial SerialUSB      //SerialUSB can be used as Debugging Port 
#endif
