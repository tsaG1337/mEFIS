#define DEBUGSerial SerialUSB      //SerialUSB can be used as Debugging Port 

#ifdef DEBUG    
#define DEBUG_PRINT(x)  char msg[32]; sprintf(msg,"[" + millis() "]" + x); serial.print(msg);
#define DEBUG_PRINTLN(x)  char msg[32]; sprintf(msg,"[" + millis() "]" + x); serial.println(msg);
 
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

