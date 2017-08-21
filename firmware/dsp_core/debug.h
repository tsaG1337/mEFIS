#ifndef _H_LOGGER_H
#define _H_LOGGER_H

#define xDEBUG_OUTPUT
#define DEBUGSerial SerialUSB

#if  defined(DEBUG)
#define LOG(x, ...)  {char msg[128]; sprintf(msg, "%d [LOG: %s] ",millis(),DEBUG); DEBUGSerial.print(msg); DEBUGSerial.print(x); DEBUGSerial.println(__VA_ARGS__);}
#define DEBUG_PRINTLN(x) {DEBUGSerial.println(x);}

#else
#define LOG(x, ...)
#define DEBUG_PRINTLN(x)
#endif
#endif
