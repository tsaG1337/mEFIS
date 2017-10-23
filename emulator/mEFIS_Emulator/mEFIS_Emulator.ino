#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFiUdp.h>


//********** Configuration **********//
#define DEBUG                       //uncomment to remove Debug messages on the Serial @9600 baud

#define WHO_AM_I 101                // Device Name
#define ID 1000                     //Serial Number
#define HARDVERSION 2               //Hardware Version
#define SOFTVERSION 1               //Software Version

const char *ssid = "mEFIS";         //Accesspoint SSID

#define loop1interval 10         //Loop1 Interval
#define loop2interval 1000         //Loop2 Interval

#define MPU9250_ADDRESS 0x68
#define DSP_CORE_ADDRESS 0x22

#define RS232_Baudrate 57600        // Baudrate for external RS232 Device

#define errorLEDPin 0               //Diagnostic LED
#define RS232_RXPin 2              //RS232 RX Pin
#define RS232_TXPin 13              //RS232 TX Pin
#define AudioOutPin 15              //Audio Pin connected to the Intercom

//******** Variables **********//

uint32_t loop1PreMill = 0;        //Last time Loop1 was updated
uint32_t loop2PreMill = 0;        //Last time Loop2 was updated



// UDP variables
unsigned int UDPlocalPort = 8888;
unsigned int UDPremotePort = 1234;



//******** Class Objects ********//

WiFiUDP UDP;

void setup()
{
  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  delay(500);
}

void loop() {
  uint32_t currentMillis = millis();

  //Loop1
  if (currentMillis - loop1PreMill >= loop1interval) {
    loop1PreMill = currentMillis;
  }

  //Loop2
  if (currentMillis - loop2PreMill >= loop2interval) {
    loop2PreMill = currentMillis;
    UDPSendData();
  
  }
  yield();

}
void UDPSendData(void) {
 
  IPAddress broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();  //Build the Broadcast IP Adress

  //$RPYL
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write("$RPYL,010,020,0990,0,012,1,");
  UDP.endPacket();

  //$RPYL
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write("$APENV1,230,2500,0,0,0,1234,");
  UDP.endPacket();

  //$APENG1
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write("$APENG1,02,095,87,55,120,0,");
  UDP.endPacket();

    //$APENG2
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write("$APENG2,2300,");
  UDP.endPacket();

      //$APPOWER
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write("$APPOWER,1230,1,");
  UDP.endPacket();

   //$GPRMC
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write("$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62");
  UDP.endPacket();
}

