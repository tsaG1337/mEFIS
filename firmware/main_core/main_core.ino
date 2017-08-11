#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "mpu9250.h"
#include "tinyGPSpp.h"
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

// UDP variables
unsigned int UDPlocalPort = 8888;
unsigned int UDPremotePort = 1234;

//********** Serial Ports **********//

#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif


//******** Class Objects ********//

TinyGPSPlus gps;                  //Initialize GPS
MPU9250 imu;                      //initialize onboard IMU 4
WiFiUDP UDP;

//********* Variables ***********//

bool INTERNAL_IMU = 0;
bool INTERNAL_GPS = 0;
byte DSP_CORE_WHOAMI = 0;
uint16_t DSP_CORE_SERIAL = 0;
uint8_t HARD_VERSION = 0;
uint8_t DSP_CORE_SOFT_VERSION = 0;

byte UNKNOW_I2C_DEVICES = 0;
byte error, address;
int nDevices = 0;

uint32_t loop1PreMill = 0;        //Last time Loop1 was updated
uint32_t loop2PreMill = 0;        //Last time Loop2 was updated

boolean udpConnected = false;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char ReplyBuffer[200] = "ACK"; // a string to send back
//Variables for internal GPS only

//location
bool locIsValid = false;
double lat;
double lng;
double course;
//date
uint32_t value;
uint16_t year;
uint8_t month;
uint8_t day;
//time
uint8_t hour;
uint8_t minute;
uint8_t second;
//speed
double GPS_Speed_Knots;
double GPS_Altitude_Meter;



void setup()
{
  Serial.begin(RS232_Baudrate);
  delay(1000);
  DEBUG_PRINTLN("Initizializing mEFIS Main Core");

  DEBUG_PRINT ("Serial: ");           DEBUG_PRINTLN (ID);
  DEBUG_PRINT ("Hardware Version: "); DEBUG_PRINTLN (HARDVERSION);
  DEBUG_PRINT ("Software Version: "); DEBUG_PRINTLN (SOFTVERSION);
  DEBUG_PRINTLN ("");
  DEBUG_PRINTLN ("Booting up WiFi Network");
  DEBUG_PRINT ("SSID: ");
  DEBUG_PRINTLN (*ssid);
  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  DEBUG_PRINT("AP IP address: ");
  DEBUG_PRINTLN(myIP);
  DEBUG_PRINTLN ("Initializing OTA feature");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DEBUG_PRINT("Start updating ");
    DEBUG_PRINTLN(type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_PRINTLN("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINT("Progress: %u%%\r");
    DEBUG_PRINTLN(progress / (total / 100));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINT("Error[%u]: ");
    DEBUG_PRINT(error);
    if (error == OTA_AUTH_ERROR) DEBUG_PRINTLN("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG_PRINTLN("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG_PRINTLN("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINTLN("Receive Failed");
    else if (error == OTA_END_ERROR) DEBUG_PRINTLN("End Failed");
  });
  ArduinoOTA.begin();
  delay(500);
  DEBUG_PRINTLN ("");
  DEBUG_PRINTLN("Initializing Ports");
  pinMode(errorLEDPin, INPUT); //TEST
  DEBUG_PRINTLN ("Joining I2C Bus as master");
  Wire.begin();
  DEBUG_PRINTLN ("Scanning I2C Ports for peripherals.");
  scanI2CDevices();

  syncDSPCore_info();



}

void loop() {
  uint32_t currentMillis = millis();

  //Loop1
  if (currentMillis - loop1PreMill >= loop1interval) {
    loop1PreMill = currentMillis;
    //playTone(500);

    //imu.readAccelData();  // Read the x/y/z adc values
    //imu.readGyroData();
    //imu.readMagData();
    //imu.getAres();

  }

  //Loop2
  if (currentMillis - loop2PreMill >= loop2interval) {
    loop2PreMill = currentMillis;
    //Serial.println(DSP_CORE_WHOAMI, HEX);
    //DEBUG_PRINT("Time: "); DEBUG_PRINT(gps.time.hour()); DEBUG_PRINT(":"); DEBUG_PRINT(gps.time.minute());  DEBUG_PRINT(" and "); DEBUG_PRINT(gps.time.second()); DEBUG_PRINTLN(" seconds.");
    //digitalWrite(errorLEDPin, !digitalRead(errorLEDPin));
    UDPSendData();
  }
  while (Serial.available() > 0) {
    char bytesRead = Serial.read();
    DEBUG_PRINT (bytesRead);
    gps.encode(bytesRead);
  }
  yield();




}
void scanI2CDevices() {
  for (address = 1; address < 127; address++ )
  {
    yield();
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
}


void syncDSPCore_info() {

  DSP_CORE_WHOAMI = readBytefromDSPCore(DSP_CORE_ADDRESS, 3 );

}

void playTone(int _frequency) {
  analogWriteFreq(_frequency);
  analogWrite(AudioOutPin, 512);
}
uint8_t readBytefromDSPCore(uint8_t address, uint8_t pointer ) {
  Wire.beginTransmission(address);
  Wire.write(pointer); // set the Pointer
  Wire.endTransmission();
  delay(2); //give the DSP-Core time to arrange things.
  int n = Wire.requestFrom(address, 1); // request one byte from Slave
  while (n != 1) {
    if (n == 1) {
      // Read low byte into buffer
      return Wire.read();
    }
  }
}
void UDPSendData(void) {
  char buffer[200];
  sprintf(buffer, "$GPRMC,%u,%d,", gps.time.value(), gps.location.isValid());
  dtostrf(gps.location.lat(), 4, 5, &buffer[strlen(buffer)]);
  strcat(buffer, ",");
  dtostrf(gps.location.lng(), 4, 5, &buffer[strlen(buffer)]);
  strcat(buffer, ",");
  dtostrf(gps.speed.knots(), 3, 0, &buffer[strlen(buffer)]);
  strcat(buffer, ",");
  IPAddress broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();  //Build the Broadcast IP Adress
  UDP.beginPacket(broadcastIp, UDPremotePort);
  UDP.write(buffer);
  UDP.endPacket();

}

