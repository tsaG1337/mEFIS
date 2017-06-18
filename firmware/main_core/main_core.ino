#include <ESP8266WiFi.h>
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
#define loop2interval 100         //Loop2 Interval

#define MPU9250_ADDRESS 0x68
#define MEFIS_ANALOGCORE_ADRESS 0x22

#define RS232_Baudrate 57600        // Baudrate for external RS232 Device

#define errorLEDPin 0               //Diagnostic LED
#define RS232_RXPin 2              //RS232 RX Pin
#define RS232_TXPin 13              //RS232 TX Pin
#define AudioOutPin 15              //Audio Pin connected to the Intercom
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

//********* Variables ***********//

bool INTERNAL_IMU = 0;
bool INTERNAL_GPS = 0;
bool ANALOGCORE_FOUND = 0;
byte UNKNOW_I2C_DEVICES = 0;


uint32_t loop1PreMill = 0;        //Last time Loop1 was updated
uint32_t loop2PreMill = 0;        //Last time Loop2 was updated

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
    DEBUG_PRINTLN("Start");
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


  DEBUG_PRINTLN ("");
  DEBUG_PRINTLN("Initializing Ports");
  pinMode(errorLEDPin, OUTPUT);
  DEBUG_PRINTLN ("Joining I2C Bus as master");
  Wire.begin();
  DEBUG_PRINTLN ("Scanning I2C Ports for peripherals.");
  if (scanI2CDevices(MPU9250_ADDRESS, 0x75, 0x71)) {
    DEBUG_PRINTLN ("MPU9250 found!");
    INTERNAL_IMU = 1;
  } else {
    DEBUG_PRINTLN ("MPU9250 not found!");
    INTERNAL_IMU = 0;
    digitalWrite(errorLEDPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  if (scanI2CDevices(MEFIS_ANALOGCORE_ADRESS, 0x01, 0xB)) {
    DEBUG_PRINTLN ("AnalogCore found!");
    ANALOGCORE_FOUND = 1;
  } else {
    DEBUG_PRINTLN ("AnalogCore not found!");
    ANALOGCORE_FOUND = 0;
    digitalWrite(errorLEDPin, HIGH);   // turn the LED on (HIGH is the voltage level)
  }

  DEBUG_PRINTLN ("Initializing Serial Ports:");
  DEBUG_PRINT ("GPS Port with Baudrate:"); DEBUG_PRINTLN (RS232_Baudrate);
  Serial.begin(RS232_Baudrate);          //Start Serial connection on the RS232 Port


}

void loop() {
  uint32_t currentMillis = millis();

  //Loop1
  if (currentMillis - loop1PreMill >= loop1interval) {
    loop1PreMill = currentMillis;
    playTone(AudioOutPin, 500, 100);

    imu.readAccelData();  // Read the x/y/z adc values

    imu.readGyroData();
    imu.readMagData();
    imu.getAres();
  }

  //Loop2
  if (currentMillis - loop2PreMill >= loop2interval) {
    loop2PreMill = currentMillis;

    //DEBUG_PRINT("Time: "); DEBUG_PRINT(gps.time.hour()); DEBUG_PRINT(":"); DEBUG_PRINT(gps.time.minute());  DEBUG_PRINT(" and "); DEBUG_PRINT(gps.time.second()); DEBUG_PRINTLN(" seconds.");
  }
  while (Serial.available() > 0) {
    gps.encode(Serial.read());

  }
  yield();
}

bool scanI2CDevices(byte address, byte subAddress, byte result) {
  byte error;

  Wire.beginTransmission(address);
  error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.print("I2C device found at address 0x");
    if (address < 16) {
      Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
    }
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    delay(10);
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
    uint8_t data = Wire.read();                      // Fill Rx buffer with result
    if (data == result) {
      Serial.println("I2C device identified.");
      return true;                          //return String matching!
    } else {
      return false;                           //return String does not match
    }
  }
  else if (error == 4)
  {
    Serial.print("Unknown error at address 0x");
    if (address < 16)
      Serial.print("0");
    Serial.println(address, HEX);
    return false;
  }

}

void syncAnalogCore_AnalogSensors() {

}

void playTone(int _pin, int _frequency, int _length) {
  analogWriteFreq(_frequency);
  analogWrite(_pin, 512);
  delay(_length);
  analogWrite(_pin, 0);
}
