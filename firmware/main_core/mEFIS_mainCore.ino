


#define SerialDebug true   // set to true to get Serial output for debugging
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "mpu9250.h"
#include "tinyGPSpp.h"
#include "HSS.h"
//********** Configuration **********//
#define DEBUG                       //uncomment to remove Debug messages on the USBSerial @9600 baud

#define WHO_AM_I 101                // Device Name
#define ID 1000                     //Serial Number
#define HARDVERSION 2               //Hardware Version
#define SOFTVERSION 1               //Software Version

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
TXLED0;

//******** Class Objects ********//

SoftwareSerial RS232Serial(RS232_RXPin, RS232_TXPin, false, 256); //initialize RS232 Serial
TinyGPSPlus gps;                  //Initialize GPS
MPU9250 imu;                      //initialize onboard IMU 4
HSS diffPress(15, -60, 60); //Initialize Differential Pressure Sensor with +-60mBar
HSS absPress(16, 0, 1600); //Initialize Differential Pressure Sensor with +-60mBar

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
  Serial.begin(115200);
  DEBUG_PRINTLN("Initizializing mEFIS Main Core");
  DEBUG_PRINTLN ("Booting up!");
  DEBUG_PRINT ("Serial: ");           DEBUG_PRINTLN (ID);
  DEBUG_PRINT ("Hardware Version: "); DEBUG_PRINTLN (HARDVERSION);
  DEBUG_PRINT ("Software Version: "); DEBUG_PRINTLN (SOFTVERSION);
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
  //RS232Serial.begin(RS232_Baudrate);          //Start Serial connection on the RS232 Port


}

void loop() {
  uint32_t currentMillis = millis();

  //Loop1
  if (currentMillis - loop1PreMill >= loop1interval) {
    loop1PreMill = currentMillis;
    diffPress.readSensor();
    absPress.readSensor();
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
    
    Serial.print("X-acceleration: "); Serial.print(1000*imu.ax); Serial.print(" mg ");
    Serial.print("Y-acceleration: "); Serial.print(1000*imu.ay); Serial.print(" mg ");
    Serial.print("Z-acceleration: "); Serial.print(1000*imu.az); Serial.println(" mg ");
 
    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); Serial.print(imu.gx, 3); Serial.print(" degrees/sec "); 
    Serial.print("Y-gyro rate: "); Serial.print(imu.gy, 3); Serial.print(" degrees/sec "); 
    Serial.print("Z-gyro rate: "); Serial.print(imu.gz, 3); Serial.println(" degrees/sec"); 
    
    // Print mag values in degree/sec
    Serial.print("X-mag field: "); Serial.print(imu.mx); Serial.print(" mG "); 
    Serial.print("Y-mag field: "); Serial.print(imu.my); Serial.print(" mG "); 
Serial.print("Z-mag field: "); Serial.print(imu.mz); Serial.println(" mG"); 


  }
  while (RS232Serial.available() > 0) {
    //gps.encode(RS232Serial.read());

  }
  yield();
}
//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================
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
