# mEFIS

>mEFIS is an abbreviation for "my Electronic Flight Instrument System" and is based on open source Software. It has been designed for permanent integration into the ultralight aircraft cockpit.

## Hardware!

  You can find all the necessary hardware files in the hardware folder. The Gerber files are in the Gerber RS-274X format which are supported by most PCB Factories.
  - The Hardware files have been updated to V1.1 which fixes a reset bug. The DSP now doesnt need a custom Bootloader anymoe. Problem was: As soon as the DSP transmitts data over USB.Serial it would have reset the Main Core (TX LED output was connected to Main Core LED.

## Firmware!

The Main Core hosts a Wifi network which can be encrypted (not recommended for Android users) and fully configured by recompiling the firmware.

The data is send via UDP on the specified Port. You can change the port in the configuration header of the Firmware by changing `UDPremotePort = 1234;` to the desired value.
The Datastream is broadcasted to every device in the network using the broadcast adress. 
DHCP is supported, however there seems to be a small hickup in the DHCP library by Espressif / Arduino that stops allocating IP adresses to more than 1 devices. It is therefore recommended to statically allocate a static IP on the tabled device itself.
#### NMEA

The mEFIS communication is in "NMEA 0183 / pseudo NMEA”. These are also commonly used by iLEVIL or iCFly devices. These include:
##### Movement and Attitude
^
```
$RPYL, RRR, PPP, YYYYY, 0, YRR, GG,
```
| Value | Description |
| --- | --- |
| RRR | Roll [tenths of a degree] |
| PPP | Pitch [tenths of a degree] |
| YYYYY | YAW (Magnetic heading) [tenths of a degree] |
| YRR | YAW Rate (Turning Indicator) [degree] |
| GG | G-Force |

##### Barometric Values
^
```
$APENV1, SSS, AAA, 0,0,0,VVVV,
```
| Value | Description |
| --- | --- |
| SSS | Indicated Airspeed [knots] |
| AAA | Barometric Altitude [feet] |
| VVVV | Barometric Vertical Speed [feet/min]|

##### Engine Data
^
```
$APENG1, PP, TTT, LLL,RRR,FFF,VS,
```
| Value | Description |
| --- | --- |
| PP | Oil Pressure [bar] |
| TTT | Oil Temperature [°C] |
| LLL | Left Fuel Gauge [%] |
| RRR | Right Fuel Gauge [%] |
| FFF | Fuel Pressure [tenth of bar] |

```
$APENG2, UUUU,
```
| Value | Description |
| --- | --- |
| UUUU | Engine RPM |

##### Power
^
```
$APPOWER, VVVV, B,
```
| Value | Description |
| --- | --- |
| VVVV | Supply Voltage (milliVolt) |
| B | Battery Charging Status ( 1 / 0 ) |

##### Flight Status
^

```
$PFA,FLT,f,s,dddddd,tttt,u,llll,v,aaaaaa*hh
```
| Value | Description |
| --- | --- |
| FLT | fixed string, identifying flight status message |
| f | flight status: F = in flight, G = on ground |
| s | determination source for flight status, I for IAS, G for GPS |
| dddddd | date of takeoff in format DDMMYY, empty before takeoff |
| ttt | time of takeoff in format HHMMSS, empty before takeoff |
| u | reference system of takeoff time is UTC |
| llllll | time of landing in format HHMMSS, empty before landing |
| v | reference system of landing time is UTC |
| aaaaaa | duration airborne in format HHMMSS, empty before takeoff |
| *hh | Checksum |

Besides this custom messages, the standard minimum recommended GPS information `$GPRMC` and `$GPGGA` Global Positioning System Fix Data is also send. 

 #### GPS
```
$GPRMC,hhmmss.ss,A,llll.ll,a1,yyyyy.yy,a2,x.x1,x.x2,ddmmyy,x.x3,a3*hh
```
| Value | Description |
| --- | --- |
|hhmmss | UTC of position fix |
|A    |Data status (V=navigation receiver warning)|
| llll.ll |Latitude of fix |
|a1 | N (-orth) or S (-outh)|
|yyyyy.yy | Longitude of fix|
| a2 | E (-ast) or W (-est)|
| x.x1 | (GPS-) Speed over ground in knots |
| x.x2 |    Track made good in degrees True |
| ddmmyy | UT date |
| x.x3 | Magnetic variation degrees (Easterly var. subtracts from true course) |
|a3   | E (-ast) or W (-est)|
|hh   | Checksum |

```
$GPGGA,hhmmss.ss,llll.ll,a1,yyyyy.yy,a2,x,xx,x.x1,x.x2,M1,x.x3,M2,x.x4,xxxx*hh
```

| Value | Description |
| --- | --- |
|hhmmss | UTC of position |
| llll.ll | Latitude |
| a1 | N (-orth) or S (-outh) |
| yyyyy.yy | Longitude |
| a2 | E (-ast) or W (-est)|
| x | GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix) |
| xx    | Number of satellites in use [not those in view] |
| x.x1    | Horizontal dilution of position |
| x.x2 | Antenna altitude above/below mean sea level (geoid) |
| M1 | Meters  (Antenna height unit) |
| x.x3 | Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level.  -=geoid is below WGS-84 ellipsoid) |
| M2  | Meters  (Units of geoidal separation) |
| x.x4 |  = Age in seconds since last update from diff. reference station |
| xxxx |   = Diff. reference station ID# |
| hh | Checksum |

For more information about the specific NMEA sentences, you can checkout the GPS-NMEA 0183 reference document at [wikipedia](https://de.wikipedia.org/wiki/NMEA_0183). For mor information visit the [iCFly manual](https://www.siebert.aero/media/products/Handbuch_ICflyAHRSII.pdf) manual.

###### Compiling and burning firmware

I recommend using the Arduino IDE for compiling. The hardware Version 1.1 supports the standard Arduino Zero bootloader which can be choosen in the Target menu.

- if you have a V1.0 (or earlier) Board, you have to install the custom Bootloader and Pin mapping using the Board Manager with the following URL
```http://bihnairy.de/hardware/mefis/Arduino_hardware_def/package_mefis_index.json```


The Drivers and Bootloaders for the Main Core (Espressif ESP8266) have to be installed through the Board manager using the following URL
```http://arduino.esp8266.com/stable/package_esp8266com_index.json```

The Main Core has to be flashed using the UART interface on J2. This can be done using a simple FTDI Breakout Board. Before flashing, the Chip has to boot into Flash mode which is done by tapping the reset button on the board while connecting GPIO0 (SS_CAN) to ground (The DSP resets the Main Core on every boot). The default Baudrate for flashing is 115200. Theoretically, it is also possible to program the Board using the RS232 interface if you dont have a FTDI Board at hand.

The DSP Core can be flashed using the USB Port with the Arduino IDE default settings.

## Software!

Since the mEFIS is using widely used avionics standards it can be used with several applications such as the iLevil AHRS Utility, Air Navigation Pro or Sky-Map. However, if you are a "no risk no fun", "I dont need reliability" or a "I need a hassle everytime Im using my avionics" type, you can also use an Adroid device. Since todays pilots are more the "better safe than sorry" type, there are not many Android avionic apps available and you are on your own.
