# mEFIS

[![N|Solid](http://bihnairy.de/img/case.1_small.png)](http://bihnairy.de)

>mEFIS is an abbreviation for "my Electronic Flight Instrument System" and is based on open source Software. It has been designed for permanent integration into the ultralight aircraft cockpit.

## Hardware!

  You can find all the necessary hardware files in the hardware folder. The Gerber files are in the Gerber RS-274X format which are supported by most PCB Factories.
  - The Hardware files have been updated to V1.1 which fixes a reset bug. The DSP now doesnt need a custom Bootloader anymoe. Problem was: As soon as the DSP transmitts data over USB.Serial it would have reset the Main Core (TX LED output was connected to Main Core LED.

## Firmware!

The Main Core hosts a Wifi network which can be encrypted (not recommended for Android users) and fully configured by recompiling the firmware.

The data is send via UDP on the specified Port. You can change the port in the configuration header of the Firmware by changing `UDPremotePort = 1234;` to the desired value.
The Datastream is broadcasted to every device in the network using the broadcast adress. 
DHCP is supported, however there seems to be a small hickup in the DHCP library by Espressif / Arduino that stops allocating IP adresses to more than 1 devices. It is therefore recommended to statically allocate a static IP on the tabled device itself.
###### NMEA

The mEFIS sends out standard and custom NMEA sentences. These are also commonly used by iLEVIL or iCFly devices. These include:
```
$RPYL, Roll, Pitch, Yaw (Magnetic heading), Inclination, Yaw Rate (turn coordinator), G force,
$RPYL,82,69,2021,83,14,1013,0,<CR><LF>
```
```
$APENV1, Airspeed, Altitude, 0,0,0,Vertical Speed
$APENV1,124,179,0,0,0,500,<CR><LF>
```
```
$APPOWER, Voltage, backup battery charging status
$APPOWER,1212,65,1<CR><LF>
```

Besides this custom messages, the standard minimum recommended GPS information `$GPRMC` is also send. For more information about the specific NMEA sentences, you can also checkout the [LEVIL](http://aviation.levil.com) webpage or the [iCFly manual](https://www.siebert.aero/media/products/Handbuch_ICflyAHRSII.pdf)

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

Find more information at the [BihnAiry Page](http://bihnairy.de/).
