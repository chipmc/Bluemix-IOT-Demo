
  Bluemix-IOT-Demo
  Project
  ----------------------------------
  Developed with embedXcode

  Project Bluemix-IOT-Demo
  Created by Charles McClelland on 10/5/16
  Copyright © 2016 Charles McClelland
  Licence GNU General Public Licence



  References
  ----------------------------------
/* Chip McClelland - Cellular Data Logger
BSD license, Please keep my name and credits in any redistribution

Requirements:
- Account on Ubidots.  http://www.ubidots.com
- Adafruit CC3000 Breakout Board - see below for link
- Adafruit Ultimate GPS: https://www.adafruit.com/product/746
- Teensy 3.1
Credits:
- Ubidots Demo Code from: Mateo Velez for Ubidots,Inc. Modified 15 Nov 2014
Configuration:
- You will need to add your SSID and password information for WiFi
Hardware Connections: (Teensy 3.1)
Adafruit CC3000 Breakout   Function
VCC        +3.3V           External Power - not from Teensy
GND        GND             GND
2          INT             Interrupt (Must be an interrupt pin - Mico has 5)
5          EN              WiFi Enable (Can be any ping)
10         CS              SPI Chip Select
11         MOSI            SPI MOSI
12         MISO            SPI MISO
13         SCK             SPI Clock
Adafruit Ultimate GPs
VCC        +3.3V           External Power - not from Teensy
GND        GND
7          Enable          Turn on and off module
8          Fix             Do we have a fix?
0          TX->            Hardware UART Rx
1          RX<-            Hardware UART Tx
Sparkfun MMA8452 Breakout
VCC        +3.3V          External Power - not from Teensy
GND         GND
18          SDA0          i2c data - should have 4.7k pullup
19          SCL0          i2c clock - should have 4.7k pullup
3           I2            To wake up on tap - should have 4.7k pullup
4           I1            Not used by wired for future use - should have 4.7k pullup
Sensitivity 10k Trim Pots
14          A0            Delay Time
15          A1            Sensitivity Time
Indicator Leds (all three have switched ground)
16          LED1          Tap - Red
17          LED2          TBD - Yellow
VCC         LED3          Power
/******************** Library Credits *******************************
This is an example for the Adafruit CC3000 Wifi Breakout & Shield

Designed specifically to work with the Adafruit WiFi products:
----> https://www.adafruit.com/products/1469

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried & Kevin Townsend for Adafruit Industries.
BSD license, all text above must be included in any redistribution

Improved Wire Library for Teensy 3.1 Used below (Wire.h works as well)
Credit - Brian - Nox77 - https://github.com/nox771/i2c_t3
Thanks Brian!


****************************************************************/


  embedXcode
  embedXcode+
  ----------------------------------
  Embedded Computing on Xcode
  Copyright © Rei VILO, 2010-2016
  All rights reserved
  http://embedXcode.weebly.com

