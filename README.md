# BNO055 AT style driver
#

### (c) by Thomas Welsch / 2016 under the GNU Lesser General Public License

This driver implements a simple "AT" style interface for the BNO055 sensor
chip (BOSCH).
The BNO055 is connected via the I2C interface, see wiring below.

Tested  with Arduino Pro Micro (virtual Serial interface on USB or real
serial interface on pins 0/1) and Arduino Nano.

### Current commands:
```
AT     : OK
ATH    : Display help
ATI    : Display sensor details
ATS    : Display sensor status
ATC    : Display sensor status (calibration)
ATT    : Display sensor temperatur in C degree
ATO    : Display sensor orientation in Euler degree
ATO1   :   Enable continuous Euler degree output
ATO0   :   Disable E1
ATA    : Display sensor accelerometer in m/s^2
ATA1   :   Enable continuous accelerometer output
ATA0   :   Disable A1
ATY    : Display sensor gyroscope in rad/s
ATY1   :   Enable continuous gyroscope output
ATY0   :   Disable Y1
ATM    : Display sensor magnetometer in uT
ATM1   :   Enable continuous magnetometer output
ATM0   :   Disable G1
ATG    : Display sensor gravity in m/s^2
ATG1   :   Enable continuous gravity output
ATG0   :   Disable G1
ATL    : Display sensor linear accelerometer in m/s^2
ATL1   :   Enable continuous linear accelerometer output
ATL0   :   Disable L1
AT1    : Enable continuous linear accelerometer output of all sensor data
AT0    : Disable AT1
```

### Example:
```
ATO1
OK
Ori: 243.5000 -37.1875 1.3125
Ori: 208.6875 -15.0000 -22.7500
Ori: 207.1250 -29.9375 -39.5625
Ori: 223.8750 -40.0625 -35.2500
Ori: 260.6875 -51.0625 -14.5000
......
ATO0
OK
```

### ToDo:
- Setting continuous output interval via new AT command.
- Using the proper calibration flags (eg. accelerometer/gyroscope/magnet.)
  instead always using the system calibration.
- Adding continuous output for the calibration and temperature.
- Adding output for the internal BNO055 versions (Bootloader, SW Revision ID,
  GYRO chip ID...)
- Adding support for setting up an HC-06 modul (setup, pairing, baudrate)
- Adding echo on/off? (for echoing the AT commands)

### Additional needed software:
This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
which provides a common 'type' for sensor data and some helper functions.
To use this driver you will also need to download the Adafruit_Sensor
library and include it in your libraries folder.

### Connections between Arduino and BNO055 chip:
```
Connect SCL to BNO055 SCL
Connect SDA to BNO055 SDA
Connect VDD to 5V DC
Connect GROUND to common ground
```

### Connections between Arduino and HC-06 modul for using Bluetooth Serial (Arduino Pro Micro, 5V version)
```
Connect TX to HC-06 RX
Connect RX to HC-06 TX
Connect VDD to 5V DC
Connect GROUND to common ground
```
You should setup the HC-06 modul to the right baudrate separately.

### Credits:
Based on some examples from adafruit / KTOWN, but not much left anymore :-)
Thanks to adafruit.


#### Have fun,
####   Thomas
  