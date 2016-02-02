/*
   =============================================================================
   (c) by Thomas Welsch / 2016 under the GNU Lesser General Public License
   =============================================================================

   This driver implements a simple "AT" style interface for the BNO055 sensor
   chip (BOSCH).
   The BNO055 is connected via the I2C interface, see wiring below.

   Tested  with Arduino Pro Micro (virtual Serial interface on USB or real
   serial interface on pins 0/1) and Arduino Nano.

   =============================================================================
   Current commands:
   =============================================================================
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

   =============================================================================
   Example:
   =============================================================================
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
    
   =============================================================================
   ToDo:
   =============================================================================
   - Setting continuous output interval via new AT command.
   - Using the proper calibration flags (eg. accelerometer/gyroscope/magnet.)
     instead always using the system calibration.
   - Adding continuous output for the calibration and temperature.
   - Adding output for the internal BNO055 versions (Bootloader, SW Revision
     ID, GYRO chip ID...)
   - Adding support for setting up an HC-06 modul (setup, pairing, baudrate)
   - Adding echo on/off? (for echoing the AT commands)

   =============================================================================
   Additional needed software:
   =============================================================================
   This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   =============================================================================
   Connections between Arduino and BNO055 chip:
   =============================================================================
   Connect SCL to BNO055 SCL
   Connect SDA to BNO055 SDA
   Connect VDD to 5V DC
   Connect GROUND to common ground

   =============================================================================
   Connections between Arduino and HC-06 modul for using Bluetooth Serial:
   (Arduino Pro Micro, 5V version)
   =============================================================================
   Connect TX to HC-06 RX
   Connect RX to HC-06 TX
   Connect VDD to 5V DC
   Connect GROUND to common ground

   You should setup the HC-06 modul to the right baudrate separately.

   =============================================================================
   Credits:
   =============================================================================
   Based on some examples from adafruit / KTOWN, but not much left anymore :-)
   Thanks to adafruit.

*/



#include "Stream.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



//==============================================================================
// BMo055 sensor from Adafruit library
//==============================================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//==============================================================================
// Streams we use for communication. The user communication is done via the
// in/out Stream. The Arduino Pro Micro has a real serial port on pins 0/1. In
// this default configuration I'm using an HC-06 modul attached on that serial
// port. If you are using an Arduino Nano or don't use this modul use the
// default "Serial" port (which "ends" on the USB port) instead "Serial1".
// See also the setup() function for initialization of the boudrate!
//==============================================================================
Stream& out     = Serial1;
Stream& in      = Serial1;

// Just for infos or debugging
// Stream& infoOut = Serial;

//==============================================================================

// Buffer for collecting request;
String cmdBuffer                           = String("");

// Current status for reply:
String currentStatus                       = String("OK");

// Set if we have a working BNO055 connetion:
boolean gotBno                             = false;

// MS between enabled continuous outputs. Should be settable via AT command...
int continuousOutputDelay                  = 500;

// Flags for continuous outputs, set by ATx1 or AT1, cleared by ATx0 or AT0
boolean displayEulerValues                 = false;
boolean displayAccelerometerValues         = false;
boolean displayGyroscopeValues             = false;
boolean displayMagnetometerValues          = false;
boolean displayGravityValues               = false;
boolean displayLinearAccelerometerValues   = false;

// Last output time for the continuous output in ms:
unsigned long lastOutputMs;

//==============================================================================
// Arduino setup function (automatically called at startup)
//==============================================================================
void setup(void)
{
  // Hardware Seriel:
  Serial1.begin(2400);
  Serial1.println(F("Orientation_Bluetooth_Server Ver. 0.1")); Serial1.println("");

  // USB:
  // Serial.begin(9600);
  // Serial.println(F("Orientation_Bluetooth_Server Ver. 0.1")); Serial.println("");

  lastOutputMs = millis();
}

//==============================================================================
// Arduino loop function, called once 'setup' is complete in an endless loop
//==============================================================================
void loop(void)
{

  initBno();    // Try to init if not done...

  //----------------------------------------------------------------------------
  // Checking for incomming AT commands. We are collecting the characters
  // in cmdBuffer and start parsing after an we got a \n.
  //----------------------------------------------------------------------------
  if (in.available()) {

    char c = toLowerCase( (char) in.read() );

    if (c != '\n' && c != '\r') {
      if (cmdBuffer.length() < 10) cmdBuffer = cmdBuffer +  c;
    }

    if (c == '\n') {
      parseCommand(out);
      cmdBuffer = "";
    }
  }

  //----------------------------------------------------------------------------
  // Output enabled continuous outputs.
  //----------------------------------------------------------------------------
  unsigned long now = millis();
  unsigned long msSinceLastOutput;

  // Handle 50 days rollover:
  if (now > lastOutputMs)
    msSinceLastOutput = now - lastOutputMs;
  else
    msSinceLastOutput = (0 - lastOutputMs) + now;

  if (msSinceLastOutput >= continuousOutputDelay) {
    lastOutputMs = now;

    if (displayEulerValues)               displayEuler(out);
    if (displayAccelerometerValues)       displayAccelerometer(out);
    if (displayGyroscopeValues)           displayGyroscope(out);
    if (displayMagnetometerValues)        displayMagnetometer(out);
    if (displayGravityValues)             displayGravity(out);
    if (displayLinearAccelerometerValues) displayLinearAccelerometer(out);
  }

}

//==============================================================================
// Try to setup the BNO055, if not done yet.
//==============================================================================
void initBno()
{
  if (gotBno) return;

  // Initialise the sensor

  if (!bno.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    currentStatus = F("ERROR: NO BNO055 detected");
    return;
  }

  bno.setExtCrystalUse(true);

  gotBno = true;
}

//==============================================================================
// Get the free memory.
//==============================================================================
int freeRam ()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//==============================================================================
// Parse global cmdBuffer for commands. Part I/II: handle currentStatus
//==============================================================================
void parseCommand(Stream &out)
{

  if (!gotBno) {
    out.println(currentStatus);
    return;
  }

  currentStatus = "OK";

  byte handled = parseCommandInternal(out);

  if (handled) {
    out.println(currentStatus);
  }
  else {
    out.println("ERROR: Unknown command |" + cmdBuffer + "|");
  }

}

//==============================================================================
// Parse global cmdBuffer for commands. Part II/II: Command lookup.
//==============================================================================
byte parseCommandInternal(Stream &out)
{

  //----------------------------------------------------------------------------

  if (cmdBuffer == "at") {
    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "ath") {
    out.println(F("HELP:"));
    out.println(F("  AT     : OK"));
    out.println(F("  ATH    : Display help"));
    out.println(F("  ATI    : Display sensor details"));
    out.println(F("  ATS    : Display sensor status"));
    out.println(F("  ATC    : Display sensor status (calibration)"));
    out.println(F("  ATT    : Display sensor temperatur in C degree"));
    out.println(F("  ATO    : Display sensor orientation in Euler degree"));
    out.println(F("  ATO1   :   Enable continuous orientation output"));
    out.println(F("  ATO0   :   Disable O1"));
    out.println(F("  ATA    : Display sensor accelerometer in m/s^2"));
    out.println(F("  ATA1   :   Enable continuous accelerometer output"));
    out.println(F("  ATA0   :   Disable A1"));
    out.println(F("  ATY    : Display sensor gyroscope in rad/s"));
    out.println(F("  ATY1   :   Enable continuous gyroscope output"));
    out.println(F("  ATY0   :   Disable Y1"));
    out.println(F("  ATM    : Display sensor magnetometer in uT"));
    out.println(F("  ATM1   :   Enable continuous magnetometer output"));
    out.println(F("  ATM0   :   Disable G1"));
    out.println(F("  ATG    : Display sensor gravity in m/s^2"));
    out.println(F("  ATG1   :   Enable continuous gravity output"));
    out.println(F("  ATG0   :   Disable G1"));
    out.println(F("  ATL    : Display sensor linear accelerometer in m/s^2"));
    out.println(F("  ATL1   :   Enable continuous linear accelerometer output"));
    out.println(F("  ATL0   :   Disable L1"));

    out.println(F("  AT1    : Enable continuous linear accelerometer output of all sensor data"));
    out.println(F("  AT0    : Disable AT1"));

    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "ati") {
    displaySensorDetails(out);
    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "ats") {
    displaySensorStatus(out);
    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "atc") {
    displayCalStatus(out);
    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "att") {
    displayTemperature(out);
    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "at0") {
    displayEulerValues                 = false;
    displayAccelerometerValues         = false;
    displayGyroscopeValues             = false;
    displayMagnetometerValues          = false;
    displayGravityValues               = false;
    displayLinearAccelerometerValues   = false;
    return 1;
  }

  //----------------------------------------------------------------------------

  if (cmdBuffer == "at1") {
    displayEulerValues                 = true;
    displayAccelerometerValues         = true;
    displayGyroscopeValues             = true;
    displayMagnetometerValues          = true;
    displayGravityValues               = true;
    displayLinearAccelerometerValues   = true;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Euler orientation
  //----------------------------------------------------------------------------
  if (cmdBuffer == "ato") {
    displayEuler(out);
    return 1;
  }

  if (cmdBuffer == "ato0") {
    displayEulerValues = false;
    return 1;
  }

  if (cmdBuffer == "ato1") {
    displayEulerValues = true;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Accelerometer
  //----------------------------------------------------------------------------
  if (cmdBuffer == "ata") {
    displayAccelerometer(out);
    return 1;
  }

  if (cmdBuffer == "ata0") {
    displayAccelerometerValues = false;
    return 1;
  }

  if (cmdBuffer == "ata1") {
    displayAccelerometerValues = true;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Gyroscope
  //----------------------------------------------------------------------------
  if (cmdBuffer == "aty") {
    displayGyroscope(out);
    return 1;
  }

  if (cmdBuffer == "aty0") {
    displayGyroscopeValues = false;
    return 1;
  }

  if (cmdBuffer == "aty1") {
    displayGyroscopeValues = true;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Magnetometer
  //----------------------------------------------------------------------------
  if (cmdBuffer == "atm") {
    displayMagnetometer(out);
    return 1;
  }

  if (cmdBuffer == "atm0") {
    displayMagnetometerValues = false;
    return 1;
  }

  if (cmdBuffer == "atm1") {
    displayMagnetometerValues = true;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Gravity
  //----------------------------------------------------------------------------
  if (cmdBuffer == "atg") {
    displayGravity(out);
    return 1;
  }

  if (cmdBuffer == "atg0") {
    displayGravityValues = false;
    return 1;
  }

  if (cmdBuffer == "atg1") {
    displayGravityValues = true;
    return 1;
  }

  //----------------------------------------------------------------------------
  // Linear Accelerometer
  //----------------------------------------------------------------------------
  if (cmdBuffer == "atl") {
    displayLinearAccelerometer(out);
    return 1;
  }

  if (cmdBuffer == "atl0") {
    displayLinearAccelerometerValues = false;
    return 1;
  }

  if (cmdBuffer == "atl1") {
    displayLinearAccelerometerValues = true;
    return 1;
  }

  return 0;

}

//==============================================================================
// Displays some basic information on this sensor from the unified
// sensor API sensor_t type (see Adafruit_Sensor for more information).
//==============================================================================
void displaySensorDetails(Stream &out)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  out.print  ("Sensor:        "); out.println(sensor.name);
  out.print  ("Driver Ver:    "); out.println(sensor.version);
  out.print  ("Unique ID:     "); out.println(sensor.sensor_id);
  out.println("");
}

//==============================================================================
// Display some basic info about the sensor status.
// You can find more informations about the status codes in Adafruit_BNO055.cpp
// or the BNO055 data sheet.
//==============================================================================
void displaySensorStatus(Stream &out)
{
  // Get the system status values (mostly for debugging purposes)
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Display the results in the Serial Monitor
  out.println("");
  out.print("System Status: 0x");
  out.println(system_status, HEX);
  out.print("Self Test:     0x");
  out.println(self_test_results, HEX);
  out.print("System Error:  0x");
  out.println(system_error, HEX);
  out.println("");
}

//==============================================================================
// Display sensor calibration status
//==============================================================================
void displayCalStatus(Stream &out)
{
  //----------------------------------------------------------------------------
  // Get the four calibration values (0..3)
  // - Any sensor data reporting 0 should be ignored,
  // - 3 means 'fully calibrated"
  //----------------------------------------------------------------------------
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // The data should be ignored until the system calibration is > 0
  if (!system) {
    out.println("No calibration yet");
    out.println("");
    //  return;
  }

  // Display the individual values
  out.print("System ");
  out.print(system, DEC);
  out.print(" Gyroscope ");
  out.print(gyro, DEC);
  out.print(" Accelerometer ");
  out.print(accel, DEC);
  out.print(" Magnetometer ");
  out.println(mag, DEC);
  out.println("");
}

//==============================================================================
// Check if we have a valid system calibration. We are only using the "system"
// calibration.
//==============================================================================
boolean hasSystemCalibration()
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  
  bno.getCalibration(&system, &gyro, &accel, &mag);

  return system != 0;
}

//==============================================================================
// Display the chip temperature.
//==============================================================================
void displayTemperature(Stream &out)
{
  uint8_t temp = bno.getTemp();

  out.print("Temperatur ");
  out.print(temp, DEC);
  out.println(" C");
  out.println("");
}


//==============================================================================
// Common output function for different sensors, see display*() functions:
//==============================================================================
void displayVector(Stream   &out,
                   String   what,
                   String   info,
                   boolean  displayRequest,
                   Adafruit_BNO055::adafruit_vector_type_t vector_type)
{
    
  // We are only using the system calibration flag, see TODO list.
  if (!hasSystemCalibration()) {
    out.println(what + ": No calibration!");
  }

  // Possible vector values can be:
  // + VECTOR_ACCELEROMETER - m/s^2
  // + VECTOR_MAGNETOMETER  - uT
  // + VECTOR_GYROSCOPE     - rad/s
  // + VECTOR_EULER         - degrees
  // + VECTOR_LINEARACCEL   - m/s^2
  // + VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> xyz = bno.getVector(vector_type);

  // Display the floating point data
  out.print(info + ": ");
  out.print(xyz.x(), 4);
  out.print(" ");
  out.print(xyz.y(), 4);
  out.print(" ");
  out.println(xyz.z(), 4);

  if (!displayRequest) out.println("");

}

//==============================================================================
void displayEuler(Stream &out)
{
  displayVector(out, "Orientation", "Ori", displayEulerValues, Adafruit_BNO055::VECTOR_EULER);
}
//==============================================================================
void displayAccelerometer(Stream &out)
{
  displayVector(out, "Accelerometer", "Accel", displayAccelerometerValues, Adafruit_BNO055::VECTOR_ACCELEROMETER);
}
//==============================================================================
void displayGyroscope(Stream &out)
{
  displayVector(out, "Gyroscope", "Gyro", displayGyroscopeValues, Adafruit_BNO055::VECTOR_GYROSCOPE);
}
//==============================================================================
void displayMagnetometer(Stream &out)
{
  displayVector(out, "Magnetometer", "Mag", displayMagnetometerValues, Adafruit_BNO055::VECTOR_MAGNETOMETER);
}
//==============================================================================
void displayLinearAccelerometer(Stream &out)
{
  displayVector(out, "Linear Accelerometer", "LinearAccel", displayLinearAccelerometerValues, Adafruit_BNO055::VECTOR_LINEARACCEL);
}
//==============================================================================
void displayGravity(Stream &out)
{
  displayVector(out, "Gravity", "Grav", displayGravityValues, Adafruit_BNO055::VECTOR_GRAVITY);
}
//==============================================================================




