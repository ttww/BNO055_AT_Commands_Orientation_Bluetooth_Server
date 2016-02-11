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
    ATH      : Display help
    ATI      : Display sensor details
    ATS      : Display sensor status
    ATC      : Display sensor status (calibration)
    ATT      : Display sensor temperatur in C degree
    ATO      : Display sensor orientation in Euler degree
    ATO1     :   Enable continuous Euler degree output
    ATO0     :   Disable E1
    ATA      : Display sensor accelerometer in m/s^2
    ATA1     :   Enable continuous accelerometer output
    ATA0     :   Disable A1
    ATY      : Display sensor gyroscope in rad/s
    ATY1     :   Enable continuous gyroscope output
    ATY0     :   Disable Y1
    ATM      : Display sensor magnetometer in uT
    ATM1     :   Enable continuous magnetometer output
    ATM0     :   Disable G1
    ATG      : Display sensor gravity in m/s^2
    ATG1     :   Enable continuous gravity output
    ATG0     :   Disable G1
    ATL      : Display sensor linear accelerometer in m/s^2
    ATL1     :   Enable continuous linear accelerometer output
    ATL0     :   Disable L1
    AT1      : Enable continuous linear accelerometer output of all sensor data
    AT0      : Disable AT1
    ATB=xxx  : Setting the baud rate. xxx can be 1200,2400,4800,9600,19200,38400,57600,115200
               You need to reconnect. If we have a HC-06, the bluetooth speed is also changed.
               This change is permanent.
    ATF      : Shows the free RAM in bytes.


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
   - Adding support for setting up an HC-06 modul:
        PIN:       todo
        NAME:      todo
        BAUDRATE:  DONE
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
   Connect A6 (=#4) HC-06 WAKEUP

   The HC-06 baudrate can be set via the ATB=xxx command.
   The HC-06 PIN can be set via the ATP=xxx command.            (TODO !)
   The HC-06 bluetooth name can be set via the ATN=xxx command. (TODO !)

   =============================================================================
   Credits:
   =============================================================================
   Based on some examples from adafruit / KTOWN, but not much left anymore :-)
   Thanks to adafruit.

*/


#include "Stream.h"

#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static const boolean USE_HC06 = true;

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
// See also the setup() function for initialization of the baudrate!
//==============================================================================
#define RXTX_PORT  Serial1

// Just for infos or debugging
//#define DEBUG_PORT Serial

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

const int MAX_COMMAND_LEN                  = 10;    // without MAX_BLUETOOTH_NAME_LEN
const int MAX_BLUETOOTH_NAME_LEN           = 20;

const int HC_06_WAKEUP_PIN                 = 4;


const int  CONFIG_EEPROM_STATUS_ADDR        = 0;
const long CONFIG_EEPROM_MAGIC              = 0x74770116L + 3;     // tw16

struct configtatusType {
  long  magic;
  long  baudrate;
  char  blueToothName[MAX_BLUETOOTH_NAME_LEN];
} configStatus;

/* Known HC-06 baudrates:
    AT+BAUD1———1200
    AT+BAUD2———2400
    AT+BAUD3———4800
    AT+BAUD4———9600 (Default)
    AT+BAUD5———19200
    AT+BAUD6———38400
    AT+BAUD7———57600
    AT+BAUD8———115200
    AT+BAUD9———230400
    AT+BAUDA———460800
    AT+BAUDB———921600
    AT+BAUDC———1382400
*/
const long BAUD_RATES[] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };

//==============================================================================
// Arduino setup function (automatically called at startup)
//==============================================================================
void setup(void)
{

  // USB:
#ifdef DEBUG_PORT
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT) {
    ; // wait for serial port to connect. Needed for native USB
  }
  DEBUG_PORT.println(F("Orientation_Bluetooth_Server Ver. 0.2")); DEBUG_PORT.println("");
#endif

  readConfigFromEEPROM();

  // Hardware serial:
  RXTX_PORT.begin(configStatus.baudrate != 0 ? configStatus.baudrate : 9600);


  // HC-06
  if (USE_HC06) {
    initHC06();
  }

  RXTX_PORT.println(F("Orientation_Bluetooth_Server Ver. 0.2")); RXTX_PORT.println("");

  //  inHC06CommandMode = true;
  //  RXTX_PORT.print("AT");
  //  RXTX_PORT.print("AT+VERSION");

  lastOutputMs = millis();
}

void initHC06()
{
  pinMode(HC_06_WAKEUP_PIN, OUTPUT);
  digitalWrite(HC_06_WAKEUP_PIN, 0);


#ifdef DEBUG_PORT
  DEBUG_PORT.println(F("HC-06 init..."));
  DEBUG_PORT.print(F("  Baudrate       = "));
  DEBUG_PORT.println(configStatus.baudrate);
  DEBUG_PORT.print(F("  Bluetooth Name = "));
  DEBUG_PORT.println(configStatus.blueToothName);
#endif

  if (configStatus.baudrate == 0) {
#ifdef DEBUG_PORT
    DEBUG_PORT.println(F("--> need HC-06 baudrate scan."));
#endif

    // Setting the HC-06 into the AT mode:
    disconnectHC06();

    char buf[5];
    for (int i = 0; i < sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]); i++) {
      long baudrate = BAUD_RATES[i];

#ifdef DEBUG_PORT
      DEBUG_PORT.print(F("  check "));
      DEBUG_PORT.print(baudrate);
#endif

      RXTX_PORT.begin(baudrate);

      delay(10);
      RXTX_PORT.print("AT");        // --> should give "OK"
      RXTX_PORT.flush();

      buf[0] = readCharFromHC06(900);   // the HC-06 needs a little time for starting the answer....
      buf[1] = readCharFromHC06(100);
      buf[2] = '\0';

#ifdef DEBUG_PORT
      DEBUG_PORT.print(F("  answer = |"));
      DEBUG_PORT.print(buf);
      DEBUG_PORT.println("|");
#endif

      if (buf[0] == 'O' && buf[1] == 'K') {
#ifdef DEBUG_PORT
        DEBUG_PORT.println(F("  FOUND !"));
#endif

        configStatus.baudrate = baudrate;
        RXTX_PORT.begin(configStatus.baudrate);
        writeConfigToEEPROM();        // Remember the baudrat
        break;
      }
    }   // for (baudrate scan

    if (configStatus.baudrate == 0) {
#ifdef DEBUG_PORT
      DEBUG_PORT.println(F("Boudrate not found, using 9600"));
#endif

      // Something is wrong, using the default baudrate 9600. Possible errors are:
      // - WAKEUP line not connected
      // - Not a HC-06 module on hardware serial port (wrong USE_HC06 on compile time?)
      configStatus.baudrate = 9600;
      RXTX_PORT.begin(configStatus.baudrate);
      // we don't write to the EEPROM. This will lead to a rescan next
      // time (give a chance for conncting the module...)
    }
  }
  else {
    RXTX_PORT.begin(configStatus.baudrate);

#ifdef DEBUG_PORT
    DEBUG_PORT.print(F("Using baudrate from EEPROM = "));
    DEBUG_PORT.println(configStatus.baudrate);
#endif
  }


}

// Read char from seriel with timeout. Stream.setTimeout() is not working...
char readCharFromHC06(int timeOutMs)
{
  while (timeOutMs > 0 && !RXTX_PORT.available()) {
    timeOutMs--;
    delay(1);
  }
  if (timeOutMs == 0) return '?';

  return RXTX_PORT.read();
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
  if (RXTX_PORT.available()) {

    //    char c = toLowerCase( (char) in.read() );
    char c = (char) RXTX_PORT.read() ;

    if (c != '\n' && c != '\r') {
      if (cmdBuffer.length() < (MAX_COMMAND_LEN + MAX_BLUETOOTH_NAME_LEN)) cmdBuffer = cmdBuffer +  c;
    }

    if (c == '\n') {
      parseCommand(RXTX_PORT);
      cmdBuffer = "";
    }
  }

  // Testing: Read USB serial and send it to the HC-06:
  //if (DEBUG_PORT.available()) {
  //  char c = DEBUG_PORT.read();
  //  DEBUG_PORT.print(c);
  //  RXTX_PORT.print(c);
  //}


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

    if (displayEulerValues)               displayEuler(RXTX_PORT);
    if (displayAccelerometerValues)       displayAccelerometer(RXTX_PORT);
    if (displayGyroscopeValues)           displayGyroscope(RXTX_PORT);
    if (displayMagnetometerValues)        displayMagnetometer(RXTX_PORT);
    if (displayGravityValues)             displayGravity(RXTX_PORT);
    if (displayLinearAccelerometerValues) displayLinearAccelerometer(RXTX_PORT);
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
int freeRam()
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
    if (currentStatus != "OK") {
      out.println("ERROR: Bad command. " + currentStatus);
    }
    else {
      out.println("ERROR: Unknown command |" + cmdBuffer + "|");
    }
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
    out.println(F("  AT       : OK"));
    out.println(F("  ATH      : Display help"));
    out.println(F("  ATI      : Display sensor details"));
    out.println(F("  ATS      : Display sensor status"));
    out.println(F("  ATC      : Display sensor status (calibration)"));
    out.println(F("  ATT      : Display sensor temperatur in C degree"));
    out.println(F("  ATO      : Display sensor orientation in Euler degree"));
    out.println(F("  ATO1     :   Enable continuous orientation output"));
    out.println(F("  ATO0     :   Disable O1"));
    out.println(F("  ATA      : Display sensor accelerometer in m/s^2"));
    out.println(F("  ATA1     :   Enable continuous accelerometer output"));
    out.println(F("  ATA0     :   Disable A1"));
    out.println(F("  ATY      : Display sensor gyroscope in rad/s"));
    out.println(F("  ATY1     :   Enable continuous gyroscope output"));
    out.println(F("  ATY0     :   Disable Y1"));
    out.println(F("  ATM      : Display sensor magnetometer in uT"));
    out.println(F("  ATM1     :   Enable continuous magnetometer output"));
    out.println(F("  ATM0     :   Disable G1"));
    out.println(F("  ATG      : Display sensor gravity in m/s^2"));
    out.println(F("  ATG1     :   Enable continuous gravity output"));
    out.println(F("  ATG0     :   Disable G1"));
    out.println(F("  ATL      : Display sensor linear accelerometer in m/s^2"));
    out.println(F("  ATL1     :   Enable continuous linear accelerometer output"));
    out.println(F("  ATL0     :   Disable L1"));

    out.println(F("  AT1      : Enable continuous linear accelerometer output of all sensor data"));
    out.println(F("  AT0      : Disable AT1"));


    out.println(F("  ATB=xxx  : Setting the baud rate. xxx can be 1200,2400,4800,9600,19200,38400,57600,115200"));
    out.println(F("             You need to reconnect. If we have a HC-06, the bluetooth speed is also changed."));
    out.println(F("             This change is permanent."));

    out.println(F("  ATF      : Shows the free RAM in bytes."));

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

  //----------------------------------------------------------------------------
  // Connection parameters (HC-06, baudrate...)
  //----------------------------------------------------------------------------
  if (cmdBuffer.startsWith("atb=")) {
    String n = cmdBuffer.substring(4);
    long baudrate = (long) n.toFloat();     // we need float, baudrate > INT2...

#ifdef DEBUG_PORT
    DEBUG_PORT.print("Baud = ");
    DEBUG_PORT.println(baudrate);
#endif

    int idxFound = -1;
    for (int i = 0; i < sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]); i++) {
      if (baudrate == BAUD_RATES[i]) {
        idxFound = i;
        break;
      }
    } // for

    if (idxFound == -1) {
      currentStatus = "Unknown baudrate";
      return 0;
    }

    if (USE_HC06) {
      disconnectHC06();
      delay(500);
      // Sending AT+BAUDx command to the HC-06. x is our index inside BAUD_RATES + 1
      RXTX_PORT.print("AT+BAUD");
      RXTX_PORT.print(idxFound + 1);
      RXTX_PORT.flush();

      // Read the answer in the old baudrate.
      while (true) {
        char c = readCharFromHC06(900);   // the HC-06 needs a little time for starting the answer....
        if (c == '?') break;

#ifdef DEBUG_PORT
        DEBUG_PORT.print(c);
#endif
      }
#ifdef DEBUG_PORT
      DEBUG_PORT.println("---Done");
#endif
    }
    configStatus.baudrate = baudrate;
    writeConfigToEEPROM();
    RXTX_PORT.begin(baudrate);

    return 1;
  }

  //----------------------------------------------------------------------------
  // Show the free memory
  //----------------------------------------------------------------------------
  if (cmdBuffer == "atf") {
    RXTX_PORT.print("FREE=");
    RXTX_PORT.println(freeRam());
    return 1;
  }

  return 0;

}

//==============================================================================
// Setting the HC-06 into the AT mode:
//==============================================================================
void disconnectHC06() {
  digitalWrite(HC_06_WAKEUP_PIN, 1);
  delay(100);
  digitalWrite(HC_06_WAKEUP_PIN, 0);
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
// Write the the current configuration from configStatus.
//==============================================================================
void writeConfigToEEPROM() {
  configStatus.magic    = CONFIG_EEPROM_MAGIC;
  EEPROM.put(CONFIG_EEPROM_STATUS_ADDR, configStatus);
}

//==============================================================================
// Read the current configuration into configStatus.
// We fill configStatus with zeros if the EEPROM contains the wrong MAGIC
// number.
//==============================================================================
void readConfigFromEEPROM() {
  EEPROM.get(CONFIG_EEPROM_STATUS_ADDR, configStatus);

  // Setup status for the first time
  if (configStatus.magic != CONFIG_EEPROM_MAGIC) {
    configStatus.baudrate = 0;
    memset(configStatus.blueToothName, 0, sizeof(configStatus.blueToothName));
  }
}


