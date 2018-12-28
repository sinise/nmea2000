
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "RTIMULib.h"
#include <PString.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

RTIMU *imu;              // the IMU object
RTIMUSettings *settings; // the settings object

float magd = -5.0;
boolean outputhdm = true; // magnetic heading
boolean outputhdt = false; // true heading
boolean outputrot = true; // rate of turn
boolean outputshr = true; // pitch and roll
//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL 300 // interval between pose displays

#define SERIAL_PORT_SPEED 115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;
float r = M_PI / 180.0f; // degrees to radians
float d = 180.0f / M_PI; // radians to degrees

void setup()
{
  int errcode;

  NMEA2000.SetN2kCANSendFrameBufSize(100);
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Message sender teensy heading",  // Manufacturer's Model ID
                                 "1.0.0.1 (2017-01-01)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2017-01-01)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(190, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );
  // Uncomment 3 rows below to see, what device will send to bus
   Serial.begin(115200);
//   NMEA2000.SetForwardStream(&Serial);  // PC output on due programming port
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  // NMEA2000.SetForwardOwnMessages();

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  //NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)

  NMEA2000.Open();
  Serial.begin(SERIAL_PORT_SPEED);
  while (!Serial)
  {
    ; // wait for serial port to connect.
  }
  Wire.begin();
  settings = new RTIMUSettings();
  imu = RTIMU::createIMU(settings); // create the imu object

  Serial.print("TeensyIMU10 starting using device ");
  Serial.print(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0)
  {
    Serial.print("Failed to init IMU: ");
    Serial.println(errcode);
  }
  if (imu->getCompassCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  imu->setSlerpPower(0.02);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);

  lastDisplay = lastRate = millis();
  sampleCount = 0;
}
byte checksum(char *str)
{
  byte cs = 0;
  for (unsigned int n = 1; n < strlen(str) - 1; n++)
  {
    cs ^= str[n];
  }
  return cs;
}
void loop()
{
  unsigned long now = millis();
  unsigned long delta;
  tN2kMsg N2kMsg;
  RTIMU_DATA imuData;
  NMEA2000.ParseMessages();
  if (imu->IMURead())
  { // get the latest data if ready yet      Serial.println();
    imuData = imu->getIMUData();
    sampleCount++;
    if ((delta = now - lastRate) >= 1000)
    {
      if (!imu->getCompassCalibrationValid())
      {
        if (imu->getRuntimeCompassCalibrationValid())
          Serial.print(", runtime mag cal valid");
        else
          Serial.print(", runtime mag cal not valid");
      }
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL)
    {
      lastDisplay = now;
      RTVector3 pose = imuData.fusionPose;
      RTVector3 gyro = imuData.gyro;
      float roll = pose.y() * -1 * d; // negative is left roll
      float pitch = pose.x() * d;     // negative is nose down
      float yaw = pose.z() * d;       // negative is to the left of 270 magnetic
      float rot = gyro.z() * d;       // negative is to left
      float hdm = yaw - 90;           // 0 yaw = 270 magnetic; converts to mag degrees
      if (yaw < 90 && yaw >= -179.99)
      {
        hdm = yaw + 270;
      }
      float hdt = hdm - magd; // calculate true heading
      if (hdt > 360)
      {
        hdt = hdt - 360;
      }
      if (hdt < 0.0)
      {
        hdt = hdt + 360;
      }
      if (outputhdm == true)
      {
        char hdmSentence[23];
        byte csm;
        PString strm(hdmSentence, sizeof(hdmSentence));
        strm.print("$HCHDM,");
        strm.print(hdm); // lround simply rounds out the decimal, since a single degree is fine enough of a resolution
        SetN2kMagneticHeading(N2kMsg, 0, DegToRad(hdm), DegToRad(-3.0), DegToRad(5.5));
        NMEA2000.SendMsg(N2kMsg);

        strm.print(",M*");
        csm = checksum(hdmSentence);
        if (csm < 0x10)
          strm.print('0');
        strm.print(csm, HEX);
        Serial.println(hdmSentence);
      }
      if (outputhdt == true)
      {
        char hdtSentence[23];
        byte cst;
        PString strt(hdtSentence, sizeof(hdtSentence));
        strt.print("$HCHDT,");
        strt.print(hdt);
        strt.print(",T*");
        cst = checksum(hdtSentence);
        if (cst < 0x10)
          strt.print('0');
        strt.print(cst, HEX);
        Serial.println(hdtSentence);
      }

      if (outputrot == true)
      {
        // ROT Rate of Turn //
        char rotSentence[18];
        byte csr;
        PString strr(rotSentence, sizeof(rotSentence));
        strr.print("$TIROT,");
        strr.print(rot * 60); // multiply by 60, since ROT is measured in degrees per minute
        strr.print(",A*");
        csr = checksum(rotSentence);
        if (csr < 0x10)
          strr.print('0');
        strr.print(csr, HEX);
        Serial.println(rotSentence);
      }
      if (outputshr == true){
        // SHR Pitch and Roll (no heave... yet...)//
        char shrSentence [50];
        byte csp;
        PString strp(shrSentence, sizeof(shrSentence));
        strp.print("$INSHR,,");
        strp.print(hdt);
        strp.print(",T,");
        strp.print(roll);
        strp.print(",");
        strp.print(pitch);
        strp.print(",,,,,1,0*");
        csp = checksum(shrSentence);
        if (csp < 0x10) strp.print('0');
        strp.print(csp, HEX);
        Serial.println(shrSentence);
      }
    }
  }
}
