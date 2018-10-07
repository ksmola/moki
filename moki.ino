#include "src/moki.h"
#include "src/BMA280.h"
#include "src/SENTRAL.h"
#include "src/QFILTER.h"
#include "src/Globals.h"

/// GPS ///
#include "src/NeoGPS/src/NMEAGPS_cfg.h"
#include "src/NeoGPS/src/ublox/ubxGPS.h"
#include "src/NeoGPS/src/GPSport.h"
#include "src/NeoGPS/src/Streamers.h"

#define SerialDebug true
#define FW_VERSION "004b"
#define BUILD_DATE "2018-07-4"

float lat, lng;
gps_fix fix;

class MyGPS : public ubloxGPS
{
public:
  enum
  {
    GETTING_STATUS,
    GETTING_LEAP_SECONDS,
    GETTING_UTC,
    RUNNING
  } state NEOGPS_BF(8);

  MyGPS(Stream *device) : ubloxGPS(device)
  {
    state = GETTING_STATUS;
  }

  //--------------------------

  void get_status()
  {
    static bool acquiring = false;

    if (fix().status == gps_fix::STATUS_NONE)
    {
      static uint32_t dotPrint;
      bool requestNavStatus = false;

      if (!acquiring)
      {
        acquiring = true;
        dotPrint = millis();
        DEBUG_PORT.print(F("Acquiring..."));
        requestNavStatus = true;
      }
      else if (millis() - dotPrint > 1000UL)
      {
        dotPrint = millis();
        DEBUG_PORT << '.';

        static uint8_t requestPeriod;
        if ((++requestPeriod & 0x07) == 0)
          requestNavStatus = true;
      }

      if (requestNavStatus)
        // Turn on the UBX status message
        enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_STATUS);
    }
    else
    {
      if (acquiring)
        DEBUG_PORT << '\n';
      DEBUG_PORT << F("Acquired status: ") << (uint8_t)fix().status << '\n';

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
    defined(UBLOX_PARSE_TIMEGPS)

      if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS))
        DEBUG_PORT.println(F("enable TIMEGPS failed!"));

      state = GETTING_LEAP_SECONDS;
#else
      start_running();
      state = RUNNING;
#endif
    }
  } // get_status

  //--------------------------

  void get_leap_seconds()
  {
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
    defined(UBLOX_PARSE_TIMEGPS)

    if (GPSTime::leap_seconds != 0)
    {
      DEBUG_PORT << F("Acquired leap seconds: ") << GPSTime::leap_seconds << '\n';

      if (!disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS))
        DEBUG_PORT.println(F("disable TIMEGPS failed!"));

#if defined(UBLOX_PARSE_TIMEUTC)
      if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC))
        DEBUG_PORT.println(F("enable TIMEUTC failed!"));
      state = GETTING_UTC;
#else
      start_running();
#endif
    }
#endif

  } // get_leap_seconds

  //--------------------------

  void get_utc()
  {
#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE) & \
    defined(UBLOX_PARSE_TIMEUTC)

    lock();
    bool safe = is_safe();
    NeoGPS::clock_t sow = GPSTime::start_of_week();
    NeoGPS::time_t utc = fix().dateTime;
    unlock();

    if (safe && (sow != 0))
    {
      DEBUG_PORT << F("Acquired UTC: ") << utc << '\n';
      DEBUG_PORT << F("Acquired Start-of-Week: ") << sow << '\n';

      start_running();
    }
#endif

  } // get_utc

  //--------------------------

  void start_running()
  {
    bool enabled_msg_with_time = false;

#if (defined(GPS_FIX_LOCATION) |     \
     defined(GPS_FIX_LOCATION_DMS) | \
     defined(GPS_FIX_ALTITUDE)) &    \
    defined(UBLOX_PARSE_POSLLH)
    if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_POSLLH))
      DEBUG_PORT.println(F("enable POSLLH failed!"));

    enabled_msg_with_time = true;
#endif

#if (defined(GPS_FIX_SPEED) | defined(GPS_FIX_HEADING)) & \
    defined(UBLOX_PARSE_VELNED)
    if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_VELNED))
      DEBUG_PORT.println(F("enable VELNED failed!"));

    enabled_msg_with_time = true;
#endif

#if defined(UBLOX_PARSE_DOP)
    if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_DOP))
      DEBUG_PORT.println(F("enable DOP failed!"));
    else
      DEBUG_PORT.println(F("enabled DOP."));

    enabled_msg_with_time = true;
#endif

#if (defined(GPS_FIX_SATELLITES) | defined(NMEAGPS_PARSE_SATELLITES)) & \
    defined(UBLOX_PARSE_SVINFO)
    if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_SVINFO))
      DEBUG_PORT.println(F("enable SVINFO failed!"));

    enabled_msg_with_time = true;
#endif

#if defined(UBLOX_PARSE_TIMEUTC)

#if defined(GPS_FIX_TIME) & defined(GPS_FIX_DATE)
    if (enabled_msg_with_time &&
        !disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC))
      DEBUG_PORT.println(F("disable TIMEUTC failed!"));

#elif defined(GPS_FIX_TIME) | defined(GPS_FIX_DATE)
    // If both aren't defined, we can't convert TOW to UTC,
    // so ask for the separate UTC message.
    if (!enable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC))
      DEBUG_PORT.println(F("enable TIMEUTC failed!"));
#endif

#endif

    state = RUNNING;
    trace_header(DEBUG_PORT);

  } // start_running

  //--------------------------

  bool running()
  {
    switch (state)
    {
    case GETTING_STATUS:
      get_status();
      break;
    case GETTING_LEAP_SECONDS:
      get_leap_seconds();
      break;
    case GETTING_UTC:
      get_utc();
      break;
    }

    return (state == RUNNING);

  } // running

} NEOGPS_PACKED;

// Construct the GPS object and hook it to the appropriate serial device
static MyGPS gps(&gpsPort);

#ifdef NMEAGPS_INTERRUPT_PROCESSING
static void GPSisr(uint8_t c)
{
  gps.handle(c);
}
#endif

//--------------------------

static void configNMEA(uint8_t rate)
{
  for (uint8_t i = NMEAGPS::NMEA_FIRST_MSG; i <= NMEAGPS::NMEA_LAST_MSG; i++)
  {
    ublox::configNMEA(gps, (NMEAGPS::nmea_msg_t)i, rate);
  }
}

//--------------------------

static void disableUBX()
{
  gps.disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEGPS);
  gps.disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_TIMEUTC);
  gps.disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_VELNED);
  gps.disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_POSLLH);
  gps.disable_msg(ublox::UBX_NAV, ublox::UBX_NAV_DOP);
}

/// end GPS ///

//Specify BMA280 configuration
uint8_t AAscale = AAFS_2G, BW = BW_125Hz, power_Mode = normal_Mode, sleep_dur = sleep_0_5ms, tapStatus, tapType;

bool newData = false;
bool newTap = false;
///end BMA setup

bool passThru = false;
String ble_msg;
typedef enum
{
  LED_ON,
  TXT
} ble_msg_options; 

moki moki;
BMA280 BMA280(intPin, intPin);
Sentral Sentral;
QFilter QFilter;

int blinkcolor = 0;
int blinkint = 0;
bool blink = false;

void setup()
{
  Dash.begin();
  Wire.begin();
  delay(5000);
  Serial.begin(38400);

  Serial0.begin(115200); // enable BLE
  Serial2.begin(9600);   //GPS port

  // hardware interrupt pin
  pinMode(imu_int, INPUT);

  // Set up the LED pin, it's set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);

  // turn on GPS
  pinMode(gps_enable, OUTPUT);
  digitalWrite(gps_enable, LOW);

  // indicate boot
  moki.RGBon(YELLOW);

  moki.CloudConnect(); //connect to Cloud

  Sentral.I2Cscan(); // should detect SENtral at 0x28 and BMA280 at 0x18.

  // Read SENtral device information
  delay(3000);
  Sentral.ReadDeviceInfo(); //reads sensor info (ROM/RAM/PRODUCT ID) and prints it

  // Read BMA280 Device Register
  byte BMA_ID = BMA280.getChipID();
  delay(1000); // give some time to read the screen

  // Check which sensors can be detected by the EM7180
  Sentral.FindEM7180Sensors();
  delay(1000); // give some time to read the screen

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  Sentral.CheckStatus();
  delay(1000); // give some time to read the screen

  // Set up the SENtral as sensor bus in normal operating mode
  Sentral.SetMode(passThru, SerialDebug);

  byte c = BMA280.readByte(BMA280_ADDRESS, BMA280_BGW_CHIPID); // Read CHIP_ID register for BMA280
  if (c == 0xFB)
  {
    Serial.println("BMA280 is online");

    aRes = BMA280.getAres(AAscale);
    BMA280.selfTestBMA280();
    BMA280.resetBMA280();
    delay(1000);
    BMA280.initBMA280(AAscale, BW, power_Mode, sleep_dur);
  }
  moki.RGBoff();
  // moki.SendMessage("moki v002!", "slack_message");
}

void loop()
{
  Sentral.Update(passThru); //gets new data from Sentral
  // keep track of rates
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
 
  QFilter.MadgwickQuaternionUpdate(-ay, -ax, az, gy * PI / 180.0f, gx * PI / 180.0f, -gz * PI / 180.0f, mx, my, mz);

  Sentral.PrintSerial(SerialDebug);
  Sentral.CheckMotionDetection();
  Sentral.CalcAngles();
  if (SerialDebug)
  {
    BMA280.readBMA280AccelData(aaccelCount);
    bma_ax = (float)aaccelCount[0] * aRes / 4.0f; // get actual g value, this depends on scale being set
    bma_ay = (float)aaccelCount[1] * aRes / 4.0f;
    bma_az = (float)accelCount[2] * aRes / 4.0f;
    // BMA280.MotionDetection(threshold_x, threshold_y, threshold_z, interval); //needs tuning
    BMA280.MotionDetection();

    if (Serial0.available() > 0)
    { //check for incoming BLE messages
      ble_msg = Serial0.readStringUntil('\n');

      if (ble_msg == "v")
      {
        Serial0.print("Firmware version: ");
        Serial0.println(FW_VERSION);
        Serial0.print("Build date: ");
        Serial0.println(BUILD_DATE);
      }
      else if (ble_msg == "slack")
      {
        Serial0.println("Enter message to send to Slack");
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        HologramCloud.sendMessage(ble_msg, "slack_message");
        Serial0.println("Message sent!");
        moki.RGBoff();
      }
      else if (ble_msg == "smshadi")
      {
        Serial0.println("Enter message to send via SMS");
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        HologramCloud.sendMessage(ble_msg, "sms_hadi");
        Serial0.println("Message sent!\n");
        moki.RGBoff();
      }
      else if (ble_msg == "smskon")
      {
        Serial0.println("Enter message to send via SMS");
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        HologramCloud.sendMessage(ble_msg, "sms_kon");
        Serial0.println("Message sent!");
        moki.RGBoff();
      }
      else if (ble_msg == "smsstefan")
      {
        Serial0.println("Enter message to send via SMS");
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        HologramCloud.sendMessage(ble_msg, "sms_stefan");
        Serial0.println("Message sent!");
        moki.RGBoff();
      }
      else if (ble_msg == "battery")
      {
        Serial0.println("Current battery info:");
        Serial0.print("battery level: ");
        Serial0.print(Charger.batteryMillivolts());
        Serial0.print("mV, ");
        Serial0.print(Charger.batteryPercentage());
        Serial0.println("%");
        Serial0.println(moki.ChargerStatus());
        if (Charger.isEnabled())
        {
          Serial0.println("Charging circuit is enabled!");
        }
        else
          Serial0.println("Charging circuit not enabled!");
      }
      else if (ble_msg == "iccid")
      {
        Serial0.println(moki.GetICCID());
      }
      else if (ble_msg == "conn")
      {
        Serial0.println(moki.ConnectionStatus());
      }
      else if (ble_msg == "signal")
      {
        Serial0.println(moki.GetSignalStrength());
      }
      else if (ble_msg == "accel")
      {
        Serial0.println("Accelerometer printout: (X, Y, Z)\n");
        delay(150);
        bma_printout = 1;
        blinkcolor = FOREST_GREEN;
        blinkint = 350;
        moki.RGBblink(blinkcolor, blinkint);
      }
      else if (ble_msg == "stop")
      {
        bma_printout = 0;
        moki.RGBoff();
      }
      else if (ble_msg == "th")
      {
        moki.RGBon(ROYAL_BLUE);
        Serial0.print("current: ");
        Serial0.print(threshold_x);
        Serial0.print(", ");
        Serial0.print(threshold_y);
        Serial0.print(", ");
        Serial0.println(threshold_z);
        Serial0.println("Enter new global threshold:");

        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        if (ble_msg.toFloat())
        {
          threshold_x = ble_msg.toFloat();
          threshold_y = ble_msg.toFloat();
          threshold_z = ble_msg.toFloat();
          Serial0.print("global threshold set to: ");
          Serial0.print(threshold_x);
          Serial0.println(" !\n");
        }
        else
        {
          Serial0.println("no valid input!");
        }

        moki.RGBoff();
      }
      else if (ble_msg == "calib")
      {
        moki.RGBon(ROYAL_BLUE);
        Serial0.println("starting calibration - keep moki steady!");
        BMA280.Calibrate();
        calibrated = true;
        Serial0.println("Done!");
        moki.RGBoff();
      }
      else if (ble_msg == "arm")
      {
        armed = true;
        Serial0.println("ARMED!");
      }
      else if (ble_msg == "unlock")
      {
        armed = false;
        HologramCloud.attachTag("MOKI_UNLOCK");
        moki.SendMessage("moki unlocked!", "slack_message");
        Serial0.println("UNLOCKED!");
      }
      else if (ble_msg == "percent")
      {
        moki.RGBon(ROYAL_BLUE);
        Serial0.println("Enter threshold scale in percent (0.90 = 90%");

        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        if (ble_msg.toFloat())
        {
          threshold_x *= ble_msg.toFloat();
          threshold_y *= ble_msg.toFloat();
          threshold_z *= ble_msg.toFloat();
          Serial0.print(threshold_x, 6);
          Serial0.print(", ");
          Serial0.print(threshold_y, 6);
          Serial0.print(", ");
          Serial0.println(threshold_z, 6);
          Serial0.println("Done!");
          moki.RGBoff();
        }
      }
      else if (ble_msg == "x")
      {
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        if (ble_msg.toFloat())
        {
          threshold_x = ble_msg.toFloat();
          Serial0.print("Set x to ");
          Serial0.println(threshold_x, 5);
          moki.RGBoff();
        }
      }
      else if (ble_msg == "y")
      {
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        if (ble_msg.toFloat())
        {
          threshold_x = ble_msg.toFloat();
          Serial0.print("Set y to ");
          Serial0.println(threshold_y, 5);
          moki.RGBoff();
        }
      }
      else if (ble_msg == "z")
      {
        while (Serial0.available() == 0)
        { // wait for incoming message
          blinkcolor = WHITE_SMOKE;
          blinkint = 150;
          moki.RGBblink(blinkcolor, blinkint);
        }
        ble_msg = Serial0.readStringUntil('\n');
        if (ble_msg.toFloat())
        {
          threshold_z = ble_msg.toFloat();
          Serial0.print("Set z to ");
          Serial0.println(threshold_z, 5);
          moki.RGBoff();
        }
      }
      else if (ble_msg == "blink")
      {
        rgbblink = true;
        blinkcolor = YELLOW;
        blinkint = 100;
      }
    }

    if (rgbblink == true)
      moki.RGBblink(blinkcolor, blinkint);

    if (bma_printout == 1)
    {
      Serial0.print((int)1000 * bma_ax);
      Serial0.print(" , ");
      Serial0.print((int)1000 * bma_ay);
      Serial0.print(" , ");
      Serial0.println((int)1000 * bma_az);
      delay(100);
    }

    Dash.setLED(bma_motion);

    if (armed && bma_motion)
    {
      HologramCloud.attachTag("MOKI_ALARM");
      moki.SendMessage("motion detected!", "slack_message");
      armed = false;
    }

    // tapType = BMA280.getTapType();
    // if (tapType & 0x20)
    //   Serial.println("Single tap detected!");

    uint8_t interrupt_status = BMA280.readByte(BMA280_ADDRESS, BMA280_INT_STATUS_0);

    uint8_t low_int = (((interrupt_status >> 0) & 0x01));
    uint8_t high_int = (((interrupt_status >> 1) & 0x01));
    uint8_t slope_int = (((interrupt_status >> 2) & 0x01));
    uint8_t slo_not_mot_int = (((interrupt_status >> 3) & 0x01));

    uint8_t d_tap_int = (((interrupt_status >> 4) & 0x01));
    uint8_t s_tap_int = (((interrupt_status >> 5) & 0x01));
    uint8_t orient_int = (((interrupt_status >> 6) & 0x01));
    uint8_t flat_int = (((interrupt_status >> 7) & 0x01));
  }
  
  digitalWrite(myLed, !digitalRead(myLed));
  count = millis();
  sumCount = 0;
  sum = 0;
}
