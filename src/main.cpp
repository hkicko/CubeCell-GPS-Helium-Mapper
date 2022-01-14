/* Based on the CubeCell GPS example from libraries\LoRa\examples\LoRaWAN\LoRaWAN_Sensors\LoRaWan_OnBoardGPS_Air530\LoRaWan_OnBoardGPS_Air530.ino
 * and on Jas Williams version from https://github.com/jas-williams/CubeCell-Helium-Mapper.git  
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530.h" // Enable this for board version 1.0 and 1.0_1
//#include "GPS_Air530Z.h" // Enable this for board version 1.1
#include "HT_SSD1306Wire.h"

//#define DEBUG // Enable/Disable debug output over the serial console

extern SSD1306Wire            display;    // Defined in LoRaWan_APP.cpp
extern uint8_t                isDispayOn; // Defined in LoRaWan_APP.cpp
#ifdef GPS_Air530_H
Air530Class                   GPS;
#endif
#ifdef GPS_Air530Z_H
Air530ZClass                  GPS;
#endif

// Commend out/uncomment this line to disable/enable the auto sleep/wake up by vibration sensor feature
//#define VIBR_SENSOR           GPIO5     // Change the pin where the sensor is connected if different
// Comment out/uncomment this line to disable/enable the functionality  where the vibration sensor wakes the device from "deep" sleep (VIBR_SENSOR must be enabled)
//#define VIBR_WAKE_FROM_SLEEP
// If put to sleeep from the menu, this will disable the wake up by vibration and only allow it to work when auto sleep was activated in some way (like stopped for too long)
//#define MENU_SLEEP_DISABLE_VIBR_WAKEUP
// Enable this to activate the auto sleep on no vibration function for the cases when the device is left stationary indoors and GPS generates fake movement so it can't go to sleep 
//#define VIBR_AUTOSLEEP_TIMEOUT 300000

#define GPS_READ_RATE         1000      // How often to read GPS (in ms)
#define MIN_DIST              25        // Minimum distance in meters from the last sent location before we can send again. A hex is about 340m, divide by this value to get the pings per hex.
#define MAX_GPS_WAIT          660000    // Max time to wait for GPS before going to sleep (in ms)
#define AUTO_SLEEP_TIMER      300000    // If no movement for this amount of time (in ms), the device will go to sleep. Comment out if you don't want this feature. 
#define MENU_IDLE_TIMEOUT     30000     // Auto exit the menu if no button pressed in this amount of ms
#define VBAT_CORRECTION       1.004     // Edit this for calibrating your battery voltage
//#define CAYENNELPP_FORMAT   

/*
   set LoraWan_RGB to Active,the RGB active in loraWan
   RGB red means sending;
   RGB purple means joined done;
   RGB blue means RxWindow1;
   RGB yellow means RxWindow2;
   RGB green means received done;
*/

/* OTAA para*/

uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t devAddr =  ( uint32_t )0x00000000;

#if defined( REGION_EU868 )
/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6] = { 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
#else
uint16_t userChannelsMask[6] = { 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };
#endif

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
/* Start with non-zero value, for the first transmission with previously stored JOIN,
 * but it will be changed later depending on the mode */
uint32_t appTxDutyCycle = 1000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

#define APP_PORT_DEFAULT 2
#define APP_PORT_LASTLOC 3

/* Application port */
uint8_t appPort = APP_PORT_DEFAULT;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:
  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)
  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

const uint8_t helium_logo_bmp[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0xff, 0xff, 0x03, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0x01, 0x7c, 0x80, 0x03, 0x00,
   0x00, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0x71,
   0x7c, 0x80, 0x03, 0x00, 0x00, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xf0, 0xff, 0x03, 0xf8, 0xf8, 0x80, 0x03, 0x00, 0x00, 0xe0, 0xe0, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x00, 0xf8, 0xf8, 0x80, 0x03, 0x00,
   0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x7f, 0x00, 0xf8,
   0xf8, 0x81, 0x03, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xf8, 0x3f, 0x00, 0x78, 0xfc, 0x81, 0x03, 0x00, 0x00, 0xe0, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xfc, 0x1f, 0xf8, 0x07, 0xfc, 0x83, 0xf3, 0x01,
   0xfe, 0xe0, 0xe0, 0x38, 0x70, 0x98, 0x1f, 0x3f, 0xfc, 0x1f, 0x9c, 0x07,
   0xfe, 0x83, 0xff, 0x03, 0xff, 0xe1, 0xe0, 0x38, 0x70, 0x98, 0xbf, 0x7f,
   0xfc, 0x1f, 0x06, 0x86, 0xff, 0x83, 0x0f, 0x87, 0x83, 0xe1, 0xe0, 0x38,
   0x70, 0x78, 0xf8, 0x70, 0xfc, 0x1f, 0x02, 0x84, 0xff, 0x83, 0x07, 0x87,
   0x01, 0xe3, 0xe0, 0x38, 0x70, 0x38, 0x70, 0x60, 0xfc, 0x0f, 0x03, 0x84,
   0xff, 0x83, 0x03, 0xc7, 0x01, 0xe3, 0xe0, 0x38, 0x70, 0x38, 0x70, 0x60,
   0xfc, 0x0f, 0x03, 0x84, 0xff, 0x83, 0x03, 0xc7, 0x01, 0xe7, 0xe0, 0x38,
   0x70, 0x38, 0x70, 0x60, 0xfc, 0x0f, 0x02, 0x84, 0xff, 0x83, 0x03, 0xc7,
   0xff, 0xe7, 0xe0, 0x38, 0x70, 0x38, 0x70, 0x60, 0xfc, 0x1f, 0x06, 0x86,
   0xff, 0x83, 0x03, 0xc7, 0xff, 0xe7, 0xe0, 0x38, 0x70, 0x38, 0x70, 0x60,
   0xfc, 0x07, 0x0e, 0x83, 0xff, 0x83, 0x03, 0xc7, 0x01, 0xe0, 0xe0, 0x38,
   0x70, 0x38, 0x70, 0x60, 0xfc, 0x03, 0xfe, 0x81, 0xff, 0x83, 0x03, 0xc7,
   0x01, 0xe0, 0xe0, 0x38, 0x70, 0x38, 0x70, 0x60, 0xf8, 0xe3, 0x61, 0xc0,
   0xff, 0x81, 0x03, 0x87, 0x01, 0xe3, 0xe0, 0x38, 0x70, 0x38, 0x70, 0x60,
   0xf8, 0xf1, 0x01, 0xe0, 0xff, 0x81, 0x03, 0x87, 0x83, 0xe3, 0xe0, 0x70,
   0x78, 0x38, 0x70, 0x60, 0xf8, 0xf1, 0x01, 0xf0, 0xff, 0x81, 0x03, 0x07,
   0xff, 0xc1, 0xe3, 0xf0, 0x6f, 0x38, 0x70, 0x60, 0xf0, 0xf1, 0x01, 0xf8,
   0xff, 0x80, 0x03, 0x07, 0x7e, 0x80, 0xe3, 0xe0, 0x67, 0x38, 0x70, 0x60,
   0xf0, 0xe1, 0x08, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0xf8, 0xff, 0x7f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x07, 0xfc, 0xff,
   0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x0f, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff,
   0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xfe, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

bool      sleepMode               = false;
bool      loopingInSend           = false;
bool      menuMode                = false;
bool      screenOffMode           = false; // Enable normal operation with the screen off - for more battery saving
uint32_t  lastScreenPrint         = 0;
uint32_t  joinStart               = 0;
uint32_t  gpsSearchStart          = 0;
uint32_t  lastSend                = 0;
int       currentMenu             = 0;
bool      displayBatPct           = false; // Change here if you want to see the battery as percent vs voltage (not recommended because it is inacurate unless you go edit some min and max voltage values in the base libraries with values specific to your battery)
bool      sleepActivatedFromMenu  = false;
bool      gpsTimerSet             = false;
#ifdef VIBR_AUTOSLEEP_TIMEOUT
uint32_t  lastVibrEvent           = 0;
#endif

bool      trackerMode             = false;
bool      sendLastLoc             = false;
bool      lastLocSet              = false;
uint32_t  last_lat                = 0;
uint32_t  last_lon                = 0;
double    last_send_lat           = 0;
double    last_send_lon           = 0;
uint32_t  min_dist_moved          = MIN_DIST;
uint32_t  dist_moved              = UINT32_MAX;
bool      nonstopMode             = false;
bool      gps_debug               = false;

#define MENU_CNT 8

char* menu[MENU_CNT] = {"Screen OFF", "Sleep", "Send now", "Faster Upd", "Slower Upd", "Tracker mode", "Nonstop mode", "Debug Info"}; //"Reset GPS", "Bat V/%"

enum eMenuEntries
{
  SCREEN_OFF,
  SLEEP,
  SEND_NOW,
  FASTER_UPD,
  SLOWER_UPD,
  TRACKER_MODE,
  NONSTOP_MODE,
  DEBUG_INFO
  //RESET_GPS,
  //BAT_V_PCT
};

void userKey();
void setVibrAutoWakeUp();

// Timer to schedule wake ups for GPS read before going to sleep 
static TimerEvent_t GPSCycleTimer;
// Timer to auto close the menu after certain period of inactivity
static TimerEvent_t menuIdleTimeout;

int32_t fracPart(double val, int n)
{
  return (int32_t)abs(((val - (int32_t)(val)) * pow(10, n)));
}

// RGB LED power on
void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

// RGB LED power off
void VextOFF(void) 
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void displayGPSInfo()
{
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  int index = sprintf(str, "%02d-%02d-%02d", GPS.date.year(), GPS.date.month(), GPS.date.day());
  str[index] = 0;
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, str);
  
  index = sprintf(str, "%02d:%02d:%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second());
  str[index] = 0;
  display.drawString(60, 0, str);

  if (GPS.location.age() < 1000)
  {
    display.drawString(120, 0, "A");
  }
  else
  {
    display.drawString(120, 0, "V");
  }

  if (GPS.speed.kmph() > 1.2)
  {
    display.drawString(107, 0, "M");
  }
  else
  {
    display.drawString(107, 0, "S");
  }
  
  index = sprintf(str, "alt: %d.%d", (int)GPS.altitude.meters(), fracPart(GPS.altitude.meters(), 2));
  str[index] = 0;
  display.drawString(0, 16, str);
   
  index = sprintf(str, "hdop: %d.%d", (int)GPS.hdop.hdop(), fracPart(GPS.hdop.hdop(), 2));
  str[index] = 0;
  display.drawString(0, 32, str); 
 
  index = sprintf(str, "lat :  %d.%d", (int)GPS.location.lat(), fracPart(GPS.location.lat(), 4));
  str[index] = 0;
  display.drawString(60, 16, str);   
  
  index = sprintf(str, "lon: %d.%d", (int)GPS.location.lng(), fracPart(GPS.location.lng(), 4));
  str[index] = 0;
  display.drawString(60, 32, str);

  index = sprintf(str, "speed: %d.%d km/h", (int)GPS.speed.kmph(), fracPart(GPS.speed.kmph(), 2));
  str[index] = 0;
  display.drawString(0, 48, str);

  index = sprintf(str, "sats: %d", (int)GPS.satellites.value());
  str[index] = 0;
  display.drawString(88, 48, str);
  display.display();
}

#ifdef DEBUG
void printGPSInfo()
{
  Serial.print("Date/Time: ");
  if (GPS.date.isValid())
  {
    Serial.printf("%d/%02d/%02d", GPS.date.year(), GPS.date.day(), GPS.date.month());
  }
  else
  {
    Serial.print("INVALID");
  }

  if (GPS.time.isValid())
  {
    Serial.printf(" %02d:%02d:%02d.%02d", GPS.time.hour(), GPS.time.minute(), GPS.time.second(), GPS.time.centisecond());
  }
  else
  {
    Serial.print(" INVALID");
  }
  Serial.println();
  
  Serial.print("LAT: ");
  Serial.print(GPS.location.lat(), 6);
  Serial.print(", LON: ");
  Serial.print(GPS.location.lng(), 6);
  Serial.print(", ALT: ");
  Serial.print(GPS.altitude.meters());

  Serial.println(); 
  
  Serial.print("SATS: ");
  Serial.print(GPS.satellites.value());
  Serial.print(", HDOP: ");
  Serial.print(GPS.hdop.hdop());
  Serial.print(", AGE: ");
  Serial.print(GPS.location.age());
  Serial.print(", COURSE: ");
  Serial.print(GPS.course.deg());
  Serial.print(", SPEED: ");
  Serial.println(GPS.speed.kmph());
  Serial.println();
}
#endif

// Call this from other display methods (assumes the display is initialized and awake)
void displayBatteryLevel()
{
  uint16_t batteryVoltage;
  uint8_t batteryLevel;
  float_t batteryLevelPct;
  char str[30];  
  int index;
  
  detachInterrupt(USER_KEY); // reading battery voltage is messing up with the pin and driving it down, which simulates a long press for our interrupt handler 

  if (displayBatPct)
  {    
    //get Battery Level 1-254 Returned by BoardGetBatteryLevel
    /*                                0: USB,
    *                                 1: Min level,
    *                                 x: level
    *                               254: fully charged,
    *                               255: Error
    */
    batteryLevel = BoardGetBatteryLevel();
    batteryLevelPct = ((float_t)batteryLevel - BAT_LEVEL_EMPTY) * 100 / (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);
    switch (batteryLevel)
    {
      case 0:
        index = sprintf(str, "%s", "USB"); 
        break;
      case BAT_LEVEL_EMPTY: 
        index = sprintf(str, "%s", "LOW"); 
        break;
      case 255:
        index = sprintf(str, "%s", "ERR"); 
        break;
      default:
        index = sprintf(str, "%3u%%", (uint8_t)batteryLevelPct);        
        break;
    }  
  }
  else
  {
    batteryVoltage = getBatteryVoltage();
    float_t batV = ((float_t)batteryVoltage * VBAT_CORRECTION)/1000;  // Multiply by the appropriate value for your own device to adjust the measured value after calibration
    index = sprintf(str, "%d.%02dV", (int)batV, fracPart(batV, 2));       
    
    // #ifdef DEBUG
    // Serial.println();
    // Serial.print("Bat V: ");
    // Serial.print(batteryVoltage); 
    // Serial.print(" (");
    // Serial.print(batV);
    // Serial.println(")");
    // #endif 
  }
  str[index] = 0;

  attachInterrupt(USER_KEY, userKey, FALLING);  // Attach again after voltage reading is done

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, str);
}

void displayLogoAndMsg(String msg, uint32_t wait_ms)
{
  display.clear();
  display.drawXbm(0, 0, 128, 42, helium_logo_bmp);
  displayBatteryLevel();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 54-16/2, msg);
  #ifdef DEBUG
  Serial.println(msg);
  #endif
  display.display();

  if (wait_ms)
  {
    delay(wait_ms);
  }    
}

int8_t loraDataRate()
{
  MibRequestConfirm_t mibReq;
  LoRaMacStatus_t status;
  int8_t ret = -1;
  
  mibReq.Type = MIB_CHANNELS_DATARATE;
  status = LoRaMacMibGetRequestConfirm( &mibReq );
  if (status == LORAMAC_STATUS_OK)
  {
    ret = mibReq.Param.ChannelsDatarate;
  }

  return ret;
}

void displayJoinTimer()
{
  char str[30];
  int index; 

  if ((millis() - lastScreenPrint) > 500)
  {
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.clear();
    display.drawString(58, 22, "JOINING");    
    if (millis() > joinStart)
    {
      index = sprintf(str,"%ds", (millis() - joinStart) / 1000);
      str[index] = 0; 
      display.drawString(64, 48, str);
    }    
    display.display();
    lastScreenPrint = millis();    
  }
}

void displayGPSInfoEverySecond()
{
  if (((millis() - lastScreenPrint) > 500) && GPS.time.isValid() && GPS.time.isUpdated())
  {            
    #ifdef DEBUG
    printGPSInfo();
    if (screenOffMode)
    {
      delay(15);
    }
    #endif
    if (!screenOffMode)
    {
      if (!isDispayOn)
      {
        display.wakeup();
        isDispayOn = 1;
      }    
      displayGPSInfo();
    }
    lastScreenPrint = millis();
  }
}

void displayMenu()
{
  int prev; 
  int next; 
  String currentOption = menu[currentMenu]; 
  currentOption.toUpperCase();

  prev = currentMenu - 1;

  if (prev < 0)
  {
    prev = MENU_CNT - 1;
  }

  next = currentMenu + 1;

  if (next >= MENU_CNT)
  {
    next = 0;
  }

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  
  display.drawString(64, 0, menu[prev]);   
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, (display.getHeight() - 16) / 2, currentOption); 
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, display.getHeight() - 10, menu[next]); 
  displayBatteryLevel();
  display.display();
}

void displayDebugInfo()
{
  char str[30];
  int index; 

  display.clear();  
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  index = sprintf(str,"%s: %um", "Update every", min_dist_moved);
  str[index] = 0;
  display.drawString(0, 0, str);
  index = sprintf(str,"%s: %um", "Current distance", dist_moved);
  str[index] = 0;
  display.drawString(0, 10, str);
  index = sprintf(str,"%s: %u", "loopingInSend", loopingInSend);
  str[index] = 0;
  display.drawString(0, 20, str);
  index = sprintf(str,"%s: %u DR: %i", "LoRaJoined", IsLoRaMacNetworkJoined, loraDataRate());
  str[index] = 0;
  display.drawString(0, 30, str);
  index = sprintf(str,"%s: %i", "Tracker mode", trackerMode);
  str[index] = 0;
  display.drawString(0, 40, str);
  index = sprintf(str,"%s: %i", "Nonstop mode", nonstopMode);
  str[index] = 0;
  display.drawString(0, 50, str);
  display.display();

  delay(4000);    
}

void displayGPSWaitWithCounter()
{  
  char str[30];
  int index;

  if (((millis() - lastScreenPrint) > 500) && (millis() - gpsSearchStart > 1000))
  {
    if (!isDispayOn)
    {
      display.wakeup(); 
      isDispayOn = 1;
    }
    display.clear();
    display.drawXbm(0, 0, 128, 42, helium_logo_bmp);
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);  
    display.drawString(0, 54-16/2, "GPS fix wait");           
    index = sprintf(str,"%d", (millis() - gpsSearchStart) / 1000);
    str[index] = 0; 
    display.setTextAlignment(TEXT_ALIGN_RIGHT);  
    display.drawString(128, 54-16/2, str);
    display.display();
    lastScreenPrint = millis();
  }  
}

void startGPS()
{
  GPS.begin(115200); // If you are sure that you have selected the right include directive for your GPS chip, you can use GPS.begin(115200) here. 
  // Air530Z code has setmode(MODE_GPS_BEIDOU_GLONASS) call in begin(), but for Air530 we will need to set it ourselves
  #ifdef GPS_Air530_H
  GPS.setmode(MODE_GPS_GLONASS); //Enable dual mode - GLONASS and GPS   
  #endif
  GPS.setNMEA(NMEA_RMC | NMEA_GGA); // decrease the amount of unnecessary of data the GPS sends, NMEA_RMC has most of what we need, except altitude, NMEA_GGA has altitude but does not have date and speed
  gpsSearchStart = millis();
}

void cycleGPS()
{
  uint32_t cycleGPStimer;
  #define maxBuff 1000
  char gpsBuff[maxBuff];
  int gpsBuffPtr = 0;

  // read the location to clear the updated flag
  GPS.location.rawLat();

  cycleGPStimer = millis();  

  while (millis() - cycleGPStimer < GPS_READ_RATE)
  {
    while (GPS.available() > 0)
    {
      gpsBuff[gpsBuffPtr] = GPS.read();
      GPS.encode(gpsBuff[gpsBuffPtr]);
      gpsBuffPtr++;
      if (gpsBuffPtr >= maxBuff)
      {
        gpsBuffPtr = sprintf(gpsBuff,"++OF++"); // indicator for overflow
      }
      gpsBuff[gpsBuffPtr] = 0;
    }

    if (GPS.location.isUpdated())
    {
      break;
    }
    else
    {
      if (loopingInSend && !screenOffMode)
      {
        displayGPSWaitWithCounter();
      }
    }
  }

  if (GPS.location.age() < GPS_READ_RATE)
  {
    dist_moved = GPS.distanceBetween(last_send_lat, last_send_lon, GPS.location.lat(), GPS.location.lng());

    if (trackerMode) // store the last lat/long
    {
      last_lat    = ((GPS.location.lat() + 90) / 180.0) * 16777215;
      last_lon    = ((GPS.location.lng() + 180) / 360.0) * 16777215;
      lastLocSet  = true;
    }
  }

  if (gps_debug)
  {
    Serial.println(gpsBuff);
  }
}

void stopGPS()
{
  GPS.end();
}

void switchModeToSleep()
{
  sleepMode = true;
  if (!screenOffMode)
  {
    if (!isDispayOn)
    {
      display.wakeup();
      isDispayOn = 1;
    } 
    displayLogoAndMsg("Sleeping...", 4000);
    display.sleep();
    isDispayOn = 0;
  }
  #ifdef DEBUG
  else
  {
    Serial.println("Going to sleep...");
  }
  #endif
  stopGPS();
  Radio.Sleep(); // Not sure this is needed. It is called by LoRaAPP.cpp in various places after TX done or timeout. Most probably in 99% of the cases it will be already called when we get here. 
  sendLastLoc = trackerMode; // After wake up, if tracker mode enabled - send the last known location before waiting for GPS
  #ifdef VIBR_SENSOR
  setVibrAutoWakeUp();
  #endif
  deviceState = DEVICE_STATE_SLEEP;
}

void switchModeOutOfSleep()
{
  sleepMode = false;
  sleepActivatedFromMenu = false;
  if (!screenOffMode)
  {
    if (!isDispayOn)
    {
      display.wakeup();
      isDispayOn = 1;
    }
    displayLogoAndMsg("Waking Up...", 4000);
    display.clear();
    display.display();
  }
  #ifdef DEBUG
  else
  {
    Serial.println("Waking Up...");
  }
  #endif
  startGPS();
  if (sendLastLoc) // If we are in tracker mode and we have to send last known location on wake up - go to SEND immediately 
  {
    deviceState = DEVICE_STATE_SEND;
  }
  else
  {
    deviceState = DEVICE_STATE_SLEEP;
  }
  loopingInSend = false;
  #ifdef VIBR_AUTOSLEEP_TIMEOUT
  lastVibrEvent = millis(); // reset variable to prevent auto sleep immediately after wake up
  #endif
  #ifdef AUTO_SLEEP_TIMER
  lastSend = millis(); // reset variable to prevent auto sleep immediately after wake up
  #endif
}

void switchScrenOffMode()
{
  screenOffMode = true;  
  //displayLogoAndMsg("Scren off....", 2000);          
  VextOFF();
  display.stop();
  isDispayOn = 0;   
}

void switchScreenOnMode()
{
  screenOffMode = false;  
  VextON();
  display.init();
  isDispayOn = 1;
  displayLogoAndMsg("Screen on...", 1000);  
  display.clear();
  display.display();
}

void autoSleepIfNoGPS()
{
  #ifdef MAX_GPS_WAIT
  if (!nonstopMode && (millis() - gpsSearchStart > MAX_GPS_WAIT))
  {
    switchModeToSleep();
  }
  #endif
}

static void OnGPSCycleTimerEvent()
{
  TimerStop(&GPSCycleTimer);

  if (!loopingInSend)
  {
    cycleGPS();  

    // if we moved more than min_dist_moved, then send
    if (dist_moved >= min_dist_moved)
    {
      deviceState = DEVICE_STATE_SEND;
    }
  }
  gpsTimerSet = false;
}

#ifdef CAYENNELPP_FORMAT
bool prepareTxFrame(uint8_t port)
{  
  int32_t lat = GPS.location.lat() * 10000;
  int32_t lon = GPS.location.lng() * 10000;
  int32_t alt = GPS.altitude.meters() * 100;

  appDataSize = 0;
  appData[appDataSize++] = 0x01;
  appData[appDataSize++] = 0x88;
  appData[appDataSize++] = lat >> 16;
  appData[appDataSize++] = lat >> 8;      
  appData[appDataSize++] = lat;
  appData[appDataSize++] = lon >> 16;
  appData[appDataSize++] = lon >> 8;
  appData[appDataSize++] = lon;
  appData[appDataSize++] = alt >> 16;
  appData[appDataSize++] = alt >> 8;
  appData[appDataSize++] = alt;

  return true;
}
#else
bool prepareTxFrame(uint8_t port)
{
  /*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
    appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
    if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
    if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
    for example, if use REGION_CN470,
    the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
  */

  uint32_t  lat, lon;
  int       alt, course, speed, hdop, sats;
  
  unsigned char *puc;
  bool      ret = false;
  
  appDataSize = 0;

  switch (port)
  {
    case APP_PORT_DEFAULT:
      
      if (GPS.location.isValid())
      {
        last_send_lat = GPS.location.lat(); // store the value for distance calculation
        last_send_lon = GPS.location.lng(); // store the value for distance calculation

        lat     = ((last_send_lat + 90) / 180.0) * 16777215;
        lon     = ((last_send_lon + 180) / 360.0) * 16777215;

        alt     = (uint16_t)GPS.altitude.meters();
        course  = GPS.course.deg();
        speed   = (uint16_t)GPS.speed.kmph();
        sats    = GPS.satellites.value();
        hdop    = GPS.hdop.hdop();

        detachInterrupt(USER_KEY); // reading battery voltage is messing up with the pin and driving it down, which simulates a long press for our interrupt handler 
        uint16_t batteryVoltage = ((float_t)((float_t)((float_t)getBatteryVoltage() * VBAT_CORRECTION)  / 10) + .5);  

        puc = (unsigned char *)(&lat);
        appData[appDataSize++] = puc[2];
        appData[appDataSize++] = puc[1];
        appData[appDataSize++] = puc[0];

        puc = (unsigned char *)(&lon);
        appData[appDataSize++] = puc[2];
        appData[appDataSize++] = puc[1];
        appData[appDataSize++] = puc[0];

        puc = (unsigned char *)(&alt);
        appData[appDataSize++] = puc[1];
        appData[appDataSize++] = puc[0];

        puc = (unsigned char *)(&speed);
        appData[appDataSize++] = puc[0];
        
        appData[appDataSize++] = (uint8_t)((batteryVoltage-200) & 0xFF);

        appData[appDataSize++] = (uint8_t)(sats & 0xFF);

        #ifdef DEBUG
        Serial.print("Speed ");
        Serial.print(speed);
        Serial.println(" kph");

        //get Battery Level 1-254 Returned by BoardGetBatteryLevel
        uint8_t batteryLevel = BoardGetBatteryLevel();
        //Convert to %
        batteryLevel = (uint8_t)((float_t)batteryLevel - BAT_LEVEL_EMPTY) * 100 / (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);

        Serial.print("Battery Level ");
        Serial.print(batteryLevel);
        Serial.println(" %");
        
        Serial.print("BatteryVoltage: ");
        Serial.println(batteryVoltage);

        Serial.print("SleepMode = ");
        Serial.println(sleepMode);
        Serial.println();  
        #endif
        attachInterrupt(USER_KEY, userKey, FALLING);  // Attach again after voltage reading is done 

        ret = true;
      }
      break;

    case APP_PORT_LASTLOC:

      puc = (unsigned char *)(&last_lat);
      appData[appDataSize++] = puc[2];
      appData[appDataSize++] = puc[1];
      appData[appDataSize++] = puc[0];

      puc = (unsigned char *)(&last_lon);
      appData[appDataSize++] = puc[2];
      appData[appDataSize++] = puc[1];
      appData[appDataSize++] = puc[0];

      ret = true;
      break;      
  }

  return ret;
}
#endif

#ifdef VIBR_SENSOR
void vibration(void)
{
  detachInterrupt(VIBR_SENSOR);
  #ifdef VIBR_AUTOSLEEP_TIMEOUT
  lastVibrEvent = millis();
  #endif
  
  // #ifdef DEBUG
  // Serial.println("Vibration detected");
  // switch (deviceState)
  // {
  //   case DEVICE_STATE_SLEEP:
  //     Serial.println("Curent State = SLEEP");
  //     break;
  //   case DEVICE_STATE_SEND:
  //     Serial.println("Curent State = SEND");
  //     break;
  //   case DEVICE_STATE_CYCLE:
  //     Serial.println("Current State = CYCLE");
  //     break;    
  // }
  // #endif
  
  if (!menuMode) // Ignore vibrations while in the menu
  {
    if (sleepMode)
    {
      #ifdef VIBR_WAKE_FROM_SLEEP     
      #ifdef MENU_SLEEP_DISABLE_VIBR_WAKEUP // If menu sleep overwrites the "vibration wake up from sleeep", then add the IF statement to not wake up when sleep was initiated from the menu
      if (!sleepActivatedFromMenu)
      #endif  
      switchModeOutOfSleep();
      #endif
    }    
  }
}

void setVibrAutoWakeUp()
{
  bool setupVibr = false; // Default operation is - don't wait for vibration

  // Except if we have VIBR_WAKE_FROM_SLEEP enabled, in which case we need to attach in sleep mode
  #ifdef VIBR_WAKE_FROM_SLEEP 
    // But if MENU_SLEEP_DISABLE_VIBR_WAKEUP enabled, only if sleep was not activated from the menu
    #ifdef MENU_SLEEP_DISABLE_VIBR_WAKEUP
    if (!sleepActivatedFromMenu)
    #endif
      setupVibr = setupVibr | sleepMode;  
  #endif
  
  if (setupVibr)
  {
    attachInterrupt(VIBR_SENSOR, vibration, FALLING);
  }  
}

void autoSleepIfNoVibr()
{
  #ifdef VIBR_AUTOSLEEP_TIMEOUT          
  // if too long since last vibration, force it to go to sleep
  if (!nonstopMode && (millis() - lastVibrEvent > VIBR_AUTOSLEEP_TIMEOUT))
  {
    switchModeToSleep();    
  }          
  #endif  
}
#endif

void autoSleepIfNoMovement()
{
  #ifdef AUTO_SLEEP_TIMER
  if (!nonstopMode && (millis() - lastSend > AUTO_SLEEP_TIMER))
  {
    switchModeToSleep();
  }
  #endif
}

static void OnMenuIdleTimeout()
{
  TimerStop(&menuIdleTimeout);

  if (menuMode)
  {
    menuMode = false;
    if (!screenOffMode)
    {
      display.clear();
      display.display();
    }
  }
}

void executeMenu(void)
{
  TimerStop(&menuIdleTimeout);
  
  switch (currentMenu)
  {
    case SCREEN_OFF:
      switchScrenOffMode();
      menuMode = false;      
      break;

    case SLEEP:
      sleepActivatedFromMenu = true;
      switchModeToSleep();
      menuMode = false;
      break;

    case SEND_NOW:
      deviceState = DEVICE_STATE_SEND;
      menuMode = false;
      break;

    case FASTER_UPD:
      if (min_dist_moved > 10)
      {
        min_dist_moved -= 10;
      }
      menuMode = false;
      break;

    case SLOWER_UPD:
      if (min_dist_moved < 500)
      {
        min_dist_moved += 10;
      }
      menuMode = false;
      break;

    case TRACKER_MODE:
      trackerMode = !trackerMode;
      menuMode = false;
      break;

    case NONSTOP_MODE:
      nonstopMode = !nonstopMode;
      menuMode = false;
      break;

    case DEBUG_INFO:
      displayDebugInfo();
      menuMode = false;
      break;

    // case RESET_GPS:
    //   stopGPS();
    //   delay(1000);
    //   startGPS();
    //   deviceState = DEVICE_STATE_CYCLE;  
    //   menuMode = false;
    //   break;

    // case BAT_V_PCT:
    //   displayBatPct = !displayBatPct;
    //   menuMode = false;
    //   break;

    default:
      menuMode = false;
      break;
  }
  if (!screenOffMode)
  {
    display.clear();
    display.display();
  }
}

void userKey(void)
{
  delay(10);
  if (digitalRead(USER_KEY) == LOW)
  {
    uint16_t keyDownTime = 0;
    while (digitalRead(USER_KEY) == LOW)
    {
      delay(1);
      keyDownTime++;
      if (keyDownTime >= 1000)
        break;
    }

    if (keyDownTime < 700)
    {
      if (sleepMode)
      {        
        if (screenOffMode)
        {
          screenOffMode = false;  
          VextON();
          display.init();
          isDispayOn = 1;
        }
        switchModeOutOfSleep();
      }
      else if (screenOffMode)
      {
        switchScreenOnMode();
      }
      else
      {
        if (menuMode)
        {
          currentMenu++;
          if (currentMenu >= MENU_CNT)
          {
            currentMenu = 0;
          }
        }
        else
        {
          menuMode = true;
          currentMenu = 0;
          deviceState = DEVICE_STATE_SLEEP;
        }        
        TimerSetValue(&menuIdleTimeout, MENU_IDLE_TIMEOUT);
        TimerStart(&menuIdleTimeout);
      }
    }
    else
    {
      if (menuMode)
      {
        executeMenu();
      }
    }
  }
}

// Use AT+GPSDBG=1 to enable GPS debug output
bool checkUserAt(char * cmd, char * content)
{
  if (strcmp(cmd, "GPSDBG")==0)
  {
    gps_debug = (atoi(content) == 1);
    Serial.printf("+GPSDBG=%d", gps_debug);
    Serial.println();
    return true;
  }
  return false;
}

void setup() 
{
  boardInitMcu();

  Serial.begin(115200);  

  #if(AT_SUPPORT)
  enableAt();
  #endif

  // Display branding image. If we don't want that - the following 2 lines can be removed  
  display.init(); // displayMcuInit() will init the display, but if we want to show our logo before that, we need to init ourselves.   
  isDispayOn = 1;
  displayLogoAndMsg("MAPPER", 4000);

  LoRaWAN.displayMcuInit(); // This inits and turns on the display  
  
  deviceState = DEVICE_STATE_INIT;
  
  /* This will switch deviceState to DEVICE_STATE_SLEEP and schedule a SEND timer which will 
    switch to DEVICE_STATE_SEND if saved network info exists and no new JOIN is necessary */
  LoRaWAN.ifskipjoin(); 
  
  if (deviceState != DEVICE_STATE_INIT)
  {
    /* This messes up with LoRaWAN.init() so it can't be called before it, 
      but if we are not going to call LoRaWAN.init(), then we have to do it here. */
    startGPS(); 
  }
  //Setup user button - this must be after LoRaWAN.ifskipjoin(), because the button is used there to cancel stored settings load and initiate a new join
  pinMode(USER_KEY, INPUT);
  attachInterrupt(USER_KEY, userKey, FALLING);  

  #ifdef VIBR_SENSOR
  pinMode(VIBR_SENSOR, INPUT);
  #endif

  TimerInit(&GPSCycleTimer, OnGPSCycleTimerEvent);
  TimerInit(&menuIdleTimeout, OnMenuIdleTimeout);
}

void loop()
{
  switch (deviceState)
  {
    case DEVICE_STATE_INIT:
    {
      #if(AT_SUPPORT)
      getDevParam();
      #endif
      printDevParam();
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDataRateForNoADR(0); // Set DR_0       
      deviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      startGPS();
      LoRaWAN.displayJoining();
      LoRaWAN.join();
      joinStart = millis();
      lastScreenPrint = joinStart;
      break;
    }
    case DEVICE_STATE_SEND:
    {
      if (menuMode || sleepMode)
      {
        deviceState = DEVICE_STATE_SLEEP;
      }
      else
      {
        if (sendLastLoc && lastLocSet)
        {
          appPort = APP_PORT_LASTLOC;
          if (prepareTxFrame(appPort))
          {
            LoRaWAN.send();   
          }
          sendLastLoc = false;
          appPort = APP_PORT_DEFAULT;
        }
        
        if (!loopingInSend) // We are just getting here from some other state
        {
          loopingInSend = true; // We may be staying here for a while, but we want to reset the below variables only once when we enter.
          /* Reset both these variables. The goal is to skip the first unnecessary display of the GPS Fix Wait screen 
            and only show it if there was more than 1s without GPS fix and correctly display the time passed on it */
          gpsSearchStart = lastScreenPrint = millis(); 
        }
        
        cycleGPS(); // Read anything queued in the GPS Serial buffer, parse it and populate the internal variables with the latest GPS data
        
        if ((GPS.location.age() < GPS_READ_RATE) && (GPS.hdop.hdop() < 2))
        {
          if (prepareTxFrame(appPort)) // Don't send bad data (the method will return false if GPS coordinates are 0)
          {
            if (!menuMode && !sleepMode) // In case user pressed the button while prepareTxFrame() was running
            {
              if (!screenOffMode)
              {
                LoRaWAN.displaySending();
              }
              LoRaWAN.send();
              #ifdef AUTO_SLEEP_TIMER
              lastSend = millis();
              #endif
            }
            // if (!screenOffMode)
            // {
            //   display.sleep();
            //   isDispayOn = 0;
            //   VextOFF();
            // }
          }
          deviceState = DEVICE_STATE_SLEEP; // Schedule GPS timer and go to sleep
        }   
        else
        {
          // if (!screenOffMode)
          // {
          //   displayGPSWaitWithCounter();
          // }
          autoSleepIfNoGPS(); // If the wait for GPS is too long, automatically go to sleep
        }   
      }
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      loopingInSend = false;

      // not sure if this is necessary
      // if (!gpsTimerSet)
      // {
      //   // Schedule a wakeup for GPS read before going to sleep
      //   TimerSetValue(&GPSCycleTimer, GPS_READ_RATE);
      //   TimerStart(&GPSCycleTimer);
      //   gpsTimerSet = true;
      // }
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      loopingInSend = false;
      if (menuMode)
      {
        if (!isDispayOn)
        {
          display.wakeup();
          isDispayOn = 1;
        }
        displayMenu();
      }
      else if (!IsLoRaMacNetworkJoined)
      {
        if (!screenOffMode)
        {
          displayJoinTimer(); // When not joined yet, it will display the seconds passed, so the user knows it is doing something
        }
      }
      else if (!sleepMode) // When not in sleep mode - display the current GPS every second
      {
        autoSleepIfNoMovement();

        if (!sleepMode) // checking if autoSleepIfNoMovement() changed it
        {
          displayGPSInfoEverySecond();

          if (!gpsTimerSet) 
          {
            // Schedule a wakeup for GPS read before going to sleep
            TimerSetValue(&GPSCycleTimer, GPS_READ_RATE);
            TimerStart(&GPSCycleTimer);
            gpsTimerSet = true;          
          }
        }
      }
      else // going to deep sleep, no need to keep the display on
      {
        if (!screenOffMode && isDispayOn) // only if screen on mode (otherwise the dispaly object is not initialized and calling methods from it will cause a crash)
        {
          display.sleep();
          VextOFF();
          isDispayOn = 0;
        }
      }
      
      if (deviceState == DEVICE_STATE_SLEEP) // because the exit from the menu may change it to Cycle or Send and we don't want to go to sleep without having scheduled a wakeup
      {
        LoRaWAN.sleep();
      }
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}