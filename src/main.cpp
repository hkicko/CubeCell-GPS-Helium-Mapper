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
//#define VIBR_SENSOR           GPIO7     // Change the pin where the sensor is connected if different
// Comment out/uncomment this line to disable/enable the functionality  where the vibration sensor wakes the device from "deep" sleep (VIBR_SENSOR must be enabled)
//#define VIBR_WAKE_FROM_SLEEP
// If put to sleeep from the menu, this will disable the wake up by vibration and only allow it to work when auto sleep was activated in some way (like stopped for too long)
//#define MENU_SLEEP_DISABLE_VIBR_WAKEUP  

#define MOVING_UPDATE_RATE    5000      // Update rate when moving
#define STOPPED_UPDATE_RATE   60000     // Update rate when stopped
#define SLEEPING_UPDATE_RATE  21600000  // Update every 6hrs when sleeping
#define MAX_GPS_WAIT          660000    // Max time to wait for GPS before going to sleep
#define MIN_STOPPED_CYCLES    5         // How many consecutive MOVING_UPDATE_RATE cycles after detecting no movement we should switch to STOPPED_UPDATE_RATE - this is to improve the experience in walk mode
//#define MAX_STOPPED_CYCLES    8         // Max consecutive stopped cycles before going to sleep, keep in mind, the first MIN_STOPPED_CYCLES of these will be at MOVING_UPDATE_RATE and the next after that will be at STOPPED_UPDATE_RATE
#define VBAT_CORRECTION       1.004     // Edit this for calibrating your battery voltage
//#define CAYENNELPP_FORMAT   

/*
  How many past readings to use for avg speed calc.
  This is also the number of "good" readings we need before sending.
  "Good" readings are readings not older than 1s.
*/
#define SPEED_HISTORY_BUFFER_SIZE 5     

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

/* Application port */
uint8_t appPort = 2;
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
double    speedHistory[SPEED_HISTORY_BUFFER_SIZE];
uint8_t   speedHistoryPointer     = 0;
uint8_t   spdHistBuffFull         = 0; /* Counter to tell us how full the buffer is - we want it min SPEED_HISTORY_BUFFER_SIZE before we 
                                        could send and we use that instead of waiting a specific time for the readings to "stabilize" */
float     avgSpeed                = 0;
int       stoppedCycle            = 0;
int       currentMenu             = 0;
uint32_t  movingUpdateRate        = MOVING_UPDATE_RATE;
bool      displayBatPct           = false;
bool      sleepActivatedFromMenu  = false;
enum eDeviceState_LoraWan stateAfterMenu;


#define MENU_CNT 7

char* menu[MENU_CNT] = {"Screen OFF", "Sleep", "Debug Info", "Faster Upd", "Slower Upd", "Reset GPS", "Bat V/%"};

enum eMenuEntries
{
  SCREEN_OFF,
  SLEEP,
  DEBUG_INFO,
  FASTER_UPD,
  SLOWER_UPD,
  RESET_GPS,
  BAT_V_PCT
};

void userKey();

int32_t fracPart(double val, int n)
{
  return (int32_t)abs(((val - (int32_t)(val)) * pow(10, n)));
}

// oled power on
void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

// oled power off
void VextOFF(void) 
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void clearSpeedHistory()
{
  for (int i = 0; i < SPEED_HISTORY_BUFFER_SIZE; i++)
  {
    speedHistory[i] = 0;
  }
  avgSpeed            = 0;
  speedHistoryPointer = 0;
  spdHistBuffFull     = 0;
}

void addSpeedReading(double speedValue)
{
  speedHistory[speedHistoryPointer] = speedValue;

  speedHistoryPointer++;
  if (speedHistoryPointer >= SPEED_HISTORY_BUFFER_SIZE)
  {
    speedHistoryPointer = 0;
  }  

  if (spdHistBuffFull < SPEED_HISTORY_BUFFER_SIZE)
  {
    spdHistBuffFull++;
  }  
}

bool enoughSpeedHistory()
{
  return (spdHistBuffFull == SPEED_HISTORY_BUFFER_SIZE);
}

void calcAvgSpeed()
{
  double total;
  uint8_t tempPointer = speedHistoryPointer;

  for (int i = 0; i < SPEED_HISTORY_BUFFER_SIZE; i++)
  {    
    if (tempPointer == 0)  
    {
      tempPointer = SPEED_HISTORY_BUFFER_SIZE;
    }
    
    tempPointer--;

    total += speedHistory[tempPointer];
  }

  avgSpeed = total / SPEED_HISTORY_BUFFER_SIZE;
}

bool onTheMove()
{
  return avgSpeed > 1.2;
}

void displayGPSInfo()
{
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  int index = sprintf(str, "%02d-%02d-%02d", GPS.date.year(), GPS.date.day(), GPS.date.month());
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

  if (onTheMove())
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
  display.display();

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
  Serial.print("SPEED Average: ");
  Serial.println(avgSpeed);  
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
  if ((millis() - lastScreenPrint) > 1000) 
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
  index = sprintf(str,"%s: %u", "Update rate", movingUpdateRate);
  str[index] = 0; 
  display.drawString(0, 0, str);  
  index = sprintf(str,"%s: %u", "loopingInSend", loopingInSend);
  str[index] = 0; 
  display.drawString(0, 10, str);  
  index = sprintf(str,"%s: %u", "LoRaJoined", IsLoRaMacNetworkJoined);
  str[index] = 0; 
  display.drawString(0, 20, str);  
  index = sprintf(str,"%s: %i", "DR", loraDataRate());
  str[index] = 0; 
  display.drawString(0, 30, str);    
  display.display();

  delay(4000);    
}

void startGPS()
{
  GPS.begin();
  // Air530Z code has setmode(MODE_GPS_BEIDOU_GLONASS) call in begin(), but for Air530 we will need to set it ourselves
  #ifdef GPS_Air530_H
  GPS.setmode(MODE_GPS_GLONASS); //Enable dual mode - GLONASS and GPS   
  #endif
  gpsSearchStart = millis();
}

void cycleGPS()
{
  while (GPS.available() > 0)
  {
    GPS.encode(GPS.read());
  }

  if (GPS.location.age() < 1000)
  {
    addSpeedReading(GPS.speed.kmph());
    calcAvgSpeed();        
  }
  else
  {
    clearSpeedHistory(); // Force the start of new collection of good values    
  }
}

void stopGPS()
{
  GPS.end();
  clearSpeedHistory();
}

void displayGPSWaitWithCounter()
{  
  char str[30];
  int index;

  if ((millis() - lastScreenPrint) > 500)
  {
    if (!isDispayOn)
    {
      display.wakeup(); 
      isDispayOn = 1;
    }
    display.clear();
    display.drawXbm(0, 0, 128, 42, helium_logo_bmp);
    display.setFont(ArialMT_Plain_16);
    if (millis() - gpsSearchStart > 1000) // This is to prevent showing the counter when 0. Some times it recovers very quickly and only shows the screen once and it makes no sense to show with 0 counter if we are not going to be counting up. 
    {
      display.setTextAlignment(TEXT_ALIGN_LEFT);  
      display.drawString(0, 54-16/2, "GPS fix wait");           
      index = sprintf(str,"%d", (millis() - gpsSearchStart) / 1000);
      str[index] = 0; 
      display.setTextAlignment(TEXT_ALIGN_RIGHT);  
      display.drawString(128, 54-16/2, str);      
    }
    else
    {
      display.setTextAlignment(TEXT_ALIGN_CENTER);  
      display.drawString(64, 54-16/2, "GPS fix wait");
    }
    
    display.display();
    lastScreenPrint = millis();
  }  
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
  deviceState = DEVICE_STATE_CYCLE;  
  stoppedCycle = 0;
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
  deviceState = DEVICE_STATE_CYCLE;
  stoppedCycle = 0;
  loopingInSend = false;
  clearSpeedHistory();
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
  if (millis() - gpsSearchStart > MAX_GPS_WAIT)
  {
    switchModeToSleep();
  }
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
  
  lat     = ((GPS.location.lat() + 90) / 180.0) * 16777215;
  lon     = ((GPS.location.lng() + 180) / 360.0) * 16777215;

  if (lat == 0 || lon == 0)
  {
    return false;
  }

  alt     = (uint16_t)GPS.altitude.meters();
  course  = GPS.course.deg();
  speed   = (uint16_t)avgSpeed;  
  sats    = GPS.satellites.value();
  hdop    = GPS.hdop.hdop();

  detachInterrupt(USER_KEY); // reading battery voltage is messing up with the pin and driving it down, which simulates a long press for our interrupt handler 

  uint16_t batteryVoltage = ((float_t)((float_t)((float_t)getBatteryVoltage() * VBAT_CORRECTION)  / 10) + .5);  
  
  //Build Payload
  unsigned char *puc;
  appDataSize = 0;
  
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
  
  return true;
}
#endif

#ifdef VIBR_SENSOR
void vibration(void)
{
  detachInterrupt(VIBR_SENSOR);
  //stoppedCycle = 0;
  
  #ifdef DEBUG
  Serial.println("Vibration detected");
  switch (deviceState)
  {
    case DEVICE_STATE_SLEEP:
      Serial.println("Curent State = SLEEP");
      break;
    case DEVICE_STATE_SEND:
      Serial.println("Curent State = SEND");
      break;
    case DEVICE_STATE_CYCLE:
      Serial.println("Current State = CYCLE");
      break;    
  }
  #endif
  
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
    else
    {
      // We use vibration to shorten the "stopped update rate" cycle. Meaning - if we reached the point where it updates at 
      // stopepd update rate (default 1 min) and there is a vibration before this timer expires, we want to start sending 
      // and not wait for the timer to expire and then send.     
      if (deviceState == DEVICE_STATE_SLEEP && stoppedCycle > MIN_STOPPED_CYCLES) 
      {
        deviceState = DEVICE_STATE_CYCLE;      
        stoppedCycle = MIN_STOPPED_CYCLES;
      }  
    }
  }
}
#endif

void executeMenu(void)
{
  switch (currentMenu)
  {
    case SCREEN_OFF:
      switchScrenOffMode();
      deviceState = stateAfterMenu;
      menuMode = false;      
      break;

    case SLEEP:
      switchModeToSleep();         
      menuMode = false;
      sleepActivatedFromMenu = true;
      break;

    case DEBUG_INFO:
      displayDebugInfo();
      deviceState = stateAfterMenu;
      menuMode = false;
      break;

    case FASTER_UPD:
      if (movingUpdateRate > 1000)
      {
        movingUpdateRate -= 1000;
      }
      deviceState = DEVICE_STATE_CYCLE;  
      stoppedCycle = 0;
      menuMode = false;
      break;

    case SLOWER_UPD:
      if (movingUpdateRate < STOPPED_UPDATE_RATE)
      {
        movingUpdateRate += 1000;
      }
      deviceState = DEVICE_STATE_CYCLE;  
      stoppedCycle = 0;
      menuMode = false;
      break;

    case RESET_GPS:
      stopGPS();
      delay(1000);
      startGPS();
      deviceState = DEVICE_STATE_CYCLE;  
      menuMode = false;
      break;

    case BAT_V_PCT:
      displayBatPct = !displayBatPct;
      deviceState = stateAfterMenu;
      menuMode = false;
      break;
    
    default:
      menuMode = false;
      deviceState = stateAfterMenu;
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
          stateAfterMenu = deviceState;  
          deviceState = DEVICE_STATE_SLEEP;
        }        
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
      if (menuMode) // User pressed the button while we were waiting for the next send timer
      {
        stateAfterMenu = deviceState; // If while waiting inside the menu, a Cycle timer ended and sent us here, after exiting the menu, don't waste time going back to Cycle, go directly to Send
        deviceState = DEVICE_STATE_SLEEP;
      }
      else if (sleepMode)
      {
        deviceState = DEVICE_STATE_CYCLE; // Send to Cycle so it could setup a sleep timer if not done yet
      }
      else 
      {  
        cycleGPS(); // Read anything queued in the GPS Serial buffer, parse it and populate the internal variables with the latest GPS data
        
        if (!loopingInSend) // We are just getting here from some other state
        {
          loopingInSend = true; // We may be staying here for a while, but we want to reset the below variables only once when we enter.
          /* Reset both these variables. The goal is to skip the first unnecessary display of the GPS Fix Wait screen 
            and only show it if there was more than 1s without GPS fix and correctly display the time passed on it */
          gpsSearchStart = lastScreenPrint = millis(); 
        }
        
        if (GPS.location.age() < 1000) 
        {
          // Only send if it had enough time to stabilize, otherwise just display on screen
          if (enoughSpeedHistory()) 
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
              }
              if (!screenOffMode)
              {
                display.sleep();
                isDispayOn = 0;
                VextOFF();
              }
            }
            if (!menuMode)
            {
              deviceState = DEVICE_STATE_CYCLE; // Schedule next send
            }
          }
          else 
          {
            if (!sleepMode)
            {
              displayGPSInfoEverySecond(); // No need to wakeup the display, if we are looping here, it should be already on
            }       
          }
        }   
        else
        {
          if (!screenOffMode)
          {
            displayGPSWaitWithCounter();
          }
          autoSleepIfNoGPS(); // If the wait for GPS is too long, automatically go to sleep
        }   
      }
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      loopingInSend = false;
      if (menuMode)
      {
        stateAfterMenu = deviceState; // This may not be necessary, because most of the menu options exist to Cycle anyway, but feels like the right thing to do
      }
      else
      {
        // Schedule next packet transmission
        if (sleepMode)
        {
            stoppedCycle = 0;
            appTxDutyCycle = SLEEPING_UPDATE_RATE;
            // Schedule wake up by vibration if vibration sensor is enabled/available
            #ifdef VIBR_SENSOR
            #ifdef VIBR_WAKE_FROM_SLEEP // No need to attach to the interrupt if we won't be using the vibration sensor to wake up from sleep
            #ifdef MENU_SLEEP_DISABLE_VIBR_WAKEUP // If menu sleep overwrites the "vibration wake up from sleeep", then add the IF statement to not attach to the interrupt when sleep was initiated from the menu
            if (!sleepActivatedFromMenu)
            #endif
            attachInterrupt(VIBR_SENSOR, vibration, FALLING);
            #endif
            #endif
        }
        else
        {
          appTxDutyCycle = movingUpdateRate;

          if (onTheMove())
          {
            stoppedCycle = 0;
            #ifdef DEBUG
            Serial.println();
            Serial.print("Speed = ");
            Serial.print(avgSpeed);
            Serial.println(" MOVING");
            #endif
          }
          else
          {
            stoppedCycle++;
          
            if (stoppedCycle > MIN_STOPPED_CYCLES) // Do not switch to STOPPED too fast, wait a few more cycles for movement to resume and only if it does not, then switch
            { 
              appTxDutyCycle = STOPPED_UPDATE_RATE;
              #ifdef DEBUG
              Serial.println();
              Serial.print("Speed = ");
              Serial.print(avgSpeed);
              Serial.println(" STOPPED");
              if (screenOffMode)
              {
                delay(5);
              }
              #endif
              // Schedule wake up by vibration if vibration sensor is enabled/available
              #ifdef VIBR_SENSOR
              attachInterrupt(VIBR_SENSOR, vibration, FALLING);
              #endif
              #ifdef MAX_STOPPED_CYCLES
              // Auto sleep mode - if stopped for too many cycles, go to sleep
              if (stoppedCycle > MAX_STOPPED_CYCLES)
              {
                switchModeToSleep();
                appTxDutyCycle = SLEEPING_UPDATE_RATE;
              }
              #endif
            }
          }
        }
        txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
      }
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
      else if (!sleepMode && onTheMove()) // When not in sleep mode and moving - display the current GPS every second
      {
        displayGPSInfoEverySecond();                    
      }
      else // either going into deep sleep or stopped - turn display off
      {
        if (!screenOffMode)
        {
          display.sleep();
          VextOFF();
          isDispayOn = 0;
        }
      }
      //cycleGPS();
      LoRaWAN.sleep();   
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}