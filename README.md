# Overview
**CubeCell-GPS Helium Mapper** based on https://github.com/jas-williams/CubeCell-Helium-Mapper.git with the following improvements:

- No longer stopping the GPS after each SEND, allowing for faster GPS Fix next time.
- Added counters on the JOIN and GPS Fix Wait screens so the user could see how long it takes (could be used to compare antennas, like for example the difference between internal and external GPS antenna is clearly visible, but now you can roughly measure in seconds how much faster you get a fix with one GPS antenna vs another)
- Enabled the built in function for storing the network parameters after susccessful join, so it does not need to re-join next time after reboot
- Added Battery Level display on some screens
- Improved movement detection

This device is used for mapping the Helium networks LoRaWAN coverage. 
The initial settings are - send every 5 seconds while moving, every minute when stopped. 
A quick press on the user button puts the GPS in a sleep mode. The sleep mode decreases the updates to once every 6 hours. 
Pressing the user button while in sleep mode wakes it up and resumes normal operation.

Revision changes:
- Improved movement detection (min stop cycles before switching to stopped update rate). Added battery level display.
- Added vibration sensor wake up. 
- Added Auto Sleep after stopped for too long. 
- Added Auto Sleep when wait for GPS is too long.
- Added Air530Z GPS support for board version 1.1

# Uploading the code

- Install Serial Driver
Find Directions [here.](https://heltec-automation-docs.readthedocs.io/en/latest/general/establish_serial_connection.html)

- Install Visual Studio Code
https://code.visualstudio.com/Download

- Install PlatformIO IDE
https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide


- Clone this repository and open the folder

Edit file main.cpp in folder \src.

Comment out/uncomment the appropriate line for your board version (for GPS Air530 or Air530Z)

Comment out/uncomment the #define lines for VIBR_SENSOR, VIBR_WAKE_FROM_SLEEP, MAX_STOPPED_CYCLES and edit the values for the timers if desired 

Enter DevEUI(msb), AppEUI(msb), and AppKey(msb) from Helium Console, at the respective places in main.cpp. The values must be in MSB format. From console press the expand button to get the ID's as shown below.

![Console Image](https://gblobscdn.gitbook.com/assets%2F-M21bzsbFl2WA7VymAxU%2F-M6fLGmWEQ0QxjrJuvoC%2F-M6fLi5NzuMeWSzzihV-%2Fcubecell-console-details.png?alt=media&token=95f5c9b2-734a-4f84-bb88-523215873116)

```
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
```

Modify platformio.ini if you need to change LoRaWAN settings like region.

Click the PlatformIO: Build button

Connect the CubeCell to the computer with USB cable

Click the PlatformIO: Upload button

# Debug using Serial connection via USB

(Optional) Uncomment the line enabling the DEBUG code and build again.
```
//#define DEBUG // Enable/Disable debug output over the serial console
```
Click the PlatformIO: Serial Monitor button

# Setting up a Decoder

In helium Console create a new function call it Heltec decoder => Type Decoder => Custom Script

Copy and paste the decoder into the custom script pane

```
function Decoder(bytes, port) {
  return {
    latitude:
      ((bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3]) / 1E7,
    longitude:
      ((bytes[4] << 24) | (bytes[5] << 16) | (bytes[6] << 8) | bytes[7]) / 1E7,
    altitude:
      0,
    sats:
      (bytes[9]),
    speed:
      (((bytes[8]))/1.609).toFixed(2), 
    battery:
      (((bytes[10])*0.2)/10).toFixed(2),
    accuracy:
      2.5
  };
}

```

Create a new Label for the decoder then save the function.

You need to create two integrations one for CARGO and one for MAPPERS add the label created above to both integrations.

The payload and the decoder allows the data to update

[Mappers](http://mappers.helium.com) and [Cargo](https://cargo.helium.com)

[Integration information with Mappers](https://docs.helium.com/use-the-network/coverage-mapping/mappers-api/)

[Integration information for Cargo](https://docs.helium.com/use-the-network/console/integrations/cargo/)

Here is a youtube Video uploaded by Joey, but note: Joey adds 3 labels, you only need one label. Assign the one label to the device, function, and the 2 integrations [Youtube Video from Joey](https://youtu.be/WIIC_DvZyz0)
