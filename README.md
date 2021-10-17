# Overview
Version with a timer to constantly read the GPS. This should help with movement detection when stopped and will almost remove the need for a vibration sensor (unless we want to use vibration for wake up from deep sleep). That may impact battery life so we want to evaluate it separately from the other version.

# New version - Distance based send!
This version gets rid of the timers for sending, we no longer have moving and stopepd update rates. We no longer collect last x speed readings and calculate average speed to detect movement or stop.

Now we send based on distance traveled. You can adjust the min distance moved that would trigger a send if you want to send more or less often. 

The default value is 25m. Please feel free to increase it, if you think it is consuming too much DC. 

The Faster/Slower Upd menu options add or subtract 10m from the current value every time you click them with a hard limit of 10 min and 500 max. You can use that to test various values and once you find one that suits you - modify the default in the code and flash again.

I removed the Reset GPS and Bat V/% menu options because they were not very useful and I don&apos;t want the menu to have too many options - it makes it difficult to get to the one you want. 

I added a "Send now" option to the menu - in case you haven&apos;t moved much but you want to send and see if hotspot will receive it.

Also added a "Non-stop mode" menu option - this will disable the timers that would put the device to sleep for inactivity (no gps found, no vibration for x seconds, no send for x seconds). Useful if you are about to go on a bus/train/tube/metro ride that goes underground and loses GPS or stops for long periods and you don&apos;t want your device to go to sleep and stop sending. 

