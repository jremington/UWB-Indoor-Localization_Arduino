# arduino-dw1000 
## Updated version by @pizzo00

A library that offers basic functionality to use Decawave's DW1000 chips/modules with Arduino
(see https://www.decawave.com/products/dwm1000-module).

DW1000 library was created by Thomas Trojer (https://github.com/thotro/arduino-dw1000) \
and maintained by S. James Remington (https://github.com/jremington/UWB-Indoor-Localization_Arduino)

This is an updated version by Pizzolato Davide (https://github.com/pizzo00/UWB-Indoor-Localization_Arduino)

Project state
-------------

**Development:**

Currently (2024) **under development**.

**Improvements:**

- Added informations about the known anchors on blink message so only the unkown will respond
- After a poll an anchor that is not on the poll can respond with a ranging_init on some free time slots (to avoid waiting until a blink)
- I have increased (by reducing the transmitted data) the maximum number of anchor per tag to 6, the tag can still "know" more than 6 anchor and query only the 6 with the best signal
- The system is working with multiple tags, the limit is the occupation of the channel so the number of tags supported depends on the update frequency
- Tag would not wait for the last poll ack to arrive before sending the range anymore (so if the last anchor is offline you had to wait for it to be remove for inactivity). Now it wait for the last one or use a timeout, so the range is always sent.
- Range report to the tag can be opt-out using a flag
- Removed long address
- Add a minimal log library instead of Serial.print

**TODOs:**
* Create a attachCustomPackageHandler for maintenance operation throught uwb (like changing the esp ip remotely)



License
-------
Apache License 2.0 (see [LICENSE](https://github.com/jremington/UWB-Indoor-Localization_Arduino/blob/main/LICENSE))
