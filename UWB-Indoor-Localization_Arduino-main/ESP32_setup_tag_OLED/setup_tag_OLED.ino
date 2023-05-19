// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.
// added Adafruit monomchrome OLED display so that range tests can be conducted without connection to a computer.
// OLED reports anchor ID and range in meters.

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// ESP32 SDA = GPIO21 = Stemma Blue
// ESP32 SCL = GPIO22 = Stemma Yellow
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("UWB: Setting up tag with default Antenna Delay"));
  unsigned long start;

  // set up the OLED power (does not work, although pin voltage is measured to be 3.28V -- sjr)
  //    pinMode(2, OUTPUT); //supposed to source up to 40 mA max domain 3
  //    digitalWrite(2, HIGH); // power up display
  //    delay(2000);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner

  display.println("UWB tag ");
  display.display();


  //initialize the UWB configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  // attach callbacks
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address

  // range tests 5/3/2022  outdoors, PCB antennas upright and parallel, clear line of sight

  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);  //range 33 m
  //  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);  // range 7 m  smart power 10 m
  //  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_FAST_LOWPOWER, false);  // 6.5 m
  //  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);  // 5 m, smart power 5 m
  //  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_FAST_ACCURACY, false);  // 5 m
  //  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);  // 5 m
  /*
    The following modes are pre-configured and one of them needs to be chosen:
    - `MODE_LONGDATA_RANGE_LOWPOWER` (basically this is 110 kb/s data rate, 16 MHz PRF and long preambles)
    - `MODE_SHORTDATA_FAST_LOWPOWER` (basically this is 6.8 Mb/s data rate, 16 MHz PRF and short preambles)
    - `MODE_LONGDATA_FAST_LOWPOWER` (basically this is 6.8 Mb/s data rate, 16 MHz PRF and long preambles)
    - `MODE_SHORTDATA_FAST_ACCURACY` (basically this is 6.8 Mb/s data rate, 64 MHz PRF and short preambles)
    - `MODE_LONGDATA_FAST_ACCURACY` (basically this is 6.8 Mb/s data rate, 64 MHz PRF and long preambles)
    - `MODE_LONGDATA_RANGE_ACCURACY` (basically this is 110 kb/s data rate, 64 MHz PRF and long preambles)
  */
  //   DW1000Class::useSmartPower(true); //enable higher power for short messages

  // print configuration
  char msgBuffer[100] = {0}; //be generous with msgBuffer size!
  DW1000Class::getPrintableDeviceMode(msgBuffer);
  Serial.println(msgBuffer);
}

void loop()
{
  DW1000Ranging.loop();
}

void newRange()
{
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(",");
  Serial.println(DW1000Ranging.getDistantDevice()->getRange());

  // same data displayed on 128x32 OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  display.print(": ");
  display.println(DW1000Ranging.getDistantDevice()->getRange(), 1);
  display.display();
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
