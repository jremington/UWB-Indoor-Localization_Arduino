// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.
// added 8x2 3.3V LCD display so that range tests can be conducted without connection to a computer.
// LCD reports anchor ID and range in meters.

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

#include <LiquidCrystal.h>

// initialize the LCD library with the numbers of the interface pins
//                RS   E  D4  D5  D6  D7
LiquidCrystal lcd(25, 26, 12, 13, 14, 15);
// LCD Vdd on pin GPIO2 (3.3V LCD!!)
// LCD R/W connected GND

void setup()
{
  unsigned long start;
  // set up the LCD
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // power up LCD
  start = millis();
  while (millis() - start < 100UL); //wait 100 ms. delay() does not work here.
  start = millis();

  lcd.begin(8, 2); //columns and rows
  while (millis() - start < 100UL); //wait 100 ms
  // Print a message to the LCD.

  lcd.print("UWB tag ");

  Serial.begin(115200);
  start = millis();
  while (millis() - start < 1000UL); //wait 1 second

  Serial.println(" Setting up tag with default Antenna Delay");
  
  //initialize the UWB configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

// attach callbacks
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address

  // range tests 5/3/2022  outdoors, clear line of sight

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
  
  // same data displayed on 8x2 LCD
  lcd.setCursor(0, 0);
  lcd.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  lcd.print("  ");
  lcd.setCursor(0, 1);
  lcd.print(DW1000Ranging.getDistantDevice()->getRange());
  lcd.print("  ");
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
