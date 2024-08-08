// This program calibrates an ESP32_UWB module intended for use as a fixed anchor point
// uses binary search to find anchor antenna delay to calibrate against a known distance
//
// modified version of Thomas Trojer's DW1000 library is required!

// Remote tag (at origin) must be set up with default antenna delay (library default = 16384)

// user input required, possibly unique to each tag:
// 1) accurately measured distance from anchor to tag
// 2) address of anchor
//
// output: antenna delay parameter for use in final anchor setup.
// S. James Remington 2/20/2022

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// ESP32_UWB pin definitions

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin


char this_anchor_addr[] = "84:00:22:EA:82:60:3B:9C";
float this_anchor_target_distance = 296*0.0254; //measured distance to anchor in m

uint16_t this_anchor_Adelay = 16600; //starting value
uint16_t Adelay_delta = 100; //initial binary search step size


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin


  Serial.print("Starting Adelay "); Serial.println(this_anchor_Adelay);
  Serial.print("Measured distance "); Serial.println(this_anchor_target_distance);
  
  DW1000.setAntennaDelay(this_anchor_Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);

  //start the module as anchor, don't assign random short address
  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

}

void loop()
{
  DW1000Ranging.loop();
}

void newRange()
{
  static float last_delta = 0.0;
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

    float dist = DW1000Ranging.getDistantDevice()->getRange();

  Serial.print(",");
  Serial.print(dist); 
  if (Adelay_delta < 3) {
    Serial.print(", final Adelay ");
    Serial.println(this_anchor_Adelay);
//    Serial.print("Check: stored Adelay = ");
//    Serial.println(DW1000.getAntennaDelay());
    while(1);  //done calibrating
  }

  float this_delta = dist - this_anchor_target_distance;  //error in measured distance

  if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
    last_delta = this_delta;
  
  if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
  else this_anchor_Adelay -= Adelay_delta;
  
  Serial.print(", Adelay = ");
  Serial.println (this_anchor_Adelay);
//  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000.setAntennaDelay(this_anchor_Adelay);
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
