//anchor #4 setup


// be sure to edit anchor_addr and select the previously calibrated anchor delay
// my naming convention is anchors 1, 2, 3, ... have the lowest order byte of the MAC address set to 81, 82, 83, ...

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

uint16_t shortAddress = 132; //84:00
const char *macAddress = "5B:D5:A9:9A:E2:9C";

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16580;

// previously determined calibration results for antenna delay
// #1 16630
// #2 16610
// #3 16607
// #4 16580

// calibration distance
float dist_m = (285 - 1.75) * 0.0254; //meters

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// NOTE: Any change on this parameters will affect the transmission time of the packets
// So if you change this parameters you should also change the response times on the DW1000 library 
// (actual response times are based on experience made with these parameters)
static constexpr byte MY_MODE[] = {DW1000.TRX_RATE_6800KBPS, DW1000.TX_PULSE_FREQ_16MHZ, DW1000.TX_PREAMBLE_LEN_64};

void setup()
{
  Serial.begin(115200);
  delay(1000); //wait for serial monitor to connect
  Serial.println("Anchor config and start");
  Serial.print("Antenna delay ");
  Serial.println(Adelay);
  Serial.print("Calibration distance ");
  Serial.println(dist_m);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.init(BoardType::ANCHOR, shortAddress, macAddress, false, MY_MODE, PIN_RST, PIN_SS, PIN_IRQ);

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
}

void loop()
{
  DW1000Ranging.loop();
}

void newRange(DW1000Device *device)
{
  //    Serial.print("from: ");
  Serial.print(device->getShortAddress(), HEX);
  Serial.print(", ");

#define NUMBER_OF_DISTANCES 1
  float dist = 0.0;
  for (int i = 0; i < NUMBER_OF_DISTANCES; i++) {
    dist += device->getRange();
  }
  dist = dist/NUMBER_OF_DISTANCES;
  Serial.println(dist);
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
