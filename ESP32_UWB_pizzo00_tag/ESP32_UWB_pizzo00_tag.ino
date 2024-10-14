// currently tag is module #5
// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.

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
uint16_t shortAddress = 125; //7D:00
const char *macAddress = "22:EA:82:60:3B:9C";

// NOTE: Any change on this parameters will affect the transmission time of the packets
// So if you change this parameters you should also change the response times on the DW1000 library 
// (actual response times are based on experience made with these parameters)
static constexpr byte MY_MODE[] = {DW1000.TRX_RATE_6800KBPS, DW1000.TX_PULSE_FREQ_16MHZ, DW1000.TX_PREAMBLE_LEN_64};

void setup()
{
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.init(BoardType::TAG, shortAddress, macAddress, false, MY_MODE, PIN_RST, PIN_SS, PIN_IRQ);

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
  Serial.print(device->getShortAddress(), HEX);
  Serial.print(",");
  Serial.println(device->getRange());
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
