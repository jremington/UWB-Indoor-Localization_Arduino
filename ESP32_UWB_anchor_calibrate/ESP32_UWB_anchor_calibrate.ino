// This code allows the user to calibrate a given anchor.
// procedure: identify an ESP32_UWB module that will be the tag.
// setup the tag without specifying "antenna delay" so that the default is the DW1000 library default of 16384
// set the tag and a uniquely identified anchor a know distance apart. Decawave recommends about 8 meters
// collect distances from the tag. The code makes 1000 measurements and reports the average.

// in subsequent runs, adjust the ANCHOR antenna delay so that the reported distance is correct. 
// expect antenna delay values in the range of 16550 to 16650. 
// Larger antenna delays result in shorter reported ranges.
// S. James Remington 1/2022

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// unique anchor andress. First two bytes become "short address", 
// The last three bits of first byte = anchor ID in 2D and 3D localization code.

char anchor_addr[]= "82:00:5B:D5:A9:9A:E2:9C";  //#2

//trial Antenna Delay setting for this anchor
uint16_t Adelay=16610;

// calibration distance
float dist_m = 7.19; //this example, meters

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

void setup()
{
    Serial.begin(115200);
    while(!Serial); //wait for serial monitor to connect

    Serial.println("Anchor config and start");
    Serial.print("Antenna delay ");
    Serial.println(Adelay);
    Serial.print("Calibration distance ");
    Serial.println(dist_m);
    
    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000.setAntennaDelay(Adelay);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    //start the module as an anchor. DO NOT assign random short address

    DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY);
    // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop()
{
    DW1000Ranging.loop();
}

void newRange()
{
//    Serial.print("range from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print(", ");
	// get 1000 range measurements
    float dist=0.0;
    for(int i=0; i<1000; i++) {
    dist += DW1000Ranging.getDistantDevice()->getRange();
    }
    dist /= 1000.0;  
    Serial.println(dist);
 }

void newBlink(DW1000Device *device)
{
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX); 
}
