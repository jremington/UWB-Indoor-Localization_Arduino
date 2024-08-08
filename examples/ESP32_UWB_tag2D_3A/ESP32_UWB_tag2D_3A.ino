// currently tag is module #5
// this version is 2D (X,Y) only, 3 anchors.
// The Z coordinates of anchors and tag are all assumed to be zero, so for highest accuracy
// the anchors should be approximately in the same horizontal plane as the tag.
// S. James Remington 1/2022

// This code does not average multiple position measurements!

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

//#define DEBUG_TRILAT   //prints in trilateration code
//#define DEBUG_DIST     //print anchor distances

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

// variables for position determination
#define N_ANCHORS 3
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds)

// global variables, input and output

float anchor_matrix[N_ANCHORS][3] = { //list of anchor coordinates, relative to chosen origin.
  {0.0, 0.0, 0.97},  //Anchor labeled #1
  {3.99, 5.44, 1.14},//Anchor labeled #2
  {3.71, -0.3, 0.6}, //Anchor labeled #3
};  //Z values are ignored in this code

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports

float current_tag_position[2] = {0.0, 0.0}; //global current position (meters with respect to anchor origin)
float current_distance_rmse = 0.0;  //rms error in distance calc => crude measure of position error (meters).  Needs to be better characterized

void setup()
{
  Serial.begin(115200);
  delay(1000);

  //initialize configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address

  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop()
{
  DW1000Ranging.loop();
}

// collect distance data from anchors, presently configured for 4 anchors
// solve for position if all four current

void newRange()
{
  int i; //indices, expecting values 1 to 4
  //index of this anchor, expecting values 1,2,3
  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x03; 
  
  if (index > 0) {
    last_anchor_update[index - 1] = millis();  //decrement for array index
    float range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance[index-1] = range;
    if (range < 0.0 || range > 30.0)     last_anchor_update[index - 1] = 0;  //sanity check, ignore this measurement
  }
  
  int detected = 0;

  //reject old measurements
  for (i = 0; i < N_ANCHORS; i++) {
    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }

  if ( detected == 3) { //three measurements TODO: check millis() wrap

#ifdef DEBUG_DIST
    // print distance and age of measurement
    uint32_t current_time = millis();
    for (i = 0; i < N_ANCHORS; i++) {
      Serial.print(last_anchor_distance[i]);
      Serial.print("\t");
      Serial.println(current_time - last_anchor_update[i]); //age in millis
    }
#endif

    trilat2D_3A();

    //output the values (X, Y and error estimate)
    Serial.print("P= ");
    Serial.print(current_tag_position[0]);
    Serial.write(',');
    Serial.print(current_tag_position[1]);
    Serial.write(',');
    Serial.println(current_distance_rmse);
  }
}  //end newRange

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

int trilat2D_3A(void) {

  // for method see technical paper at
  // https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
  // S. James Remington 1/2022
  //
  // A nice feature of this method is that the normal matrix depends only on the anchor arrangement 
  // and needs to be inverted only once. Hence, the position calculation should be robust.
  //
  static bool first = true;  //first time through, some preliminary work
  float b[N_ANCHORS], d[N_ANCHORS]; //temp vector, distances from anchors

  static float Ainv[2][2], k[N_ANCHORS]; //these are calculated only once

  int i;
  // copy distances to local storage
  for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];

#ifdef DEBUG_TRILAT
  char line[60];
  snprintf(line, sizeof line, "d: %6.2f %6.2f %6.2f", d[0], d[1], d[2]);
  Serial.println(line);
#endif

  if (first) {  //intermediate fixed vectors
    first = false;

    float x[N_ANCHORS], y[N_ANCHORS]; //intermediate vectors
    float A[2][2];  //the A matrix for system of equations to solve

    for (i = 0; i < N_ANCHORS; i++) {
      x[i] = anchor_matrix[i][0];
      y[i] = anchor_matrix[i][1];
      k[i] = x[i] * x[i] + y[i] * y[i];
    }

    // set up least squares equation

    for (i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
#ifdef DEBUG_TRILAT
      snprintf(line, sizeof line, "A  %5.2f %5.2f \n", A[i - 1][0], A[i - 1][1]);
      Serial.println(line);
#endif
    }
    //invert A
    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabs(det) < 1.0E-4) {
      Serial.println("***Singular matrix, check anchor coordinates***");
      while (1) delay(1); //hang
    }

#ifdef DEBUG_TRILAT
    snprintf(line, sizeof line, "det A %8.3e\n", det);
    Serial.println(line);
#endif

    det = 1.0 / det;
    //scale adjoint
    Ainv[0][0] =  det * A[1][1];
    Ainv[0][1] = -det * A[0][1];
    Ainv[1][0] = -det * A[1][0];
    Ainv[1][1] =  det * A[0][0];
  } //end if (first);

  for (i = 1; i < N_ANCHORS; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
  }

  //least squares solution for position
  //solve:  2 A rc = b

  current_tag_position[0] = 0.5 * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
  current_tag_position[1] = 0.5 * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);

  // calculate rms error for distances
  float rmse = 0.0, dc0 = 0.0, dc1 = 0.0;
  for (i = 0; i < N_ANCHORS; i++) {
    dc0 = current_tag_position[0] - anchor_matrix[i][0];
    dc1 = current_tag_position[1] - anchor_matrix[i][1];
    dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1);
    rmse += dc0 * dc0;
  }
  current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS));

  return 1;
}  //end trilat2D_3A
