// currently tag is module labeled #5
// This code calculates the (X,Y,Z) position in meters of a UWB tag, based on the known locations
// of four UWB anchors, labeled 1 to 4
// S. James Remington 1/2022

// This code does not average position measurements!

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include "util/m33v3.h"   //matrix and vector macro library, all loops unrolled

//#define DEBUG_TRILAT   //debug output in trilateration code
//#define DEBUG_DISTANCES   //print collected anchor distances for algorithm
//#define DEBUG_ANCHOR_ID  // print anchor IDs and raw distances

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
float current_tag_position[3] = {0}; //tag current position (meters with respect to origin anchor)
float current_distance_rmse = 0.0;  //error in distance calculations. Crude measure of coordinate error (needs to be characterized)

// variables for position determination
#define N_ANCHORS 4   //THIS VERSION WORKS ONLY WITH 4 ANCHORS. May be generalized to 5 or more.
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds)

float anchor_matrix[N_ANCHORS][3] = { //list of anchor coordinates
  {0.0, 0.0, 0.97},
  {3.99, 5.44, 1.14},
  {3.71, -0.3, 0.61},
  { -0.56, 4.88, 0.15}
};

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports

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
// solve for position if all four beacons are current

void newRange()
{
  int i;

  //index of this anchor, expecting values 1 to 4
  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07; //expect devices 1 to 7
  if (index > 0 && index < 5) {
    last_anchor_update[index - 1] = millis();  //(-1) => array index
    float range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance[index-1] = range;
    if (range < 0.0 || range > 30.0)     last_anchor_update[index - 1] = 0;  //sanity check, ignore this measurement
  }

#ifdef DEBUG_ANCHOR_ID
  Serial.print(index); //anchor ID, raw range
  Serial.print(" ");;
  Serial.println(range);
#endif
  //check for four measurements within the last interval
  int detected = 0;  //count anchors recently seen

  for (i = 0; i < N_ANCHORS; i++) {

    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }
  if ( (detected == N_ANCHORS)) { //four recent measurements

#ifdef DEBUG_DISTANCES
    // print distance and age of measurement
    uint32_t current_time = millis();
    for (i = 0; i < N_ANCHORS; i++) {
      Serial.print(last_anchor_distance[i]);
      Serial.print("\t");
      Serial.println(current_time - last_anchor_update[i]); //age in millis
    }
#endif

    trilat3D_4A();
    Serial.print("P= ");  //result
    Serial.print(current_tag_position[0]);
    Serial.write(',');
    Serial.print(current_tag_position[1]);
    Serial.write(',');
    Serial.print(current_tag_position[2]);
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

int trilat3D_4A(void) {

  // for method see technical paper at
  // https://www.th-luebeck.de/fileadmin/media_cosa/Dateien/Veroeffentlichungen/Sammlung/TR-2-2015-least-sqaures-with-ToA.pdf
  // S. J. Remington 1/2022
  //
  // A nice feature of this method is that the normal matrix depends only on the anchor arrangement
  // and needs to be inverted only once. Hence, the position calculation should be robust.
  //
  static bool first = true;  //first time through, some preliminary work
  float b[3], d[N_ANCHORS]; //distances from anchors

  static float Ainv[3][3], k[N_ANCHORS]; //these are calculated only once

  int i;
  // copy distances to local storage
  for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];

#ifdef DEBUG_TRILAT
  char line[60];
  snprintf(line, sizeof line, "d: %6.2f %6.2f %6.2f d= %6.2f", d[0], d[1], d[2], d[3]);
  Serial.println(line);
#endif

  if (first) {  //intermediate fixed vectors
    first = false;

    float x[N_ANCHORS], y[N_ANCHORS], z[N_ANCHORS]; //intermediate vectors
    float A[3][3];  //the A matrix for system of equations to solve

    for (i = 0; i < N_ANCHORS; i++) {
      x[i] = anchor_matrix[i][0];
      y[i] = anchor_matrix[i][1];
      z[i] = anchor_matrix[i][2];
      k[i] = x[i] * x[i] + y[i] * y[i] + z[i] * z[i];
    }

    // set up the A matrix
    for (i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
      A[i - 1][2] = z[i] - z[0];
#ifdef DEBUG_TRILAT
      snprintf(line, sizeof line, "A %6.2f %6.2f %6.2f", A[i - 1][0], A[i - 1][1], A[i - 1][2]);
      Serial.println(line);
#endif
    }

    float det;
    DETERMINANT_3X3 (det, A);

#ifdef DEBUG_TRILAT
    //    check solution stability (small or zero)
    Serial.print("Determinant of A matrix");
    Serial.println(det);
#endif
    if (fabs(det) < 1.0e-4) {  //TODO : define as parameter
      Serial.println("***Singular matrix, check anchor coordinates***");
      while (1) delay(1); //hang
    }

    det = 1.0 / det;
    SCALE_ADJOINT_3X3 (Ainv, det, A);  //Ainv is static

  } //end if (first)

  // set up least squares equation
  for (i = 1; i < 4; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
  }

  // solve:  2 A x posn = b

  float posn2[3];
  MAT_DOT_VEC_3X3(posn2, Ainv, b);
  // copy to global current_tag_position[]
  for (i = 0; i < 3; i++) current_tag_position[i] = posn2[i] * 0.5; //remove factor of 2

  //rms error in measured versus calculated distances
  float x[3] = {0}, rmse = 0.0, dc = 0.0;
  for (i = 0; i < N_ANCHORS; i++) {
    x[0] = anchor_matrix[i][0] - current_tag_position[0];
    x[1] = anchor_matrix[i][1] - current_tag_position[1];
    x[2] = anchor_matrix[i][2] - current_tag_position[2];
    dc = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    rmse += (d[i] - dc) * (d[i] - dc);
  }
  current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS)); //copy to global

  return 1;
}  //end trilat3D_4A
