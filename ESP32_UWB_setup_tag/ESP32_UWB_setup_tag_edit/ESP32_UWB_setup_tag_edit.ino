// currently tag is module #5
// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// Wi-Fi settings
const char* ssid = "Guy"; // Add your wifi ssid
const char* password = "0967949888"; // Add your wifi password

// MQTT settings
const char* mqttBroker = "localhost"; // Add your mqtt broker
const int mqttPort = 1883;
const char* mqttClientId = "tag";
const char* mqttTopic = "tag_data";

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// connection pins
const uint8_t PIN_RST = 27;  // reset pin
const uint8_t PIN_IRQ = 34;  // irq pin
const uint8_t PIN_SS = 4;    // spi select pin

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "8D:00:22:EA:82:60:3B:9C";

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to MQTT broker
  client.setServer(mqttBroker, mqttPort);
  while (!client.connected()) {
    if (client.connect(mqttClientId)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }

  delay(1000);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);  //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address

  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
  // DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_FAST_LOWPOWER, false);
}

void loop() {
  DW1000Ranging.loop();

  // Maintain MQTT connection
  if (!client.connected()) {
    reconnect();
  }

  client.loop();
}

void newRange() {
  // Create a JSON payload with the anchor data
  String payload = String(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX) + "," + String(DW1000Ranging.getDistantDevice()->getRange());

  // Publish the data to MQTT
  client.publish(mqttTopic, payload.c_str());

  // Print the data to serial monitor
  Serial.println(payload);
}

void newDevice(DW1000Device* device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(mqttClientId)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
