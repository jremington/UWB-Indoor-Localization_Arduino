/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Ranging.h
 * Arduino global library (header file) working with the DW1000 library
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */

#include "DW1000.h"
#include "DW1000Time.h"
#include "DW1000Device.h"
#include "DW1000Mac.h"

//Log tags

// messages used in the ranging protocol
enum class MessageType : byte 
{
	POLL = 0,
	POLL_ACK = 1,
	RANGE = 2,
	RANGE_REPORT = 3,
	BLINK = 4,
	RANGING_INIT = 5,
	TYPE_ERROR = 254,
	RANGE_FAILED = 255,
};

#define LEN_DATA 90

// Max devices we put in the networkDevices array ! Each DW1000Device is 74 Bytes in SRAM memory for now.
#define MAX_DEVICES 12

// One blink every x polls
#define BLINK_INTERVAL 5

// Default Pin for module:
#define DEFAULT_RST_PIN 9
#define DEFAULT_SPI_SS_PIN 10
#define DEFAULT_SPI_IRQ_PIN 2

// Default value
// in ms
#define DEFAULT_RESET_PERIOD 2000
// in us
#define DEFAULT_REPLY_DELAY_TIME 3000

// sketch type (anchor or tag)
enum class BoardType : byte 
{
	TAG = 0,
	ANCHOR = 1,
};

// default timer delay
#define DEFAULT_RANGE_INTERVAL 1500

#define ENABLE_RANGE_REPORT false

class DW1000RangingClass
{
public:
	// Initialization
	static void init(BoardType type, uint16_t shortAddress, const char *wifiMacAddress, bool high_power, const byte mode[], uint8_t myRST = DEFAULT_RST_PIN, uint8_t mySS = DEFAULT_SPI_SS_PIN, uint8_t myIRQ = DEFAULT_SPI_IRQ_PIN);

	static void loop();

	// Handlers
	static void attachNewRange(void (*handleNewRange)(DW1000Device *)) { _handleNewRange = handleNewRange; };
	static void attachBlinkDevice(void (*handleBlinkDevice)(DW1000Device *)) { _handleBlinkDevice = handleBlinkDevice; };
	static void attachNewDevice(void (*handleNewDevice)(DW1000Device *)) { _handleNewDevice = handleNewDevice; };
	static void attachInactiveDevice(void (*handleInactiveDevice)(DW1000Device *)) { _handleInactiveDevice = handleInactiveDevice; };
	static void attachRemovedDeviceMaxReached(void (*handleRemovedDeviceMaxReached)(DW1000Device *)) { _handleRemovedDeviceMaxReached = handleRemovedDeviceMaxReached; };
	
private:
	// Initialization
    static void initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ);
	
	// variables
	// data buffer
	static byte receivedData[LEN_DATA];
	static byte sentData[LEN_DATA];

	// Initialization
	static void configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[]);
	static void generalStart(bool high_power);
	static boolean addNetworkDevices(DW1000Device *device);
	static void removeNetworkDevices(uint8_t index);

	// Setters
	static void setResetPeriod(uint32_t resetPeriod);

	// Getters
	static byte *getCurrentAddress() { return _ownLongAddress; };
	static byte *getCurrentShortAddress() { return _ownShortAddress; };
	static uint8_t getNetworkDevicesNumber() { return _networkDevicesNumber; };

	// Utils
	static MessageType detectMessageType(byte datas[]);
	static DW1000Device *searchDistantDevice(byte shortAddress[]);
	static void copyShortAddress(byte address1[], byte address2[]);

	// FOR DEBUGGING
	static void visualizeDatas(byte datas[]);

private:
	// Other devices in the network
	static DW1000Device _networkDevices[MAX_DEVICES];
	static volatile uint8_t _networkDevicesNumber;
	static byte _ownLongAddress[8];
	static byte _ownShortAddress[2];
	static byte _lastSentToShortAddress[2];
	static DW1000Mac _globalMac;
	static uint32_t lastTimerTick;
	static uint32_t _replyTimeOfLastPollAck;
	static uint32_t _timeOfLastPollSent;
	static uint16_t _addressOfExpectedLastPollAck;
	static int16_t counterForBlink;

	// Handlers
	static void (*_handleNewRange)(DW1000Device *);
	static void (*_handleBlinkDevice)(DW1000Device *);
	static void (*_handleNewDevice)(DW1000Device *);
	static void (*_handleInactiveDevice)(DW1000Device *);
	static void (*_handleRemovedDeviceMaxReached)(DW1000Device *);

	// Board type (tag or anchor)
	static BoardType _type;
	// Message flow state
	static volatile MessageType _expectedMsgId;
	// Message sent/received state
	static volatile boolean _sentAck;
	static volatile boolean _receivedAck;
	// Protocol error state
	static boolean _protocolFailed;
	// Reset line to the chip
	static uint8_t _RST;
	static uint8_t _SS;
	// Watchdog and reset period
	static uint32_t _lastActivity;
	static uint32_t _resetPeriod;
	// Timer Tick delay
	static uint16_t _timerDelay;
	// Millis between one range and another
	static uint16_t _rangeInterval;
	// Ranging counter (per second)
	static uint32_t _rangingCountPeriod;

	// Methods
	static void handleSent();
	static void handleReceived();
	static void noteActivity();
	static void resetInactive();

	// Global functions:
	static void checkForReset();
	static void checkForInactiveDevices();

	// ANCHOR ranging protocol
	static void transmitInit();
	static void transmit(byte datas[]);
	static void transmit(byte datas[], DW1000Time time);
	static void transmitBlink();
	static void transmitRangingInit(u_int16_t delay = 0);
	static void transmitPollAck(DW1000Device *myDistantDevice, u_int16_t delay);
	static void transmitRangeReport(DW1000Device *myDistantDevice, u_int16_t delay);
	static void transmitRangeFailed(DW1000Device *myDistantDevice);
	static void receiver();

	// TAG ranging protocol
	static void transmitPoll();
	static void transmitRange();

	// Methods for range computation
	static void timerTick();
	static void computeRangeAsymmetric(DW1000Device *myDistantDevice, DW1000Time *myTOF);
	static uint16_t getReplyTimeOfIndex(int i);
};

extern DW1000RangingClass DW1000Ranging;
