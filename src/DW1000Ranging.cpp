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
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */


#include "DW1000Ranging.h"
#include "DW1000Device.h"

DW1000RangingClass DW1000Ranging;


//other devices we are going to communicate with which are on our network:
DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];
byte         DW1000RangingClass::_currentAddress[8];
byte         DW1000RangingClass::_currentShortAddress[2];
byte         DW1000RangingClass::_lastSentToShortAddress[2];

volatile uint8_t DW1000RangingClass::_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t      DW1000RangingClass::_lastDistantDevice    = 0; // TODO short, 8bit?
DW1000Mac    DW1000RangingClass::_globalMac;

//module type (anchor or tag)
int16_t      DW1000RangingClass::_type; // TODO enum??
uint8_t      DW1000RangingClass::_master = 0;
uint32_t     DW1000RangingClass::synchronizedTime=0;


// message flow state
volatile byte    DW1000RangingClass::_expectedMsgId;

// range filter
volatile boolean DW1000RangingClass::_useRangeFilter = false;
uint16_t DW1000RangingClass::_rangeFilterValue = 15;

// message sent/received state
volatile boolean DW1000RangingClass::_sentAck     = false;
volatile boolean DW1000RangingClass::_receivedAck = false;

// protocol error state
boolean          DW1000RangingClass::_protocolFailed = false;

// timestamps to remember
uint32_t          DW1000RangingClass::timer           = 0;
uint8_t 		  DW1000RangingClass::counterForBlink = 0;

uint32_t          DW1000RangingClass::lastSyncTime    = 0;
uint32_t          DW1000RangingClass::roundTripTime   = 0;
uint32_t 		  DW1000RangingClass::myTimeSlotStart = 0; 
uint32_t 		  DW1000RangingClass::myTimeSlotEnd   = 0;
uint32_t 		  DW1000RangingClass::SYNC_INTERVAL   = 0;  //102

// data buffer
byte      DW1000RangingClass::data[LEN_DATA];
// reset line to the chip
uint8_t   DW1000RangingClass::_RST;
uint8_t   DW1000RangingClass::_SS;
// watchdog and reset period
uint32_t  DW1000RangingClass::_lastActivity;
uint32_t  DW1000RangingClass::_resetPeriod;
// reply times (same on both sides for symm. ranging)
uint16_t  DW1000RangingClass::_replyDelayTimeUS;
//timer delay
uint32_t  DW1000RangingClass::_timerDelay;
uint32_t  DW1000RangingClass::DEFAULT_TIMER_DELAY;

// ranging counter (per second)
uint16_t  DW1000RangingClass::_successRangingCount = 0;
uint32_t  DW1000RangingClass::_rangingCountPeriod  = 0;
//Here our handlers
void (* DW1000RangingClass::_handleNewRange)(void) = 0;
void (* DW1000RangingClass::_handleBlinkDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleNewDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleInactiveDevice)(DW1000Device*) = 0;

uint32_t DW1000RangingClass::MICROS_TO_MILLIS = 1000;

uint8_t DW1000RangingClass::messageCounter = 0;
bool DW1000RangingClass::lastTimeslotState;

const uint8_t DW1000RangingClass::kRangeDeviceSize = 17;
const uint8_t DW1000RangingClass::kPollDeviceSize = 4;


/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

void DW1000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ, const uint32_t Default_Timer_Delay) {
	// reset line to the chip
	_RST              = myRST;
	_SS               = mySS;
	_resetPeriod      = DEFAULT_RESET_PERIOD;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;
	//we set our timer delay
	
	DEFAULT_TIMER_DELAY = Default_Timer_Delay;
	//default timer delay
	// 80 defines Poll Delay - Duration 102ms
	// 40 defines Poll Delay - Duration  62ms
	// 25 defines Poll Delay - Duration  47ms
	_timerDelay       = DEFAULT_TIMER_DELAY;
	
	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}

void DW1000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[], const byte channel) {
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.enableMode(mode);
	//DW1000.setChannel(channel);
	DW1000.commitConfiguration();
}

void DW1000RangingClass::generalStart() {
	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message
	
	if(DEBUG) {
		// DEBUG monitoring
		Serial.print("\nDW1000-arduino");
		// initialize the driver
		
		Serial.print("\nconfiguration..");
		// DEBUG chip info and registers pretty printed
		char msg[90];
		DW1000.getPrintableDeviceIdentifier(msg);
		Serial.print("\nDevice ID: ");
		Serial.print(msg);
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		Serial.print("\nUnique ID: ");
		Serial.print(msg);
		char string[6];
		sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
		Serial.print("\n short: ");
		Serial.print(string);
		
		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		Serial.print("\nNetwork ID & Device Address: ");
		Serial.print(msg);
		DW1000.getPrintableDeviceMode(msg);
		Serial.print("\nDevice mode: ");
		Serial.print(msg);
	}
	
	
	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
	// for first time ranging frequency computation
	_rangingCountPeriod = (esp_timer_get_time()/MICROS_TO_MILLIS);
}

//Anchor
void DW1000RangingClass::startAsAnchor(char address[], const byte mode[], const bool randomShortAddress, 
	const bool master, const uint32_t SYNC_Periode, const byte channel) {
	
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("\ndevice address: ");
	Serial.print(address);
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}

	SYNC_INTERVAL = SYNC_Periode;
	
	//configure the network for mac filtering
	//(device Address, network ID, frequency)
	
	uint16_t currShortAddr = _currentShortAddress[0]*256+_currentShortAddress[1];
	DW1000Ranging.configureNetwork(currShortAddr, 0xDECA, mode, channel);
	//general start:
	generalStart();
	
	//defined type as anchor
	_type = ANCHOR;

	//define anchor as master if master == 1
	_master = master;
	if(_master == MASTER){
		Serial.print("\nI am the MASTER-ANCHOR");
	}

	Serial.print("\nANCHOR short address: ");
	Serial.print(currShortAddr, HEX);
	
}

//Tag
void DW1000RangingClass::startAsTag(char address[], const byte mode[], const bool randomShortAddress, const uint32_t TimeSlotStart, 
	const uint32_t TimeSlotEnd, const byte channel) {
	
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("\ndevice address: ");
	Serial.print(address);

	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	} else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode, channel);
	
	generalStart();
	//defined type as tag
	_type = TAG;
	lastSyncTime = (esp_timer_get_time()/MICROS_TO_MILLIS);
	myTimeSlotStart = TimeSlotStart; 
	myTimeSlotEnd   = TimeSlotEnd;
	Serial.print("\n### TAG ###");
}

// Tag & Anchor
boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device, boolean shortAddress) {
    // Check if the device already exists in the network
    for (uint8_t i = 0; i < _networkDevicesNumber; ++i) {
        if ((!shortAddress && _networkDevices[i].isAddressEqual(device)) ||
            (shortAddress && _networkDevices[i].isShortAddressEqual(device))) {
            // The device already exists, do not add it again
            return false;
        }
    }

    // Check if there's space for a new device
    if (_networkDevicesNumber >= MAX_DEVICES) {
        return false; // No space left in the network devices array
    }

    // Add the new device to the network
    device->setRange(0);
    memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
    _networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
    _networkDevicesNumber++;

	if(DEBUG){
		Serial.print("\nAdded device at index: " + String(_networkDevicesNumber - 1));
		Serial.print(", Total devices: " + String(_networkDevicesNumber));
	}
    return true;
}


// Tag & Anchor
boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device) {
    // Check if the device already exists in the network
    for (uint8_t i = 0; i < _networkDevicesNumber; ++i) {
        if (_networkDevices[i].isAddressEqual(device) && _networkDevices[i].isShortAddressEqual(device)) {
            // The device already exists, do not add it again
            return false;
        }
    }

    // Add the new device to the network
    memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device)); // Directly copy the device
    _networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);       // Set the index for the new device
    _networkDevicesNumber++;
	if(DEBUG){
		Serial.print("\nDevice count: " + String(_networkDevicesNumber));
	}
    return true;
}

// Tag & Anchor
void DW1000RangingClass::removeNetworkDevices(int16_t index) {
    if (index < 0 || index >= _networkDevicesNumber) {
        // Invalid index, do nothing
        return;
    }

    // If there is only one device, set the number to 0
    if (_networkDevicesNumber == 1) {
        _networkDevicesNumber = 0;
        return;
    }

    // If the last element is being removed
    if (index == _networkDevicesNumber - 1) {
        _networkDevicesNumber--;
        return;
    }

    // Shift all elements after the one being removed
    for (uint8_t i = index; i < _networkDevicesNumber - 1; ++i) {
        _networkDevices[i] = _networkDevices[i + 1]; // Direct assignment instead of memcpy
        _networkDevices[i].setIndex(i);
    }
    _networkDevicesNumber--; // Reduce the number of devices
}


/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */

//setters
// Tag & Anchor
void DW1000RangingClass::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }

// Tag & Anchor
void DW1000RangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }

// Tag & Anchor
DW1000Device* DW1000RangingClass::searchDistantDevice(const byte shortAddress[]) {
	//we compare the 2 bytes address with the others
	for(uint8_t i = 0; i < _networkDevicesNumber; ++i) {
		if(memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0) {
			//we have found our device !
			return &_networkDevices[i];
		}
	}
	return nullptr;
}

DW1000Device* DW1000RangingClass::getDistantDevice() {
	//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
	return &_networkDevices[_lastDistantDevice];
}


/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

 // Tag & Anchor
void DW1000RangingClass::checkForReset() {
	if(!_sentAck && !_receivedAck) {
		// check if inactive
		if((esp_timer_get_time()/MICROS_TO_MILLIS) - _lastActivity > _resetPeriod) {
			resetInactive();
		}
	}
}

// Tag & Anchor
void DW1000RangingClass::checkForInactiveDevices() {
    for (int16_t i = _networkDevicesNumber - 1; i >= 0; --i) {
        if (_networkDevices[i].isInactive()) {
            // Call the handler for inactive devices if set
            if (_handleInactiveDevice) {
                (*_handleInactiveDevice)(&_networkDevices[i]);
            }
            // Remove the inactive device from the array
            removeNetworkDevices(i);  
            if (DEBUG){
                Serial.print("\nInactive device removed. Remaining devices: " + String(_networkDevicesNumber));
            }
        }
    }
}

// Tag & Anchor
MessageType DW1000RangingClass::detectMessageType(const byte datas[]) {
    switch (datas[0]) {
        case FC_1_BLINK:
            return BLINK;
        case FC_1_SYNC:
            return SYNC;
        case FC_1:
            if (datas[1] == FC_2) {
                return RANGING_INIT; // Long MAC frame message (ranging init)
            }
            if (datas[1] == FC_2_SHORT) {
                // Short MAC frame message (poll, range, range report, etc.)
                switch (datas[SHORT_MAC_LEN]) {
                    case 0:  return POLL;
                    case 1:  return POLL_ACK;
                    case 2:  return RANGE;
                    case 3:  return RANGE_REPORT;
                    case 255: return RANGE_FAILED;
                    default: return ERROR;
                }
            }
            break;
    }
    return ERROR;
}

// Tag
bool DW1000RangingClass::isMyTimeSlot() {
    uint32_t TIMER_MARGIN = DEFAULT_TIMER_DELAY + 22;
    uint32_t currentTime = (esp_timer_get_time()/MICROS_TO_MILLIS) - synchronizedTime;
    return (currentTime >= myTimeSlotStart) && (currentTime + TIMER_MARGIN < myTimeSlotEnd);
}

// Tag & Anchor
void DW1000RangingClass::loop() {
    uint32_t currentTime = (esp_timer_get_time() / MICROS_TO_MILLIS);
    checkForReset();

    // Periodically send a sync signal if configured as master
    if (_master == MASTER) {
        if (currentTime - lastSyncTime >= SYNC_INTERVAL) {
            transmitSync();
            if(DEBUG){
                Serial.printf("\nSync sent: %u ms since last sync.", currentTime - lastSyncTime);
            }
            lastSyncTime = currentTime;
        }
    }

	handlePeriodicTasks((esp_timer_get_time() / MICROS_TO_MILLIS)); 
    handleSentAck();         // Handle the sending of ACKs
    handleReceivedMessage(); // Handle received messages
}

// Tag & Anchor
void DW1000RangingClass::handlePeriodicTasks(uint32_t currentTime) {
    // Check if timer has expired
    
	if (currentTime - timer < _timerDelay) {
        return; // Not enough time has passed yet
    }
	
    // Tasks for TAGs
    
	if (_type == TAG) {
		if (counterForBlink == 0) {
            transmitBlink(); // Sending Blink signal
        } else if ( _networkDevicesNumber > 0){
			_expectedMsgId = POLL_ACK;
			//transmitPoll_2(isMyTimeSlot());

			if(isMyTimeSlot()){
				transmitPoll(nullptr);
			}
			
		}
    }

    // Check for inactive devices (TAGs and ANCHORs)
    if (counterForBlink == 0) {
        checkForInactiveDevices();
    }

    // Updating counter
    counterForBlink++;
    if (counterForBlink > MAX_BLINK_COUNTER) {
        counterForBlink = 0;
    }
    timer = currentTime;
}

// Tag & Anchor
void DW1000RangingClass::handleSentAck() {
	// Check if an ACK has been sent
	if (!_sentAck) {
		return; // No action required
	}

	// Reset the flag to ensure that it is only handled once
	_sentAck = false;

	// Determine the message type of the ACK sent
	MessageType messageType = detectMessageType(data);

	// If the message type is not relevant for the ranging logs, exit the function
	if (messageType != POLL_ACK && messageType != POLL && messageType != RANGE) {
		if (DEBUG) {
			Serial.print("\nhandleSentAck: Ignore message type: " + String(messageType));
		}
		return;
	}

  	// Handle the ACK based on the device type (ANCHOR or TAG)
    switch (_type) {
        case ANCHOR:
            handleSentAckAnchor(messageType);
            break;
        case TAG:
            handleSentAckTag(messageType);
            break;
        default:
            if (DEBUG) {
                Serial.print("\nhandleSentAck: Unknown device type.");
            }
            break;
    }
}

//Anchor
void DW1000RangingClass::handleSentAckAnchor(MessageType messageType) {
	if (messageType != POLL_ACK) {
        return; // Exit early if the message type is not relevant
    }
    DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
    if (myDistantDevice) {
      	DW1000.getTransmitTimestamp(myDistantDevice->timePollAckSent);
    }
}

// Tag
void DW1000RangingClass::handleSentAckTag(MessageType messageType) {
  	if (messageType != POLL && messageType != RANGE) {
        return; // Exit early if the message type is not relevant
	}
	// Retrieve the transmit timestamp for the specific message type (Poll, Range)
    DW1000Time transmitTimestamp;
    DW1000.getTransmitTimestamp(transmitTimestamp);
	updateDeviceTimeStamps(_lastSentToShortAddress, transmitTimestamp, messageType);
}

// Tag
void DW1000RangingClass::updateDeviceTimeStamps(byte* shortAddress, DW1000Time time, MessageType messageType) {
	if (shortAddress[0] == 0xFF && shortAddress[1] == 0xFF) {
		for (uint16_t i = 0; i < _networkDevicesNumber; ++i) {
			if (messageType == POLL) {
				_networkDevices[i].timePollSent = time;
			} else if (messageType == RANGE) {
				_networkDevices[i].timeRangeSent = time;
			}
		}
	} else {
		DW1000Device* myDistantDevice = searchDistantDevice(shortAddress);
		if (myDistantDevice) {
			if (messageType == POLL) {
				myDistantDevice->timePollSent = time;
			} else if (messageType == RANGE) {
				myDistantDevice->timeRangeSent = time;
			}
		}
	}
}

// Tag & Anchor
void DW1000RangingClass::handleReceivedMessage() {
	if (!_receivedAck) {
		return;
	}
	_receivedAck = false;
	DW1000.getData(data, LEN_DATA);

	MessageType messageType = detectMessageType(data);

	switch (messageType) {
		case SYNC:
			if (_type == TAG) {
				handleSync();
			}
			break;
		case BLINK:
			if (_type == ANCHOR) {
				handleBlink();
			}
			break;
		case RANGING_INIT:
			if (_type == TAG) {
				handleRangingInit();
			}
			break;
		default:
			processShortMacMessage(messageType);
			break;
	}
}

// Tag 
void DW1000RangingClass::handleSync() {
	uint32_t currentTime = (esp_timer_get_time()/MICROS_TO_MILLIS);
	if (DEBUG) {
		Serial.print("\nhandleSync: " + String(currentTime - synchronizedTime));
	}
	synchronizedTime = currentTime;
}

// Anchor
void DW1000RangingClass::handleBlink() {
	byte address[8];
	byte shortAddress[2];
	_globalMac.decodeBlinkFrame(data, address, shortAddress);

	DW1000Device myTag(address, shortAddress);
	if (addNetworkDevices(&myTag)) {
		if (_handleBlinkDevice != 0) {
			(*_handleBlinkDevice)(&myTag);
		}
		transmitRangingInit(&myTag);
		noteActivity();
	}
	_expectedMsgId = POLL;
}

// Tag
void DW1000RangingClass::handleRangingInit() {
	byte address[2];
	_globalMac.decodeLongMACFrame(data, address);
	DW1000Device myAnchor(address, true);
	if (addNetworkDevices(&myAnchor, true)) {
		if (_handleNewDevice != 0) {
			(*_handleNewDevice)(&myAnchor);
		}
	}
	noteActivity();
}

// Tag & Anchor
void DW1000RangingClass::processShortMacMessage(MessageType messageType) {
	byte address[2];
	_globalMac.decodeShortMACFrame(data, address);
	DW1000Device* myDistantDevice = searchDistantDevice(address);

	if ((_networkDevicesNumber == 0) || (myDistantDevice == nullptr)) {
		if (DEBUG) {
			Serial.print("\nNot found");
		}
		return;
	}

	if (_type == ANCHOR) {
		processAnchorMessage(messageType, myDistantDevice);
	} else if (_type == TAG) {
		processTagMessage(messageType, myDistantDevice);
	}
}

// Anchor
void DW1000RangingClass::processAnchorMessage(MessageType messageType, DW1000Device* myDistantDevice) {
	if (messageType != _expectedMsgId) {
		_protocolFailed = true;
		return;
	}

	if (messageType == POLL) {
		handlePoll(myDistantDevice);
	} else if (messageType == RANGE) {
		handleRange(myDistantDevice);
	}
}

void DW1000RangingClass::handlePoll(DW1000Device* myDistantDevice) {
    if (!myDistantDevice) {
        if(DEBUG){
            Serial.print("\nhandlePoll: Invalid device pointer");
        }
        return;
    }

	const uint8_t numberDevices = data[SHORT_MAC_LEN + 1];
    const uint8_t* deviceData = data + SHORT_MAC_LEN + 2;

	if(DEBUG){
		Serial.print("\nhandlePoll: numberDevices = ");
		Serial.print(numberDevices);

		Serial.print("\nhandlePoll: deviceData = ");
		for (int i = 0; i < numberDevices * 4; ++i) { // Anzeige für alle Geräte (4 Bytes pro Gerät)
			Serial.print(deviceData[i], HEX);
			Serial.print(" ");
		}
		Serial.print("\nhandlePoll: _currentShortAddress = ");
		Serial.print(_currentShortAddress[0], HEX);
		Serial.print(" ");
		Serial.print(_currentShortAddress[1], HEX);
	}


	for (uint8_t i = 0; i < numberDevices; i++) {
		if (memcmp(deviceData, _currentShortAddress, 2) == 0) {
			// Device found, process the data
			_replyDelayTimeUS = *reinterpret_cast<const uint16_t*>(deviceData + 2);
			_protocolFailed = false;
			DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
			myDistantDevice->noteActivity();
			_expectedMsgId = RANGE;
			transmitPollAck(myDistantDevice);
			noteActivity();
			return;  // Exit the loop once the device is found and processed
		}
		deviceData += 4; // Move to the next device data
	}

	if(DEBUG){
        Serial.print("\nhandlePoll: Device not found in the poll message");
    }
}

// Anchor
void DW1000RangingClass::handleRange(DW1000Device* myDistantDevice) {
	uint8_t numberDevices = 0;
	memcpy(&numberDevices, data + SHORT_MAC_LEN + 1, 1);

	for (uint8_t i = 0; i < numberDevices; i++) {
		byte shortAddress[2];
		memcpy(shortAddress, data + SHORT_MAC_LEN + 2 + i * 17, 2);

		if (shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
			DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
			noteActivity();
			_expectedMsgId = POLL;

			if (!_protocolFailed) {
				myDistantDevice->timePollSent.setTimestamp(data + SHORT_MAC_LEN + 4 + 17 * i);
				myDistantDevice->timePollAckReceived.setTimestamp(data + SHORT_MAC_LEN + 9 + 17 * i);
				myDistantDevice->timeRangeSent.setTimestamp(data + SHORT_MAC_LEN + 14 + 17 * i);

				DW1000Time myTOF;
				computeRangeAsymmetric(myDistantDevice, &myTOF);

				float distance = myTOF.getAsMeters();

				if (_useRangeFilter) {
					if (myDistantDevice->getRange() != 0.0f) {
						distance = filterValue(distance, myDistantDevice->getRange(), _rangeFilterValue);
					}
				}

				myDistantDevice->setRXPower(DW1000.getReceivePower());
				myDistantDevice->setRange(distance);
				myDistantDevice->setFPPower(DW1000.getFirstPathPower());
				myDistantDevice->setQuality(DW1000.getReceiveQuality());

				transmitRangeReport(myDistantDevice);
				uint32_t currentTime = esp_timer_get_time()/MICROS_TO_MILLIS;
				Serial.print("\nNew Range - round trip time: " + String(currentTime - roundTripTime));
				roundTripTime = currentTime;
				_lastDistantDevice = myDistantDevice->getIndex();
				if (_handleNewRange != 0) {
					(*_handleNewRange)();
				}
			} else {
				transmitRangeFailed(myDistantDevice);
			}
			return;
		}
	}
}

// Tag 
void DW1000RangingClass::processTagMessage(MessageType messageType, DW1000Device* myDistantDevice) {
    if (!myDistantDevice) {
        if(DEBUG){
            Serial.print("\nprocessTagMessage: Invalid device pointer");
        }
        return;
    }

    if (messageType != _expectedMsgId) {
        if(DEBUG){
            Serial.printf("\nUnexpected message type. Expected: %d, Received: %d\n", _expectedMsgId, messageType);
        }
        _expectedMsgId = POLL_ACK;
        return;
    }

    switch (messageType) {
        case POLL_ACK:
            handlePollAck(myDistantDevice);
            break;
        case RANGE_REPORT:
            handleRangeReport(myDistantDevice);
            break;
        case RANGE_FAILED:
            if(DEBUG){
                Serial.print("\nRange measurement failed");
            }
            // Consider if any action is needed for RANGE_FAILED
            break;
        default:
            if(DEBUG){
                Serial.printf("\nUnhandled message type: %d\n", messageType);
            }
            break;
    }
}

// Tag
void DW1000RangingClass::handlePollAck(DW1000Device* device) {
    DW1000.getReceiveTimestamp(device->timePollAckReceived);
    device->noteActivity();
    if (device->getIndex() == _networkDevicesNumber - 1) {
        _expectedMsgId = RANGE_REPORT;
        transmitRange(nullptr);
		//transmitRange_2(isMyTimeSlot());
		//transmitRange(device);
    }
}

// Tag
void DW1000RangingClass::handleRangeReport(DW1000Device* device) {
    float curRange, curRXPower;
    memcpy(&curRange, data + 1 + SHORT_MAC_LEN, sizeof(float));
    memcpy(&curRXPower, data + 5 + SHORT_MAC_LEN, sizeof(float));

    if (_useRangeFilter && device->getRange() != 0.0f) {
        curRange = filterValue(curRange, device->getRange(), _rangeFilterValue);
    }

    device->setRange(curRange);
    device->setRXPower(curRXPower);

    _lastDistantDevice = device->getIndex();
    if (_handleNewRange) {
        (*_handleNewRange)();
    }

    if(DEBUG){
        Serial.printf("\nRange: %.2f m, RX Power: %.2f dBm\n", curRange, curRXPower);
    }
}


void DW1000RangingClass::useRangeFilter(boolean enabled) {
	_useRangeFilter = enabled;
}

void DW1000RangingClass::setRangeFilterValue(uint16_t newValue) {
	if (newValue < 2) {
		_rangeFilterValue = 2;
	}else{
		_rangeFilterValue = newValue;
	}
}


/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */


void DW1000RangingClass::handleSent() {
	// status change on sent success
	_sentAck = true;
}

void DW1000RangingClass::handleReceived() {
	//if received frame is corrupt, ignore it
	if(DW1000.isReceiveFailed()){
		return;
	}
	// status change on received success
	_receivedAck = true;
}


void DW1000RangingClass::noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	_lastActivity = (esp_timer_get_time()/MICROS_TO_MILLIS);
}

void DW1000RangingClass::resetInactive() {
	//if inactive
	if(_type == ANCHOR) {
		_expectedMsgId = POLL;
		receiver();
	}
	noteActivity();
}

void DW1000RangingClass::copyShortAddress(byte address1[], byte address2[]) {
	*address1     = *address2;
	*(address1+1) = *(address2+1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

// Tag & Anchor
void DW1000RangingClass::transmitInit() {
	DW1000.newTransmit();
	DW1000.setDefaults();
}

// Tag & Anchor
void DW1000RangingClass::transmit(byte datas[]) {
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}

// Tag & Anchor
void DW1000RangingClass::transmit(byte datas[], uint16_t len) {
	DW1000.setData(datas, len);
	DW1000.startTransmit();
}

// Tag & Anchor
void DW1000RangingClass::transmit(byte datas[], DW1000Time time) {
	DW1000.setDelay(time);
	DW1000.setData(data, LEN_DATA);
	DW1000.startTransmit();
}

// Tag & Anchor
void DW1000RangingClass::transmit(byte datas[], uint16_t len, DW1000Time time) {
	DW1000.setDelay(time);
	DW1000.setData(data, len);
	DW1000.startTransmit();
}

//Tag
void DW1000RangingClass::transmitBlink() {
	transmitInit();
	_globalMac.generateBlinkFrame(data, _currentAddress, _currentShortAddress);
	transmit(data, 12); //transmit(data);
}

//Anchor
void DW1000RangingClass::transmitSync() {
	transmitInit();
	_globalMac.generateSyncFrame(data, _currentAddress, _currentShortAddress);
	transmit(data, 12);
}

//Anchor
void DW1000RangingClass::transmitRangingInit(DW1000Device* myDistantDevice) {
	transmitInit();
	//we generate the mac frame for a ranging init message
	_globalMac.generateLongMACFrame(data, _currentShortAddress, myDistantDevice->getByteAddress());
	//we define the function code
	data[LONG_MAC_LEN] = RANGING_INIT;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data);
}

//Tag - broadcast
void DW1000RangingClass::transmitPoll(DW1000Device* myDistantDevice) {
	
	transmitInit();
	
	if(myDistantDevice == nullptr) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/MICROS_TO_MILLIS);
		
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = POLL;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//each devices have a different reply delay time.
			_networkDevices[i].setReplyTime((2*i+1)*DEFAULT_REPLY_DELAY_TIME);
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+kPollDeviceSize*i, _networkDevices[i].getByteShortAddress(), 2);
			//we add the replyTime
			uint16_t replyTime = _networkDevices[i].getReplyTime();
			memcpy(data+SHORT_MAC_LEN+2+2+kPollDeviceSize*i, &replyTime, 2);
			
		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		transmit(data, SHORT_MAC_LEN+2+kPollDeviceSize*_networkDevicesNumber);
		
	}
	else {
		//we redefine our default_timer_delay for just 1 device;
		_timerDelay = DEFAULT_TIMER_DELAY;
		
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		
		data[SHORT_MAC_LEN]   = POLL;
		data[SHORT_MAC_LEN+1] = 1;
		uint16_t replyTime = myDistantDevice->getReplyTime();
		memcpy(data+SHORT_MAC_LEN+2, &replyTime, sizeof(uint16_t)); // todo is code correct?
		
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
		transmit(data, SHORT_MAC_LEN+2+kPollDeviceSize);
	}
	
}

//Tag
void DW1000RangingClass::transmitPoll_2(bool timeslot) {
	
	DeviceIndices indices = calculateDeviceIndices(timeslot);
	
	if (indices.deviceCount == 0) {
		return;
	}

	transmitInit();
	_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(indices.deviceCount*3*DEFAULT_REPLY_DELAY_TIME/MICROS_TO_MILLIS);
	data[SHORT_MAC_LEN] = POLL;
	data[SHORT_MAC_LEN + 1] = indices.deviceCount;

	byte shortBroadcast[2] = {0xFF, 0xFF};
	_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);

	uint8_t device_index = 0;
	for (uint8_t i = indices.startIndex; i <= indices.endIndex; i++) {
		_networkDevices[i].setReplyTime((2 * device_index + 1) * DEFAULT_REPLY_DELAY_TIME);
		memcpy(data+SHORT_MAC_LEN + 2 + kPollDeviceSize * device_index, _networkDevices[i].getByteShortAddress(), 2);
		uint16_t replyTime = _networkDevices[i].getReplyTime();
		memcpy(data+SHORT_MAC_LEN + 2 + 2 + kPollDeviceSize * device_index, &replyTime, 2);
		device_index++;
	}
	copyShortAddress(_lastSentToShortAddress, shortBroadcast);
	transmit(data, SHORT_MAC_LEN + 2 + kPollDeviceSize * indices.deviceCount); 
}

DeviceIndices DW1000RangingClass::calculateDeviceIndices(bool timeslot) {
    DeviceIndices result = {255, 255, 0};

    switch (_networkDevicesNumber) {
        case 1:
            if (timeslot) {
                result.deviceCount = 1;
                result.startIndex = 0;
                result.endIndex = 0;
            }
            break;
        case 2:
            result.deviceCount = 1;
            result.startIndex = timeslot ? 0 : 1;
            result.endIndex = result.startIndex;
            break;
        case 3:
            result.deviceCount = timeslot ? 2 : 1;
            result.startIndex = timeslot ? 0 : 2;
            result.endIndex = timeslot ? 1 : 2;
            break;
        case 4:
            result.deviceCount = 2;
            result.startIndex = timeslot ? 0 : 2;
            result.endIndex = timeslot ? 1 : 3;
            break;
    }
    return result;
}

//Anchor
void DW1000RangingClass::transmitPollAck(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = POLL_ACK;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	//transmit(data, SHORT_MAC_LEN+1, deltaTime); //
	transmit(data, deltaTime);
}

//Tag
void DW1000RangingClass::transmitRange(DW1000Device* myDistantDevice) {
	//transmit range need to accept broadcast for multiple anchor
	transmitInit();
	
	if(myDistantDevice == nullptr) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/MICROS_TO_MILLIS);
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = RANGE;
		//we enter the number of devices
		data[SHORT_MAC_LEN + 1] = _networkDevicesNumber;
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime     = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
		DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//we write the short address of our device:
			memcpy(data + SHORT_MAC_LEN + 2 + kRangeDeviceSize * i, _networkDevices[i].getByteShortAddress(), 2);
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			_networkDevices[i].timeRangeSent = timeRangeSent;
			_networkDevices[i].timePollSent.getTimestamp(data + SHORT_MAC_LEN + 4 + kRangeDeviceSize * i);
			_networkDevices[i].timePollAckReceived.getTimestamp(data + SHORT_MAC_LEN + 9 + kRangeDeviceSize * i);
			_networkDevices[i].timeRangeSent.getTimestamp(data+SHORT_MAC_LEN + 14 + kRangeDeviceSize * i);
		}
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		transmit(data, SHORT_MAC_LEN + 2 + kRangeDeviceSize *_networkDevicesNumber);
	}
	else{
			// Timer-Delay setzen (nur für ein Gerät)
			_timerDelay = DEFAULT_TIMER_DELAY + (uint16_t)(3 * DEFAULT_REPLY_DELAY_TIME / MICROS_TO_MILLIS);
		
			// MAC-Frame generieren
			_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
			data[SHORT_MAC_LEN] = static_cast<byte>(MessageType::RANGE);
			data[SHORT_MAC_LEN+1] = 1; // Nur ein Gerät
		
			// Verzögerung für das Senden der Nachricht
			DW1000Time deltaTime = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
			DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
		
			// Daten für das spezifische Gerät schreiben
			memcpy(data + SHORT_MAC_LEN + 2, myDistantDevice->getByteShortAddress(), 2);
			myDistantDevice->timeRangeSent = timeRangeSent;
			myDistantDevice->timePollSent.getTimestamp(data + SHORT_MAC_LEN + 4);
			myDistantDevice->timePollAckReceived.getTimestamp(data + SHORT_MAC_LEN + 9);
			myDistantDevice->timeRangeSent.getTimestamp(data + SHORT_MAC_LEN + 14);
		
			copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
			transmit(data, SHORT_MAC_LEN + 2 + 17); // 17 Bytes für ein Gerät
	}
}

//Tag
void DW1000RangingClass::transmitRange_2(bool timeslot) {
	//transmit range need to accept broadcast for multiple anchor
	DeviceIndices indices = calculateDeviceIndices(timeslot);

	transmitInit();
	
	//we need to set our timerDelay:
	_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(indices.deviceCount *3*DEFAULT_REPLY_DELAY_TIME/MICROS_TO_MILLIS);
	
	byte shortBroadcast[2] = {0xFF, 0xFF};
	_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
	data[SHORT_MAC_LEN]   = RANGE;
	//we enter the number of devices
	data[SHORT_MAC_LEN+1] = indices.deviceCount;
	
	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime     = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
	DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);

	uint8_t device_index = 0;
	for(uint8_t i = indices.startIndex; i <= indices.endIndex; i++) {
		//we write the short address of our device:
		memcpy(data+SHORT_MAC_LEN+2+17*device_index, _networkDevices[i].getByteShortAddress(), 2);
		//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
		_networkDevices[i].timeRangeSent = timeRangeSent;
		_networkDevices[i].timePollSent.getTimestamp(data+SHORT_MAC_LEN+4+17*device_index);
		_networkDevices[i].timePollAckReceived.getTimestamp(data+SHORT_MAC_LEN+9+17*device_index);
		_networkDevices[i].timeRangeSent.getTimestamp(data+SHORT_MAC_LEN+14+17*device_index);
		device_index++;
	}
	copyShortAddress(_lastSentToShortAddress, shortBroadcast);
	transmit(data, SHORT_MAC_LEN+2+17*indices.deviceCount);
}


// Anchor
void DW1000RangingClass::transmitRangeReport(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_REPORT;
	// write final ranging result
	float curRange   = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	
	//We add the Range and then the RXPower
	memcpy(data+1+SHORT_MAC_LEN, &curRange, 4);
	memcpy(data+5+SHORT_MAC_LEN, &curRXPower, 4);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, SHORT_MAC_LEN+9, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS)); //transmit(data, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS));
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_FAILED;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, SHORT_MAC_LEN+1); //transmit(data);
}

void DW1000RangingClass::receiver() {
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}


/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */


void DW1000RangingClass::computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF) {
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceived-myDistantDevice->timePollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent-myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived-myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSent-myDistantDevice->timePollAckReceived).wrap();
	
	myTOF->setTimestamp((round1*round2-reply1*reply2)/(round1+round2+reply1+reply2));
	if(DEBUG){
		Serial.print("\ntimePollAckReceived ");myDistantDevice->timePollAckReceived.print();
		Serial.print("\ntimePollSent ");myDistantDevice->timePollSent.print();
		Serial.print("\nround1 "); Serial.print((long)round1.getTimestamp());
		
		Serial.print("\ntimePollAckSent ");myDistantDevice->timePollAckSent.print();
		Serial.print("\ntimePollReceived ");myDistantDevice->timePollReceived.print();
		Serial.print("\nreply1 "); Serial.print((long)reply1.getTimestamp());
		
		Serial.print("\ntimeRangeReceived ");myDistantDevice->timeRangeReceived.print();
		Serial.print("\ntimePollAckSent ");myDistantDevice->timePollAckSent.print();
		Serial.print("\nround2 "); Serial.print((long)round2.getTimestamp());
		
		Serial.print("\ntimeRangeSent ");myDistantDevice->timeRangeSent.print();
		Serial.print("\ntimePollAckReceived ");myDistantDevice->timePollAckReceived.print();
		Serial.print("\nreply2 "); Serial.print((long)reply2.getTimestamp());
	}
}


/* FOR DEBUGGING*/
void DW1000RangingClass::visualizeDatas(byte datas[]) {
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7], datas[8], datas[9], datas[10], datas[11], datas[12], datas[13], datas[14], datas[15]);
	Serial.print(string);
}



/* ###########################################################################
 * #### Utils  ###############################################################
 * ######################################################################### */

float DW1000RangingClass::filterValue(float value, float previousValue, uint16_t numberOfElements) {
	
	float k = 2.0f / ((float)numberOfElements + 1.0f);
	return (value * k) + previousValue * (1.0f - k);
}