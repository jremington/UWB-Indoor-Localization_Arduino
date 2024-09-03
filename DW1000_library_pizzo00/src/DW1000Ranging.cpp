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
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */

#include "DW1000Ranging.h"
#include "DW1000Device.h"
#include "m_log.h"

DW1000RangingClass DW1000Ranging;

constexpr short rangeDeviceSize = 12;
constexpr short pollDeviceSize = 4;
constexpr uint8_t devicePerPollTransmit = 4;
constexpr uint8_t pollAckTimeSlots = 6;

DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];
byte DW1000RangingClass::_ownLongAddress[8];
byte DW1000RangingClass::_ownShortAddress[2];
byte DW1000RangingClass::_lastSentToShortAddress[2];
DW1000Mac DW1000RangingClass::_globalMac;
BoardType DW1000RangingClass::_type;
volatile MessageType DW1000RangingClass::_expectedMsgId;
byte DW1000RangingClass::receivedData[LEN_DATA];
byte DW1000RangingClass::sentData[LEN_DATA];
uint8_t DW1000RangingClass::_RST;
uint8_t DW1000RangingClass::_SS;
uint32_t DW1000RangingClass::_lastActivity;
uint32_t DW1000RangingClass::_resetPeriod;
uint16_t DW1000RangingClass::_timerDelay;
volatile uint8_t DW1000RangingClass::_networkDevicesNumber;
volatile boolean DW1000RangingClass::_sentAck;
volatile boolean DW1000RangingClass::_receivedAck;
boolean DW1000RangingClass::_protocolFailed;
uint32_t DW1000RangingClass::lastTimerTick;
uint32_t DW1000RangingClass::_replyTimeOfLastPollAck;
uint32_t DW1000RangingClass::_timeOfLastPollSent;
uint16_t DW1000RangingClass::_addressOfExpectedLastPollAck;
int16_t DW1000RangingClass::counterForBlink;
uint16_t DW1000RangingClass::_rangeInterval;
uint32_t DW1000RangingClass::_rangingCountPeriod;
void (*DW1000RangingClass::_handleNewRange)(DW1000Device *);
void (*DW1000RangingClass::_handleBlinkDevice)(DW1000Device *);
void (*DW1000RangingClass::_handleNewDevice)(DW1000Device *);
void (*DW1000RangingClass::_handleInactiveDevice)(DW1000Device *);
void (*DW1000RangingClass::_handleRemovedDeviceMaxReached)(DW1000Device *);

void DW1000RangingClass::init(BoardType type, uint16_t shortAddress, const char *wifiMacAddress, bool high_power, const byte mode[], uint8_t myRST, uint8_t mySS, uint8_t myIRQ)
{
	_networkDevicesNumber = 0;
	_sentAck = false;
	_receivedAck = false;
	_protocolFailed = false;
	lastTimerTick = 0;
	_replyTimeOfLastPollAck = 0;
	_addressOfExpectedLastPollAck = 0;
	_timeOfLastPollSent = 0;
	counterForBlink = 0; // TODO 8 bit?
	_rangeInterval = DEFAULT_RANGE_INTERVAL;
	_rangingCountPeriod = 0;
	_handleNewRange = 0;
	_handleBlinkDevice = 0;
	_handleNewDevice = 0;
	_handleInactiveDevice = 0;
	_handleRemovedDeviceMaxReached = 0;

	initCommunication(myRST, mySS, myIRQ);

	// convert the address
	DW1000.convertToByte(shortAddress, _ownLongAddress);
	DW1000.convertToByte(wifiMacAddress, _ownLongAddress + 2, 6);

	// we use first two bytes in address for short address
	_ownShortAddress[0] = _ownLongAddress[0];
	_ownShortAddress[1] = _ownLongAddress[1];

	// write the address on the DW1000 chip
	DW1000.setEUI(_ownLongAddress);

	// we configure the network for mac filtering
	//(device Address, network ID, frequency)
	configureNetwork(shortAddress, 0xDECA, mode);

	// general start
	generalStart(high_power);

	// defined type
	_type = type;

	if (_type == BoardType::ANCHOR)
		m_log::log_inf(LOG_DW1000, "### ANCHOR ###");
	else if (type == BoardType::TAG)
		m_log::log_inf(LOG_DW1000, "### TAG ###");

	char msg[6];
	sprintf(msg, "%02X:%02X", _ownShortAddress[0], _ownShortAddress[1]);
	m_log::log_inf(LOG_DW1000, "Short address: %s", msg);
}

/* ###########################################################################
 * #### Init and end #########################################################
 * ########################################################################### */

void DW1000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ)
{
	// reset line to the chip
	_RST = myRST;
	_SS = mySS;
	_resetPeriod = DEFAULT_RESET_PERIOD;

	// we set our timer delay
	_timerDelay = _rangeInterval;

	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}

void DW1000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[])
{
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.enableMode(mode);
	DW1000.commitConfiguration();
}

void DW1000RangingClass::generalStart(bool high_power)
{
	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message

	/*
	// initialize the driver
	m_log::log_vrb(LOG_DW1000, "configuration...");
	// DEBUG chip info and registers pretty printed
	char msg[90];

	DW1000.getPrintableDeviceIdentifier(msg);
	m_log::log_vrb(LOG_DW1000, "Device ID: %s", msg);

	DW1000.getPrintableExtendedUniqueIdentifier(msg);
	m_log::log_vrb(LOG_DW1000, "Unique ID: %s", msg);

	char string[6];
	sprintf(string, "%02X:%02X", _ownShortAddress[0], _ownShortAddress[1]);
	m_log::log_vrb(LOG_DW1000, " short: %s", string);

	DW1000.getPrintableNetworkIdAndShortAddress(msg);
	m_log::log_vrb(LOG_DW1000, "Network ID & Device Address: %s", msg);

	DW1000.getPrintableDeviceMode(msg);
	m_log::log_vrb(LOG_DW1000, "Device mode: %s", msg);
	*/

	if(high_power)
		DW1000.high_power_init();

	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
	// for first time ranging frequency computation
	_rangingCountPeriod = millis();
}

boolean DW1000RangingClass::addNetworkDevices(DW1000Device *device)
{
	// if (device->getShortAddress() == 114 ||
	// 	device->getShortAddress() == 115)
	// 	return true;

	// we test our network devices array to check
	// we don't already have it
	uint8_t worstQuality = 0;
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		if (_networkDevices[i].isShortAddressEqual(device))
			return false; // the device already exists

		if (_networkDevices[i].getQuality() < _networkDevices[worstQuality].getQuality())
			worstQuality = i;
	}

	device->setRange(0);
	if (_networkDevicesNumber < MAX_DEVICES)
	{
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
	}
	else
	{
		// Reached max devices count, replace the worst (farthest) one
		if (_handleRemovedDeviceMaxReached != 0)
		{
			(*_handleRemovedDeviceMaxReached)(&_networkDevices[worstQuality]);
		}
		memcpy(&_networkDevices[worstQuality], device, sizeof(DW1000Device));
		_networkDevices[worstQuality].setIndex(worstQuality);
	}
	return true;
}

void DW1000RangingClass::removeNetworkDevices(uint8_t index)
{
	if (index != _networkDevicesNumber - 1) // if we do not delete the last element
	{
		// we replace the element we want to delete with the last one
		memcpy(&_networkDevices[index], &_networkDevices[_networkDevicesNumber - 1], sizeof(DW1000Device));
		_networkDevices[index].setIndex(index);
	}
	_networkDevicesNumber--;
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ########################################################################### */

// setters
void DW1000RangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }

DW1000Device *DW1000RangingClass::searchDistantDevice(byte shortAddress[])
{
	// we compare the 2 bytes address with the others
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		if (memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0)
		{
			// we have found our device !
			return &_networkDevices[i];
		}
	}

	return nullptr;
}

/* ###########################################################################
 * #### Public methods #######################################################
 * ########################################################################### */

void DW1000RangingClass::checkForReset()
{
	if (!_sentAck && !_receivedAck)
	{
		resetInactive();
		return; // TODO cc
	}
}

void DW1000RangingClass::checkForInactiveDevices()
{
	uint8_t inactiveDevicesNum = 0;
	uint8_t inactiveDevices[MAX_DEVICES];
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		if (_networkDevices[i].isInactive())
		{
			inactiveDevices[inactiveDevicesNum++] = i;
			if (_handleInactiveDevice != 0)
			{
				(*_handleInactiveDevice)(&_networkDevices[i]);
			}
		}
	}

	// we need to delete the device from the array:
	for (uint8_t i = 0; i < inactiveDevicesNum; i++)
	{
		removeNetworkDevices(inactiveDevices[i]);
	}
}

MessageType DW1000RangingClass::detectMessageType(byte datas[])
{
	if (datas[0] == FC_1_BLINK)
	{
		return MessageType::BLINK;
	}
	else if (datas[0] == FC_1 && datas[1] == FC_2)
	{
		// we have a long MAC frame message (ranging init)
		return static_cast<MessageType>(datas[LONG_MAC_LEN]);
	}
	else if (datas[0] == FC_1 && datas[1] == FC_2_SHORT)
	{
		// we have a short mac frame message (poll, range, range report, etc..)
		return static_cast<MessageType>(datas[SHORT_MAC_LEN]);
	}
	return MessageType::TYPE_ERROR;
}

uint32_t DEBUGtimePollSent;
uint32_t DEBUGRangeSent;

void DW1000RangingClass::loop()
{
	// we check if needed to reset!
	checkForReset();
	uint32_t currentTime = millis();
	if (currentTime - lastTimerTick > _timerDelay)
	{
		lastTimerTick = currentTime;
		timerTick();
	}

	if (!_sentAck && !_receivedAck)
	{
		if (_replyTimeOfLastPollAck != 0 && currentTime - _timeOfLastPollSent > _replyTimeOfLastPollAck + 3)
		{
			m_log::log_vrb(LOG_DW1000_MSG, "RANGE ON TIMEOUT");
			transmitRange();
		}
	}

	if (_sentAck)
	{
		_sentAck = false;

		MessageType messageType = detectMessageType(sentData);
		switch (messageType)
		{
		case MessageType::POLL:
			m_log::log_dbg(LOG_DW1000_MSG, "POLL");
			break;
		case MessageType::POLL_ACK:
			m_log::log_dbg(LOG_DW1000_MSG, "POLL_ACK");
			break;
		case MessageType::RANGE:
			m_log::log_dbg(LOG_DW1000_MSG, "RANGE");
			break;
		case MessageType::RANGE_REPORT:
			m_log::log_dbg(LOG_DW1000_MSG, "RANGE_REPORT");
			break;
		case MessageType::BLINK:
			m_log::log_dbg(LOG_DW1000_MSG, "BLINK");
			break;
		case MessageType::RANGING_INIT:
			m_log::log_dbg(LOG_DW1000_MSG, "RANGING_INIT");
			break;
		case MessageType::TYPE_ERROR:
			m_log::log_dbg(LOG_DW1000_MSG, "TYPE_ERROR");
			break;
		case MessageType::RANGE_FAILED:
			m_log::log_dbg(LOG_DW1000_MSG, "RANGE_FAILED");
			break;
		};

		if (messageType != MessageType::POLL_ACK && messageType != MessageType::POLL && messageType != MessageType::RANGE)
			return;

		// A msg was sent. We launch the ranging protocol when a message was sent
		if (_type == BoardType::ANCHOR)
		{
			if (messageType == MessageType::POLL_ACK)
			{
				DW1000Device *myDistantDevice = searchDistantDevice(_lastSentToShortAddress);

				if (myDistantDevice)
				{
					// myDistantDevice->noteActivity(); // Not active, just a submission
					DW1000.getTransmitTimestamp(myDistantDevice->timePollAckSent);
				}
			}
		}
		else if (_type == BoardType::TAG)
		{
			if (messageType == MessageType::POLL)
			{
				DW1000Time timePollSent;
				DW1000.getTransmitTimestamp(timePollSent);

				DEBUGtimePollSent = millis();

				// we save the value for all the devices !
				for (uint8_t i = 0; i < _networkDevicesNumber; i++)
				{
					_networkDevices[i].timePollSent = timePollSent;
					_networkDevices[i].hasSentPoolAck = false;
				}
			}
			else if (messageType == MessageType::RANGE)
			{
				DW1000Time timeRangeSent;
				DW1000.getTransmitTimestamp(timeRangeSent);
				// we save the value for all the devices !
				for (uint8_t i = 0; i < _networkDevicesNumber; i++)
				{
					_networkDevices[i].timeRangeSent = timeRangeSent;
				}
			}
		}
	}

	// check for new received message
	if (_receivedAck)
	{
		_receivedAck = false;

		// we read the datas from the modules:
		//  get message and parse
		DW1000.getData(receivedData, LEN_DATA);

		MessageType messageType = detectMessageType(receivedData);

		switch (messageType)
		{
		case MessageType::POLL:
			m_log::log_dbg(LOG_DW1000_MSG, "<=POLL");
			break;
		case MessageType::POLL_ACK:
			m_log::log_dbg(LOG_DW1000_MSG, "<=POLL_ACK");
			break;
		case MessageType::RANGE:
			m_log::log_dbg(LOG_DW1000_MSG, "<=RANGE");
			break;
		case MessageType::RANGE_REPORT:
			m_log::log_dbg(LOG_DW1000_MSG, "<=RANGE_REPORT");
			break;
		case MessageType::BLINK:
			m_log::log_dbg(LOG_DW1000_MSG, "<=BLINK");
			break;
		case MessageType::RANGING_INIT:
			m_log::log_dbg(LOG_DW1000_MSG, "<=RANGING_INIT");
			break;
		case MessageType::TYPE_ERROR:
			m_log::log_dbg(LOG_DW1000_MSG, "<=TYPE_ERROR");
			break;
		case MessageType::RANGE_FAILED:
			m_log::log_dbg(LOG_DW1000_MSG, "<=RANGE_FAILED");
			break;
		};

		// we have just received a BLINK message from tag
		if (messageType == MessageType::BLINK && _type == BoardType::ANCHOR)
		{
			byte shortAddress[2];
			_globalMac.decodeBlinkFrame(receivedData, shortAddress);

			bool knownByTheTag = false;

			uint8_t numberDevices = receivedData[BLINK_MAC_LEN];
			for (uint8_t i = 0; i < numberDevices; i++)
			{
				// we check if the tag know us
				byte shortAddress[2];
				memcpy(shortAddress, receivedData + BLINK_MAC_LEN + 1 + i * 2, 2);
				// we test if the short address is our address
				if (shortAddress[0] == _ownShortAddress[0] &&
					shortAddress[1] == _ownShortAddress[1])
					knownByTheTag = true;
			}

			// we create a new device with the tag
			DW1000Device myTag(shortAddress);
			myTag.setRXPower(DW1000.getReceivePower());
			myTag.setFPPower(DW1000.getFirstPathPower());
			myTag.setQuality(DW1000.getReceiveQuality());

			if (addNetworkDevices(&myTag))
			{
				if (_handleBlinkDevice != 0)
				{
					(*_handleBlinkDevice)(&myTag);
				}
			}

			if (!knownByTheTag)
			{
				// we reply by the transmit ranging init message
				constexpr short slotQty = 7;
				constexpr u_int16_t slotDuration = 2.5 * DEFAULT_REPLY_DELAY_TIME;
				int randomSlot = random(0, slotQty) + 1;
				// randomSlot = (_ownShortAddress[0] % slotQty) + 1;
				u_int16_t delay = slotDuration * randomSlot;
				// Serial.println(delay);
				transmitRangingInit(delay);
			}
			noteActivity();

			_expectedMsgId = MessageType::POLL;
		}
		else if (messageType == MessageType::RANGING_INIT && _type == BoardType::TAG)
		{

			byte address[2];
			_globalMac.decodeShortMACFrame(receivedData, address);
			// we crate a new device with the anchor
			DW1000Device myAnchor(address);
			myAnchor.setRXPower(DW1000.getReceivePower());
			myAnchor.setFPPower(DW1000.getFirstPathPower());
			myAnchor.setQuality(DW1000.getReceiveQuality());

			m_log::log_vrb(LOG_DW1000_MSG, "RANGING_INIT from %x", myAnchor.getShortAddress());

			if (addNetworkDevices(&myAnchor))
			{
				if (_handleNewDevice != 0)
				{
					(*_handleNewDevice)(&myAnchor);
				}
			}

			noteActivity();
		}
		else
		{
			// we have a short mac layer frame !
			byte address[2];
			_globalMac.decodeShortMACFrame(receivedData, address);

			// we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			DW1000Device *myDistantDevice = searchDistantDevice(address);

			// then we proceed to range protocol
			if (_type == BoardType::ANCHOR)
			{
				if (myDistantDevice != nullptr && messageType != _expectedMsgId)
				{
					// unexpected message, start over again (except if already POLL)
					_protocolFailed = true;
				}
				if (messageType == MessageType::POLL)
				{
					if (myDistantDevice == nullptr)
					{
						// we create a new device with the tag
						DW1000Device myTag(address);
						myTag.setRXPower(DW1000.getReceivePower());
						myTag.setFPPower(DW1000.getFirstPathPower());
						myTag.setQuality(DW1000.getReceiveQuality());
						if (addNetworkDevices(&myTag))
						{
							myDistantDevice = &myTag;
							if (_handleNewDevice != 0)
								(*_handleNewDevice)(&myTag);
						}
						else
						{
							return;
						}
					}

					// we receive a POLL which is a broadcast message
					// we need to grab info about it
					uint8_t numberDevices = receivedData[SHORT_MAC_LEN + 1];

					for (uint8_t i = 0; i < numberDevices; i++)
					{
						// we need to test if this value is for us:
						// we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, receivedData + SHORT_MAC_LEN + 2 + i * pollDeviceSize, 2);

						// we test if the short address is our address
						if (shortAddress[0] == _ownShortAddress[0] &&
							shortAddress[1] == _ownShortAddress[1])
						{
							myDistantDevice->noteActivity(); // Poll is for us

							// we grab the replytime which is for us
							uint16_t replyTime = getReplyTimeOfIndex(i);
							memcpy(&replyTime, receivedData + SHORT_MAC_LEN + 2 + 2 + i * pollDeviceSize, 2);

							// on POLL we (re-)start, so no protocol failure
							_protocolFailed = false;

							DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
							// we indicate our next receive message for our ranging protocol
							_expectedMsgId = MessageType::RANGE;
							transmitPollAck(myDistantDevice, replyTime);
							noteActivity();

							return;
						}
					}
					// Remove mydistantdevice, non ci conosce, oppure send ranginginit
					// removeNetworkDevices(myDistantDevice->getIndex());

					int randomSlot = random(0, pollAckTimeSlots - numberDevices);
					uint16_t replyTime = getReplyTimeOfIndex(randomSlot);
					transmitRangingInit(replyTime);
				}
				else if (messageType == MessageType::RANGE)
				{
					if (myDistantDevice == nullptr)
					{
						// we don't have the short address of the device in memory
						m_log::log_err(LOG_DW1000, "Device not found");
						return;
					}

					// we receive a RANGE which is a broadcast message
					// we need to grab info about it
					uint8_t numberDevices = 0;
					memcpy(&numberDevices, receivedData + SHORT_MAC_LEN + 1, 1);

					for (uint8_t i = 0; i < numberDevices; i++)
					{
						// we need to test if this value is for us:
						// we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, receivedData + SHORT_MAC_LEN + 2 + i * rangeDeviceSize, 2);

						// we test if the short address is our address
						if (shortAddress[0] == _ownShortAddress[0] && shortAddress[1] == _ownShortAddress[1])
						{
							myDistantDevice->noteActivity();

							// we grab the replytime which is for us
							DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
							noteActivity();
							_expectedMsgId = MessageType::POLL;

							if (!_protocolFailed)
							{

								myDistantDevice->timePollAckReceivedMinusPollSent.setTimestamp(receivedData + SHORT_MAC_LEN + 4 + rangeDeviceSize * i);
								myDistantDevice->timeRangeSentMinusPollAckReceived.setTimestamp(receivedData + SHORT_MAC_LEN + 9 + rangeDeviceSize * i);

								// myDistantDevice->timePollSent.setTimestamp(receivedData + SHORT_MAC_LEN + 4 + 17 * i);
								// myDistantDevice->timePollAckReceived.setTimestamp(receivedData + SHORT_MAC_LEN + 9 + 17 * i);
								// myDistantDevice->timeRangeSent.setTimestamp(receivedData + SHORT_MAC_LEN + 14 + 17 * i);

								// (re-)compute range as two-way ranging is done
								DW1000Time myTOF;
								computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM

								float distance = myTOF.getAsMeters();

								myDistantDevice->setRange(distance);

								myDistantDevice->setRXPower(DW1000.getReceivePower());
								myDistantDevice->setFPPower(DW1000.getFirstPathPower());
								myDistantDevice->setQuality(DW1000.getReceiveQuality());

								if (ENABLE_RANGE_REPORT)
								{
									uint16_t replyTime = getReplyTimeOfIndex(i);

									// we send the range to TAG
									transmitRangeReport(myDistantDevice, replyTime);
								}

								// we have finished our range computation. We send the corresponding handler
								if (_handleNewRange != 0)
								{
									(*_handleNewRange)(myDistantDevice);
								}
							}
							// else
							// {
							// 	transmitRangeFailed(myDistantDevice);
							// }

							return;
						}
					}
				}
			}
			else if (_type == BoardType::TAG)
			{
				if (myDistantDevice == nullptr)
				{
					// we don't have the short address of the device in memory
					m_log::log_err(LOG_DW1000, "Device not found");
					return;
				}

				myDistantDevice->noteActivity();
				// get message and parse
				if (messageType != _expectedMsgId)
				{
					// unexpected message, start over again
					// not needed ?
					return;
					_expectedMsgId = MessageType::POLL_ACK;
					return;
				}
				// we test if the short address is our address
				if (receivedData[6] != _ownShortAddress[0] ||
					receivedData[5] != _ownShortAddress[1])
				{
					return;
				}

				if (messageType == MessageType::POLL_ACK)
				{
					DW1000.getReceiveTimestamp(myDistantDevice->timePollAckReceived);
					// we note activity for our device:
					myDistantDevice->noteActivity();
					myDistantDevice->hasSentPoolAck = true;

					// Serial.println(DW1000.getReceivePower());
					// Serial.println(DW1000.getFirstPathPower());
					// Serial.println(DW1000.getReceiveQuality());

					// in the case the message come from our last device:
					if (_replyTimeOfLastPollAck != 0 && myDistantDevice->getShortAddress() == _addressOfExpectedLastPollAck)
					{
						m_log::log_vrb(LOG_DW1000_MSG, "RANGE LAST POLLACK");
						transmitRange();

						DEBUGRangeSent = millis();
						m_log::log_vrb(LOG_DW1000_MSG, "MILLIS: %d", DEBUGRangeSent - DEBUGtimePollSent);
					}
				}
				else if (messageType == MessageType::RANGE_REPORT)
				{
					float curRange;
					memcpy(&curRange, receivedData + 1 + SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, receivedData + 5 + SHORT_MAC_LEN, 4);

					// we have a new range to save !
					myDistantDevice->setRange(curRange);
					myDistantDevice->setRXPower(curRXPower);

					// We can call our handler !
					// we have finished our range computation. We send the corresponding handler
					if (_handleNewRange != 0)
					{
						(*_handleNewRange)(myDistantDevice);
					}
				}
				else if (messageType == MessageType::RANGE_FAILED)
				{
					// not needed as we have a timer;
					return;
					_expectedMsgId = MessageType::POLL_ACK;
				}
			}
		}
	}
}

/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ########################################################################### */

void DW1000RangingClass::handleSent()
{
	// status change on sent success
	_sentAck = true;
}

void DW1000RangingClass::handleReceived()
{
	// status change on received success
	_receivedAck = true;
}

void DW1000RangingClass::noteActivity()
{
	// update activity timestamp, so that we do not reach "resetPeriod"
	_lastActivity = millis();
}

void DW1000RangingClass::resetInactive()
{
	// if inactive
	if (millis() - _lastActivity > _resetPeriod)
	{
		if (_type == BoardType::ANCHOR)
		{
			_expectedMsgId = MessageType::POLL;
			receiver();
		}
		noteActivity();
	}
}

void DW1000RangingClass::timerTick()
{
	if (counterForBlink == 0)
	{
		if (_type == BoardType::TAG)
		{
			transmitBlink();
		}
		// check for inactive devices if we are a TAG or ANCHOR
		checkForInactiveDevices();
	}
	else
	{
		if (_networkDevicesNumber > 0 && _type == BoardType::TAG)
		{
			_expectedMsgId = MessageType::POLL_ACK;
			// send a multicast poll
			transmitPoll();
		}
	}
	counterForBlink = (counterForBlink + 1) % BLINK_INTERVAL;
}

void DW1000RangingClass::copyShortAddress(byte to[], byte from[])
{
	to[0] = from[0];
	to[1] = from[1];
}

/* ###########################################################################
 * #### Methods for ranging protocol   #######################################
 * ########################################################################### */

void DW1000RangingClass::transmitInit()
{
	DW1000.newTransmit();
	DW1000.setDefaults();
}

void DW1000RangingClass::transmit(byte datas[])
{
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000RangingClass::transmit(byte datas[], DW1000Time time)
{
	DW1000.setDelay(time);
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000RangingClass::transmitBlink()
{
	// we need to set our timerDelay:
	_timerDelay = _rangeInterval + (uint16_t)(10 * 3 * DEFAULT_REPLY_DELAY_TIME / 1000);

	transmitInit();
	_globalMac.generateBlinkFrame(sentData, _ownShortAddress);

	sentData[BLINK_MAC_LEN] = _networkDevicesNumber;
	for (uint8_t i = 0; i < _networkDevicesNumber; i++)
	{
		memcpy(sentData + BLINK_MAC_LEN + 1 + i * 2, _networkDevices[i].getByteShortAddress(), 2);
	}
	transmit(sentData);

	byte shortBroadcast[2] = {0xFF, 0xFF};
	copyShortAddress(_lastSentToShortAddress, shortBroadcast);
}

void DW1000RangingClass::transmitRangingInit(u_int16_t delay)
{
	transmitInit();
	// we generate the mac frame for a ranging init message
	byte shortBroadcast[2] = {0xFF, 0xFF};
	_globalMac.generateShortMACFrame(sentData, _ownShortAddress, shortBroadcast);
	// we define the function code
	sentData[SHORT_MAC_LEN] = static_cast<byte>(MessageType::RANGING_INIT);

	copyShortAddress(_lastSentToShortAddress, shortBroadcast);

	DW1000Time deltaTime = DW1000Time(delay, DW1000Time::MICROSECONDS);
	transmit(sentData, deltaTime);
}

void DW1000RangingClass::transmitPoll()
{
	transmitInit();

	// we need to set our timerDelay:
	_timerDelay = _rangeInterval + (uint16_t)(pollAckTimeSlots * 3 * DEFAULT_REPLY_DELAY_TIME / 1000); // TODO meglio fermare il timer forse

	uint8_t devicesCount = _networkDevicesNumber < devicePerPollTransmit ? _networkDevicesNumber : devicePerPollTransmit;

	byte shortBroadcast[2] = {0xFF, 0xFF};
	_globalMac.generateShortMACFrame(sentData, _ownShortAddress, shortBroadcast);
	sentData[SHORT_MAC_LEN] = static_cast<byte>(MessageType::POLL);
	// we enter the number of devices
	sentData[SHORT_MAC_LEN + 1] = devicesCount;

	uint8_t freeSlots = pollAckTimeSlots - devicesCount;

	for (uint8_t i = 0; i < devicesCount; i++)
	{
		// each devices have a different reply delay time.
		_networkDevices[i].setReplyTime(getReplyTimeOfIndex(i+freeSlots));

		// we write the short address of our device:
		memcpy(sentData + SHORT_MAC_LEN + 2 + i * pollDeviceSize, _networkDevices[i].getByteShortAddress(), 2);

		// we add the replyTime
		uint16_t replyTime = _networkDevices[i].getReplyTime();
		memcpy(sentData + SHORT_MAC_LEN + 2 + 2 + i * pollDeviceSize, &replyTime, 2);

		_addressOfExpectedLastPollAck = _networkDevices[i].getShortAddress();
	}

	// if (_networkDevicesNumber > 0)
	// 	_replyTimeOfLastPollAck = getReplyTimeOfIndex(_networkDevicesNumber - 1) / 1000;
	_replyTimeOfLastPollAck = getReplyTimeOfIndex(pollAckTimeSlots - 1) / 1000;

	_timeOfLastPollSent = millis();

	copyShortAddress(_lastSentToShortAddress, shortBroadcast);

	transmit(sentData);
}

void DW1000RangingClass::transmitPollAck(DW1000Device *myDistantDevice, u_int16_t delay)
{
	transmitInit();
	_globalMac.generateShortMACFrame(sentData, _ownShortAddress, myDistantDevice->getByteShortAddress());
	sentData[SHORT_MAC_LEN] = static_cast<byte>(MessageType::POLL_ACK);
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(delay, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(sentData, deltaTime);
}

void DW1000RangingClass::transmitRange()
{
	// Disable range send on timeout
	_replyTimeOfLastPollAck = 0;
	_addressOfExpectedLastPollAck = 0;
	_timeOfLastPollSent = 0;

	_expectedMsgId = ENABLE_RANGE_REPORT ? MessageType::RANGE_REPORT : MessageType::POLL_ACK;

	constexpr uint8_t devicePerTransmit = 6;

	uint8_t devicesCount = 0;
	DW1000Device *devices[devicePerTransmit];
	for (uint8_t i = 0; i < _networkDevicesNumber && devicesCount < devicePerTransmit; i++)
	{
		if (_networkDevices[i].hasSentPoolAck)
		{
			devices[devicesCount++] = &_networkDevices[i];
		}
	}

	// we need to set our timerDelay:
	_timerDelay = _rangeInterval + (uint16_t)(devicesCount * 3 * DEFAULT_REPLY_DELAY_TIME / 1000);

	transmitInit();

	byte shortBroadcast[2] = {0xFF, 0xFF};
	_globalMac.generateShortMACFrame(sentData, _ownShortAddress, shortBroadcast);
	sentData[SHORT_MAC_LEN] = static_cast<byte>(MessageType::RANGE);
	// we enter the number of devices
	sentData[SHORT_MAC_LEN + 1] = devicesCount;

	// delay sending the message and remember expected future sent timestamp
	DW1000Time deltaTime = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
	DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);

	for (uint8_t i = 0; i < devicesCount; i++)
	{
		if (ENABLE_RANGE_REPORT)
			// each devices have a different reply delay time.
			devices[i]->setReplyTime(getReplyTimeOfIndex(i));

		// we write the short address of our device:
		memcpy(sentData + SHORT_MAC_LEN + 2 + rangeDeviceSize * i, devices[i]->getByteShortAddress(), 2);

		// we get the device which correspond to the message which was sent (need to be filtered by MAC address)
		devices[i]->timeRangeSent = timeRangeSent;
		devices[i]->timePollAckReceivedMinusPollSent = devices[i]->timePollAckReceived - devices[i]->timePollSent;
		devices[i]->timeRangeSentMinusPollAckReceived = devices[i]->timeRangeSent - devices[i]->timePollAckReceived;
		devices[i]->timePollAckReceivedMinusPollSent.getTimestamp(sentData + SHORT_MAC_LEN + 4 + rangeDeviceSize * i);
		devices[i]->timeRangeSentMinusPollAckReceived.getTimestamp(sentData + SHORT_MAC_LEN + 9 + rangeDeviceSize * i);
	}

	copyShortAddress(_lastSentToShortAddress, shortBroadcast);

	transmit(sentData);
}

void DW1000RangingClass::transmitRangeReport(DW1000Device *myDistantDevice, u_int16_t delay)
{
	transmitInit();
	_globalMac.generateShortMACFrame(sentData, _ownShortAddress, myDistantDevice->getByteShortAddress());
	sentData[SHORT_MAC_LEN] = static_cast<byte>(MessageType::RANGE_REPORT);
	// write final ranging result
	float curRange = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	// We add the Range and then the RXPower
	memcpy(sentData + 1 + SHORT_MAC_LEN, &curRange, 4);
	memcpy(sentData + 5 + SHORT_MAC_LEN, &curRXPower, 4);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(sentData, DW1000Time(delay, DW1000Time::MICROSECONDS));
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device *myDistantDevice)
{
	transmitInit();
	_globalMac.generateShortMACFrame(sentData, _ownShortAddress, myDistantDevice->getByteShortAddress());
	sentData[SHORT_MAC_LEN] = static_cast<byte>(MessageType::RANGE_FAILED);

	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(sentData);
}

void DW1000RangingClass::receiver()
{
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}

uint16_t DW1000RangingClass::getReplyTimeOfIndex(int i)
{
	return (2 * i + 1) * DEFAULT_REPLY_DELAY_TIME;
}

/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ########################################################################### */

void DW1000RangingClass::computeRangeAsymmetric(DW1000Device *myDistantDevice, DW1000Time *myTOF)
{
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceivedMinusPollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent - myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived - myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSentMinusPollAckReceived).wrap();

	myTOF->setTimestamp((round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2));

	/*
	m_log::log_vrb(LOG_DW1000_MSG, "timePollAckReceivedMinusPollSent %d", myDistantDevice->timePollAckReceivedMinusPollSent.getTimestamp());
	m_log::log_vrb(LOG_DW1000_MSG, "round1 %d", (long)round1.getTimestamp());

	m_log::log_vrb(LOG_DW1000_MSG, "timePollAckSent %d", myDistantDevice->timePollAckSent.getTimestamp());
	m_log::log_vrb(LOG_DW1000_MSG, "timePollReceived %d", myDistantDevice->timePollReceived.getTimestamp());
	m_log::log_vrb(LOG_DW1000_MSG, "reply1 %d", (long)reply1.getTimestamp());

	m_log::log_vrb(LOG_DW1000_MSG, "timeRangeReceived %d", myDistantDevice->timeRangeReceived.getTimestamp());
	m_log::log_vrb(LOG_DW1000_MSG, "timePollAckSent %d", myDistantDevice->timePollAckSent.getTimestamp());
	m_log::log_vrb(LOG_DW1000_MSG, "round2 %d", (long)round2.getTimestamp());

	m_log::log_vrb(LOG_DW1000_MSG, "timeRangeSentMinusPollAckReceived %d", myDistantDevice->timeRangeSentMinusPollAckReceived.getTimestamp());
	m_log::log_vrb(LOG_DW1000_MSG, "reply2 ", (long)reply2.getTimestamp());
	*/
}