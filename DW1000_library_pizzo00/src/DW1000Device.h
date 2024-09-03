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
 * @file DW1000Device.h
 * Arduino global library (header file) working with the DW1000 library
 * for the Decawave DW1000 UWB transceiver IC.
 *
 */

#define INACTIVITY_TIME 2000

#ifndef _DW1000Device_H_INCLUDED
#define _DW1000Device_H_INCLUDED

#include "DW1000Time.h"

class DW1000Device
{
public:
	// Constructor and destructor
	DW1000Device();
	DW1000Device(byte shortAddress[]);
	~DW1000Device();

	// Setters
	void setShortAddress(byte address[]);
	void setIndex(uint8_t index) { _index = index; }
	void setRange(float range) { _range = range; }
	void setRXPower(float power) { _RXPower = power; }
	void setFPPower(float power) { _FPPower = power; }
	void setQuality(float quality) { _quality = quality; }
	void setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUs = replyDelayTimeUs; }

	// Getters
	uint8_t getIndex() { return _index; }
	byte *getByteShortAddress() { return _shortAddress; }
	uint16_t getShortAddress();
	uint16_t getReplyTime() { return _replyDelayTimeUs; }

	float getRange() { return _range; }
	float getRXPower() { return _RXPower; }
	float getFPPower() { return _FPPower; }
	float getQuality() { return _quality; }

	boolean isAddressEqual(DW1000Device *device);
	boolean isShortAddressEqual(DW1000Device *device);

	// functions which contains the date: (easier to put as public)
	//  timestamps to remember
	DW1000Time timePollSent;
	DW1000Time timePollReceived;
	DW1000Time timePollAckSent;
	DW1000Time timePollAckReceived;
	DW1000Time timeRangeSent;
	DW1000Time timeRangeReceived;

	bool hasSentPoolAck;

	DW1000Time timePollAckReceivedMinusPollSent;
	DW1000Time timeRangeSentMinusPollAckReceived;

	void noteActivity();
	boolean isInactive();

private:
	byte _shortAddress[2];
	unsigned long _activity;
	uint16_t _replyDelayTimeUs;
	uint8_t _index;

	float _range;
	float _RXPower;
	float _FPPower;
	float _quality;
};

#endif
