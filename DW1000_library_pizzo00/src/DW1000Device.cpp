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
 * @file DW1000Device.cpp
 * Arduino global library (source file) working with the DW1000 library
 * for the Decawave DW1000 UWB transceiver IC.
 *
 */

#include "DW1000Device.h"
#include "DW1000.h"

// Constructor and destructor
DW1000Device::DW1000Device()
{
	noteActivity();
}

DW1000Device::DW1000Device(byte shortAddress[])
{
	// we set the 2 bytes address
	setShortAddress(shortAddress);
	noteActivity();
}

DW1000Device::~DW1000Device() { }

// setters:
void DW1000Device::setShortAddress(byte deviceAddress[]) { memcpy(_shortAddress, deviceAddress, 2); }

uint16_t DW1000Device::getShortAddress()
{
	return _shortAddress[1] * 256 + _shortAddress[0];
}

boolean DW1000Device::isShortAddressEqual(DW1000Device *device)
{
	return memcmp(this->getByteShortAddress(), device->getByteShortAddress(), 2) == 0;
}

void DW1000Device::noteActivity()
{
	_activity = millis();
}

boolean DW1000Device::isInactive()
{
	// One second of inactivity
	return millis() - _activity > INACTIVITY_TIME;
}
