/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *       ScalarMag.cpp - QuSpin TFM sensor class
 *       Sensor is connected to UART port (which one?)
 *
 */

// AVR LibC Includes
#include <inttypes.h>

#include <AP_HAL.h>
#include "ScalarMag.h"

extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////
bool ScalarMag::read()
{
	bool retVal = false;
	
	// read from uart Rx, process and return data if present
	// set retVal to something like: # chars available, or if packetizing 
	// is done here, return true (data available)) and load the data into the int32 data words
	// which is part of the public interface

	return retVal;
}
bool ScalarMag::readSimulated() {
	
	// quick and dirty periodic function
	magData = (magData +2) % 4000;
	
	return true;
}

bool ScalarMag::init()
{
    uint8_t buff[24];

	_updates(false);
	// psuedocode:
	// _SendCommandForStringResult("png");
	// if (_stringResult == "OK") {  healthy = true;  }
	// else {  healthy = false }
	healthy = true;
	
	_updates(true);
    return true;
}

bool			updates(bool state) {
    _updates(state);
    return (true);
}
	
bool			serialNumber() {
	return false;
}
	
bool			sensorDescription() {
	return false;
}
	
bool			firmwareRevision() {
	return false;
}

void _updates(bool state) {
	if (state) { 
		// _sendCommand("\");  
	}
	else { 
		// _sendCommand("!"); 
	}
}




