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
 *       AP_ScalarMag.cpp - QuSpin TFM sensor class
 *       Sensor is connected to PixHawk port Serial 4
 *
 */

// AVR LibC Includes
#include <inttypes.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_ScalarMag/AP_ScalarMag.h>

extern const AP_HAL::HAL& hal;

// Private Methods ///////////////////////////////////////////////////////////////
void _updates(bool state) {
    if (state) {
        // _sendCommand("\"");
    }
    else {
        // _sendCommand("!");
    }
}


/*
  AP_ScalarMag constructor
 */
AP_ScalarMag::AP_ScalarMag()
{
    //AP_Param::setup_object_defaults(this, var_info);
}

// Public Methods //////////////////////////////////////////////////////////////
bool AP_ScalarMag::read(void)
{
	bool retVal = false;
	
	// read from uart Rx, process and return data if present
	// set retVal to something like: # chars available, or if packetizing 
	// is done here, return true (data available)) and load the data into the int32 data words
	// which is part of the public interface

	return retVal;
}
bool AP_ScalarMag::readSimulated(void) {
	
	// quick and dirty periodic function
	//magData = (magData +2) % 4000;
	
	return true;
}

bool AP_ScalarMag::init(void)
{
    uint8_t buff[24];

    pktReceived = false;

	this->updates(false);
	// psuedocode:
	// _SendCommandForStringResult("png");
	// if (_stringResult == "OK") {  healthy = true;  }
	// else {  healthy = false }
	healthy = true;
	
	this->updates(true);
    return true;
}

bool AP_ScalarMag::updates(bool state) {
    _updates(state);
    return (true);
}
	
bool AP_ScalarMag::serialNumber() {
	return false;
}
	
bool AP_ScalarMag::sensorDescription() {
	return false;
}
	
bool AP_ScalarMag::firmwareRevision() {
	return false;
}





