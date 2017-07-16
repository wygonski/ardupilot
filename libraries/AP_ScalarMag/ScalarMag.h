/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __SCALARMAG_H__
#define __SCALARMAG_H__


class ScalarMag 
{
public:
    /* ScalarMag public interface: */
	
    // init - initialize the TFM sensor
    bool            init();
	
	// updates() - send command to the sensor to turn on/off streaming of sensor data
	bool			updates(bool state);	
	
	// read() - called periodically to obtain ASCII data from sensor and parse it into packets
	//			returns true if a packet is decoded and new int32_t data is available 
	bool			read();
	
	// readSimulated() - schedule this for SITL-like testing of ScalarMag driver
	//						packets returned comprise a periodic waveform for testing 
	//						returns true if a packet is decoded and new int32_t data is available 
	bool			readSimulated();
	
	// serialNumber - obtains the serial number string of the TFM
	bool			serialNumber();
	
	// sensorNumber - obtains the sensor description string of the TFM
	bool			sensorDescription();
	
	// serialNumber - obtains the firmware revision string of the TFM
	bool			firmwareRevision();
	
    // healthy - return true if the sensor is healthy
	bool			healthy;
	
	// public properties
	
	// magData - 
	int32_t			magData;
	// signalStrength - 
	int32_t			signalStrength;
	// cycleCount
	int32_t			cycleCount;
	
	// float			results[6];
	
	char		snString[40];		// TFM serial number
	
	char		sdString[40];		// TFM sensor description
	
	char		fwrString[40];		// TFM firmware revision
	
private:
	int _state;
	
};

#endif // __SCALARMAG_H__
