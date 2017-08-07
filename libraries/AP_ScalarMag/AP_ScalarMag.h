/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __SCALARMAG_H__
#define __SCALARMAG_H__

#define DELIMITER_MAGDATA           '!'
#define DELIMITER_SIGNALSTRENGTH    '@'
#define DELIMITER_CYCLECOUNT        '^'

class AP_ScalarMag
{
public:
    /* ScalarMag public interface: */

    // constructor
	AP_ScalarMag();
	
    // init - initialize the TFM sensor
    bool            init(void);

    // TFM sensor value in nT * 2**31  (verify this)
    uint32_t getMagData(void) const { return tfmSensor.magData; }

    // range 0-100
    uint16_t getSignalStrength(void) const {return tfmSensor.signalStrength;}

    // value increases by 1 for each packet received from TFM.  Useful to detect if packets were dropped
    uint32_t getCycleCounter(void) const { return tfmSensor.cycleCount; }

	// updates() - send command to the sensor to turn on/off streaming of sensor data
	bool			updates(bool state);	
	
	// read() - called periodically to obtain ASCII data from sensor and parse it into packets
	//			returns true if a packet is decoded and new int32_t data is available 
	static bool			read(void);
	
	// readSimulated() - schedule this for SITL-like testing of ScalarMag driver
	//						packets returned comprise a periodic waveform for testing 
	//						returns true if a packet is decoded and new int32_t data is available 
	bool			readSimulated(void);
	
	// serialNumber - obtains the serial number string of the TFM
	bool			serialNumber();
	
	// sensorNumber - obtains the sensor description string of the TFM
	bool			sensorDescription();
	
	// serialNumber - obtains the firmware revision string of the TFM
	bool			firmwareRevision();
	
    // healthy - return true if the sensor is healthy
	bool			healthy;

    bool    pktReceived;

	// public properties
	
	// these are now in a struct so they're accessible in the DataFlash log framework
	// magData - 
	//int32_t			magData;
	// signalStrength - 
	//int16_t			signalStrength;
	// cycleCount
	//int32_t			cycleCount;
	
	// float			results[6];
	
	char		snString[40];		// TFM serial number
	
	char		sdString[40];		// TFM sensor description
	
	char		fwrString[40];		// TFM firmware revision
	
	// this should be private, but need to access the string (for now) directly from usercode
    struct sensor {
        char        rawMagData[64];
        uint32_t    magData;         // convert to nT
        uint16_t    signalStrength;  // range 0-100
        uint32_t    cycleCount;      // increasing counter
    } tfmSensor;

private:
	int _state;



};

#endif // __SCALARMAG_H__
