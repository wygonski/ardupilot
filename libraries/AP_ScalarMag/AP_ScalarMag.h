/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __SCALARMAG_H__
#define __SCALARMAG_H__

/*================
class Packet {
public:
    // Handles exchange and parsing of incoming packets consisting of
    // 1-character header plus ASCII string rep of integer, terminated by CRLF (e.g. "!8380618") or
    // "!8380618@99^1234567"
    //

    // constructor
    Packet();

    // public Packet() {
      void pktInit(void);
*/

    /**
     * @brief   Call this method passing a group of characters from the input stream.
     *          They will be added to the ring buffer for parsing into packets
     * @param inBuf
     * @param nChars
     */
//    void enqChars (char *inBuf, int nChars);
    // was      void enqBytes (byte[] inBuf, int nChars);

    /**
     * @brief
     *          If a full packet is found, the packet will be parsed into its constituent parts.
     *          If a packet is found and there are characters remaining in the ring buffer, they are left
     *          for subsequent parsing as they are probably part of the next packet.
     * @return Returns true if a full packet was detected and parsing results are available, and false if
     *          more characters are needed to form a full packet.
     */
//    bool parseMessage(void);


    /*
     *  Put a received character in the circular buffer.
     */
//    void  serPutRxChar(char c);

    /**
     *  @brief This function is called by the background app to see if any character is available from the
     *  communications buffer.  If at least one character is available, the function returns
     *  false, true otherwise.
     */
//    bool serIsRxEmpty(void);


    /*
  string   rxMessage;
*/

/**
 *  @brief This function is called by the application to see if any more characters can be placed
 *  in the Tx buffer (check to see if the Tx buffer is full).
 *  If the buffer is full, the function returns true, false otherwise.
 */    /*===========================================================================*/
//bool serIsTxFull(void);

/**
 *   @brief Called to send a character on the communications
 *   channel.  The character to send is first inserted into the Tx buffer and will be sent by
 *   the Tx thread.
 */
//int serPutChar(char  c);


//char  _serGetTxChar(void);

/**
 *      @brief This function is called by the application to obtain a character from the
 *      communications channel.
 *
 *      Returns the next character in the Rx buffer or NUL if the buffer is empty.
 */
//char serGetChar(void);

//bool _isSingleCharHeader(char ch);

/*===========================================================================*
*
*===========================================================================*/

//int pktReceive(void );

/*===========================================================================*
*
*===========================================================================*/

//int pktTransmit(void);

/*===========================================================================*
*   This is the thread that handles all packet transactions.
*   ARGUMENTS:
*    arg is unused, but its declaration is necessary to conform to template for function pointers
*===========================================================================*/
//void    tskPktMgr (void);

/*=====================
};
====================*/

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
	
	// this should be  , but need to access the string (for now) directly from usercode
    struct sensor {
        uint8_t     rawMagData[64];
        uint32_t    magData;         // convert to nT
        uint16_t    signalStrength;  // range 0-100
        uint32_t    cycleCount;      // increasing counter
        uint8_t     *rawMagDataPtr;  // points to the array of received chars
        uint32_t    rawMagDataSize;
    } tfmSensor;

private:
	int _state;



};

#endif // __SCALARMAG_H__
