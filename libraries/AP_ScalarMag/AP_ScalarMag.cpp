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

/**
 *  Constructor
 */
// public Packet() {
  void Packet::pktInit(void) {
//how does this ctor get executed?  we probably need an explicit pktInit(); again
    // Initialize the Tx and Rx message buffers
    //TxMsgBuf = new [TX_MSG_BUF_SIZE];
    for (int i=0; i<TX_MSG_BUF_SIZE; i++) TxMsgBuf[i]=0;
    idxTxDataBuf = 0;

    //RxMsgBuf = new [2][RX_MSG_BUF_SIZE];
    for (int i=0; i<RX_MSG_BUF_SIZE; i++) {
        RxMsgBuf[0][i]=0;
        RxMsgBuf[1][i]=0;
    }
    idxRxMsgRcvd = 0;       // pRxMsgRcvd=&RxMsgBuf[0][0];
    idxRxMsgBufWr = 0;  // pRxMsgBufWr=&RxMsgBuf[0][0];
    idxRxMsgValid = 1;  // pRxMsgValid=&RxMsgBuf[1][0];

    /* Initialize the ring buffer   */
    //RingBufRx = new [SER_RX_BUF_SIZE];
    //RingBufTx = new [SER_TX_BUF_SIZE];
    RingBufRxCtr    = 0;
    RingBufRxInIdx  = 0;
    RingBufRxOutIdx = 0;
    RingBufTxCtr    = 0;
    RingBufTxInIdx  = 0;
    RingBufTxOutIdx = 0;

    for (int i=0; i< SER_RX_BUF_SIZE; i++) { RingBufRx[i] = (char) NUL;}
    for (int i=0; i< SER_TX_BUF_SIZE; i++) { RingBufTx[i] = (char) NUL;}

    pktTxState = TX_IDLE;   // PTS
    pktRxState = RX_HDR;    // PRS
    pktState = OFFLINE;     // PS

    bCmdPktReadyToSend = false;
    bPktReceived = false;

    //Command      = new [COMMAND_BUF_SIZE];
    //Payload      = new [PAYLOAD_BUF_SIZE];
    //AllButHeader = new [PAYLOAD_BUF_SIZE];

}


/**
 * @brief   Call this method passing a group of characters from the input stream.
 *          They will be added to the ring buffer for parsing into packets
 * @param inBuf
 * @param nChars
 */
void Packet::enqChars (char * inBuf, int nChars) {
    for (int i = 0; i<nChars; i++) serPutRxChar(inBuf[i]);     // add all the received chars to the message processor
}

/**
 * @brief
 *          If a full packet is found, the packet will be parsed into its constituent parts.
 *          If a packet is found and there are characters remaining in the ring buffer, they are left
 *          for subsequent parsing as they are probably part of the next packet.
 * @return Returns true if a full packet was detected and parsing results are available, and false if
 *          more characters are needed to form a full packet.
 */
  bool Packet::parseMessage() {
    while (!bPktReceived && !serIsRxEmpty()) tskPktMgr();   // parse incoming characters until exhausted or until a packet is found
    if (bPktReceived) {
        //Log.i(TAG, ".. " + Header);
    }
    return(bPktReceived);
}


/**
 *  @brief This function is called by the application to see if any more characters can be placed
 *  in the Tx buffer (check to see if the Tx buffer is full).
 *  If the buffer is full, the function returns true, false otherwise.
 */    /*===========================================================================*/
  bool Packet::serIsTxFull()
{
    bool        full;

    if (RingBufTxCtr < SER_TX_BUF_SIZE) {           /* See if buffer is full                    */
        full = false;
    } else {
        full = true;
    }
    return (full);
}

/**
 *   @brief Called to send a character on the communications
 *   channel.  The character to send is first inserted into the Tx buffer and will be sent by
 *   the Tx thread.
 */
  int Packet::serPutChar(char  c)
{

    //while(true == serIsTxFull()); cannot block here
    if ( RingBufTxCtr < SER_TX_BUF_SIZE) {           /* See if buffer is full                    */
        RingBufTxCtr++;                             /* No, increment character count            */
        RingBufTx[RingBufTxInIdx++] = c;
        if ( RingBufTxInIdx ==  SER_TX_BUF_SIZE-1) { /* Wrap IN pointer           */
             RingBufTxInIdx = 0;
        }
        return (SER_NO_ERR);
    } else {
        return (SER_TX_FULL);
    }
}



char    Packet::_serGetTxChar()
{
    char    c;

    if ( RingBufTxCtr > 0) {                          /* See if buffer is empty                   */
         RingBufTxCtr--;                              /* No, decrement character count            */
        c =  RingBufTx[RingBufTxOutIdx++];
        if ( RingBufTxOutIdx == SER_TX_BUF_SIZE-1) {     /* Wrap OUT pointer     */
             RingBufTxOutIdx = 0;
        }
        return (c);                                        /* Characters are still available           */
    } else {
        return (NUL);                                      /* Buffer is empty                          */
    }
}



/**
 *  @brief This function is called by the background app to see if any character is available from the
 *  circular buffer.  If at least one character is available, the function returns
 *  false, true otherwise.
 */
  bool Packet::serIsRxEmpty()
{
    bool empty;

    if ( RingBufRxCtr > 0) {                          /* See if buffer is empty                   */
        empty = false;                                     /* Buffer is NOT empty                      */
    } else {
        empty = true;                                      /* Buffer is empty                          */
    }
    return (empty);

}


/**
 *      @brief This function is called by the application to obtain a character from the
 *      communications buffer.
 *
 *      Returns the next character in the Rx buffer or NUL if the buffer is empty.
 */
char   Packet::serGetChar()
{
    char  c;
//    int incomingByte = 0;

    if ( RingBufRxCtr > 0) {                          /* See if buffer is empty                   */
        RingBufRxCtr--;                              /* No, decrement character count            */
        c = RingBufRx[RingBufRxOutIdx++];
        if ( RingBufRxOutIdx == SER_RX_BUF_SIZE-1) {     /* Wrap OUT pointer     */
             RingBufRxOutIdx = 0;
        }
        return (c);                                        /* Characters are still available           */
    } else {
        return (NUL);                                      /* Buffer is empty                          */
    }
}


/*
 * Adds a received character to the circular buffer
 */
  void  Packet::serPutRxChar(char  c)
// was__inline void  serPutRxChar( c)
{
    if ( RingBufRxCtr < SER_RX_BUF_SIZE) {           /* See if buffer is full                    */
         RingBufRxCtr++;                             /* No, increment character count            */
         RingBufRx[RingBufRxInIdx++] = c;
        if ( RingBufRxInIdx == SER_RX_BUF_SIZE-1) { /* Wrap IN pointer           */
             RingBufRxInIdx = 0;
        }
    }
}

  /*
String Packet::pktGetPacket() {
bPktReceived = false;
return rxMessage;
}
*/

  bool Packet::_isSingleCharHeader(char ch) {
    bool bResult = false;

    for (int i=0; i<headerCount; i++) {
        if (_headerChars[i] == ch) bResult = true;
    }
    return bResult;
}

/*===========================================================================*
*
*   pktReceive()
*
*   DESCRIPTION:
*   Receive, validate, and acknowledge a command
*
*   ARGUMENTS:
*
*   RETURN VALUE:
*   Returns the state of the receiver.
*
*===========================================================================*/

  int Packet::pktReceive( )
{
    int Result;
    bool bResult;
    char    ch;
    int tempValue;

//        extern unsigned int hRxMsgPipe;

    while ( !(bResult = serIsRxEmpty()) )
    {
        ch=serGetChar();
        switch (pktRxState)
        {
            case RX_HDR:
                if ((ch == DATA_RX_HEADER)  ) { // find the data header first because we need to process streaming data faster than command responses
                    Header = ch;
                    pktRxState=RX_CR;   // will hunt for terminator while loading message buffer
                    idxRxMsgBufWr = 0;  //pRxMsgBufWr=pRxMsgRcvd;   // prep to load message
                    RxCount = 0;
                    //Log.i(TAG, " " + Header);
                }
                else if ((ch == PKT_RX_HEADER) || ( _isSingleCharHeader(ch)) ) {
                    Header = ch;
                    pktRxState=RX_CR;   // will hunt for terminator while loading message buffer
                    idxRxMsgBufWr = 0;  //pRxMsgBufWr=pRxMsgRcvd;   // prep to load message
                    RxCount = 0;
                    //Log.i(TAG, " " + Header);
                    // @TODO start a timer to prevent the other end from wedging the receiver
                }
                break;
            case RX_CR:   //  hunt for RX_CR while collecting cmd message
                if ( (RxCount < MAX_MSG_LEN))
                {
                    if (ch == 0x0D ) {
                        pktRxState=RX_LF;
                        rxMsgSize = RxCount;
                        RxMsgBuf[idxRxMsgRcvd][idxRxMsgBufWr++] = (char)'\0';   // this terminates the characters in the buffer annd saves us from the overhead of clearing it.
                    }
                    else {
                        RxMsgBuf[idxRxMsgRcvd][idxRxMsgBufWr++] = ch;   // *pRxMsgBufWr++=ch;       // load the message buffer
                        RxCount++;
                    }
                }
                else
                {
//                        #ifdef PKT_DEVICE
//                        serPutChar('-');         // send a NAK
                    pktRxState=RX_HDR;
                    //Log.i(TAG,"Packet error");
//                        #endif
                }
                break;

            case RX_LF:
                if ( ch == 0x0A )
                {
                    if (Header == DATA_RX_HEADER) { // !<integer string>CRLF or !<integer string>@<integer string>^<integer string>CRLF
                        Command[0] = (char)0;
                        for (int i=0; i<rxMsgSize; i++) AllButHeader[i] = RxMsgBuf[idxRxMsgRcvd][i];
                        AllButHeader[rxMsgSize] = 0;     // terminate with ASCII null
                        sizeAllButHeader = rxMsgSize+1;
                        /*  no longer needed?
                        for (int i=0; i<rxMsgSize; i++) Payload[i] = RxMsgBuf[idxRxMsgRcvd][i];
                        Payload[rxMsgSize] = 0;     // terminate with ASCII null
                        */
                    }
                    else if (Header==PKT_RX_HEADER) {    // 3-char command packet
                        for (int i=0; i<3; i++) Command[i] = RxMsgBuf[idxRxMsgRcvd][i];
                        Command[4] = 0;
                        for (int i=3; i<rxMsgSize; i++) Payload[i-3] = RxMsgBuf[idxRxMsgRcvd][i];
                        Payload[rxMsgSize-3] = 0;     // terminate with ASCII null
                    }
                    else {
                        Command[0] = (char)0;
                        for (int i=0; i<rxMsgSize; i++) AllButHeader[i] = RxMsgBuf[idxRxMsgRcvd][i];
                        AllButHeader[rxMsgSize] = 0;     // terminate with ASCII null
                    }
                    idxTemp = idxRxMsgValid;
                    idxRxMsgValid = idxRxMsgRcvd;
                    idxRxMsgRcvd = idxTemp;
                    bPktReceived = true;
                    pktRxState=RX_HDR;
                    //Log.i(TAG, ". " + Header);
                    // return here with a full packet, otherwise parsing continues and can
                    // overwrite packets if there's many more characters qued
                    return((int)pktRxState);
                }
                else
                {
//                serPutChar('-');         // send a NAK
                    pktRxState=RX_HDR;
                    //Log.i(TAG,"Packet error");
                }
                break;
        } // switch END
    } // while END
    return((int)pktRxState);
}


/*===========================================================================*
*
*   pktTransmit()
*
*   DESCRIPTION:
*
*   ARGUMENTS:
*
*   RETURN VALUE:
*   none
*
*===========================================================================*/

  int Packet::pktTransmit()
{
    char    ch;
    int Result;


    switch (pktTxState)
    {
        case TX_IDLE:
            idxTxDataBuf = 0;   // pTxDataBuf = &TxMsgBuf[0];
            TxCount=0;
            pktTxState=TX_HDR;
            // fall through
        case TX_HDR:
            ch = PKT_TX_HEADER;
            if ( (Result=serPutChar(ch)) != SER_NO_ERR)  return(pktTxState);    // busy, wait til next time
            else  {
                pktTxState = TX_MSG;
            }
            // fall through
        case TX_MSG:
            while ( TxCount < TxMsgSize )  {
                if ( (Result=serPutChar(TxMsgBuf[idxTxDataBuf])) != SER_NO_ERR) return(pktTxState); // busy, wait til next time
                idxTxDataBuf++;
                TxCount++;
            }
            pktTxState = TX_CR;
            TxCount=0;
            //break; fall through so crlf is appended and sent out ahead of a competing output message e.g. SerSendFormatted
        case TX_CR:
//                if ( (Result=serPutChar(0x0D)) != SER_NO_ERR)  return(pktTxState);    // busy, wait til next time
            if ( (Result=serPutChar((char)'\r')) != SER_NO_ERR)  return(pktTxState);    // busy, wait til next time
            else pktTxState=TX_LF;
        case TX_LF:
//                if ( (Result=serPutChar(0x0A)) != SER_NO_ERR)  return(pktTxState);    // busy, wait til next time
            if ( (Result=serPutChar((char)'\n')) != SER_NO_ERR)  return(pktTxState);    // busy, wait til next time
            else {
                pktTxState=TX_IDLE;
                TxMsgBuf[0]= '\0';      // will clear the string that was in the msg buffer
            // Notify writer that we're done
                bCmdPktReadyToSend = false;
                return(pktTxState);
            }
            //break;

    } // switch END
    return(pktTxState);
}


/*===========================================================================*
*
*   tskPktMgr()
*
*   DESCRIPTION:
*   This is the thread that handles all packet transactions.
*
*   ARGUMENTS:
*    arg is unused, but its declaration is necessary to conform to template for function pointers
*
*   RETURN VALUE:
*   none
*
*===========================================================================*/

  void    Packet::tskPktMgr (void) {
    int i;
    bool bResult;
    int Result;
    //int Err;

    switch (pktState) {
        case OFFLINE:
            bCmdPktReadyToSend = false;
            bPktReceived = false;
            pktState = IDLE;
        // start receiving characters now
            //serRxIntEn();

            break;
        case IDLE:
            // check to see if we have the start of an incoming packet:
            if (!(bResult = serIsRxEmpty())) {
                //if (PKT_TX_HEADER != Serial.peek()) {    // not an incoming pkt, dispatch the other commnad processor
                //    comm();
                //}
                //else
                if ((Result = pktReceive()) != RX_HDR) {
                    pktState = RCV;
                }
                else {
                    pktState = IDLE;
                }
            } else {
                if (bCmdPktReadyToSend) {
                    if ((Result = pktTransmit()) != TX_IDLE) pktState = XMT;
                    else pktState = IDLE;
                }
            }
            break;
        case XMT:
            if ((Result = pktTransmit()) != TX_IDLE) pktState = XMT;
            if (pktTxState == TX_IDLE) {
                //#ifdef PKT_CONTROLLER
                TimerCount = 0;          // start the timer
                pktState = WAIT;          // wait for response from peripheral device
                //#else
                //pktState=IDLE;          // allow Rx now that we're done
                //#endif
            }
            break;
        case RCV:
            if ((Result = pktReceive()) != 0) pktState = RCV;
            else pktState = IDLE;  // receiver is hunting for a header
            break;
        //#ifdef PKT_CONTROLLER
        case WAIT:
            if ((bResult = serIsRxEmpty()) == false) {
                // check the buffer for '+' or '-', and report the result
                // if neither + or -, we have an error
                pktState = IDLE;
            } else {
                if (++TimerCount > MAX_TIMER_COUNT) {
                    // report the timeout error
                    pktState = IDLE;
                }
            }
            break;
        //#endif
    }
    //Log.i(TAG, "tskPktMgr returning");
}







