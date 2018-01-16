#include "Copter.h"
#include <DataFlash/DataFlash.h>          // ArduPilot Mega Flash Memory Library

#include <AP_ScalarMag/AP_ScalarMag.h>
/////////////////////////////////////////////
#define DELIMITER_MAGDATA           '!'
#define DELIMITER_SIGNALSTRENGTH    '@'
#define DELIMITER_CYCLECOUNT        '^'

#define MAX_TIMER_COUNT  10000

#define PKT_RX_HEADER  '$'      // command echo will have '~' as header, so it will be ignored
                                                            // $ is the response, which will be parsed here
#define PKT_TX_HEADER  '~'
#define DATA_RX_HEADER  '!'     // data stream should be handled with expedience

// TODO Changed the Ring buffer sizes from 32 to 256 and the appinfo commands now work, figure out why:
        // because "~asn\r\n$asnquspin9999, offset 3275\r\n" is 35 chars, so you need a bigger ring buffer or it will miss chars
#define SER_RX_BUF_SIZE  256
#define SER_TX_BUF_SIZE  256
#define NUL  0

#define    MAX_MSG_LEN  (64+8)                        /* max chars for the app's message definition  */

#define RX_MSG_BUF_SIZE  64
#define TX_MSG_BUF_SIZE  64

#define COMMAND_BUF_SIZE  16
#define PAYLOAD_BUF_SIZE  64

// parts comprising a parsed received packet
  char   Header;
  char  Command[COMMAND_BUF_SIZE];
  char  Payload[PAYLOAD_BUF_SIZE];       // for 3-char command responses. ASCII chars representing a 32-bit integer, a string, etc. (depends on the command)
  char  AllButHeader[PAYLOAD_BUF_SIZE];  // Contains remainder of |, ~, # responses
  int32_t   sizeAllButHeader;               // num chars in the received message contained in AllButHeader, including the '\0'
  int  payloadUIntValue;
  bool  bPktReceived;

  /*======================
private:
======================*/

  typedef enum {
      OFFLINE,
      IDLE,
      XMT,
      RCV,
      WAIT
  }PS;

  int TimerCount = 0;

// @TODO These are hard-coded for now.  Later, write an add method so that the app can add the header characters that are app-specific
// OTOH, there's so many of them that maybe ard-coding them here makes sense.
char _headerChars[4] = {(char)'#',(char)'$',(char)'~' ,(char)'|' };
int headerCount = 4;

  typedef enum {
     RX_HDR=0,
//     RX_MSG,  // state not handled in switch, will cause compiler error
     RX_CR,
     RX_LF
 }PRS;

 typedef enum {
    TX_IDLE=0,
    TX_HDR,
    TX_MSG,
    TX_CR,
    TX_LF
}PTS;

  volatile PTS pktTxState = TX_IDLE;
  volatile PRS pktRxState = RX_HDR;
  volatile PS pktState = OFFLINE;
  uint16_t RxCount;
  uint16_t TxCount;

typedef enum  {
    SER_NO_ERR =    0,       /**< Function call was successful                       */
    SER_RX_EMPTY,            /**< Rx buffer is empty, no character available         */
    SER_TX_FULL,             /**< Tx buffer is full, could not deposit character     */
    SER_TX_EMPTY             /**< If the Tx buffer is empty.                         */
} SER_ERR;

  char    RxMsgBuf[2][RX_MSG_BUF_SIZE];   // double-buffered
  int idxRxMsgBufWr;
  int idxRxMsgRcvd;
  int idxRxMsgValid;
  int idxTemp;
  int rxMsgSize;

  char    TxMsgBuf[TX_MSG_BUF_SIZE];      // only transmit one msg at a time
  int     TxMsgSize;
  int     idxTxDataBuf;

  bool  bCmdPktReadyToSend;

  int       RingBufRxCtr;    /* Number of characters in the Rx ring buffer              */
  int       RingBufRxInIdx;  /* index where next character will be inserted        */
  int       RingBufRxOutIdx; /* index where next character will be extracted     */
  char      RingBufRx[SER_RX_BUF_SIZE];     /* Ring buffer character storage (Rx)                      */
  int       RingBufTxCtr;    /* Number of characters in the Tx ring buffer              */
  int       RingBufTxInIdx;  /* index where next character will be inserted        */
  int       RingBufTxOutIdx; /* index where next character will be extracted     */
  char      RingBufTx[SER_TX_BUF_SIZE];     /* Ring buffer character storage (Tx)                      */



/**
 *  @brief This function is called by the application to see if any more characters can be placed
 *  in the Tx buffer (check to see if the Tx buffer is full).
 *  If the buffer is full, the function returns true, false otherwise.
 */    /*===========================================================================*/
  bool  serIsTxFull()
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
  int  serPutChar(char  c)
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



char     _serGetTxChar()
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
  bool  serIsRxEmpty()
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
char    serGetChar()
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
  void   serPutRxChar(char  c)
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
String  pktGetPacket() {
bPktReceived = false;
return rxMessage;
}
*/

  bool  _isSingleCharHeader(char ch) {
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

  int  pktReceive( )
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

  int  pktTransmit()
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

  void     tskPktMgr (void) {
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


  void  pktInit(void) {
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

void  enqChars (char * inBuf, int nChars) {
    for (int i = 0; i<nChars; i++) serPutRxChar(inBuf[i]);     // add all the received chars to the message processor
}
*/
/**
 * @brief
 *          If a full packet is found, the packet will be parsed into its constituent parts.
 *          If a packet is found and there are characters remaining in the ring buffer, they are left
 *          for subsequent parsing as they are probably part of the next packet.
 * @return Returns true if a full packet was detected and parsing results are available, and false if
 *          more characters are needed to form a full packet.
 */
  bool  parseMessage() {
    while (!bPktReceived && !serIsRxEmpty()) tskPktMgr();   // parse incoming characters until exhausted or until a packet is found
    if (bPktReceived) {
        //Log.i(TAG, ".. " + Header);
    }
    return(bPktReceived);
}




///////////////////////////////////////////////
// moved this to Copter.h so it has scope in Copter class for UserHook init
//AP_ScalarMag scalarMag;
//Packet pktMag;

/* JJW // Write a ScalarMag packet
static void Log_Write_ScalarMag()
{
    struct log_ScalarMag pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SCALARMAG_MSG),
        val1              : scalarMag.MagData,
        val2              : scalarMag.signalStrength,
        val3              : scalarMag.cycleCount
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}*/


// Write a SCALARMAG packet
// Moved here from LogFile.cpp
// added declaration to Copter.h
void Copter::Log_Write_ScalarMag(AP_ScalarMag &sm, uint64_t time_us)
// was void Copter::Log_Write_ScalarMag(AP_ScalarMag &scalarMag, uint64_t time_us)
{
    Location loc;

    if (time_us == 0) {
        time_us = AP_HAL::micros64();   // @TODO we want GPS time--this is uC clock ticks?
    }
    ahrs.get_position(loc);
    struct log_SCALARMAG logpkt = {
        LOG_PACKET_HEADER_INIT(LOG_SCALARMAG_MSG),
        time_us       : time_us,
//        rawMagData      : { },
        magData         : sm.getMagData(),
        signalStrength  : sm.getSignalStrength(),
        cycleCounter    : sm.getCycleCounter(),
    };
    /* JJW removed writing the raw string to dataflash
    strncpy((char *)logpkt.rawMagData, (char *)sm.tfmSensor.rawMagData, sm.tfmSensor.rawMagDataSize); */
    DataFlash.WriteBlock(&logpkt, sizeof(logpkt));
    // was WriteBlock(&logpkt, sizeof(logpkt));
}


/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}
// JJW setup PixHawk uart for scalar data Tx
static void scalarSetupUart(AP_HAL::UARTDriver *uart, int32_t baudRate)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(baudRate);
    uart->flush();   // likely just waits for serial Tx to finish, if any
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("\r\n>>>4>>>scalarMagTask at %.3f s over %s \n",
                                (double)(AP_HAL::millis() * 0.001f), name);
}
#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    setup_uart(hal.uartC, "uartC");  // uartC is TELEM1
//    setup_uart(hal.uartD, "uartD");  // uartD is TELEM2
    // set up to read scalarMag on serial 4
    scalarSetupUart(hal.uartE, 115200);  // PixHawk Serial 4
    hal.console->printf("\r\n>>scalarSetupUart()-- DONE\r\n ");  // to MP Terminal
    //setup_uart(hal.uartF, "uartF");  // is *not* Serial 5
    UserCount=0;

    // JJW
     pktInit();
    scalarMag.init();
    bFirstTime = true;
    hal.console->printf("\r\n>>scalarMag.init()-- DONE\r\n ");  // to MP Terminal
}
#endif

void Copter::scalarMagTask()
{
    char    chMagData[12];
    char    chSignalStrength[12];
    char    cycleCounter[12];
    char    *pHdr;
    int     charCount;

    if (bFirstTime) {
        bFirstTime = false;
        while (hal.uartE->available()) { hal.uartE->read();}  // clears Rx buffer at startup
        hal.console->printf("\r\n>>Serial Rx cleared -- DONE\r\n ");  // to MP Terminal
        return;
    }

    //uint8_t dataByte = 0;
//////////////////////////////////////////////////
    //char buffer[1024];      // CHECK THIS--DO WE NEED SOMETHING OF THIS SIZE?
    //char * packetAllButHeader;
    //char packetHeader;
    //int bytes;
    uint16_t i;
    uint8_t h1 = '!'; // char
    uint8_t h2 = '@'; // char
    uint8_t h3 = '^'; // char
    int numConverted;

    hal.uartE->write('U');  // DEBUG write a sentinel, monitor on scope
    while (! serIsRxEmpty()) { // this checks the ring buffer
        // parsing...;
        if ( parseMessage()) {   // run tskPktMgr() and pktReceive()
            //Log.i(TAG, " " + pktMag.Header);
            // we have a packet
             bPktReceived = false;
            if ( Header == '!') {
                if ( sizeAllButHeader <= 62) {  // guard against errors causing packet corruption (but error could have occurred earlier in pktMag and crashed)
                    scalarMag.tfmSensor.rawMagDataSize =  sizeAllButHeader;
                    for (i=0; i< sizeAllButHeader; i++)  {
                        scalarMag.tfmSensor.rawMagData[i] =  AllButHeader[i];
                    }
                    // array AllButHeader() now contains <magData>@<signalStrength>^<cycleCounter>\0
                    //                       or contains <magData>@<signalStrength>\0
#if 1   // attempt to parse all data values using sscanf calls...
        /*  NOTE: using any of the scanf will probably be SLOWER than just looping over your buffers to extract strings of
         *      characters [A-Za-z]. This is because, with any of the scanf functions, the code first needs to parse
         *      your format string to figure out what you're looking for, and then actually parse the input.
         */
                    pHdr = NULL;
                    pHdr = strchr(AllButHeader, (int)h3);
                    if (pHdr != NULL) { // string contains <magData>@<signalStrength>^<cycleCounter>\0
                        numConverted = sscanf( AllButHeader, "%s@%s^%d", chMagData, chSignalStrength, &scalarMag.tfmSensor.cycleCount ) ;  // last one is already terminated with '\0' so convert directly
                        // was numConverted = sscanf( AllButHeader, "%[^@]@%[^^]^%d", chMagData, chSignalStrength, &scalarMag.tfmSensor.cycleCount ) ;  // last one is already terminated with '\0' so convert directly
                        if (numConverted == 3) {
                            sscanf(chMagData, "%d", &scalarMag.tfmSensor.magData);
                            sscanf(chSignalStrength, "%hd", &scalarMag.tfmSensor.signalStrength);   // crucial to use %hd in sscanf for int16_t of SignalStrength
                            hal.console->printf("\r\n>> %d %hd %d \r\n ", scalarMag.tfmSensor.magData, scalarMag.tfmSensor.signalStrength, scalarMag.tfmSensor.cycleCount);  // to MP Terminal
                        }
                        else {
                            hal.console->printf("\r\n>>FAIL-converted %d of 3 values in %s\r\n ", numConverted, AllButHeader );  // to MP Terminal
                        }
                    }
                    else { // string contains <magData>@<signalStrength>\0
                        numConverted = sscanf( AllButHeader, "%s@%hd", chMagData, &scalarMag.tfmSensor.signalStrength ) ;  // last one is already terminated with '\0' so convert directly
                        // was, didn't work: numConverted = sscanf( AllButHeader, "%[^@]@%hd", chMagData, &scalarMag.tfmSensor.signalStrength ) ;  // last one is already terminated with '\0' so convert directly
                        if (numConverted == 2) {
                            sscanf(chMagData, "%d", &scalarMag.tfmSensor.magData);
                            //hal.console->printf("\r\n>> %d %d \r\n ", scalarMag.tfmSensor.magData, scalarMag.tfmSensor.signalStrength);  // to MP Terminal
                        }
                        else {
                            //hal.console->printf("\r\n>>FAIL - only converted %d of 2 values\r\n ", numConverted );  // to MP Terminal
                        }
                    }
#else
                    pHdr = strchr(AllButHeader, (int)h2); // find the end of <magData>
                    if (pHdr != NULL) {
                        memcpy(chMagData, AllButHeader, pHdr - AllButHeader );
                        //@TODO this is likely slow as atoi, so find the fast version in your Android code
                        sscanf(chMagData, "%d", &scalarMag.tfmSensor.magData);
                    }
                    else {
                        chMagData[0] = '\0';
                    }
#endif
                    // do not use scalarMag.tfmSensor.rawMagDataPtr =  (uint8_t *)(&pktMag.AllButHeader);
                    // time the log write
                    //hal.uartE->write('U');  // DEBUG write a sentinel, monitor on scope
                    Log_Write_ScalarMag(scalarMag, 0); /* TODO Call Log_Write_ScalarMAg() with time_usvalue obtained here, otherwise you'll be timestamping it witht he time of writeing to flash */
                    hal.uartE->write('P');  // 0x50 DEBUG write a sentinel, monitor on scope
              }
                //hal.console->printf("\r\n>>Rx packet size %d ",  sizeAllButHeader);  // to MP Terminal
            }
            // I don't think we need to transfer the received packet into another buffer, just log it.
            //for (i = 0; i < pktMag.AllButHeader.length && pktMag.AllButHeader[i] != 0; i++) {
            //} // nothing to do, we're just finding the position of the null-terminator
            //packetAllButHeader = Arrays.copyOfRange(pktMag.AllButHeader, 0, i);
            //packetHeader = pktMag.Header;
            //// Send the obtained bytes to the UI Activity
            ////mHandler.obtainMessage(Constants.MESSAGE_READ, bytes, -1, buffer.clone()).sendToTarget();
            //// Send the obtained bytes to the UI Activity
            ////mHandler.obtainMessage(Constants.MESSAGE_READ, packetHeader, i, packetAllButHeader).sendToTarget();
        }
    }
    // Read from the uart and enqueue anything received
    // here, want toread a char and enQ it, look into details and see what's most efficient.
    // this part copies Rx chars into a byte array and then enQs the array.
    ///////
    // probably better to do uart.read and then serPutCHar
    ////////
    // read bytes from scalarMag
    // scalarMag::read()
    uint32_t nBytes = hal.uartE->available();
    if (nBytes > 0) {
        for (i=0; i<nBytes; i++) {
            // @TODO more efficient to read all chars into an array, and then add the array using enqChars
            // saves call to serPutRxChar() for each character
             serPutRxChar((uint8_t)hal.uartE->read());   // enqueues the received bytes
        }
    }
//    hal.uartE->write('P');  // 0x50 DEBUG write a sentinel, monitor on scope

    /////////////////////////////////////////////////
    /*
    // Replaced with above section to parse the incoming data stream.
    if ((nBytes>0) && (nBytes<64)) {  // WARNING- Fails if we got a partial packet?
        hal.uartE->write('U');  // DEBUG write a sentinel, monitor on scope
        for (uint32_t i=0; i<nBytes; i++) {
            dataByte = (uint8_t)hal.uartE->read();
            // no longer valid--do not use scalarMag.tfmSensor.rawMagData[i] = dataByte;
            receivedBytes[i] = dataByte;
            // echo bytes over radio link
            hal.uartC->write(dataByte); // uartC is TELEM1
            if (dataByte == '\n') {
                hal.console->printf("\r\n>> ");  // to MP Terminal
                hal.console->write(scalarMag.tfmSensor.rawMagData,nBytes);
                ///////////////////////////////////
                Log_Write_ScalarMag(scalarMag,0);
                ////////////////////////////////////
                break;
            }
        }
    }
    */
    //hal.console->printf("\r\n>>ScalarMag read %d bytes\r\n ", nBytes);  // to MP Terminal
    //hal.console->printf("\r\n>>ScalarMag count %d\r\n ", UserCount++);  // to MP Terminal
    //test_uart(hal.uartD, "uartD");  // telem2 is 915MHz radio to base at 57600
}


#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    /* JJW
    if(scalarMag.healthy){
        if ( scalarMag.read() ) {
            hal.console->printf("magData %d\r\n", scalarMag.magData);
            // Log_Write_ScalarMag();
        }
    }
    else
        ScalarMag.init();
    */

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    hal.console->printf("\r\n>>ScalarMag count %d\r\n ", UserCount++);  // to MP Terminal
}
#endif
