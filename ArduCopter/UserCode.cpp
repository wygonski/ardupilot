#include "Copter.h"
#include <DataFlash/DataFlash.h>          // ArduPilot Mega Flash Memory Library

#include <AP_ScalarMag/AP_ScalarMag.h>

// moved this to Copter.h so it has scope in Copter class for UserHook init
//AP_ScalarMag scalarMag;
//Packet pkt;

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
    if (time_us == 0) {
        time_us = AP_HAL::micros64();
    }
    struct log_SCALARMAG logpkt = {
        LOG_PACKET_HEADER_INIT(LOG_SCALARMAG_MSG),
        time_us       : time_us,
        rawMagData      : { },
        magData         : sm.getMagData(),
        signalStrength  : sm.getSignalStrength(),
        cycleCounter    : sm.getCycleCounter(),
        /* was
        magData         : scalarMag.getMagData(),
        signalStrength  : scalarMag.getSignalStrength(),
        cycleCounter    : scalarMag.getCycleCounter(),
        */
    };
    strncpy((char *)logpkt.rawMagData, (char *)sm.tfmSensor.rawMagDataPtr, sm.tfmSensor.rawMagDataSize);
    // was strncpy((char *)logpkt.rawMagData, (char *)sm.tfmSensor.rawMagData, sizeof(logpkt.rawMagData));
    // was strncpy(pkt.rawMagData, scalarMag.tfmSensor.rawMagData, sizeof(pkt.rawMagData));
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
    pkt.pktInit();
    scalarMag.init();
    bFirstTime = true;
    hal.console->printf("\r\n>>scalarMag.init()-- DONE\r\n ");  // to MP Terminal
}
#endif

void Copter::scalarMagTask()
{
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

    hal.uartE->write('U');  // DEBUG write a sentinel, monitor on scope
    while (!pkt.serIsRxEmpty()) { // this checks the ring buffer
        // parsing...;
        if (pkt.parseMessage()) {   // run tskPktMgr() and pktReceive()
            //Log.i(TAG, " " + pkt.Header);
            // we have a packet
            pkt.bPktReceived = false;
            if (pkt.Header == '!') {
                if (pkt.sizeAllButHeader <= 50) {  // guard against errors causing packet corruption (but error could have occurred earlier in pkt and crashed)
                    scalarMag.tfmSensor.rawMagDataSize = pkt.sizeAllButHeader;
                    scalarMag.tfmSensor.rawMagDataPtr =  (uint8_t *)(&pkt.AllButHeader);
                    // time the log write
                    //hal.uartE->write('U');  // DEBUG write a sentinel, monitor on scope
                    Log_Write_ScalarMag(scalarMag, 0);
                    //hal.uartE->write('U');  // DEBUG write a sentinel, monitor on scope
                }
                hal.console->printf("\r\n>>Rx packet size %d ", pkt.sizeAllButHeader);  // to MP Terminal
            }
            // I don't think we need to transfer the received packet into another buffer, just log it.
            //for (i = 0; i < pkt.AllButHeader.length && pkt.AllButHeader[i] != 0; i++) {
            //} // nothing to do, we're just finding the position of the null-terminator
            //packetAllButHeader = Arrays.copyOfRange(pkt.AllButHeader, 0, i);
            //packetHeader = pkt.Header;
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
            pkt.serPutRxChar((uint8_t)hal.uartE->read());   // enqueues the received bytes
        }
    }
    hal.uartE->write('P');  // 0x50 DEBUG write a sentinel, monitor on scope

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
