#include "Copter.h"

//#include <AP_ScalarMag/AP_ScalarMag.h>
#include "AP_ScalarMag.h"

ScalarMag scalarMag;

/*// Write a ScalarMag packet
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

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("\r\n>>>4>>>ScalarMag %s at %.3f s\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
}
#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    setup_uart(hal.uartD, "uartD");  // telemetry 2
    UserCount=0;}

    //scalarMag.init();

#endif

void Copter::scalarMagTask()
{
    hal.console->printf("\r\n>>ScalarMag count %d\r\n ", UserCount++);
    test_uart(hal.uartD, "uartD");  // telemetry2
    // Log_Write_ScalarMag();
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
    if(scalarMag.healthy){
        if ( scalarMag.read() ) {
            hal.console->printf("magData %d\r\n", scalarMag.magData);
            // Log_Write_ScalarMag();
        }
    }
    /*
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
    hal.console->printf("\r\n>> 1 Hz loop >>MSG count %d\r\n ", UserCount++);
}
#endif
