#include "Copter.h"
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
    uart->printf("\r\n>>>4>>>MSG from %s at %.3f s\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
}
#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    // this will be called once at start-up
    setup_uart(hal.uartD, "uartD");  // telemetry 2
    UserCount=0;}
#endif

void Copter::scalarMagTask()
{
    // put your 1Hz code here
    hal.console->printf("\r\n>>>>MSG from scheduled task, count %d\r\n ", UserCount++);
    test_uart(hal.uartD, "uartD");  // telemetry2
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
    hal.console->printf("\r\n>>1>>MSG count %d\r\n ", UserCount++);
    test_uart(hal.uartD, "uartD");  // telemetry2
}
#endif
