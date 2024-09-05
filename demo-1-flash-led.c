// daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU on the spectrometer DAQ prototype.
//
// PJ
// 2024-09-05 Bring up the board by flashing the LEDs.

// This version string will be reported by the version command.
#define VERSION_STR "v0.0 AVR64EA28 SPI-ADC 2024-09-05"

#include "global_defs.h"
#include <xc.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config PERIOD = PERIOD_OFF, WINDOW = WINDOW_OFF
#pragma config SLEEP = SLEEP_DISABLE, ACTIVE = ACTIVE_DISABLE, SAMPFREQ = SAMPFREQ_128HZ, LVL = LVL_BODLEVEL0
#pragma config EESAVE = EESAVE_DISABLE, RSTPINCFG = RSTPINCFG_RESET, UPDIPINCFG = UPDIPINCFG_UPDI, CRCSEL = CRCSEL_CRC16, CRCSRC = CRCSRC_NOCRC
#pragma config SUT = SUT_64MS

static inline void LED_ON()
{
    PORTA.OUTSET = PIN0_bm;
}

static inline void LED_OFF()
{
    PORTA.OUTCLR = PIN0_bm;
}

void iopins_init(void)
{
    // UART0
    PORTA.DIRSET = PIN0_bm; // LED
    LED_OFF();
    return;
} // end iopins_init()


int main(void)
{
    // Turn off the main clock prescaler so that we run at full 20MHz.
    ccp_write_io((void *) &(CLKCTRL.MCLKCTRLB), (CLKCTRL.MCLKCTRLB & 0xfe));
    CLKCTRL.MCLKTIMEBASE = TIMEBASE_VALUE; // Needed for the ADC
    iopins_init();
    _delay_ms(10); // Let the pins settle, to reduce garbage on the RX pin.
    while (1) {
        LED_ON();
        _delay_ms(250);
        LED_OFF();
        _delay_ms(250);
    }
    return 0;
}
