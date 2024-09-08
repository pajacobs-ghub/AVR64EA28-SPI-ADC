// spi_daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU on the spectrometer DAQ prototype.
//
// PJ
// 2024-09-05 Bring up the board by flashing the LEDs.
// 2024-09-08 SPI slave firmware to respond to commands as in document TB3215.

// This version string will be reported by the version command.
#define VERSION_STR "v0.0 AVR64EA28 SPI-ADC 2024-09-08"

#include "global_defs.h"
#include <xc.h>
#include <avr/cpufunc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config PERIOD = PERIOD_OFF, WINDOW = WINDOW_OFF
#pragma config SLEEP = SLEEP_DISABLE, ACTIVE = ACTIVE_DISABLE, SAMPFREQ = SAMPFREQ_128HZ, LVL = LVL_BODLEVEL0
#pragma config EESAVE = EESAVE_DISABLE, RSTPINCFG = RSTPINCFG_RESET, UPDIPINCFG = UPDIPINCFG_UPDI, CRCSEL = CRCSEL_CRC16, CRCSRC = CRCSRC_NOCRC
#pragma config SUT = SUT_64MS

// The SPI peripheral will exchange data with the master MCU
// via the following buffers.
#define NBUF 64
volatile uint8_t inBuf[NBUF];
volatile uint8_t outBuf[NBUF];
volatile uint8_t nBytes = 0; // Number of bytes in inBuf after the last exchange.
volatile uint8_t newData = 0; // A flag to indicate that new data has arrived.


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

void spi0_init(void)
{
    PORTA.DIR &= ~PIN4_bm; // MOSI as input
    PORTA.DIR |= PIN5_bm; // MISO as output
    PORTA.DIR &= ~PIN6_bm; // SCK as input
    PORTA.DIR &= ~PIN7_bm; // SS as input
    //
    SPI0.CTRLA = SPI_ENABLE_bm & (~SPI_MASTER_bm); // Slave mode
    SPI0.INTCTRL = SPI_IE_bm;
}

ISR(SPI0_INT_vect)
{
    // For the moment, we follow TB3215 closely
    // and just handle a single byte.
    inBuf[0] = SPI0.DATA;
    SPI0.DATA = outBuf[0]; // Will be sent next time.
    nBytes = 1;
    newData = 1;
    SPI0.INTFLAGS = SPI_IF_bm;
}

void do_command()
{
    // We are working here with the interrupts off, so do not dally.
    int nchar;
    uint8_t cmd = inBuf[0];
    switch (cmd) {
        case 0:
            // Nil command, to allow previously-set outBuf data
            // to be exchanged with the master MCU.
            break;
        case 125:
            for (uint8_t i=0; i < NBUF; ++i) outBuf[i] = 0;
            nchar = snprintf(outBuf, NBUF, "%s", VERSION_STR);
            break;
        case 127:
            LED_ON();
            outBuf[0] = 1;
            break;
        case 126:
            LED_OFF();
            outBuf[0] = 0;
            break;
    }
}

int main(void)
{
    // Turn off the main clock pre-scaler so that we run at full 20MHz.
    ccp_write_io((void *) &(CLKCTRL.MCLKCTRLB), (CLKCTRL.MCLKCTRLB & 0xfe));
    CLKCTRL.MCLKTIMEBASE = TIMEBASE_VALUE; // Needed for the ADC
    iopins_init();
    spi0_init();
    ei();
    while (1) {
        di();
        if (newData) {
            do_command();
            newData = 0;
        }
        ei();
        // Do low priority tasks here.
        // Eventually, this will be the analog data sampling.
        _delay_ms(1);
    }
    return 0;
}
