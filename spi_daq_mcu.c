// spi_daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU on the spectrometer DAQ prototype.
//
// PJ
// 2024-09-05 Bring up the board by flashing the LEDs.
// 2024-09-08 SPI slave firmware to respond to commands as in document TB3215.
//            Change to using SPI buffered mode so we can send an array.
// 2025-09-26 Update for use on the manufactured PCB.

// This version string will be reported by the version command (124)
#define VERSION_STR "v0.2 2025-09-26"

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
#define NBUF 32
volatile uint8_t inBuf[NBUF];
volatile uint8_t outBuf[NBUF];
volatile uint8_t nBytesIn = 0; // Number of bytes in inBuf after the last exchange.
volatile uint8_t nBytesOut = 0;
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
    // We will use the SS pin transition interrupts
    // to start and stop the SPI transaction.
    PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    PORTA.INTFLAGS |= PIN7_bm;
    //
    SPI0.CTRLA = SPI_ENABLE_bm & (~SPI_MASTER_bm); // Slave mode
    SPI0.CTRLB = SPI_BUFEN_bm | SPI_BUFWR_bm; // Buffer mode, don't write dummy byte
    // default SPI mode 0
    SPI0.INTCTRL = SPI_RXCIE_bm | SPI_TXCIE_bm | SPI_DREIE_bm;
}

ISR(PORTA_PORT_vect)
{
    if (PORTA.INTFLAGS & PIN7_bm) {
        if (PORTA.IN & PIN7_bm) {
            // Pin has gone high for end of the SPI transaction.
            newData = 1;
        } else {
            // Pin has gone low; Start a new SPI transaction.
            nBytesIn = 0;
            nBytesOut = 0;
            // Because we only start putting the outgoing buffer's bytes
            // into the SPI.DATA register once the interrupt routine is called,
            // the first two bytes that are sent appear to be the values
            // already in the buffer and DATA register at chip-select time.
        }
    }
    PORTA.INTFLAGS = PORT_INT_gm;
}

ISR(SPI0_INT_vect)
{
    // We arrive here if the SPI's incoming buffer gets full or
    // if a transmission has completed or 
    // if the outgoing data register is empty.
    //
    // Read all available incoming bytes.
    // It we fill the incoming buffer, we just overwrite the final place.
    while (SPI0.INTFLAGS & SPI_RXCIF_bm) {
        inBuf[nBytesIn] = SPI0.DATA;
        if (nBytesIn+1 < NBUF) ++nBytesIn;
    }
    // Fill up the outgoing register if it is empty.
    // Once we run out of bytes, we reuse the last one.
    while (SPI0.INTFLAGS & SPI_DREIF_bm) {
        SPI0.DATA = outBuf[nBytesOut];
        if (nBytesOut+1 < NBUF) ++nBytesOut;
    }
    if (SPI0.INTFLAGS & SPI_TXCIF_bm) {
        SPI0.INTFLAGS = SPI_TXCIF_bm;
    }
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
        case 124:
            // Copy the version string into the outgoing buffer.
            strncpy(outBuf, VERSION_STR, NBUF-1);
            break;
        case 125:
            // Insert a known set of data bytes.
            outBuf[0] = 0xde; outBuf[1] = 0xad;
            outBuf[2] = 0xbe; outBuf[3] = 0xef;
            outBuf[4] = 0xde; outBuf[5] = 0xad;
            outBuf[6] = 0xbe; outBuf[7] = 0xef;
            for (uint8_t i=8; i < NBUF; ++i) outBuf[i] = 0;
            break;
        case 127:
            LED_ON();
            outBuf[0] = 1;
            for (uint8_t i=1; i < NBUF; ++i) outBuf[i] = 0;
            break;
        case 126:
            LED_OFF();
            outBuf[0] = 0;
            for (uint8_t i=1; i < NBUF; ++i) outBuf[i] = 0;
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
