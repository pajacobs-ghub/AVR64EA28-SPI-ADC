// spi_daq_mcu.c
// Use an AVR64EA28-I/SP as the DAQ-MCU on the spectrometer DAQ prototype.
//
// Peter J.
// 2024-09-05 Bring up the board by flashing the LEDs.
// 2024-09-08 SPI slave firmware to respond to commands as in document TB3215.
//            Change to using SPI buffered mode so we can send an array.
// 2025-09-26 Update for use on the manufactured PCB.
// 2025-10-17,18 Have the analog sampling happening.

// This version string will be reported by the version command (124)
#define VERSION_STR "v0.3 2025-10-17"

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
uint8_t outBuf[NBUF]; // Not volatile because interrupt routine only reads from it.
volatile uint8_t nBytesIn = 0; // Number of bytes in inBuf after the last exchange.
volatile uint8_t nBytesOut = 0;
volatile uint8_t newData = 0; // A flag to indicate that new data has arrived.

// The LED is used to indicate sampling activity and may be manually turned on.
// This flag is used to indicate that we have manually turned the LED on.
uint8_t overrideLED = 0;

// Virtual registers for configuration and state.
// [0] : state, 0==IDLE 1==SAMPLING
// [1] : V_REF selection, 0==V_DD, 1==1v024, 2==2v048, 3=4v096, 4=2v500
// [2] : PGA flag, 0==direct sampling, 1==via PGA
// [3] : PGA gain, 0==1X, 1==2X, 2==4X, 3==8X, 4==16X
// [4] : samples in burst mode: 0==single sample, n>0 2**n samples in burst mode
#define NREG 5
volatile uint8_t vreg[NREG] = {
    0, // IDLE
    3, // V_REF==4v096
    0, // direct sampling without PGA
    0, // 1X
    4  // 4 (16 samples in burst mode)
};
#define IDLE 0
#define SAMPLING 1
uint8_t need_to_init_ADC = 1;

// Storage for sampled analog data.
#define NCHAN 8
int16_t analogData[NCHAN];

// Bit patterns for selecting analog-input pins.
const uint8_t muxpos_pin[] = {
    ADC_MUXPOS_AIN28_gc, // [0] = PC0 (actually decimal 28)
    ADC_MUXPOS_AIN30_gc, // [1] = PC2
    ADC_MUXPOS_AIN0_gc,  // [2] = PD0
    ADC_MUXPOS_AIN2_gc,  // [3] = PD2
    ADC_MUXPOS_AIN4_gc,  // [4] = PD4
    ADC_MUXPOS_AIN6_gc,  // [5] = PD6 (actually decimal 6)
    ADC_MUXPOS_AIN23_gc, // [6] = PA3
    ADC_MUXPOS_AIN17_gc  // [7] = PF1
};
const uint8_t muxneg_pin[] = {
    ADC_MUXNEG_AIN29_gc, // [0] = PC1 (actually decimal 29)
    ADC_MUXNEG_AIN31_gc, // [1] = PC3
    ADC_MUXNEG_AIN1_gc,  // [2] = PD1
    ADC_MUXNEG_AIN3_gc,  // [3] = PD3
    ADC_MUXNEG_AIN5_gc,  // [4] = PD5
    ADC_MUXNEG_AIN7_gc,  // [5] = PD7 (actually decimal 7)
    ADC_MUXNEG_AIN22_gc, // [6] = PA2
    ADC_MUXNEG_AIN16_gc  // [7] = PF0
};

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
    PORTA.DIRSET = PIN0_bm; // LED
    LED_OFF();
    // 16 analog-in pins
    PORTC.DIRCLR = PIN0_bm; // Input for AIN28
    PORTC.DIRCLR = PIN1_bm; // Input for AIN29
    PORTC.DIRCLR = PIN2_bm; // Input for AIN30
    PORTC.DIRCLR = PIN3_bm; // Input for AIN31
    PORTD.DIRCLR = PIN0_bm; // Input for AIN0
    PORTD.DIRCLR = PIN1_bm; // Input for AIN1
    PORTD.DIRCLR = PIN2_bm; // Input for AIN2
    PORTD.DIRCLR = PIN3_bm; // Input for AIN3
    PORTD.DIRCLR = PIN4_bm; // Input for AIN4
    PORTD.DIRCLR = PIN5_bm; // Input for AIN5
    PORTD.DIRCLR = PIN6_bm; // Input for AIN6
    PORTD.DIRCLR = PIN7_bm; // Input for AIN7
    PORTA.DIRCLR = PIN3_bm; // Input for AIN23
    PORTA.DIRCLR = PIN2_bm; // Input for AIN22
    PORTF.DIRCLR = PIN1_bm; // Input for AIN17
    PORTF.DIRCLR = PIN0_bm; // Input for AIN16
    return;
} // end iopins_init()

void adc0_init(void)
{
    // Set up the ADC for low-latency conversion.
    // Once we start conversions, we will likely want to make
    // subsequent conversions quickly.
    ADC0.CTRLA |= ADC_LOWLAT_bm | ADC_ENABLE_bm;
    ADC0.CTRLB = ADC_PRESC_DIV4_gc; // ADC clock frequency 5MHz
    switch (vreg[1]) {
        case 0:
            ADC0.CTRLC = ADC_REFSEL_VDD_gc;
            break;
        case 1:
            ADC0.CTRLC = ADC_REFSEL_1V024_gc;
            break;
        case 2:
            ADC0.CTRLC = ADC_REFSEL_2V048_gc;
            break;
        case 3:
            ADC0.CTRLC = ADC_REFSEL_4V096_gc;
            break;
        case 4:
            ADC0.CTRLC = ADC_REFSEL_2V500_gc;
            break;
        default:
            ADC0.CTRLC = ADC_REFSEL_4V096_gc;
    }
    ADC0.CTRLE = 20; // SAMPDUR of 4 microseconds
    //
    // The number of samples converted in burst mode is 2^^sampnum.
    uint8_t sampnum = vreg[4];
    if (sampnum > 0x0A) sampnum = 0x0A;
    if (sampnum == 0) {
        ADC0.CTRLF = ADC_SAMPNUM_NONE_gc;
    } else {
        ADC0.CTRLF = ADC_CHOPPING_bm | ADC_FREERUN_bm | sampnum;
    }
    //
    uint8_t gain_gc = ADC_GAIN_1X_gc;
    switch (vreg[3]) {
        case 0:
            gain_gc = ADC_GAIN_1X_gc;
            break;
        case 1:
            gain_gc = ADC_GAIN_2X_gc;
            break;
        case 2:
            gain_gc = ADC_GAIN_4X_gc;
            break;
        case 3:
            gain_gc = ADC_GAIN_8X_gc;
            break;
        case 4:
            gain_gc = ADC_GAIN_16X_gc;
            break;
        default:
            gain_gc = ADC_GAIN_1X_gc;
    }
    ADC0.PGACTRL = gain_gc | ADC_PGABIASSEL_100PCT_gc | ADC_PGAEN_bm;
    while (ADC0.STATUS & ADC_ADCBUSY_bm) { /* wait for settling */ }
} // end adc0_init()

void adc0_close(void)
{
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
}

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
        case 80: //0x50
            // Write analog data values to outgoing buffer.
            // Use big-endian layout.
            for (uint8_t i=0; i < NCHAN; ++i) {
                outBuf[i*2] = (uint8_t)((analogData[i] & 0xff00) >> 8);
                outBuf[i*2+1] = (uint8_t)(analogData[i] & 0x00ff);
            }
            for (uint8_t i=NCHAN*2; i < NBUF; ++i) outBuf[i] = 0;
            break;
        case 96: //0x60
            // Write virtual register values to outgoing buffer.
            for (uint8_t i=0; i < NREG; ++i) outBuf[i] = vreg[i];
            for (uint8_t i=NREG; i < NBUF; ++i) outBuf[i] = 0;
            break;
        case 112: // 0x70
            vreg[0] = inBuf[1]; // sampling state
            adc0_close();
            need_to_init_ADC = 1;
            break;
        case 113: // 0x71
            vreg[1] = inBuf[1]; // V_REF selection
            adc0_close();
            need_to_init_ADC = 1;
            break;
        case 114: // 0x72
            vreg[2] = inBuf[1]; // PGA flag
            adc0_close();
            need_to_init_ADC = 1;
            break;
        case 115: // 0x73
            vreg[3] = inBuf[1]; // PGA gain
            adc0_close();
            need_to_init_ADC = 1;
            break;
        case 116: // 0x74
            vreg[4] = inBuf[1]; // burst-mode sampling
            adc0_close();
            need_to_init_ADC = 1;
            break;
        case 124: // 0x7c
            // Copy the version string into the outgoing buffer.
            strncpy(outBuf, VERSION_STR, NBUF-1);
            break;
        case 125: // 0x7d
            // Insert a known set of data bytes.
            outBuf[0] = 0xde; outBuf[1] = 0xad;
            outBuf[2] = 0xbe; outBuf[3] = 0xef;
            outBuf[4] = 0xde; outBuf[5] = 0xad;
            outBuf[6] = 0xbe; outBuf[7] = 0xef;
            for (uint8_t i=8; i < NBUF; ++i) outBuf[i] = 0;
            break;
        case 126: // 0x7e
            overrideLED = 0;
            LED_OFF();
            outBuf[0] = 0;
            for (uint8_t i=1; i < NBUF; ++i) outBuf[i] = 0;
            break;
        case 127: // 0x7f
            overrideLED = 1;
            LED_ON();
            outBuf[0] = 1;
            for (uint8_t i=1; i < NBUF; ++i) outBuf[i] = 0;
            break;
        default: { /* do nothing */ }
            // Another nil command, to allow previously-set outBuf data
            // to be exchanged with the master MCU.            
    }
}

int main(void)
{
    // Turn off the main clock pre-scaler so that we run at full 20MHz.
    ccp_write_io((void *) &(CLKCTRL.MCLKCTRLB), (CLKCTRL.MCLKCTRLB & 0xfe));
    CLKCTRL.MCLKTIMEBASE = TIMEBASE_VALUE; // Needed for the ADC
    iopins_init();
    spi0_init();
    adc0_init();
    vreg[0] = SAMPLING;
    ei();
    while (1) {
        di();
        if (newData) {
            do_command();
            newData = 0;
        }
        ei();
        // Do low priority task of collecting analog data samples.
        if (need_to_init_ADC) {
            // We may have to reinitialize because the configuration has changed.
            adc0_init();
            need_to_init_ADC = 0;
        }
        if (vreg[0] == SAMPLING) {
            // Sample all analog channels.
            if (!overrideLED) LED_ON();
            // Build up the ADC command-register byte from our options.
            uint8_t adc_cmd = ADC_DIFF_bm;
            adc_cmd |= ADC_START_IMMEDIATE_gc;
            if (vreg[4] > 0) {
                // Burst-mode sampling with scaling, such that we have to only pick up
                // a 16-bit result.  The results are an accumulation of the sample values
                // and will be scaled such that they are 16 times larger than the raw
                // 12-bit sample, even if fewer than 16 samples were accumulated.
                // See Table 31-3 in the data sheet.
                adc_cmd |= ADC_MODE_BURST_SCALING_gc;
            } else {
                // Default to single sample for each conversion.
                adc_cmd |= ADC_MODE_SINGLE_12BIT_gc;
            }
            for (uint8_t i= 0; i < NCHAN; ++i) {
                uint8_t pos_bits = muxpos_pin[i];
                uint8_t neg_bits = muxneg_pin[i];
                if (vreg[2]) {
                    pos_bits |= ADC_VIA_PGA_gc;
                    neg_bits |= ADC_VIA_PGA_gc;
                } else {
                    pos_bits |= ADC_VIA_DIRECT_gc;
                    neg_bits |= ADC_VIA_DIRECT_gc;
                }
                // Select ADC channel and make the conversion.
                ADC0.MUXPOS = pos_bits;
                ADC0.MUXNEG = neg_bits;
                ADC0.COMMAND = adc_cmd;
                while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) { /* wait */ }
                ADC0.COMMAND = ADC_START_STOP_gc;
                analogData[i] = (int16_t)ADC0.RESULT; // 16-bit value only
            }
            if (!overrideLED) LED_OFF();
        }
    }
    return 0;
}
