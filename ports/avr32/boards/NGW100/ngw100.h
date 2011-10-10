#ifndef _NGW100_H
#define	_NGW100_H

#include <avr32/io.h>
/* #include "usart.h" */
/* #include "pm_at32ap7000.h" */
#include "error.h"

#define CPU_FAMILY    "AVR32_AP7"
#define CPU_TYPE      "AT32AP7000"
#define MAX_FREQ_HZ   150000000

#define DEFAULT_FREQ  FOSC0
#define DEFAULT_UART  UART_A

#define FOSC32          32768     //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   3         //!< Osc32 startup time: RCOsc periods.

/* Define the on-board oscillators in Hz */
#define FOSC0           20000000  //!< Osc0 frequency: Hz.
#define FOSC1           12000000  //!< Osc1 frequency: Hz.
#define OSC0_STARTUP    3         //!< Osc0 startup time: RCOsc periods.
#define OSC1_STARTUP    3         //!< Osc1 startup time: RCOsc periods.

//! Set onboard SDRAM
#define SDRAM_PART_HDR  "mt481c2m32b2tg.h"

//! Data bus width to use the SDRAM(s)
#define SDRAM_DBW       16

/* Define the peripheral names */
#define UART_A  AVR32_USART1

/* Using privileged mode P1 to bypass cache*/
#define BOARD_SDRAM_BASE 0x90000000


#ifndef FRCOSC
  #define FRCOSC    AVR32_PM_RCOSC_FREQUENCY  //!< Default RCOsc frequency.
#endif


#endif
