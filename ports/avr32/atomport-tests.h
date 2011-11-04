/*
 * Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ATOM_PORT_TESTS_H
#define __ATOM_PORT_TESTS_H

/* Include Atomthreads kernel API */
#include "atom.h"

/* Prerequisite include for ATOMLOG() macro (via printf) */
#include <stdio.h>

extern void uprint(const char* str, ...);

/* Logger macro for viewing test results */
#define ATOMLOG(str, ...) uprint (str, ## __VA_ARGS__)

/*
 * String location macro: for platforms which need to place strings in
 * alternative locations, e.g. on avr-gcc strings can be placed in
 * program space, saving SRAM. On most platforms this can expand to
 * empty.
 */
#define PSTR(x)     x //SN: Consider when implementing tests
#define _STR(x)     PSTR(x)

/* Default thread stack size (in bytes) */
#define TEST_THREAD_STACK_SIZE      128

/* Uncomment to enable logging of stack usage to UART */
/* #define TESTS_LOG_STACK_USAGE */

#  define ONBOARD_USART               (&AVR32_USART1)
#  define ONBOARD_USART_RX_PIN        AVR32_USART1_RXD_0_PIN
#  define ONBOARD_USART_RX_FUNCTION   AVR32_USART1_RXD_0_FUNCTION
#  define ONBOARD_USART_TX_PIN        AVR32_USART1_TXD_0_PIN
#  define ONBOARD_USART_TX_FUNCTION   AVR32_USART1_TXD_0_FUNCTION

#endif /* __ATOM_PORT_TESTS_H */

