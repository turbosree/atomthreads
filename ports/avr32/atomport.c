/*
 * Copyright (c) 2011, Kelvin Lawson. All rights reserved.
 * Sreejith Naarakathil: sreejith.naarakathil@gmail.com
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


#include "atom.h"
#include "atomport-private.h"


/** Forward declarations */
static void thread_shell (void);


/**
 * \b thread_shell
 *
 * Shell routine which is used to call all thread entry points.
 *
 * This routine is called whenever a new thread is starting, and is
 * responsible for taking the entry point parameter off the TCB
 * and passing this into the thread entry point, as well as enabling
 * interrupts. This is an optional function for a port, as it is
 * also possible to do this with a regular context restore if the
 * appropriate registers are saved for context switches (parameter
 * and interrupt enable). Alternatively a flag could be used to
 * notify archFirstThreadRestore() and archContextSwitch()
 * that they should this time restore the contents of the parameter
 * registers and enable interrupts. After restoring a thread first
 * time the context restore routines need not perform those
 * operations again. This is discussed in more detail below.
 *
 * When starting new threads, ports must pass in the entry point
 * function parameter, and enable interrupts. This can be handled by
 * the first context restore for new threads if they are saved as
 * part of the thread's context. However for the AVR32 port we will 
 * save registers R0-R12. The thread shell routine is used to start 
 * all threads. The thread shell is also handy for providing port 
 * users with a place to do any initialisation that
 * must be done for each thread (e.g. opening stdio files etc).
 *
 * @return None
 */
static void thread_shell (void)
{
   ATOM_TCB *curr_tcb;

   /* Get the TCB of the thread being started */
   curr_tcb = atomCurrentContext();

   /**
    * Enable interrupts - these will not be enabled when a thread
    * is first restored.
    */
   ENABLE_ALL_INTERRUPTS();

   /* Call the thread entry point */
   if (curr_tcb && curr_tcb->entry_point)
      {
         curr_tcb->entry_point(curr_tcb->entry_param);
      }

   /* Not reached - threads should never return from the entry point */

}


/**
 * \b archThreadContextInit
 *
 * Architecture-specific thread context initialisation routine.
 *
 * This function must set up a thread's context ready for restoring
 * and running the thread via archFirstThreadRestore() or
 * archContextSwitch().
 *
 * The layout required to fill the correct register values is
 * described in archContextSwitch(). Note that not all registers
 * are restored by archContextSwitch() and archFirstThreadRestore()
 * as this port takes advantage of the fact that not all registers
 * must be stored by gcc-avr C subroutines. This means that we don't
 * provide start values for those registers, as they are "don't cares".
 *
 * The thread shell is always used as the thread entry point stored
 * in the thread context, and it does the actual calling of the
 * proper thread entry point, passing the thread entry parameter.
 * This allows us to pass the entry parameter without actually
 * storing it on the stack (the thread shell routine takes the
 * entry point and parameter from the thread's TCB). On other ports
 * you may instead choose to store the entry point and parameter
 * in the thread context and use no thread shell routine.
 *
 * Similarly we use the thread shell in this case to enable interrupts.
 * When a thread is restored and started for the first time, it must
 * also enable interrupts. This might be done by setting up the
 * appropriate value in the SREG register for enabled interrupts, which
 * would then be restored when the thread is first started. But to
 * reduce register-saves we do not save SREG on the AVR port, and
 * instead we use the thread shell routine to enable interrupts the
 * first time a thread is started.
 *
 * @param[in] tcb_ptr Pointer to the TCB of the thread being created
 * @param[in] stack_top Pointer to the top of the new thread's stack
 * @param[in] entry_point Pointer to the thread entry point function
 * @param[in] entry_param Parameter to be passed to the thread entry point
 *
 * @return None
 */

void archThreadContextInit (ATOM_TCB *tcb_ptr, void *stack_top, void (*entry_point)(uint32_t), uint32_t entry_param)
{
   uint32_t *stack_ptr;

   /** Start at stack top */
   stack_ptr = (uint32_t *)stack_top;

   /**
      The AVR32 architecture has 16 registers, with 13 of these being 
      purely general purpose. The remaining three are the program counter, 
      the stack pointer and the link register. The link register is used 
      to hold the return address of the current function. This reduces 
      the amount of stack accesses required for function calls, since 
      simple function calls do not need to access the stack at all. 
      Both the stack pointer and the link register can also be used as 
      general purpose registers. Status register - One of the system 
      registers is the status register. It is split into two parts - 
      the upper and lower halfword. User applications can only access 
      the lower halfword. The lower halfword contains several flags set 
      by results of arithmetic and logical operations, such as a zero flag,
      an overflow flag, and several others. These flags are used by 
      conditional branches and operations. The lock-bit is used to implement 
      atomic operations, the scratch bit can be used for any purpose by 
      applications, and the register remap flag is used by the Java extension
      module. The upper halfword contains the status of the processor. 
      Among other things this includes the current execution mode and 
      whether interrupts and exceptions are enabled.
   */


   /**
    * After restoring all of the context registers, the thread restore
    * routines will perform a RETS or RETE which expect to find the
    * address of the calling routine on the stack. In this case (the
    * first time a thread is run) we "return" to the entry point for
    * the thread. That is, we store the thread entry point in the
    * place that RETS and RETE will look for the return address: In case
    * of AVR32, the link register (lr) if CPU is in Supervisor mode and 
    * RAR/RSR if the CPU is in any other mode than application mode.
    *
    * Note that we are using the thread_shell() routine to start all
    * threads, so we actually store the address of thread_shell()
    * here. Other ports may store the real thread entry point here
    * and call it directly from the thread restore routines.
    *
    * Because we are filling the stack from top to bottom, this goes
    * on the stack first (at the top).
    */

   /**
      Store thread_shell address so that we can restore lr 
   */
   *stack_ptr-- = (uint32_t)thread_shell;

   /**
    * Store starting register values for R0-R12
    */
   
   /**
     Stack layout AVR32B AP7000
     R14/LR -4
     R0     -8 
     R1     -12
     R2    -16
     R3    -20
     R4    -24
     R5    -28
     R6    -32
     R7    -36
     R8    -40
     R9    -44
     R10   -48
     R11   -52
     R12   -56
   */
   *stack_ptr-- = 0x00;    /* R0 */
   *stack_ptr-- = 0x00;    /* R1 */
   *stack_ptr-- = 0x00;    /* R2 */
   *stack_ptr-- = 0x00;    /* R3 */
   *stack_ptr-- = 0x00;    /* R4 */
   *stack_ptr-- = 0x00;    /* R5 */
   *stack_ptr-- = 0x00;    /* R6 */
   *stack_ptr-- = 0x00;    /* R7 */
   *stack_ptr-- = 0x00;    /* R8 */
   *stack_ptr-- = 0x00;    /* R9 */
   *stack_ptr-- = 0x00;    /* R10 */
   *stack_ptr-- = 0x00;    /* R11 */
   *stack_ptr-- = 0x00;    /* R12 */

   /**
    * All thread context has now been initialised. Save the current
    * stack pointer to the thread's TCB so it knows where to start
    * looking when the thread is started.
    */
   tcb_ptr->sp_save_ptr = stack_ptr;

}


/**
 *
 * System tick ISR.
 *
 * This is responsible for regularly calling the OS system tick handler.
 * The system tick handler checks if any timer callbacks are necessary,
 * and runs the scheduler.
 *
 * The compiler automatically saves all registers necessary before calling
 * out to a C routine. This will be (at least) SN: TBD
 *
 * The system may decide to schedule in a new thread during the call to
 * atomTimerTick(), in which case around half of the thread's context will
 * already have been saved here, ready for when we return here when the
 * interrupted thread is scheduled back in. The remaining context will be
 * saved by the context switch routine.
 *
 * As with all interrupts, the ISR should call atomIntEnter() and
 * atomIntExit() on entry and exit. This serves two purposes:
 *
 * a) To notify the OS that it is running in interrupt context
 * b) To defer the scheduler until after the ISR is completed
 *
 * We defer all scheduling decisions until after the ISR has completed
 * in case the interrupt handler makes more than one thread ready.
 *
 * @return None
 */

/**
   SN: TBD - Optimize using register shadowing.
   The interrupt attribute takes an optional argument specifying the 
   shadow-register mode for the interrupt. Registers that are shadowed 
   is not required to be saved on the stack before being used in the 
   function. The AVR32 architecture has three possible shadowing modes: 
   full, half or none. Specifying no argument to the attribute defaults 
   to none. Writing an interrupt handler function for an interrupt with 
   half shadow mode can then be done like this: 
   __attribute__((interrupt("half"))) void bar(void) { }
*/
__attribute__((__interrupt__))
static void tc_irq(void)
{
   /* Call the interrupt entry routine */
   atomIntEnter();

   /* Call the OS system tick handler */
   atomTimerTick();

   /* Call the interrupt exit routine */
   atomIntExit(TRUE);
}


/**
 *
 * Default (no handler installed) ISR.
 *
 * Installs a default handler to be called if any interrupts occur for
 * which we have not registered an ISR. This is empty and has only been
 * included to handle user-created code which may enable interrupts. The
 * core OS does not enable any interrupts other than the system timer
 * tick interrupt.
 *
 * @return None
 */
__attribute__((__interrupt__))
static void default_irq(void)
{
   /* Empty */
}


/**
 * \b avrInitSystemTickTimer
 *
 * Initialise the system tick timer. Uses the AVR's timer1 facility.
 *
 * @return None
 */
void avrInitSystemTickTimer ( void )
{
   /**
      Start a timer/counter and generate a "tick" interrupt each millisecond. 
      The selected timer input clock is the internal clock labelled TC3
      referred to as TIMER_CLOCK3 in the datasheet. TIMER_CLOCK3 is connected
      to clk_pbb / 8 (see datasheet). The 16-bit timer/counter channel will 
      cycle from 0x0000 to RC. RC is initialized to (clk_pbb / 8) / 1000, 
      so that an interrupt will be triggered every 1 ms. 
   */
   volatile avr32_tc_t *tc = SYSTEM_TC;

   /**
      Options for waveform genration.
   */
   static const tc_waveform_opt_t WAVEFORM_OPT =
      {
         .channel  = TC_CHANNEL,                        // Channel selection.

         .bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
         .beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
         .bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
         .bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

         .aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
         .aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
         .acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
         .acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

         .wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
         .enetrg   = FALSE,                             // External event trigger enable.
         .eevt     = 0,                                 // External event selection.
         .eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
         .cpcdis   = FALSE,                             // Counter disable when RC compare.
         .cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

         .burst    = FALSE,                             // Burst signal selection.
         .clki     = FALSE,                             // Clock inversion.
         .tcclks   = TC_CLOCK_SOURCE_TC3                // Internal source clock 3, connected to fPBB / 8.
      };

   /**
      Enable compare match (RC) interrupt 
   */
   static const tc_interrupt_t TC_INTERRUPT =
      {
         .etrgs = 0,
         .ldrbs = 0,
         .ldras = 0,
         .cpcs  = 1,
         .cpbs  = 0,
         .cpas  = 0,
         .lovrs = 0,
         .covfs = 0
      };   

   /**
      Reset PM. Makes sure we get the expected clocking after a 
      soft reset (e.g.: JTAG reset)
   */
   pm_reset();

   /**
      Start PLL0 giving 96 MHz clock
   */
   pm_pll_opt_t pll_opt = {
      .pll_id = 0,
      .mul = 24,
      .div = 5,
      .osc_id = 0,
      .count = 16,
      .wait_for_lock = 1,
   };
   pm_start_pll(&pll_opt);

   /**
      Divide HSB by 2 and PBA by 4 to keep them below maximum ratings
   */
   pm_set_clock_domain_scaler(PM_HSB_DOMAIN, 2);
   pm_set_clock_domain_scaler(PM_PBA_DOMAIN, 4);

   /**
      Divide 96 MHz by 8 to get 12 MHz PBB clock used for TIMER 0
   */
   pm_set_clock_domain_scaler(PM_PBB_DOMAIN, 8);

   /**
      Use PLL0 as clock source
   */
   pm_set_mclk_source(PM_PLL0);

   Disable_global_interrupt();

   /**
     Initialize interrupt vectors.
   */
   INTC_init_interrupts();

   /**
      Register the TC0 interrupt handler to the interrupt controller.
   */
   INTC_register_interrupt(&tc_irq, AVR32_TC0_IRQ0, AVR32_INTC_INT0);

   Enable_global_interrupt();

   /**
      Initialize the timer/counter. Initialize the timer/counter waveform.
   */
   tc_init_waveform(tc, &WAVEFORM_OPT);

   /**
      Set the compare triggers.
      Remember TC counter is 16-bits, so counting second is not possible 
      with clk_pbb = 12 MHz. We configure it to count ms. 
      We want: (1/(clk_pbb)) * RC = 0.001 s, 
      hence RC = (clk_pbb) / 1000 = 12000 to get an interrupt every 1 ms.
   */
   tc_write_rc(tc, TC_CHANNEL, (AVR32_CPU_HZ / 8) / 1000);

   tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

   /**
      Start the timer/counter.
   */
   tc_start(tc, TC_CHANNEL);
}
