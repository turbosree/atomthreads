/*
 * Copyright (c) 2010, Atomthreads Project. All rights reserved.
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

#include <avr32/io.h>


.section .text

/*
 * \b archContextSwitch
 *
 * Architecture-specific context switch routine.
 *
 * Note that interrupts are always locked out when this routine is
 * called. For cooperative switches, the scheduler will have entered
 * a critical region. For preemptions (called from an ISR), the
 * ISR will have disabled interrupts on entry.
 *
 * Note that this function might have been coded in C, but gcc
 * was generating prologue and epilogue code to handle the parameters.
 * Worse, with the naked attribute set it generated half of the
 * prologue/epilogue. Rather than work around the gcc code generation,
 * which may change from compiler version to compiler version, we
 * just write this function in asm, where we have absolute control
 * over the code generation. Important during register saves/restores.
 *
 * @param[in] old_tcb_ptr Pointer to the thread being scheduled out
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archContextSwitch (ATOM_TCB *old_tcb_ptr, ATOM_TCB *new_tcb_ptr)
 */
.global archContextSwitch
archContextSwitch:

    /**
     * Parameter locations:
	  * SN: TBD - Not sure! Asked in Atmel AVR32 tools list
     *  old_tcb_ptr = R0
     *  new_tcb_ptr = R1
     */

    /**
     * If this is a cooperative context switch (a thread has called us
     * to schedule itself out), gcc will have saved any of the
     * registers: SN: TBD which it does not want us to clobber.
     * Any registers of that set which it did not need to save are safe
     * not to be saved by us anyway. Hence for cooperative context
     * switches we only need to save those registers which gcc expects
     * us _not_ to modify, that is R0-R12.
     *
     * If we were called from an interrupt routine (because a thread
     * is being preemptively scheduled out), the situation is exactly
     * the same. Any ISR which calls out to a subroutine will have
     * similarly saved those registers which it needs us not to
     * clobber. In the case of an interrupt, that is every single
     * register of the set SN: TBD. (gcc cannot establish
     * which of those registers actually need to be saved because
     * the information is not available to an ISR). Again, we only
     * need to save the registers R0-R12, because these
     * are expected to be unclobbered by a subroutine.
     */

    /**
     * Save registers R0-R12.
     */
	 stm --sp,r0-r12

	 /**
		 Store SP to old_tcb_ptr->sp_save_ptr which is conveniently the
		 first member of the TCB.
	  */
	 st.w r0[0],sp 

    /**
     * At this point, all of the current thread's context has been saved
     * so we no longer care about keeping the contents of any registers.
     *
     * The stack frame if this is a cooperative switch looks as follows:
     *
     *    <Any of SN: TBD that the calling function saves>
     *    <Return address to calling function>
     *    <R0>
     *    <R1>
     *     ||
     *    <R12>
     *
     * The stack frame if this was a preemptive switch looks as follows:
     *
     *   <SN: TBD>    // saved by ISR
     *   <Return address to ISR>
     *   <Any stacking and return addresses between ISR and this call>
     *   <R0>
     *   <R1>
     *    ||
     *   <R12>
     *
     *
     * In addition, the thread's stack pointer (after context-save) is
     * stored in the thread's TCB.
     */

    /**
     * We are now ready to restore the new thread's context. We switch
     * our stack pointer to the new thread's stack pointer, and pop
     * all of its context off the stack. When we have finished popping
     * all registers (R0-R12), we are ready to return.
     *
     * Note that any further registers that needed to be saved for the
     * thread will be restored on exiting this function. If the new
     * thread previously scheduled itself out cooperatively, the
     * original calling function will restore any registers it chose
     * to save. If the new thread was preempted, we will return to the
     * ISR which will restore all other system registers, before
     * returning to the interrupted thread.
     */

    /**
		 Load new_tcb_ptr->sp_save_ptr into SP. It is conveniently the
		 first member of the TCB.
	   */
	 ld.w sp,r1
		  
    /**
     * Restore registers
     */
	 /**
		  ldm description:
		  if Reglist16[R12] == 1
		  		R12 <- *(Loadaddress++);
		  for (i = 11 to 0)
		  		if Reglist16[i] == 1 then
		  			Ri <- *(Loadaddress++);
	  */
 	 ldm sp++,r0-r12

    /**
     * The return address on the stack will now be the new thread's return
     * address - i.e. although we just entered this function from a
     * function called by the old thread, now that we have restored the new
     * thread's context, we actually return from this function to wherever
     * the new thread was when it was scheduled out. This could be either a
     * regular C routine if the new thread previously scheduled itself out
     * cooperatively, or it could be an ISR if this new thread was
     * previously preempted (on exiting the ISR, execution will return to
     * wherever the new thread was originally interrupted).
     */

	 /**
		Return address will go in lr
		*/
	 ld.w lr,sp++
		  
    /**
     * We may come in from an ISR and leave through a regular C routine for
     * another thread (and visa versa).
	  * (1) We enter from a regular thread context, but leave through
	  * an ISR return address and a RETE. SN: TBD - Will it enable interrupts
	  * for us?
     * (2) we enter from an ISR and leave back into some thread context
     * calls, interrupts will remain disabled through the regular RET
     * calls, and we will reenable interrupts in the CRITICAL_END() call
     * when we unlock interrupts.
     */

	 /**
		SN: TBD - Application mode is not supported.
		SN: TBD - ISSUE: Here CPU might continue in its current MODE
 		since we are not using appropriate ret instruction.
      Eg: archContextSwitch is called from INTL0 Mode (TC0 interrupt) and
		since we are not calling rets/rete, processor will continue in
		INTL0 Mode?
		Ref: 'Entry and Exit Mechanism' in 32-bit atmel architecture manual.

		SOLUTION: This can be implemented in another way by using RAR_x
		and RSR_x registers. In that case rets can be used instead of
		restoring lr and pc. Since we are in Supervisor mode, rete can't
		be used here.
	  */
	  mov pc,lr		  


/**
 * \b archFirstThreadRestore
 *
 * Architecture-specific function to restore and start the first thread.
 * This is called by atomOSStart() when the OS is starting.
 *
 * This function will be largely similar to the latter half of
 * archContextSwitch(). Its job is to restore the context for the
 * first thread, and finally enable interrupts.
 *
 * It expects to see the context saved in the same way as if the
 * thread has been previously scheduled out, and had its context
 * saved. That is, archThreadContextInit() will have been called
 * first (via atomThreadCreate()) to create a "fake" context save
 * area, containing the relevant register-save values for a thread
 * restore.
 *
 * Note that you can create more than one thread before starting
 * the OS - only one thread is restored using this function, so
 * all other threads are actually restored by archContextSwitch().
 * This is another reminder that the initial context set up by
 * archThreadContextInit() must look the same whether restored by
 * archFirstThreadRestore() or archContextSwitch().
 *
 * @param[in] new_tcb_ptr Pointer to the thread being scheduled in
 *
 * @return None
 *
 * void archFirstThreadRestore (ATOM_TCB *new_tcb_ptr)
 */
.global archFirstThreadRestore
archFirstThreadRestore:

    /**
     * Parameter locations:
	  * SN: TBD - Not sure! Asked in Atmel AVR32 tools list		  
     *  new_tcb_ptr = R0
     */

    /**
     * First thread restores in the AVR32 port expect to see R0-R12 and
     * stored as context. The context will look exactly like it
     * would had a thread cooperatively scheduled itself out. That is,
     * these registers will be stored on the stack, and above those will
     * be the return address of the calling function. In this case we
     * will have set up this "fake" context in archThreadContextInit(),
     * and above these registers will be the return address of the thread
     * entry point. SN: TBD - Use of "rets" or "rete" instruction
     */

    /**
		 Load new_tcb_ptr->sp_save_ptr. It is conveniently the first member
		 of the TCB. Set our stack pointer to the new thread's stack pointer,
		 from its TCB.
     */
	 ld.w sp,r0
	 	  
    /**
     * Restore registers
     */
	 /**
		  ldm description
		  if Reglist16[R12] == 1
		  		R12 <- *(Loadaddress++);
		  for (i = 11 to 0)
		   	if Reglist16[i] == 1 then
		   		Ri <- *(Loadaddress++);
	  */
 	 ldm sp++,r0-r12

	 /**
		thread_shell(return address) will go in lr
	  */
	 ld.w lr,sp++
		  
    /**
     * The "return address" left on the lr register now will be the new
     * thread's entry point. Moving it to pc will take us there as if we had
     * actually been there before calling this subroutine, whereas
     * the return address was actually set up by archThreadContextInit().
     *
     * As discussed above, this function is responsible for enabling
     * interrupts once all context has been restored.
     */

	 /**
		SN: TBD - Application mode is not supported.
      Enable interrupts (Clear Global Interrupt Mask) and return to
		thread funtion. This can be implemented in another way by using
		RAR_x and RSR_x registers. In that case rets can be used
		instead of restoring lr and pc. Since we are in Supervisor mode,
		rete can't be used here.
	  */
	  csrf 16
	  mov pc,lr		  

