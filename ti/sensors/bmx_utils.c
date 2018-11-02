//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//****************************************************************************

#include <stdbool.h>
#include <stdint.h>

#include "driverlib.h"

#include "bmx_utils.h"

//*****************************************************************************
//
//! Provides a small delay.
//!
//! \param ui32Count is the number of delay loop iterations to perform.
//!
//! This function provides a means of generating a delay by executing a simple
//! 3 instruction cycle loop a given number of times.  It is written in
//! assembly to keep the loop instruction count consistent across tool chains.
//!
//! It is important to note that this function does NOT provide an accurate
//! timing mechanism.  Although the delay loop is 3 instruction cycles long,
//! the execution time of the loop will vary dramatically depending upon the
//! application's interrupt environment (the loop will be interrupted unless
//! run with interrupts disabled and this is generally an unwise thing to do)
//! and also the current system clock rate and flash timings (wait states and
//! the operation of the prefetch buffer affect the timing).
//!
//! For better accuracy, the ROM version of this function may be used.  This
//! version will not suffer from flash- and prefect buffer-related timing
//! variability but will still be delayed by interrupt service routines.
//!
//! For best accuracy, a system timer should be used with code either polling
//! for a particular timer value being exceeded or processing the timer
//! interrupt to determine when a particular time period has elapsed.
//!
//! \return None.
//
//*****************************************************************************
#if defined(ewarm) || defined(DOXYGEN)
void
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(codered) || defined(gcc) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif
#if defined(rvmdk) || defined(__ARMCC_VERSION)
__asm void
SysCtlDelay(uint32_t ui32Count)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif
//
// For CCS implement this function in pure assembly.  This prevents the TI
// compiler from doing funny things with the optimizer.
//
#if defined(ccs)
__asm("    .sect \".text:SysCtlDelay\"\n"
      "    .clink\n"
      "    .thumbfunc SysCtlDelay\n"
      "    .thumb\n"
      "    .global SysCtlDelay\n"
      "SysCtlDelay:\n"
      "    subs r0, #1\n"
      "    bne.n SysCtlDelay\n"
      "    bx lr\n");
#endif

void DelayMs (uint32_t ulClockMS)
{
	if (ulClockMS == 1)
	{
		ulClockMS = 2;
	}
	SysCtlDelay((ulClockMS/2)*(CS_getMCLK() / (3 * 1000)));
}

void startWakeUpTimerA(uint16_t ulClockMS)
{
    ulClockMS = (ulClockMS * 32768)/1000;

    /* TimerA UpMode Configuration Parameter */
    Timer_A_UpModeConfig upConfig =
    {
            TIMER_A_CLOCKSOURCE_ACLK,              // ACLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_1,         // ACLK/1 = 32KHz
            ulClockMS,                             // tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,   // Enable CCR0 interrupt
            TIMER_A_SKIP_CLEAR                     // Clear value
    };

    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_0);

    MAP_Interrupt_enableInterrupt(INT_TA0_0);
    MAP_Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
}

void stopWakeUpTimerA(void)
{
    MAP_Interrupt_disableInterrupt(INT_TA0_0);
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
}
