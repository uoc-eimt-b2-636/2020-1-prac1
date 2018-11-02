/*
 * board.c
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

#include "simplelink.h"
#include "board.h"
#include "driverlib.h"

/*----------------------------------------------------------------------------*/

#define CC3100_IRQ_PORT             ( GPIO_PORT_P2 )
#define CC3100_IRQ_PIN              ( GPIO_PIN5 )
#define CC3100_IRQ_INT              ( INT_PORT2 )
#define CC3100_nHIB_PORT            ( GPIO_PORT_P4 )
#define CC3100_nHIB_PIN             ( GPIO_PIN1 )

/*----------------------------------------------------------------------------*/

P_EVENT_HANDLER pIraEventHandler = 0;

unsigned char IntIsMasked;

/*----------------------------------------------------------------------------*/

int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl, void* pValue)
{
    pIraEventHandler = InterruptHdl;

    return 0;
}

/*----------------------------------------------------------------------------*/

void CC3100_enable(void)
{
    MAP_GPIO_setOutputHighOnPin(CC3100_nHIB_PORT, CC3100_nHIB_PIN);
}

/*----------------------------------------------------------------------------*/

void CC3100_disable(void)
{
    MAP_GPIO_setOutputLowOnPin(CC3100_nHIB_PORT, CC3100_nHIB_PIN);
}

/*----------------------------------------------------------------------------*/

void CC3100_InterruptEnable(void)
{
    MAP_GPIO_interruptEdgeSelect(CC3100_IRQ_PORT, CC3100_IRQ_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_enableInterrupt(CC3100_IRQ_PORT, CC3100_IRQ_PIN);

    MAP_Interrupt_enableInterrupt(CC3100_IRQ_INT);
    MAP_Interrupt_setPriority(CC3100_IRQ_INT, 0xE0);
    MAP_Interrupt_enableMaster();
}

/*----------------------------------------------------------------------------*/

void CC3100_InterruptDisable(void)
{
    MAP_GPIO_disableInterrupt(CC3100_IRQ_PORT, CC3100_IRQ_PIN);
    MAP_Interrupt_disableInterrupt(CC3100_IRQ_INT);
}

/*----------------------------------------------------------------------------*/

void MaskIntHdlr(void)
{
    IntIsMasked = TRUE;
}

/*----------------------------------------------------------------------------*/

void UnMaskIntHdlr(void)
{
    IntIsMasked = FALSE;
}

/*----------------------------------------------------------------------------*/

void set_rts(void)
{
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6);
}

/*----------------------------------------------------------------------------*/

void clear_rts(void)
{
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
}

/*----------------------------------------------------------------------------*/

void initAntSelGPIO(void)
{
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
}

/*----------------------------------------------------------------------------*/

void SelAntenna(int antenna)
{
    switch (antenna)
    {
    case ANT1:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6);
        break;
    case ANT2:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);
        break;
    }
}

/*----------------------------------------------------------------------------*/

void Delay(unsigned long interval)
{
    volatile uint32_t i;
    while (interval > 0)
    {
        for (i = 48000; i > 0; i--)
            ;
        interval--;
    }
}

/*----------------------------------------------------------------------------*/

void PORT2_IRQHandler(void)
{
    if (MAP_GPIO_getInterruptStatus(CC3100_IRQ_PORT, CC3100_IRQ_PIN))
    {
        MAP_GPIO_clearInterruptFlag(CC3100_IRQ_PORT, CC3100_IRQ_PIN);
        if (pIraEventHandler)
        {
            pIraEventHandler(0);
        }
    }
}

/*----------------------------------------------------------------------------*/
