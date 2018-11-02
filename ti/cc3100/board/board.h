/*
 * board.h
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

#ifndef _BOARD_H
#define _BOARD_H

#include <stdint.h>

#include <msp432.h>

#define UART_READ_JITTER_BUFFER_SIZE       4
#define UART_READ_JITTER_RTS_GUARD         3

typedef void (*P_EVENT_HANDLER)(void* pValue);

typedef struct
{
    unsigned char          JitterBuffer[UART_READ_JITTER_BUFFER_SIZE];
    unsigned char          JitterBufferWriteIdx;
    unsigned char          JitterBufferReadIdx;
    unsigned char          JitterBufferFreeBytes;
    unsigned char          bRtsSetByFlowControl;
    unsigned char          bActiveBufferIsJitterOne;
    unsigned char          *pActiveBuffer;
    unsigned short         ActiveBufferWriteCounter;
}_uartFlowctrl;

typedef enum
{
    NO_LED,
    LED1,
    LED2
} ledEnum;

#define LED_RED   LED1
#define LED_GREEN LED2

typedef enum
{
    ANT1,
    ANT2
} antenna;

/*!
    \brief register an interrupt handler for the host IRQ

    \param[in]      InterruptHdl    -    pointer to interrupt handler function

    \param[in]      pValue          -    pointer to a memory strcuture that is
                    passed to the interrupt handler.

    \return         upon successful registration, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa
    \note           If there is already registered interrupt handler, the
                    function should overwrite the old handler with the new one
    \warning
*/
int registerInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue);

/*!
    \brief          Enables the CC3100

    \param[in]      none

    \return         none

    \note

    \warning
*/
void CC3100_enable(void);

/*!
    \brief          Disables the CC3100

    \param[in]      none

    \return         none

    \note

    \warning
*/
void CC3100_disable(void);

/*!
    \brief          Enables the interrupt from the CC3100

    \param[in]      none

    \return         none

    \note

    \warning
*/
void CC3100_InterruptEnable(void);

/*!
    \brief          Disables the interrupt from the CC3100

    \param[in]      none

    \return         none

    \note

    \warning
*/
void CC3100_InterruptDisable(void);

/*!
    \brief          Stops the Watch Dog timer

    \param[in]      none

    \return         none

    \note

    \warning
*/
void stopWDT(void);

/*!
    \brief          Initialize the system clock of MCU

    \param[in]      none

    \return         none

    \note

    \warning
*/
void initClk(void);

/*!
    \brief          Initialize the LEDs

    \param[in]      none

    \return         none

    \note

    \warning
*/
void initLEDs(void);

/*!
    \brief          Turn on the LED on MCU

    \param[in]      ledNum - LED to be turned on

    \return         none

    \note

    \warning
*/
void turnLedOn(char ledNum);

/*!
    \brief          Turn off the LED on MCU

    \param[in]      ledNum - LED to be turned of

    \return         none

    \note

    \warning
*/
void turnLedOff(char ledNum);

/*!
    \brief          Toggle the LED

    \param[in]      ledNum - LED to be toggled

    \return         none

    \note

    \warning
*/
void toggleLed(char ledNum);

/*!
    \brief          Get the LED status

    \param[in]      none

    \return         unsigned char - byte containing the status of LED

    \note

    \warning
*/
unsigned char GetLEDStatus(char ledNum);

/*!
    \brief     Produce delay in ms

    \param[in]         interval - Time in ms

    \return            none

    \note

    \warning
*/
void Delay(unsigned long interval);

/*!
    \brief      Masks the Host IRQ

    \param[in]      none

    \return         none

    \warning
*/
void MaskIntHdlr(void);

/*!
    \brief     Unmasks the Host IRQ

    \param[in]      none

    \return         none

    \warning
*/
void UnMaskIntHdlr(void);

/*!
    \brief     Set the CC3100 RTS line

    \param[in]      none

    \return         none

    \warning
*/
void set_rts(void);

/*!
    \brief     Clear the CC3100 RTS line

    \param[in]      none

    \return         none

    \warning
*/
void clear_rts(void);

/*!
    \brief     Initialize the antenna section GPIO (P2.4/P2.5)

    \param[in]      none

    \return         none

    \warning
*/
void initAntSelGPIO(void);

/*!
    \brief     Configure the GPIO for selecting antenna.

    \param[in]      antenna : antenna to be selected

    \return         none

    \warning
*/
void SelAntenna(int antenna);

#endif
