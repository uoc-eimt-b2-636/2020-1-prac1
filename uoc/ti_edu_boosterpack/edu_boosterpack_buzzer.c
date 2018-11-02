/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
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
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
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

#include "edu_boosterpack_buzzer.h"

#include "msp432.h"
#include "driverlib.h"

/*----------------------------------------------------------------------------*/
#define BUZZER_PORT             ( GPIO_PORT_P2 )
#define BUZZER_PIN              ( GPIO_PIN7 )
#define BUZZER_MODE             ( GPIO_PRIMARY_MODULE_FUNCTION )
#define BUZZER_TIMER            ( TIMER_A0_BASE )
#define BUZZER_TIMER_CCR        ( PM_TA0CCR4A )

#define TIMER_BASE              ( TIMER_A1_BASE )
#define TIMER_MODE              ( TIMER_A_UP_MODE )
#define TIMER_REGISTER          ( TIMER_A_CAPTURECOMPARE_REGISTER_0 )
#define TIMER_INTERRUPT         ( INT_TA1_0 )
/*----------------------------------------------------------------------------*/
#define VOLUME_VALUE            ( 0xF1 )
/*----------------------------------------------------------------------------*/
static Timer_A_UpModeConfig upConfig =
{
    TIMER_A_CLOCKSOURCE_ACLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_32,
    0,
    TIMER_A_TAIE_INTERRUPT_DISABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
    TIMER_A_DO_CLEAR
};
/*----------------------------------------------------------------------------*/
static Timer_A_PWMConfig pwmConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_12,
    0,
    TIMER_A_CAPTURECOMPARE_REGISTER_4,
    TIMER_A_OUTPUTMODE_RESET_SET,
    0
};
/*----------------------------------------------------------------------------*/
typedef struct {
    uint16_t bpm;
    buzzer_cb_t callback;
} buzzer_t;
/*----------------------------------------------------------------------------*/
static buzzer_t buzzer = {.callback = NULL};
/*----------------------------------------------------------------------------*/
static const uint8_t port2_remapping[] = {
    PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_TA0CCR1A, PM_NONE, PM_TA0CCR3A, BUZZER_TIMER_CCR
};
/*----------------------------------------------------------------------------*/
static note_t notes[] = {{0x13EE,0x0C}, {0x12D3,0x0B}, {0x11C1,0x0B}, {0x10C4,0x0A},
                         {0x0FD1,0x0A}, {0x0EEC,0x09}, {0x0E17,0x09}, {0x0D4C,0x08},
                         {0x0C8D,0x08}, {0x0BD9,0x07}, {0x0B2F,0x07}, {0x0A8F,0x06},
                         {0x09F7,0x06}, {0x0968,0x05}, {0x08E1,0x05}, {0x0861,0x04},
                         {0x07E8,0x04}, {0x0777,0x04}, {0x0000,0x00}, };
/*----------------------------------------------------------------------------*/
void edu_boosterpack_buzzer_init(void)
{
    MAP_PMAP_configurePorts((const uint8_t *) port2_remapping, P2MAP, 1, PMAP_DISABLE_RECONFIGURATION);

    GPIO_setAsPeripheralModuleFunctionOutputPin(BUZZER_PORT, BUZZER_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_buzzer_callback_set(buzzer_cb_t callback)
{
    buzzer.callback = callback;
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_buzzer_callback_clear(void)
{
    buzzer.callback = NULL;
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_buzzer_callback(void)
{
    if (buzzer.callback != NULL)
    {
        buzzer.callback();
    }
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_set_bpm(uint16_t bpm)
{
    buzzer.bpm = bpm;
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_buzzer_pwm(uint16_t period, uint16_t duty_cycle)
{
    pwmConfig.timerPeriod = period;
    pwmConfig.dutyCycle   = duty_cycle;

    Timer_A_generatePWM(BUZZER_TIMER, &pwmConfig);
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_buzzer_play_tune(const tune_t tune, int8_t volume) {
    uint8_t index;
    uint16_t period;
    uint16_t duty_cycle;
    uint8_t length;

    /*  Normalize volume */
    if (volume < 1) {
        volume = 1;
    } else if (volume > 10) {
        volume = 10;
    }
    /* Recover note index and length */
    index = tune.note;
    length = tune.length;

    /* Set the note period and duty cycle */
    period     = notes[index].period      + volume * VOLUME_VALUE;
    duty_cycle = notes[index].duty_cycle  + volume * VOLUME_VALUE;

    /* Set the note duration */
    upConfig.timerPeriod = length * buzzer.bpm;

    /* Configure timer for up mode */
    MAP_Timer_A_configureUpMode(TIMER_BASE, &upConfig);
    MAP_Timer_A_clearInterruptFlag(TIMER_BASE);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_BASE, TIMER_REGISTER);
    MAP_Timer_A_startCounter(TIMER_BASE, TIMER_MODE);
    MAP_Interrupt_enableInterrupt(TIMER_INTERRUPT);

    /* Start the PWM */
    edu_boosterpack_buzzer_pwm(period, duty_cycle);
}

void TA1_0_IRQHandler(void) {
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_BASE, TIMER_REGISTER);

    edu_boosterpack_buzzer_callback();
}
