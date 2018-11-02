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
#include "edu_boosterpack_microphone.h"

#include "driverlib.h"
/*----------------------------------------------------------------------------*/

#define SMCLK_FREQUENCY     ( 48000000 )
#define SAMPLE_FREQUENCY    ( 8000 )

#define BUFFER_SIZE         ( 512 )
#define BUFFER_BYTES        ( 2 * BUFFER_SIZE )

#define MICROPHONE_PORT     ( GPIO_PORT_P4 )
#define MICROPHONE_PIN      ( GPIO_PIN3 )
#define MICROPHONE_MODE     ( GPIO_TERTIARY_MODULE_FUNCTION )

#define TIMER_BASE          ( TIMER_A3_BASE )

#define ADC_CLOCK_SRC       ( ADC_CLOCKSOURCE_MCLK )
#define ADC_PRE_DIVIDER     ( ADC_PREDIVIDER_1 )
#define ADC_DIVIDER         ( ADC_DIVIDER_1 )
#define ADC_TRIGGER         ( ADC_TRIGGER_SOURCE7 )
#define ADC_ROUTE           ( ADC_NOROUTE )
#define ADC_MEMORY          ( ADC_MEM0 )
#define ADC_REFERENCE       ( ADC_VREFPOS_AVCC_VREFNEG_VSS )
#define ADC_INPUT           ( ADC_INPUT_A10 )
#define ADC_FORMAT          ( ADC_SIGNED_BINARY )

#define DMA_CHANNEL         ( DMA_CH7_ADC14 )
#define DMA_CONFIG          ( UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK )
#define DMA_NUMBER          ( 7 )
#define DMA_IRQ             ( DMA_INT1 )

#define INTERRUPT_DMA       ( INT_DMA_INT1 )
#define INTERRUPT_DMA_PRIO  ( 0xF0 )

/*----------------------------------------------------------------------------*/

typedef struct {
    microphone_cb_t callback;
    bool buffer;
    int16_t* buffer_1;
    int16_t* buffer_2;
} microphone_t;

/*----------------------------------------------------------------------------*/
static void edu_boosterpack_microphone_gpio_init(void);
static void edu_boosterpack_microphone_adc_init(void);
static void edu_boosterpack_microphone_dma_init(void);
static void edu_boosterpack_microphone_callback(void);
/*----------------------------------------------------------------------------*/

extern DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable1[32];

static int16_t buffer_1[BUFFER_SIZE];
static int16_t buffer_2[BUFFER_SIZE];

static microphone_t microphone = {.callback = NULL, .buffer = false, .buffer_1 = buffer_1, .buffer_2 = buffer_2};

/* Timer_A PWM Configuration Parameter */
static Timer_A_PWMConfig pwmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        (SMCLK_FREQUENCY/SAMPLE_FREQUENCY),
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_SET_RESET,
        (SMCLK_FREQUENCY/SAMPLE_FREQUENCY)/2
};
/*----------------------------------------------------------------------------*/
void edu_boosterpack_microphone_init(void)
{
    edu_boosterpack_microphone_gpio_init();
    edu_boosterpack_microphone_adc_init();
    edu_boosterpack_microphone_dma_init();
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_microphone_callback_set(microphone_cb_t callback)
{
    microphone.callback = callback;
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_microphone_callback_clear(void)
{
    microphone.callback = NULL;
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_microphone_start(void)
{
    /* Enable DMA and ADC to start operation */
    MAP_DMA_enableChannel(DMA_NUMBER);
    MAP_ADC14_enableConversion();
}

void edu_boosterpack_microphone_stop(void)
{
    /* Disable DMA and ADC to start operation */
    MAP_DMA_disableChannel(DMA_NUMBER);
    MAP_ADC14_disableConversion();
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_microphone_restart(void)
{
    /* Switch between primary and alternate buffers with DMA's ping-pong mode */
    if (microphone.buffer == true)
    {
        MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
        MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14, UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0], microphone.buffer_1, BUFFER_SIZE);
    }
    else
    {
        MAP_DMA_setChannelControl(UDMA_ALT_SELECT | DMA_CH7_ADC14, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
        MAP_DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14, UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0], microphone.buffer_2, BUFFER_SIZE);
    }
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_microphone_get_data(int16_t** buffer) {
    if (microphone.buffer)
    {
        *buffer = (int16_t* ) microphone.buffer_1;
    }
    else
    {
        *buffer = (int16_t* ) microphone.buffer_2;
    }
}

/*----------------------------------------------------------------------------*/
static void edu_boosterpack_microphone_callback(void)
{
    if (microphone.callback != NULL)
    {
        microphone.callback();
    }
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_microphone_gpio_init(void)
{
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(MICROPHONE_PORT, MICROPHONE_PIN, MICROPHONE_MODE);
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_microphone_adc_init(void) {
    /* Configuring Timer_A to generate the sampling period  */
    MAP_Timer_A_generatePWM(TIMER_BASE, &pwmConfig);

    /* Enable and initialize ADC module */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCK_SRC, ADC_PRE_DIVIDER, ADC_DIVIDER, ADC_NOROUTE);

    /* Set the trigger source for the ADC */
    MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER, false);

    /* Configuring ADC memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEMORY, true);
    MAP_ADC14_configureConversionMemory(ADC_MEMORY, ADC_REFERENCE, ADC_INPUT, false);

    /* Set ADC result format to signed binary */
    MAP_ADC14_setResultFormat(ADC_FORMAT);
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_microphone_dma_init(void) {
    /* Configuring DMA module */
    MAP_DMA_enableModule();
    MAP_DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable1);
    MAP_DMA_disableChannelAttribute(DMA_CHANNEL, DMA_CONFIG);

    /* Set DMA transfer from ADC14 Memory 0 to destination data array in ping-pong mode */
    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14, UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0], microphone.buffer_1, BUFFER_SIZE);
    MAP_DMA_setChannelControl(UDMA_ALT_SELECT | DMA_CH7_ADC14, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    MAP_DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14, UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0], microphone.buffer_2, BUFFER_SIZE);

    /* Assigning and enable DMA interrupts */
    MAP_DMA_assignInterrupt(DMA_IRQ, DMA_NUMBER);
    MAP_DMA_assignChannel(DMA_CHANNEL);

    /* Clear DMA interrupt flag */
    MAP_DMA_clearInterruptFlag(DMA_NUMBER);

    /* Configure and enable DMA interrupt */
    MAP_Interrupt_setPriority(INTERRUPT_DMA, INTERRUPT_DMA_PRIO);
    MAP_Interrupt_enableInterrupt(INTERRUPT_DMA);
}
/*----------------------------------------------------------------------------*/
void DMA_INT1_IRQHandler(void)
{
    /* Switch between primary and alternate buffers with DMA's ping-pong mode */
    if (MAP_DMA_getChannelAttribute(DMA_NUMBER) & UDMA_ATTR_ALTSELECT)
    {
        microphone.buffer = true;
    }
    else
    {
        microphone.buffer = false;
    }

    /* Execute callback */
    edu_boosterpack_microphone_callback();
}
/*----------------------------------------------------------------------------*/
