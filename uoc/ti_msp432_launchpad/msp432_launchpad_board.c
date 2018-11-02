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

#include "msp432_launchpad_board.h"

#include "driverlib.h"

/*----------------------------------------------------------------------------*/

static void board_init_leds(void);
static void board_init_debug(void);
static void board_init_buttons(void);
static void board_init_clk(void);

static void gpio_on(uint8_t port, uint16_t pin);
static void gpio_off(uint8_t port, uint16_t pin);
static void gpio_toggle(uint8_t port, uint16_t pin);

/*----------------------------------------------------------------------------*/

#define LED_RED_PORT            ( GPIO_PORT_P1 )
#define LED_RED_PIN             ( GPIO_PIN0 )
#define LED_RED1_PORT           ( GPIO_PORT_P2 )
#define LED_RED1_PIN            ( GPIO_PIN0 )
#define LED_GREEN_PORT          ( GPIO_PORT_P2 )
#define LED_GREEN_PIN           ( GPIO_PIN1 )
#define LED_BLUE_PORT           ( GPIO_PORT_P2 )
#define LED_BLUE_PIN            ( GPIO_PIN2 )

#define DEBUG1_PORT             ( GPIO_PORT_P4 )
#define DEBUG1_PIN              ( GPIO_PIN5 )
#define DEBUG2_PORT             ( GPIO_PORT_P4 )
#define DEBUG2_PIN              ( GPIO_PIN7 )
#define DEBUG3_PORT             ( GPIO_PORT_P5 )
#define DEBUG3_PIN              ( GPIO_PIN4 )
#define DEBUG4_PORT             ( GPIO_PORT_P5 )
#define DEBUG4_PIN              ( GPIO_PIN5 )

#define BUTTON1_PORT            ( GPIO_PORT_P1)
#define BUTTON1_PIN             ( GPIO_PIN1 )
#define BUTTON2_PORT            ( GPIO_PORT_P1)
#define BUTTON2_PIN             ( GPIO_PIN4 )

#define LCD_CS_PORT             ( GPIO_PORT_P5 )
#define LCD_CS_PIN              ( GPIO_PIN0 )

/*----------------------------------------------------------------------------*/
void board_init(void)
{
    board_init_leds();
    board_init_debug();
    board_init_buttons();
    board_init_clk();
}
/*----------------------------------------------------------------------------*/
void led_red_on(void) {
    gpio_on(LED_RED_PORT, LED_RED_PIN);
}
/*----------------------------------------------------------------------------*/
void led_red_off(void) {
    gpio_off(LED_RED_PORT, LED_RED_PIN);
}
/*----------------------------------------------------------------------------*/
void led_red_toggle(void) {
    gpio_toggle(LED_RED_PORT, LED_RED_PIN);
}
/*----------------------------------------------------------------------------*/
void led_blue_on(void) {
    gpio_on(LED_BLUE_PORT, LED_BLUE_PIN);
}
/*----------------------------------------------------------------------------*/
void led_blue_off(void) {
    gpio_off(LED_BLUE_PORT, LED_BLUE_PIN);
}
/*----------------------------------------------------------------------------*/
void led_blue_toggle(void) {
    gpio_toggle(LED_BLUE_PORT, LED_BLUE_PIN);
}
/*----------------------------------------------------------------------------*/
void led_green_on(void) {
    gpio_on(LED_GREEN_PORT, LED_GREEN_PIN);
}
/*----------------------------------------------------------------------------*/
void led_green_off(void) {
    gpio_off(LED_GREEN_PORT, LED_GREEN_PIN);
}
/*----------------------------------------------------------------------------*/
void led_green_toggle(void) {
    gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
}
/*----------------------------------------------------------------------------*/
void led_red1_on(void) {
    gpio_on(LED_RED1_PORT, LED_RED1_PIN);
}
/*----------------------------------------------------------------------------*/
void led_red1_off(void) {
    gpio_off(LED_RED1_PORT, LED_RED1_PIN);
}
/*----------------------------------------------------------------------------*/
void led_red1_toggle(void) {
    gpio_toggle(LED_RED1_PORT, LED_RED1_PIN);
}
/*----------------------------------------------------------------------------*/
void debug1_on(void) {
    gpio_on(DEBUG1_PORT, DEBUG1_PIN);
}
/*----------------------------------------------------------------------------*/
void debug1_off(void) {
    gpio_off(DEBUG1_PORT, DEBUG1_PIN);
}
/*----------------------------------------------------------------------------*/
void debug1_toggle(void) {
    gpio_toggle(DEBUG1_PORT, DEBUG1_PIN);
}
/*----------------------------------------------------------------------------*/
void debug2_on(void) {
    gpio_on(DEBUG2_PORT, DEBUG2_PIN);
}
/*----------------------------------------------------------------------------*/
void debug2_off(void) {
    gpio_off(DEBUG2_PORT, DEBUG2_PIN);
}
/*----------------------------------------------------------------------------*/
void debug2_toggle(void) {
    gpio_toggle(DEBUG2_PORT, DEBUG2_PIN);
}
/*----------------------------------------------------------------------------*/
void debug3_on(void) {
    gpio_on(DEBUG3_PORT, DEBUG3_PIN);
}
/*----------------------------------------------------------------------------*/
void debug3_off(void) {
    gpio_off(DEBUG3_PORT, DEBUG3_PIN);
}
/*----------------------------------------------------------------------------*/
void debug3_toggle(void) {
    gpio_toggle(DEBUG3_PORT, DEBUG3_PIN);
}
/*----------------------------------------------------------------------------*/
void debug4_on(void) {
    gpio_on(DEBUG4_PORT, DEBUG4_PIN);
}
/*----------------------------------------------------------------------------*/
void debug4_off(void) {
    gpio_off(DEBUG4_PORT, DEBUG4_PIN);
}
/*----------------------------------------------------------------------------*/
void debug4_toggle(void) {
    gpio_toggle(DEBUG4_PORT, DEBUG4_PIN);
}
/*----------------------------------------------------------------------------*/
static void gpio_on(uint8_t port, uint16_t pin) {
    MAP_GPIO_setOutputHighOnPin(port, pin);
}
/*----------------------------------------------------------------------------*/
static void gpio_off(uint8_t port, uint16_t pin) {
    MAP_GPIO_setOutputLowOnPin(port, pin);
}
/*----------------------------------------------------------------------------*/
static void gpio_toggle(uint8_t port, uint16_t pin) {
    MAP_GPIO_toggleOutputOnPin(port, pin);
}
/*----------------------------------------------------------------------------*/
static void board_init_leds(void)
{
    MAP_GPIO_setOutputLowOnPin(LED_RED_PORT, LED_RED_PIN);
    MAP_GPIO_setAsOutputPin(LED_RED_PORT, LED_RED_PIN);

    MAP_GPIO_setOutputLowOnPin(LED_GREEN_PORT, LED_GREEN_PIN);
    MAP_GPIO_setAsOutputPin(LED_GREEN_PORT, LED_GREEN_PIN);

    MAP_GPIO_setOutputLowOnPin(LED_RED1_PORT, LED_RED1_PIN);
    MAP_GPIO_setAsOutputPin(LED_RED1_PORT, LED_RED1_PIN);

    MAP_GPIO_setOutputLowOnPin(LED_BLUE_PORT, LED_BLUE_PIN);
    MAP_GPIO_setAsOutputPin(LED_BLUE_PORT, LED_BLUE_PIN);
}
/*----------------------------------------------------------------------------*/
static void board_init_buttons(void)
{
    MAP_GPIO_setAsInputPinWithPullUpResistor(BUTTON1_PORT, BUTTON1_PIN);
    MAP_GPIO_clearInterruptFlag(BUTTON1_PORT, BUTTON1_PIN);
    MAP_GPIO_enableInterrupt(BUTTON1_PORT, BUTTON1_PIN);

    MAP_GPIO_setAsInputPinWithPullUpResistor(BUTTON2_PORT, BUTTON2_PIN);
    MAP_GPIO_clearInterruptFlag(BUTTON2_PORT, BUTTON2_PIN);
    MAP_GPIO_enableInterrupt(BUTTON2_PORT, BUTTON2_PIN);

    MAP_Interrupt_enableInterrupt(INT_PORT1);
}
/*----------------------------------------------------------------------------*/
static void board_init_debug(void)
{
    MAP_GPIO_setOutputLowOnPin(DEBUG1_PORT, DEBUG1_PIN);
    MAP_GPIO_setAsOutputPin(DEBUG1_PORT, DEBUG1_PIN);

    MAP_GPIO_setOutputLowOnPin(DEBUG2_PORT, DEBUG2_PIN);
    MAP_GPIO_setAsOutputPin(DEBUG2_PORT, DEBUG2_PIN);

    MAP_GPIO_setOutputLowOnPin(DEBUG3_PORT, DEBUG3_PIN);
    MAP_GPIO_setAsOutputPin(DEBUG3_PORT, DEBUG3_PIN);

    MAP_GPIO_setOutputLowOnPin(DEBUG4_PORT, DEBUG4_PIN);
    MAP_GPIO_setAsOutputPin(DEBUG4_PORT, DEBUG4_PIN);
}
/*----------------------------------------------------------------------------*/
static void board_init_clk(void)
{
    /* Halt the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Ensure the FPU is enabled */
    MAP_FPU_enableModule();

    /* Configuring pins for peripheral/crystal */
    // MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Set the clock frequency in the code */
    MAP_CS_setExternalClockSourceFrequency(32768, 48000000);

    /* Change VCORE to 1 to support the 48 MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set FLASH wait states */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Starting HFXT/LFXT in non-bypass mode without a timeout */
    // MAP_CS_startHFXT(false);
    MAP_CS_startLFXT(false);

    /* Setting the DCO Frequency to 48 MHz */
    MAP_CS_setDCOFrequency(48000000);

    /* Initializing MCLK, HSMCLK, SMCLK and ACLK based on DCO and LFXT */
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Enable CPU after IRQ */
    MAP_Interrupt_disableSleepOnIsrExit();

    /* Globally enable interrupts */
    MAP_Interrupt_enableMaster();

    /* Ensure LCD CS pin is not floating to avoid problems */
    MAP_GPIO_setAsOutputPin(LCD_CS_PORT, LCD_CS_PIN);
    MAP_GPIO_setOutputHighOnPin(LCD_CS_PORT, LCD_CS_PIN);
}
/*----------------------------------------------------------------------------*/
