/******************************************************************************
 *  Filename:       tmp006.c
 *  Revised:
 *  Revision:
 *
 *  Description:    Driver for the TI TMP06 infrared thermophile sensor.
 *
 *  Copyright (C) 2014 - 2015 Texas Instruments Incorporated - http://www.ti.com/
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
 *    documentation and/or other materials provided with the distribution.
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
 ******************************************************************************/

/* -----------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "msp432.h"
#include "driverlib.h"
#include "i2c_driver.h"
#include "tmp006.h"

/* -----------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------
 */

/* CONSTANTS */
#define TMP006_SLAVE_ADDRESS    0x40

/* TEMPERATURE SENSOR REGISTER DEFINITIONS */
#define TMP006_P_VOBJ           0x00
#define TMP006_P_TABT           0x01
#define TMP006_WRITE_REG        0x02
#define TMP006_P_MAN_ID         0xFE
#define TMP006_P_DEVICE_ID      0xFF

/* CONFIGURATION REGISTER SETTINGS */
#define TMP006_RST              0x8000
#define TMP006_POWER_DOWN       0x0000
#define TMP006_POWER_UP         0x7000
#define TMP006_CR_4             0x0000
#define TMP006_CR_2             0x0200
#define TMP006_CR_1             0x0400
#define TMP006_CR_0_5           0x0600
#define TMP006_CR_0_25          0x0800
#define TMP006_EN               0x0100
#define TMP006_DRDY             0x0080

/* -----------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------
 */
static int TMP006_readDeviceId(void);
static int TMP006_readObjectVoltage(void);

/* -----------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------
 */

/* Calibration constant for TMP006 */
static long double S0 = 0;

/* -----------------------------------------------------------------------------
 *                                           Public functions
 * ------------------------------------------------------------------------------
 */

void TMP006_init(void)
{
    uint16_t val;

    /* Reset TMP006 */
    val = TMP006_RST;
    I2C_write16(TMP006_SLAVE_ADDRESS, TMP006_WRITE_REG, val);

    volatile int i;
    for (i=10000; i>0;i--);

    /* Power-up and re-enable device */
    val = TMP006_POWER_UP | TMP006_CR_2;
    I2C_write16(TMP006_SLAVE_ADDRESS, TMP006_WRITE_REG, val);
}

float TMP006_readAmbientTemperature(void)
{
    uint16_t result;

    I2C_read16(TMP006_SLAVE_ADDRESS, TMP006_P_TABT, &result);

    return result/128.0f;
}

float TMP006_readObjectTemperature(void)
{
    volatile int Vobj = 0;
    volatile int Tdie = 0;

    Vobj = TMP006_readDeviceId();

    /* Read the object voltage */
    Vobj = TMP006_readObjectVoltage();

    /* Read the ambient temperature */
    Tdie = TMP006_readAmbientTemperature();
    Tdie = Tdie >> 2;

    /* Calculate TMP006. This needs to be reviewed and calibrated */
    long double Vobj2 = (double)Vobj*.00000015625;
    long double Tdie2 = (double)Tdie*.03525 + 273.15;

    /* Initialize constants */
    S0 = 6 * pow(10, -14);
    long double a1 = 1.75*pow(10, -3);
    long double a2 = -1.678*pow(10, -5);
    long double b0 = -2.94*pow(10, -5);
    long double b1 = -5.7*pow(10, -7);
    long double b2 = 4.63*pow(10, -9);
    long double c2 = 13.4;
    long double Tref = 298.15;

    /* Calculate values */
    long double S = S0*(1+a1*(Tdie2 - Tref)+a2*pow((Tdie2 - Tref),2));
    long double Vos = b0 + b1*(Tdie2 - Tref) + b2*pow((Tdie2 - Tref),2);
    volatile long double fObj = (Vobj2 - Vos) + c2*pow((Vobj2 - Vos),2);
    volatile long double Tobj = pow(pow(Tdie2,4) + (fObj/S),.25);
    Tobj = (9.0/5.0)*(Tobj - 273.15) + 32;

    /* Return temperature of object */
    return (Tobj);
}

/* -----------------------------------------------------------------------------
 *                                           Private functions
 * ------------------------------------------------------------------------------
 */

static int TMP006_readDeviceId(void)
{
    uint16_t result;

    I2C_read16(TMP006_SLAVE_ADDRESS, TMP006_P_DEVICE_ID, &result);

    return result;
}

static int TMP006_readObjectVoltage(void)
{
    uint16_t result;

    I2C_read16(TMP006_SLAVE_ADDRESS, TMP006_P_VOBJ, &result);

    return result;
}
