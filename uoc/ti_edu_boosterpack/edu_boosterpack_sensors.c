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

#include "edu_boosterpack_sensors.h"

#include "driverlib.h"

#include "i2c_driver.h"
#include "tmp006.h"
#include "opt3001.h"

/*----------------------------------------------------------------------------*/
#define ACCELERATION_X_PORT            ( GPIO_PORT_P4 )
#define ACCELERATION_X_PIN             ( GPIO_PIN0 )
#define ACCELERATION_X_MODE            ( GPIO_TERTIARY_MODULE_FUNCTION )
#define ACCELERATION_Y_PORT            ( GPIO_PORT_P4 )
#define ACCELERATION_Y_PIN             ( GPIO_PIN2 )
#define ACCELERATION_Y_MODE            ( GPIO_TERTIARY_MODULE_FUNCTION )
#define ACCELERATION_Z_PORT            ( GPIO_PORT_P6 )
#define ACCELERATION_Z_PIN             ( GPIO_PIN1 )
#define ACCELERATION_Z_MODE            ( GPIO_TERTIARY_MODULE_FUNCTION )
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_sensors_gpio_init(void);
static void edu_boosterpack_sensors_adc_init(void);
static void edu_boosterpack_sensors_i2c_init(void);
/*----------------------------------------------------------------------------*/
void edu_boosterpack_sensors_init(void)
{
    edu_boosterpack_sensors_gpio_init();
    edu_boosterpack_sensors_i2c_init();
    edu_boosterpack_sensors_adc_init();

    TMP006_init();

    sensorOpt3001Init();
    sensorOpt3001Enable(true);
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_sensors_acceleration_read(float temperature, int16_t* x, int16_t* y, int16_t* z)
{
    int32_t scratch_x, scratch_y, scratch_z;
    float sensitivity_xy, sensitivity_z;

    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM2,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    while(MAP_ADC14_getEnabledInterruptStatus() & ADC_INT2 != ADC_INT2)
        ;

    MAP_ADC14_clearInterruptFlag(ADC_INT2);

    scratch_x = ADC14_getResult(ADC_MEM0);
    scratch_y = ADC14_getResult(ADC_MEM1);
    scratch_z = ADC14_getResult(ADC_MEM2);

    sensitivity_xy = 1000000.0f / (660.0f * (1.0f + 0.0001f * temperature));
    sensitivity_z = 1000000.0f / (660.0f * (1.0f + 0.0004f * temperature));

    *x = sensitivity_xy * ((3.3f * scratch_x / 16384.0f) - 1.65f) + (0.7f * temperature);
    *y = sensitivity_xy * ((3.3f * scratch_y / 16384.0f) - 1.65f) + (0.7f * temperature);
    *z = sensitivity_z * ((3.3f * scratch_z / 16384.0f) - 1.65f) + (0.4f * temperature);
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_sensors_temperature_read(float *temperature)
{
    *temperature = TMP006_readAmbientTemperature();
}
/*----------------------------------------------------------------------------*/
void edu_boosterpack_sensors_light_read(float *light)
{
    uint16_t rawData = 0;

    sensorOpt3001Read(&rawData);
    sensorOpt3001Convert(rawData, light);
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_sensors_gpio_init(void)
{
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ACCELERATION_X_PORT, ACCELERATION_X_PIN, ACCELERATION_X_MODE);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ACCELERATION_Y_PORT, ACCELERATION_Y_PIN, ACCELERATION_Y_MODE);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ACCELERATION_Z_PORT, ACCELERATION_Z_PIN, ACCELERATION_Z_MODE);
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_sensors_i2c_init(void)
{
    /* Initialize I2C */
    I2C_init();
}
/*----------------------------------------------------------------------------*/
static void edu_boosterpack_sensors_adc_init(void)
{
    /* Initializing ADC */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);
}
/*----------------------------------------------------------------------------*/
