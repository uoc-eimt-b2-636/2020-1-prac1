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

#include "msp432_launchpad_temperature.h"

#include "adc14.h"

static int32_t adcRefTempCal_1_2v_30;
static int32_t adcRefTempCal_1_2v_85;

void msp432_launchpad_temperature_init(void)
{
    // Read the ADC temperature reference calibration value
    adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;
    adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;

    // Initialize the shared reference module
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
    // If ref generator busy, WAIT
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);

    // Enable internal 1.2V reference and turn it on
    REF_A->CTL0 |= REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON;

    // Enable temperature sensor
    REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;

    // Configure ADC in pulse sample mode; ADC14_CTL0_SC trigger
    // ADC ON, temperature sample period > 5us
    ADC14->CTL0 |= ADC14_CTL0_SHT0_6 | ADC14_CTL0_ON | ADC14_CTL0_SHP;

    // Enable internal temperature sensor
    ADC14->CTL1 |= ADC14_CTL1_TCMAP;

    // ADC input channel A22 for temperature sensing
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_22;

    // Wait for reference generator to settle
    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));

    // ADC enable conversion
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

float msp432_launchpad_temperature_read(void)
{
    float temperature;

    // Sampling and conversion start
    ADC14->CTL0 |= ADC14_CTL0_SC;

    // Wait until conversion finishes
    while(ADC14->CTL0 & ADC14_CTL0_BUSY);

    // Temperature in Celsius
    temperature = (((float) ADC14->MEM[0] - adcRefTempCal_1_2v_30) * (85 - 30)) /
            (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;

    return temperature;
}
