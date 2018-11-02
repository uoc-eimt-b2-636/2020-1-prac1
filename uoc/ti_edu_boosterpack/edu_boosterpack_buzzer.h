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

#ifndef MSP432_EDU_BOOSTERPACK_BUZZER_H_
#define MSP432_EDU_BOOSTERPACK_BUZZER_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
/*----------------------------------------------------------------------------*/
#define G5              (  0 )
#define Gs5             (  1 )
#define A5              (  2 )
#define Bb5             (  3 )
#define B5              (  4 )
#define C6              (  5 )
#define Cs6             (  6 )
#define D6              (  7 )
#define Eb6             (  8 )
#define E6              (  9 )
#define F6              ( 10 )
#define Fs6             ( 11 )
#define G6              ( 12 )
#define Gs6             ( 13 )
#define A6              ( 14 )
#define Bb6             ( 15 )
#define B6              ( 16 )
#define C7              ( 17 )
#define M               ( 18 )
/*----------------------------------------------------------------------------*/
#define _SQ             (  1 )
#define _Q              (  2 )
#define _QP             (  3 )
#define _C              (  4 )
#define _CP             (  6 )
#define _M              (  8 )
#define _L              ( 16 )
/*----------------------------------------------------------------------------*/
typedef void (*buzzer_cb_t)(void);
/*----------------------------------------------------------------------------*/
typedef struct {
    uint16_t period;
    uint8_t duty_cycle;
} note_t;
/*----------------------------------------------------------------------------*/
typedef struct {
    uint8_t note;
    uint8_t length;
} tune_t;
/*----------------------------------------------------------------------------*/
void edu_boosterpack_buzzer_init(void);
void edu_boosterpack_buzzer_callback_set(buzzer_cb_t callback);
void edu_boosterpack_buzzer_callback_clear(void);
void edu_boosterpack_set_bpm(uint16_t bpm);
void edu_boosterpack_buzzer_pwm(uint16_t period, uint16_t duty_cycle);
void edu_boosterpack_buzzer_play_tune(tune_t tune, int8_t volume);
/*----------------------------------------------------------------------------*/

#endif /* MSP432_EDU_BOOSTERPACK_BUZZER_H_ */
