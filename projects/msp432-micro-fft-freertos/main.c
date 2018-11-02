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

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* Launchpad, Wi-Fi and Sensors includes */
#include "msp432_launchpad_board.h"
#include "edu_boosterpack_microphone.h"
//#include "cc3100_boosterpack.h"

/* TI Graphics library */
#include "st7735.h"
#include "st7735_msp432.h"
#include "grlib.h"

#include <arm_math.h>
#include "arm_const_structs.h"

/*----------------------------------------------------------------------------*/

#define MAIN_TASK_PRIORITY      ( tskIDLE_PRIORITY + 2 )
#define BLINK_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define MAIN_STACK_SIZE         ( 2048 )
#define BLINK_STACK_SIZE        ( 128 )

#define SAMPLE_LENGTH           ( 512 )

/*----------------------------------------------------------------------------*/

static SemaphoreHandle_t semaphoreMic;
static QueueHandle_t queueMic, queueFFT;

/* Graphic library context */
Graphics_Context g_sContext;

/* FFT data/processing buffers*/

static int16_t fft_values[SAMPLE_LENGTH];
static int16_t screen_values[SAMPLE_LENGTH];
static bool paint_vector[SAMPLE_LENGTH];



/*----------------------------------------------------------------------------*/

static void BlinkTask(void *pvParameters);

static void microphone_interrupt(void);

static void init_display_FFT(void);
static void init_HannWindow(float *hann);
static void compute_FFT(int16_t *data_buffer, int16_t *data_output,
                        uint32_t *maxIndex, float *hann);
static uint16_t check_display_FFT(int16_t* old_vector, int16_t* new_vector,
bool* paint_vector);
static void display_FFT(int16_t maxIndex, int16_t* data, bool* paint_vector);

/*----------------------------------------------------------------------------*/

static void BlinkTask(void *pvParameters)
{
    while (true)
    {
        /* Turn red LED on */
        led_red_on();

        /* Sleep for 10 ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Turn red LED on */
        led_red_off();

        /* Sleep for 990 ms */
        vTaskDelay(pdMS_TO_TICKS(990));
    }
}

static void MicrophoneTask(void *pvParameters)
{
    uint32_t start_time;

    /* Start the microphone */
    edu_boosterpack_microphone_init();
    edu_boosterpack_microphone_callback_set(microphone_interrupt);

    init_display_FFT();


    /* Start collecting samples from the microphone */
    edu_boosterpack_microphone_start();

    start_time = xTaskGetTickCount();

    while (true)
    {
        /* Wait to be notified from interrupt */
        if (xSemaphoreTake(semaphoreMic, portMAX_DELAY) == pdTRUE)
        {
            debug1_on();

            /* Turn green LED on */
            led_green_on();

            /* Stop collecting samples from the microphone */
            edu_boosterpack_microphone_stop();

            /* Get data samples from microphone */
            int16_t* data_buffer;
            edu_boosterpack_microphone_get_data(&data_buffer);
            xQueueSend(queueMic, &data_buffer, portMAX_DELAY);
        }
    }
}

static void FFTTask(void *pvParameters)
{
    int16_t *data_buffer;
    uint32_t maxIndex;
    float hann[SAMPLE_LENGTH];

    /* Initialize Hanning Window */
    init_HannWindow(hann);

    while (pdTRUE)
    {
        if (xQueueReceive(queueMic, &data_buffer, portMAX_DELAY) == pdTRUE)
        {

            /* Compute FFT */
            debug2_on();
            compute_FFT(data_buffer, fft_values, &maxIndex, hann);
            debug2_off();

            xQueueSend(queueFFT, &maxIndex, portMAX_DELAY);
        }
    }
}

static void DrawTask(void *pvParameters)
{
    uint32_t current_time;
    uint32_t elapsed_time;
    uint16_t changed_bins;
    uint32_t maxIndex;
    uint32_t start_time;
    uint8_t frames_string[6];
    uint16_t frames = 0;

    while (pdTRUE)
    {
        if (xQueueReceive(queueFFT, &maxIndex, portMAX_DELAY) == pdTRUE)
        {

            /* Check the number of bins that need to be updated */
            changed_bins = check_display_FFT(fft_values, screen_values,
                                             paint_vector);

            if (changed_bins < 48)
            {
                /* Restart microphone acquisition */
                edu_boosterpack_microphone_restart();
                edu_boosterpack_microphone_start();
            }

            /* Draw magnitude of FFT on LCD */
            debug3_on();
            display_FFT(maxIndex, screen_values, paint_vector);
            debug3_off();

            if (changed_bins >= 48)
            {
                /* Restart microphone acquisition */
                edu_boosterpack_microphone_restart();
                edu_boosterpack_microphone_start();
            }

            /* Upgrade number of frames */
            frames++;

            current_time = xTaskGetTickCount();
            elapsed_time = current_time - start_time;

            if (elapsed_time > configTICK_RATE_HZ)
            {
                sprintf((char *) frames_string, "%02d", frames);

                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
                Graphics_drawStringCentered(&g_sContext,
                                            (int8_t *) frames_string, 2, 96, 6,
                                            OPAQUE_TEXT);

                frames = 0;
                elapsed_time = 0;
                start_time = current_time;
            }

            /* Turn green LED off */
            led_green_off();

            debug1_off();
        }
    }

}

/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal;

    /* Initialize the board */
    board_init();

    /*
     * Create binary semaphore for synchronization
     */
    semaphoreMic = xSemaphoreCreateBinary();
    if (semaphoreMic == NULL)
    {
        led_red_on();
        while (1)
            ;
    }

    queueMic = xQueueCreate(5, sizeof(int16_t *));
    queueFFT = xQueueCreate(5, sizeof(uint32_t));

    if ((queueMic == NULL) || (queueFFT == NULL))
    {
        led_red_on();
        while (1)
            ;
    }

    /* Create blink task */
    retVal = xTaskCreate(BlinkTask, "BlinkTask",
    BLINK_STACK_SIZE,
                         NULL,
                         BLINK_TASK_PRIORITY,
                         NULL);
    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Create acquisition and display task */
    retVal = xTaskCreate(MicrophoneTask, "MicTask",
    MAIN_STACK_SIZE,
                         NULL,
                         MAIN_TASK_PRIORITY,
                         NULL);

    retVal += xTaskCreate(FFTTask, "FFTTask",
    MAIN_STACK_SIZE,
                          NULL,
                          MAIN_TASK_PRIORITY,
                          NULL);

    retVal += xTaskCreate(DrawTask, "DrawTask",
    MAIN_STACK_SIZE,
                          NULL,
                          MAIN_TASK_PRIORITY,
                          NULL);

    if (retVal < 0)
    {
        led_red_on();
        while (1)
            ;
    }

    /* Start the task scheduler */
    vTaskStartScheduler();

    return 0;
}

/*----------------------------------------------------------------------------*/

static void microphone_interrupt(void)
{
    static BaseType_t xHigherPriorityTaskWoken;

    /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR(semaphoreMic, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void init_display_FFT(void)
{
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);

    /* Draw Title, x-axis, gradation & labels */
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawLineH(&g_sContext, 0, 127, 115);
    Graphics_drawLineV(&g_sContext, 0, 115, 117);
    Graphics_drawLineV(&g_sContext, 16, 115, 116);
    Graphics_drawLineV(&g_sContext, 31, 115, 117);
    Graphics_drawLineV(&g_sContext, 32, 115, 117);
    Graphics_drawLineV(&g_sContext, 48, 115, 116);
    Graphics_drawLineV(&g_sContext, 63, 115, 117);
    Graphics_drawLineV(&g_sContext, 64, 115, 117);
    Graphics_drawLineV(&g_sContext, 80, 115, 116);
    Graphics_drawLineV(&g_sContext, 95, 115, 117);
    Graphics_drawLineV(&g_sContext, 96, 115, 117);
    Graphics_drawLineV(&g_sContext, 112, 115, 116);
    Graphics_drawLineV(&g_sContext, 127, 115, 117);

    Graphics_drawStringCentered(&g_sContext, "512-Point FFT, ",
    AUTO_STRING_LENGTH,
                                48, 6,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "fps",
    AUTO_STRING_LENGTH,
                                116, 6,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "0",
    AUTO_STRING_LENGTH,
                                4, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "1",
    AUTO_STRING_LENGTH,
                                32, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "2",
    AUTO_STRING_LENGTH,
                                64, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "3",
    AUTO_STRING_LENGTH,
                                96, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "4",
    AUTO_STRING_LENGTH,
                                125, 122,
                                OPAQUE_TEXT);
    Graphics_drawStringCentered(&g_sContext, "kHz",
    AUTO_STRING_LENGTH,
                                112, 122,
                                OPAQUE_TEXT);
}

static void init_HannWindow(float *hann)
{
    int n;
    for (n = 0; n < SAMPLE_LENGTH; n++)
    {
        hann[n] = 0.5 - 0.5 * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
    }
}

static void compute_FFT(int16_t *data_buffer, int16_t *data_output,
                        uint32_t *maxIndex, float *hann)
{
    uint32_t fftSize = SAMPLE_LENGTH;
    uint32_t ifftFlag = 0;
    uint32_t doBitReverse = 1;
    volatile arm_status status;
    int i;

    int16_t data_hannW[SAMPLE_LENGTH];
    int16_t data_input[SAMPLE_LENGTH * 2];

    /* Filter the data using a hanning window */
    for (i = 0; i < SAMPLE_LENGTH; i++)
    {
        data_hannW[i] = (int16_t) (hann[i] * data_buffer[i]);
    }

    /* Computer real FFT using the completed data buffer */
    arm_rfft_instance_q15 instance;
    status = arm_rfft_init_q15(&instance, fftSize, ifftFlag, doBitReverse);
    arm_rfft_q15(&instance, data_hannW, data_input);

    /* Calculate magnitude of FFT complex output */
    for (i = 0; i < 1024; i += 2)
    {
        data_output[i / 2] = (int32_t) (sqrtf(
                (data_input[i] * data_input[i])
                        + (data_input[i + 1] * data_input[i + 1])));
    }

    q15_t maxValue;
    *maxIndex = 0;

    /* Find the maximum value */
    arm_max_q15(data_output, fftSize, &maxValue, maxIndex);
}

static uint16_t check_display_FFT(int16_t* fft_vector, int16_t* screen_vector,
bool* paint_vector)
{
    uint16_t i, j;
    uint16_t changed;

    changed = 0;

    for (i = 0, j = 0; i < 256; i += 2, j += 1)
    {
        uint16_t new, old;
        int16_t diff;

        /* Average two consequtive bins to fit them into the screen */
        new = min(100, (int )((fft_vector[i] + fft_vector[i + 1]) / 8));

        old = screen_vector[i];
        screen_vector[i] = new;

        if (new >= old)
        {
            paint_vector[i] = false;
        }
        else
        {
            paint_vector[i] = true;
            changed++;
        }
    }

    return changed;
}

static void display_FFT(int16_t maxIndex, int16_t *data, bool* paint)
{
    uint16_t i;

    /* Draw frequency bin graph */
    for (i = 0; i < 256; i += 1)
    {
        int16_t x = data[i];

        if (paint[i] == true)
        {
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawLineV(&g_sContext, i / 2, 114 - x, 14);
        }

        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_GREEN);
        Graphics_drawLineV(&g_sContext, i / 2, 114, 114 - x);
    }
}
