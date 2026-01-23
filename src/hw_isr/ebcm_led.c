/******************************************************************************
 * @file    ebcm_led.c
 * @brief   Add brief here
 *
 * MIT License
 *
 * Copyright (c) 2026 n0stalgic
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/


/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "ebcm_cfg.h"
#include "ebcm_led.h"
#include "IfxPort.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define MAX_NUM_LEDS 2U


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/


typedef enum
{
    LED_PATTERN_OFF = 0,
    LED_PATTERN_BLINK_0_2_SEC,
    LED_PATTERN_BLINK_0_5_SEC,
    LED_PATTERN_ABS_IN_DEV,
} led_flash_e;


typedef struct
{
        led_flash_e type;
        uint8* pattern_array;
        const Ifx_SizeT length;

} led_pattern;

typedef struct
{
        ebcm_led_index EBCM_LED;
        uint16 count;
        led_pattern active_pattern;

} EBCM_led_context;

#define LED_PATTERN_OFF_SEC_LEN   10U
const led_pattern led_pattern_off     =
                { .pattern_array = (const uint8[])
                                   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                  .length        = LED_PATTERN_OFF_SEC_LEN,
                  .type          = LED_PATTERN_OFF
                };


#define LED_PATTERN_0_2_SEC_LEN   40U
const led_pattern led_pattern_0_2_sec =
                                {  .pattern_array =  (const uint8[])
                                      { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },

                                      .length = LED_PATTERN_0_2_SEC_LEN,
                                      .type = LED_PATTERN_BLINK_0_2_SEC };


#define LED_PATTERN_0_5_SEC_LEN   100U
const led_pattern led_pattern_0_5_sec = { .pattern_array = (const uint8[])
                                    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },


                                    .length = LED_PATTERN_0_5_SEC_LEN,
                                    .type   = LED_PATTERN_BLINK_0_5_SEC };

#define LED_PATTERN_ABS_INDEV   290U
const led_pattern led_pattern_abs_indev = { .pattern_array = (const uint8[])
        {   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        },

        .length = LED_PATTERN_ABS_INDEV,
        .type = LED_PATTERN_ABS_IN_DEV
};


EBCM_led_context lc_1;
EBCM_led_context lc_2;


/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/* Initialize GPIO pins for LEDs */
void init_leds(void)
{
    /* Initialize GPIO pins for LEDs */
    IfxPort_setPinMode(&MODULE_P00, EBCM_LED1, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(&MODULE_P00, EBCM_LED2, IfxPort_Mode_outputPushPullGeneral);

    /* Turn off all LEDs */
    IfxPort_setPinState(&MODULE_P00, EBCM_LED1, IfxPort_State_high);
    IfxPort_setPinState(&MODULE_P00, EBCM_LED2, IfxPort_State_high);

    lc_1.EBCM_LED = EBCM_LED1;
    lc_1.count = 0;
    lc_1.active_pattern = led_pattern_abs_indev;


    lc_2.EBCM_LED = EBCM_LED2;
    lc_2.count = 0;
    lc_2.active_pattern = led_pattern_off;

}

void set_led_pattern(EBCM_led_context* led_context, led_pattern pattern)
{
    if (led_context != NULL)
    {
        led_context->count = 0U;
        led_context->active_pattern = pattern;
    }
}

void update_led(EBCM_led_context* led_context)
{
    if (led_context != NULL)
    {
        uint8 val = led_context->active_pattern.pattern_array[led_context->count];

        // active low leds
        if (val)
        {
            IfxPort_setPinState(&MODULE_P00, led_context->EBCM_LED, IfxPort_State_low);
        }
        else
        {
            IfxPort_setPinState(&MODULE_P00, led_context->EBCM_LED, IfxPort_State_high);
        }

        if (++(led_context->count) >= led_context->active_pattern.length)
        {
            led_context->count = 0;
        }

    }

}

void ebcm_led_task(void)
{
    update_led(&lc_1);
    update_led(&lc_2);
}

