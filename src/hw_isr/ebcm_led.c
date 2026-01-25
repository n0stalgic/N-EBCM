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
} LedFlash;


typedef struct
{
    LedFlash type;
    uint8* patternArray;
    const Ifx_SizeT length;

} LedPattern;

typedef struct
{
    EbcmLedIndex ebcmLed;
    uint16 count;
    LedPattern activePattern;

} EbcmLedContext;

#define LED_PATTERN_OFF_SEC_LEN   10U
const LedPattern ledPatternOff =
                { .patternArray = (const uint8[])
                                   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                   .length        = LED_PATTERN_OFF_SEC_LEN,
                   .type          = LED_PATTERN_OFF
                 };


#define LED_PATTERN_0_2_SEC_LEN   40U
const LedPattern ledPattern0p2Sec =
                                 {  .patternArray =  (const uint8[])
                                       { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },

                                       .length = LED_PATTERN_0_2_SEC_LEN,
                                       .type = LED_PATTERN_BLINK_0_2_SEC };


#define LED_PATTERN_0_5_SEC_LEN   100U
const LedPattern ledPattern0p5Sec = { .patternArray = (const uint8[])
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
const LedPattern ledPatternAbsIndev = { .patternArray = (const uint8[])
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


EbcmLedContext lc1;
EbcmLedContext lc2;


/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/* Initialize GPIO pins for LEDs */
void EbcmHw_initLeds(void)
{
    /* Initialize GPIO pins for LEDs */
    IfxPort_setPinMode(&MODULE_P00, EBCM_LED1, IfxPort_Mode_outputPushPullGeneral);
    IfxPort_setPinMode(&MODULE_P00, EBCM_LED2, IfxPort_Mode_outputPushPullGeneral);

    /* Turn off all LEDs */
    IfxPort_setPinState(&MODULE_P00, EBCM_LED1, IfxPort_State_high);
    IfxPort_setPinState(&MODULE_P00, EBCM_LED2, IfxPort_State_high);

    lc1.ebcmLed = EBCM_LED1;
    lc1.count = 0;
    lc1.activePattern = ledPatternAbsIndev;


    lc2.ebcmLed = EBCM_LED2;
    lc2.count = 0;
    lc2.activePattern = ledPatternOff;

}

void EbcmHw_setLedPattern(EbcmLedContext* ledContext, LedPattern pattern)
{
    if (ledContext != NULL)
    {
        ledContext->count = 0U;
        ledContext->activePattern = pattern;
    }
}

void EbcmHw_updateLed(EbcmLedContext* ledContext)
{
    if (ledContext != NULL)
    {
        uint8 val = ledContext->activePattern.patternArray[ledContext->count];

        // active low leds
        if (val)
        {
            IfxPort_setPinState(&MODULE_P00, ledContext->ebcmLed, IfxPort_State_low);
        }
        else
        {
            IfxPort_setPinState(&MODULE_P00, ledContext->ebcmLed, IfxPort_State_high);
        }

        if (++(ledContext->count) >= ledContext->activePattern.length)
        {
            ledContext->count = 0;
        }

    }

}

void EbcmHw_ledTask(void)
{
    EbcmHw_updateLed(&lc1);
    EbcmHw_updateLed(&lc2);
}

