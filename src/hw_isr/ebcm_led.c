/******************************************************************************
 * @file    ebcm_led.c
 * @brief   Add brief here
 *
 * MIT License
 *
 * Copyright (c) 2026 n0stalgic
 *****************************************************************************/


/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "ebcm_cfg.h"
#include "ebcm_led.h"
#include "IfxPort.h"
#include "vfw_checkpoint.h"

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

/* VFW Checkpoints */
static VfwCheckpoint cp_led_task;
static VfwCheckpoint cp_led_update;


/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/* Initialize GPIO pins for LEDs */
void EbcmHw_initLeds(void)
{
    /* Initialize VFW Checkpoints */
    VFW_CreateCheckpoint(&cp_led_task, "EbcmHw_ledTask");
    VFW_CreateCheckpoint(&cp_led_update, "EbcmHw_updateLed");

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
    VFW_CheckpointEntry(&cp_led_update);
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
    VFW_CheckpointExit(&cp_led_update);

}

void EbcmHw_ledTask(void)
{
    VFW_CheckpointEntry(&cp_led_task);

    EbcmHw_updateLed(&lc1);
    EbcmHw_updateLed(&lc2);

    VFW_CheckpointExit(&cp_led_task);
}
