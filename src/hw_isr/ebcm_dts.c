/******************************************************************************
 * @file    ebcm_dts.c
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
#include "Ifx_Types.h"
#include "IfxAsclin_Asc.h"
#include "IfxDts_Dts.h"
#include "ebcm_cfg.h"
#include "ebcm_main.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define MIN_TEMP_LIMIT          -35     /* Lower temperature limit              */
#define MAX_TEMP_LIMIT          150     /* Upper temperature limit              */

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

volatile boolean dtsMeasureAvailable;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(EbcmHw_dtsIsr, CPU0_VECT_TABLE_ID, ISR_PRIORITY_DTS);


/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void EbcmHw_dtsIsr(void)
{
    /* Ignore first three measurements as core temperature sensor value might return 0 at the beginning (self-calibration) */
    static uint8 init_cnt;

    init_cnt++;
    if (init_cnt < 3)
    {
        return;
    }

    float32 dieTemperaturePms;
    float32 dieTemperatureCore;

    /* Read PMS die temperature */
    dieTemperaturePms = IfxDts_Dts_getTemperatureCelsius();

    /* Now read CORE die temperature from SCU module */
    dieTemperatureCore = IfxDts_Dts_convertToCelsius((uint16) MODULE_SCU.DTSCSTAT.B.RESULT);

    /* SM:DTS::DTS_RESULT from Infineon ASIL safety manual */
    /* Calculate absolute value of temperature difference and trigger an alarm if the difference is above the limit */

    float32 tempDifference =
            (dieTemperatureCore > dieTemperaturePms) ?
                    dieTemperatureCore - dieTemperaturePms : dieTemperaturePms - dieTemperatureCore;

    ebcmStatus.dieTempProfile.dieTempDifference  = tempDifference;
    ebcmStatus.dieTempProfile.dieTemperatureCore = dieTemperatureCore;
    ebcmStatus.dieTempProfile.dieTemperaturePms  = dieTemperaturePms;

    float32 higherTemp =
            ebcmStatus.dieTempProfile.dieTemperatureCore > ebcmStatus.dieTempProfile.dieTemperaturePms ?
                    ebcmStatus.dieTempProfile.dieTemperatureCore : ebcmStatus.dieTempProfile.dieTemperaturePms;
    float32 lowerTemp =
            ebcmStatus.dieTempProfile.dieTemperatureCore < ebcmStatus.dieTempProfile.dieTemperaturePms ?
                    ebcmStatus.dieTempProfile.dieTemperatureCore : ebcmStatus.dieTempProfile.dieTemperaturePms;

    /* limit checking for highest */
    if (higherTemp > ebcmStatus.dieTempProfile.dieTempHighest)
    {
        ebcmStatus.dieTempProfile.dieTempHighest = higherTemp; /* Overwrite if new value is higher as old maximum */
    }

    /* limit checking for lowest value */
    if (lowerTemp < ebcmStatus.dieTempProfile.dieTempLowest)
    {
        ebcmStatus.dieTempProfile.dieTempLowest = lowerTemp; /* Overwrite if new value is lower as old minimum */
    }

    /* TODO: latch SMU alarm */
    if (tempDifference > MAX_DIE_TEMP_DIFF)
    {
        ;
    }
}


void EbcmHw_initDts(void)
{
    dtsMeasureAvailable = FALSE;

    /* Clear upper and lower underflow/overflow of the PMS die temperature sensor */
    MODULE_PMS.DTSLIM.B.LLU = 0;
    MODULE_PMS.DTSLIM.B.UOF = 0;

    /* Get the default configuration */
    IfxDts_Dts_Config dtsConf;
    IfxDts_Dts_initModuleConfig(&dtsConf);              /* Initialize the structure with default values              */

    dtsConf.lowerTemperatureLimit = MIN_TEMP_LIMIT;     /* Set the lower temperature limit                           */
    dtsConf.upperTemperatureLimit = MAX_TEMP_LIMIT;     /* Set the upper temperature limit                           */
    dtsConf.isrPriority = ISR_PRIORITY_DTS;             /* Set the interrupt priority for new measurement events     */
    dtsConf.isrTypeOfService = IfxSrc_Tos_cpu0;         /* Set the service provider responsible for handling
                                                         * the interrupts                                            */

    IfxDts_Dts_initModule(&dtsConf);                    /* Initialize the DTS with the given configuration           */


    /* now initialize die temp sensor of the DTS core */
    uint16         password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);


    /* Clear upper and lower overflow flags */
    MODULE_SCU.DTSCLIM.B.LLU = 0;
    MODULE_SCU.DTSCLIM.B.UOF = 0;

    /* Wait for a few measurements before enabling the requested limits */
    MODULE_SCU.DTSCLIM.B.LOWER = 0;
    MODULE_SCU.DTSCLIM.B.UPPER = 4095;

    /* change to the requested limits */
    MODULE_PMS.DTSLIM.B.LOWER = IfxDts_Dts_convertFromCelsius(MIN_TEMP_LIMIT);
    MODULE_PMS.DTSLIM.B.UPPER = IfxDts_Dts_convertFromCelsius(MAX_TEMP_LIMIT);

    MODULE_SCU.DTSCLIM.B.EN = TRUE;

    IfxScuWdt_setCpuEndinit(password);

    /* Set default values */
    ebcmStatus.dieTempProfile.dieTempLowest = MIN_TEMP_LIMIT;
    ebcmStatus.dieTempProfile.dieTempHighest = MAX_TEMP_LIMIT;
    ebcmStatus.dieTempProfile.dieTempDifference = MAX_TEMP_LIMIT - MIN_TEMP_LIMIT;

}
