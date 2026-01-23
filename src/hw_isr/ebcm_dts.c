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

volatile boolean dts_measure_available;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(DTS_ISR, 0, ISR_PRIORITY_DTS);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void DTS_ISR(void)
{
    dts_measure_available = TRUE; /* Notify the system that a new measurement is ready */
}


void ebcm_init_dts(void)
{
    dts_measure_available = FALSE;

    /* Get the default configuration */
    IfxDts_Dts_Config dtsConf;
    IfxDts_Dts_initModuleConfig(&dtsConf);              /* Initialize the structure with default values              */

    dtsConf.lowerTemperatureLimit = MIN_TEMP_LIMIT;     /* Set the lower temperature limit                           */
    dtsConf.upperTemperatureLimit = MAX_TEMP_LIMIT;     /* Set the upper temperature limit                           */
    dtsConf.isrPriority = ISR_PRIORITY_DTS;             /* Set the interrupt priority for new measurement events     */
    dtsConf.isrTypeOfService = IfxSrc_Tos_cpu0;         /* Set the service provider responsible for handling
                                                         * the interrupts                                            */
    IfxDts_Dts_initModule(&dtsConf);                    /* Initialize the DTS with the given configuration           */

}

void ebcm_dts_read_task(void)
{

}
