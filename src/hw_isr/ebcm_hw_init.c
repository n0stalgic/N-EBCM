/******************************************************************************
 * @file    ebcm_hw_init.c
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
#include <vfw_sched.h>
#include "ebcm_main.h"
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "ebcm_wdt.h"
#include "ebcm_fce_crc.h"
#include "ebcm_vcom.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

EbcmStatus ebcmStatus;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void EbcmHw_initEbcm(EbcmSysInfo* ebcmInfo, IfxCpu_ResourceCpu cpuIdx)
{
    if (cpuIdx == IfxCpu_ResourceCpu_0)
    {
        /* Clear RCU flags if it was a cold reset */
        if (IfxScuRcu_readRawResetStatus() & IFXSCURCU_POWERONRESET_MASK)
        {
            /* we had a cold reset, we need to clear the flags */
            IfxScuRcu_clearColdResetStatus();
        }

        /* In case of wake up via vext ramp up, we clear the corresponding bit to avoid immediate wake up in case of standby entry */
        if (PMS_PMSWSTAT2.B.PWRWKP == 1)
        {
            uint16 endinitSftyPw = IfxScuWdt_getSafetyWatchdogPassword();
            IfxScuWdt_clearSafetyEndinit(endinitSftyPw);
            PMS_PMSWSTATCLR.B.PWRWKPCLR = 1;
            IfxScuWdt_setSafetyEndinit(endinitSftyPw);
        }

        ebcmInfo->pllFreq = IfxScuCcu_getPllFrequency();
        ebcmInfo->cpuFreq = IfxScuCcu_getCpuFrequency(cpuIdx);
        ebcmInfo->sysFreq = IfxScuCcu_getSpbFrequency();
        ebcmInfo->stmFreq = IfxStm_getFrequency(&MODULE_STM0);
    }

    init_UART();
    EbcmHw_initLeds();
    FCE_init();
    EbcmHw_initDts();

    ebcmStatus.initComplete = TRUE;

}
