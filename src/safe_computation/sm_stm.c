/******************************************************************************
 * @file    sm_stm.c
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
#include <safe_computation/smu.h>
#include "IfxStm.h"
#include "ebcm_cfg.h"
#include "ssw.h"
#include "sm_stm.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define STM_APP_RESET       0
#define REPEAT_COUNT        5

#define APP_RESET_ON_STM_PLCHK_FAIL 1

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
boolean checkForDisabledStm(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

boolean checkForDisabledStm(void)
{
    boolean isAnStmDisabled = (MODULE_STM0.CLC.B.DISS == 1 ||
                              MODULE_STM1.CLC.B.DISS == 1 ||
                              MODULE_STM2.CLC.B.DISS == 1);

    return isAnStmDisabled;
}

void stmPlausibilityCheck(IfxCpu_ResourceCpu cpuIdx)
{
    /* Only do STM monitoring if all STM modules are enabled */
    if (checkForDisabledStm())
    {
        return;
    }

    Ifx_STM* stmA;
    Ifx_STM* stmB;

    switch(cpuIdx)
    {
        case IfxCpu_ResourceCpu_0:
            stmA = &MODULE_STM0;
            stmB = &MODULE_STM1;
            break;
        case IfxCpu_ResourceCpu_1:
            stmA = &MODULE_STM1;
            stmB = &MODULE_STM2;
            break;
        case IfxCpu_ResourceCpu_2:
            stmA = &MODULE_STM2;
            stmB = &MODULE_STM0;
            break;
        default:
            while (1)
            {
                /* hang currently for debug */
                __debug();
            }
            break;
    }

    uint8 repeatNtimes = REPEAT_COUNT;

    while (repeatNtimes)
    {
        /* cross-check counters of STM A and STM B, raise alarm if deviation is great than 2 ms */
        uint64 stmACount = IfxStm_get(stmA);
        uint64 stmBCount = IfxStm_get(stmB);

        uint64 stmDiff = stmACount > stmBCount ? (stmACount - stmBCount) : (stmBCount - stmACount);

        if (stmDiff > 2 * IFX_CFG_STM_TICKS_PER_MS)
        {
            repeatNtimes--;

            /* Safety manual recommends an application reset, but we'll latch an SMU alarm eventually */
            if (repeatNtimes == 0U)
            {
#if APP_RESET_ON_STM_PLCHK_FAIL
                EbcmSsw_triggerSwReset(EBCMResetType_application);
#else
                ;
#endif
            }
        }
        else
        {
            break;
        }

    }
}
