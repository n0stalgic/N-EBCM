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
#include "ebcm_main.h"
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "ebcm_sched.h"
#include "ebcm_wdt.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

 void init_ebcm(ebcm_sys_info* ebcm_info, IfxCpu_ResourceCpu cpu_idx)
{
    if(cpu_idx == IfxCpu_ResourceCpu_0)
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
            uint16 endinitSfty_pw = IfxScuWdt_getSafetyWatchdogPassword();
            IfxScuWdt_clearSafetyEndinit(endinitSfty_pw);
            PMS_PMSWSTATCLR.B.PWRWKPCLR = 1;
            IfxScuWdt_setSafetyEndinit(endinitSfty_pw);
        }
    }

    ebcm_info->pll_freq = IfxScuCcu_getPllFrequency();
    ebcm_info->cpu_freq = IfxScuCcu_getCpuFrequency(cpu_idx);
    ebcm_info->sys_freq = IfxScuCcu_getSpbFrequency();

    switch (cpu_idx)
    {
        case IfxCpu_ResourceCpu_0: ebcm_info->stm_freq = IfxStm_getFrequency(&MODULE_STM0); break;
        case IfxCpu_ResourceCpu_1: ebcm_info->stm_freq = IfxStm_getFrequency(&MODULE_STM1); break;
        case IfxCpu_ResourceCpu_2: ebcm_info->stm_freq = IfxStm_getFrequency(&MODULE_STM2); break;
        default: while(1) __debug(); break;
    }

    init_leds();
    ebcm_sch_init_gpt12_monitor();
    init_ebcm_safety_mechanisms();

    ebcm_init_wdt(WDT_RELOAD);


    ebcm_status.init_complete = TRUE;

}

 void init_ebcm_safety_mechanisms(void)
 {

     // TODO: init safety mechanisms here like die temp sensor, FCE, LMU data integrity, CPU data, integrity, etc

     return;
 }
