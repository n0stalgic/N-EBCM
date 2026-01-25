/******************************************************************************
 * @file    ssw.c
 * @brief   Implementation for safe software startup
 *
 * MIT License
 *
 * Copyright (c) 2025 n0stalgic
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

#include <stdio.h>
#include "Ifx_Cfg_Ssw.h"
#include "IfxMtu.h"
#include "IfxPort.h"
#include "IfxScuLbist.h"
#include "IfxPmsPm.h"
#include "IfxSmuStdby.h"
#include "ssw.h"
#include "ebcm_cfg.h"
#include "ebcm_main.h"
#include "IfxSmu.h"
#include "IfxSmu_cfg.h"
#include "ssw_monbist.h"
#include "ssw_mcu_fw_chk.h"
#include "ssw_mcu_startup.h"
#include "ssw_regmon_test.h"
#include "ssw_mbist.h"
#include "ssw_alive_alarm_test.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/* @brief Stores the number of Ssw, Lbist and MCU runs */
volatile SswRunCount* sswRunCount = &SSW_STATUS_DATA_ADDRESS;  /* This variable is located in SCR XRAM memory. */
SswMonbistStatus monbistStatus;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

EbcmResetCode EbcmSsw_evalReset(void);
boolean EbcmSsw_evalStandby(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/*
 * Triggering a Warm PORST via the LBIST.
 * Note: "In AURIX(TM) TC3xx devices, LBIST execution terminates [..] with a warm reset." but
 * "The Startup Software executed afterwards follows the flow as after cold power-on [..]"
 * */
void EbcmSsw_triggerWarmPorst(void)
{
    EbcmSsw_triggerSwReset(IfxScuRcu_ResetType_warmpoweron);
}

/*
 * This function triggers either a SW Application Reset or a SW System Reset, based on the parameter resetType
 * */
void EbcmSsw_triggerSwReset(EbcmResetType resetType)
{
    /* Get the CPU EndInit password */
    uint16 password = IfxScuWdt_getCpuWatchdogPassword();

    /* Configure the request trigger in the Reset Configuration Register */
    IfxScuRcu_configureResetRequestTrigger(IfxScuRcu_Trigger_sw, (IfxScuRcu_ResetType) resetType);

    /* Clear CPU EndInit protection to write in the SWRSTCON register of SCU */
    IfxScuWdt_clearCpuEndinit(password);

    /* Trigger a software reset based on the configuration of RSTCON register */
    IfxCpu_triggerSwReset();

    /* The following instructions are not executed if a SW reset occurs */
    /* Set CPU EndInit protection */
    IfxScuWdt_setCpuEndinit(password);
}

const char* EbcmSsw_getLbistResultStr(SswTestStatus status)
{
    switch(status)
    {
        case PASSED:        return "PASSED";
        case TEST_NOT_EVAL: return "NE";
        case FAILED:        return "FAILED";
        default:            return "UNKNOWN";

    }

}

EbcmResetCode EbcmSsw_evalReset(void)
{
    IfxScuRcu_ResetCode rstCode = IfxScuRcu_evaluateReset();

    IfxScuRcu_clearColdResetStatus();

    return rstCode;
}

boolean EbcmSsw_evalStandby(void)
{
    static boolean standbyEvaluated   = FALSE;
    static boolean comingFromStandby = FALSE;

    if (standbyEvaluated == FALSE)
    {
        if ((PMS_PMSWSTAT2.U & PMSWSTAT2_WAKE_UP_FLAGS_MASK) > 0 )
        {
            uint16 endInitSftyPw = IfxScuWdt_getSafetyWatchdogPassword();
            IfxScuWdt_clearSafetyEndinit(endInitSftyPw);
            PMS_PMSWSTATCLR.U |= (PMS_PMSWSTAT2.U & PMSWSTAT2_WAKE_UP_FLAGS_MASK);
            IfxScuWdt_setSafetyEndinit(endInitSftyPw);

            comingFromStandby = TRUE;
        }
        else
        {
            comingFromStandby = FALSE;
        }
    }

    standbyEvaluated = TRUE;
    return comingFromStandby;
}

boolean EbcmSsw_lockstepInjectionTest(void)
{
    boolean passed = FALSE;

    /* Test lockstep comparator, inject a fault into LCL0 */
    IfxScuWdt_clearGlobalSafetyEndinitInline(IfxScuWdt_getGlobalSafetyEndinitPasswordInline());
    MODULE_SCU.LCLTEST.U = 0x03;
    IfxScuWdt_setGlobalSafetyEndinitInline(IfxScuWdt_getGlobalSafetyEndinitPasswordInline());


    /* Read safety management unit lockstep alarm right away */
    volatile boolean smuAlarm = IfxSmu_getAlarmStatus(IfxSmu_Alarm_CPU0_Lockstep_ComparatorError) &&
                                IfxSmu_getAlarmStatus(IfxSmu_Alarm_CPU1_Lockstep_ComparatorError);
    __nop();
    __nop();

    if (smuAlarm)
    {
      passed = TRUE;
      IfxSmu_clearAlarmStatus(IfxSmu_Alarm_CPU0_Lockstep_ComparatorError);
      IfxSmu_clearAlarmStatus(IfxSmu_Alarm_CPU1_Lockstep_ComparatorError);

      smuAlarm = IfxSmu_getAlarmStatus(IfxSmu_Alarm_CPU0_Lockstep_ComparatorError) &&
                 IfxSmu_getAlarmStatus(IfxSmu_Alarm_CPU1_Lockstep_ComparatorError);
      if (passed && !smuAlarm)
      {
          passed = TRUE;
      }
      else
      {
          passed = FALSE;
      }
    }

    return passed;
}

void EbcmSsw_runAppSwStartup(void)
{
#if EBCM_CFG_SSW_ENABLE_LBIST_BOOT || EBCM_CFG_SSW_ENABLE_LBIST_APPSW
   EbcmSsw_lbist();
#endif /* EBCM_CFG_SSW_ENABLE_LBIST_BOOT || EBCM_CFG_SSW_ENABLE_LBIST_APPSW */

   ebcmStatus.resetCode = EbcmSsw_evalReset();
   ebcmStatus.wakeupFromStby = EbcmSsw_evalStandby();

#if EBCM_CFG_SSW_ENABLE_MONBIST
    /* MONBIST Tests and evaluation
     * SMC:PMS:MONBIST_CFG
     * SM:PMS:MONBIST_RESULT
     * */
    EbcmSsw_monbist();
#endif /* SLK_CFG_SSW_ENABLE_MONBIST */

#if EBCM_CFG_SSW_ENABLE_MCU_FW_CHECK
    EbcmSsw_mcuFwCheck();
#endif /* EBCM_CFG_SSW_ENABLE_MCU_FW_CHECK */

#if EBCM_CFG_SSW_ENABLE_MCU_STARTUP
    EbcmSsw_mcuStartup();
#endif /* EBCM_CFG_SSW_ENABLE_MCU_STARTUP */

#if EBCM_CFG_SSW_ENABLE_ALIVE_ALARM_TEST
    EbcmSsw_aliveAlarmTest();
#endif /* EBCM_CFG_SSW_ENABLE_ALIVE_ALARM_TEST */

#if EBCM_CFG_SSW_ENABLE_REG_MONITOR_TEST
    EbcmSsw_regMonTest();
#endif /* EBCM_CFG_SSW_ENABLE_REG_MONITOR_TEST */

#if EBCM_CFG_SSW_ENABLE_MBIST
    EbcmSsw_mbist();
#endif /* EBCM_CFG_SSW_ENABLE_REG_MONITOR_TEST */



    /* TODO: Implement the rest of the silicon BISTs according to AN0029 Safety Critical Application Development */
}
