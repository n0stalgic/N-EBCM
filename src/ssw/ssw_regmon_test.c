/******************************************************************************
 * @file    ssw_regmon_test.c
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
#include <safe_computation/smu_stdby_FSP.h>
#include "IfxSmu.h"
#include "IfxSmu_cfg.h"
#include "IfxMtu.h"
#include "IfxIom.h"
#include "IfxIom_reg.h"
#include "IfxDma_reg.h"
#include "IfxStm.h"
#include "ebcm_main.h"
#include "ssw_regmon_test.h"
#include "IfxEmem_cfg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define SMU_REG_MONITOR_TEST_MAX_TIME_SEC 2e-6 /* Safety Manual recommends 2µs, assuming default clock configurations*/
#define TYPE_SMU_CORE   0
#define TYPE_SMU_STDBY  1

#ifdef IFXEMEM_REG_H
#define NUM_UNCORRECTABLE_ERRORS     14
#else
#define NUM_UNCORRECTABLE_ERRORS     13
#endif

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

typedef enum
{
    /* Following alarms were triggered during the execution of the REG_MONITOR_TEST of the specific module:
     * (The safety mechanisms listed in the brackets are the mechanisms which are related to the
     * SM:SMU:REG_MONITOR_TEST of the specific module)
     */
    smuRegMonitorModuleMTU = 0,             /* ALM6[0] (SM[HW]:VMT:FPI_WRITE_MONITOR, (SM[HW]:VMT:REG_MONITOR)),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleIOM = 1,             /* ALM6[1] (SM[HW]:GTM:FPI_WRITE_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleIR = 2,              /* ALM6[2] (SM[HW]:IR:FPI_WRITE_MONITOR, SM[HW]:IR:REG_MONITOR),
                                                * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleEMEM = 3,            /* ALM6[3] (SM[HW]:EMEM:FPI_WRITE_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleSCUSRU = 4,          /* ALM6[4] (SM[HW]:RESET:FF_MONITORING, SM[HW]:RESET:FPI_WRITE_MONITOR,
                                                 * SM[HW]:SCU:FPI_WRITE_MONITOR, SM[HW]:SCU:REG_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModulePMS = 5,             /* ALM6[5] (SM[HW]:PMS:REG_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleDMA = 6,             /* ALM6[6] (SM[HW]:DMA:REQUEST_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleSMUcore = 7,         /* ALM6[7] (SM[HW]:SMU:FPI_WRITE_MONITOR, SM[HW]:SMU:REG_MONITOR_TEST),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleCERBERUS = 8,        /* ALM6[23] (SM[HW]:DEBUG:CFG_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleSYSPllPERPll = 9,   /* ALM6[8] (SM[HW]:CLOCK:CFG_MONITOR, SM[HW]:CLOCK:FPI_WRITE_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    smuRegMonitorModuleCCU = 10,           /* ALM6[24] & ALM6[25] & ALM10[20] (SM[HW]:CLOCK:CFG_MONITOR),
                                                 * (ALM10[21], alarm triggered during test of every module) */
    numSmuRegMonitorModule = 11
} SmuRegMonitorModule;

typedef struct
{
    uint8   smuType;
    uint16  alarmName;
} SafetyFfUncorrectableErrorsType;


SafetyFfUncorrectableErrorsType safetyFfUncorrectableErrors[NUM_UNCORRECTABLE_ERRORS] =
{
    {TYPE_SMU_CORE,     IfxSmu_Alarm_MTU_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_IOM_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_IR_Safety_FfUncorrectableErrorDetected                 },
#ifdef IFXEMEM_REG_H
    {TYPE_SMU_CORE,     IfxSmu_Alarm_EMEM_Safety_FfUncorrectableErrorDetected               },
#endif
    {TYPE_SMU_CORE,     IfxSmu_Alarm_SCU_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_PMS_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_DMA_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_SMU_core_Safety_FfUncorrectableErrorDetected           },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_SMU_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_CERBERUS_Safety_FfUncorrectableErrorDetected           },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_SYS_PLL_PER_PLL_Safety_FfUncorrectableErrorDetected    },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_CCU_Safety_FfUncorrectableErrorDetected                },
    {TYPE_SMU_CORE,     IfxSmu_Alarm_CCU_Safety_FfCorrectableErrorDetected                  },
    {TYPE_SMU_STDBY,    smuStdbyAlarmSafetyFlipFlopUncorrectableError                       }
};

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void EbcmSsw_regMonTest(void)
{
    ebcmStatus.smuStatus.smuSafetyFlipFlopTriggerTestSts = NA;
    ebcmStatus.smuStatus.smuSafetyFlipFlopTestResultCheckSts = NA;
    ebcmStatus.smuStatus.smuSafetyFlipFlopTestAlarmFlagClearSts = NA;
    ebcmStatus.smuStatus.regMonitorTestSmu = NA;
    ebcmStatus.unlockConfig = TRUE;
    ebcmStatus.smuStatus.unlockConfigRegisterSMU = NA;

    /* Enable all clocks */
    boolean mtuWasEnabled = IfxMtu_isModuleEnabled();
    if (mtuWasEnabled == FALSE)
    {
        IfxMtu_enableModule();
    }

    boolean iomWasEnabled = TRUE;

    /* Check if IO monitor was disabled */
    if (MODULE_IOM.CLC.B.DISS != 0)
    {
        iomWasEnabled = FALSE;
        IfxIom_enableModule(&MODULE_IOM, 0x1);
    }

#ifdef IFXEMEM_REG_H
    boolean ememWasEnabled = IfxEmem_isModuleEnabled();
    if(ememWasEnabled == FALSE)
    {
        IfxEmem_enableModule(&MODULE_EMEM);
        IfxEmem_setUnlockMode(&MODULE_EMEM);
    }
#endif

    IfxScuWdt_clearCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());
    boolean dmaWasEnabled = TRUE;

    if (MODULE_DMA.CLC.B.DISS != 0)
    {
        dmaWasEnabled = FALSE;

        /* Enable DMA */
        DMA_CLC.B.DISS = 0;
    }

    boolean smuWasEnabled = TRUE;
    if (MODULE_SMU.CLC.B.DISS != 0)
    {
        smuWasEnabled = FALSE;

        /* Enable SMU */
        SMU_CLC.B.DISS = 0;
    }

    IfxScuWdt_setCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());
    ebcmStatus.unlockConfig &= IfxSmu_unlockConfigRegisters();
    if (ebcmStatus.unlockConfig == TRUE)
    {
        ebcmStatus.smuStatus.unlockConfigRegisterSMU = PASS;
    }
    else
    {
        ebcmStatus.smuStatus.unlockConfigRegisterSMU = FAIL;
    }

    uint64 tStart, tExecution;
    float64 tExecutionSec;

    boolean smuRegMonTestPassed = TRUE;

    /* Iterate through all modules listed in the SmuRegMonitorModule enumeration */
    for (SmuRegMonitorModule testModeEnable = smuRegMonitorModuleMTU;
            testModeEnable < numSmuRegMonitorModule;
                testModeEnable++)
    {

#ifndef IFXEMEM_REG_H
        /* If EMEM is not available skip the loop iteration for testModeEnable == EMEM */
        if(testModeEnable == smuRegMonitorModuleEMEM)
        {
            continue;
        }
#endif


        /* Enable the test */
        IfxSmu_setRegMonTestModeEnable(testModeEnable);

        /* Capture the time */
        tStart = IfxStm_get(&MODULE_STM0);

        /* Wait for test to complete, and also measure the execution time */
        uint32 timeout = 0xfff;
        do
        {
            tExecution = IfxStm_get(&MODULE_STM0) - tStart;
            timeout--;
        }
        while (!(IfxSmu_getRegisterMonitorStatus() & (1U << testModeEnable)) && (timeout > 0));

        /* Test completed, disable for specific module */
        IfxSmu_clearRegMonTestModeEnable(testModeEnable);

        /* Convert to seconds */
        tExecutionSec = tExecution / IfxStm_getFrequency(&MODULE_STM0);

        /* Check test result */

        if(IfxSmu_getRegisterMonitorErrorFlag() & (1U << testModeEnable))
        {
            smuRegMonTestPassed = FALSE;
        }

        if (tExecutionSec > SMU_REG_MONITOR_TEST_MAX_TIME_SEC)
        {
            smuRegMonTestPassed = FALSE;
        }
    }

    /* Update system status */
    if (smuRegMonTestPassed == TRUE)
    {
        ebcmStatus.smuStatus.smuSafetyFlipFlopTriggerTestSts  = PASS;
        ebcmStatus.smuStatus.smuSafetyFlipFlopTestResultCheckSts = PASS;
    }

    /* Clear all safety flip-flop uncorrectable errors which are raised during the test */
    boolean alarmClearSuccessful = FALSE;
    SmuStatusType result = PASS;

    for(uint8 errorId = 0; errorId < NUM_UNCORRECTABLE_ERRORS; errorId++)
    {
       if(safetyFfUncorrectableErrors[errorId].smuType == TYPE_SMU_CORE)
       {
           IfxSmu_clearAlarmStatus(safetyFfUncorrectableErrors[errorId].alarmName);
           if (IfxSmu_getAlarmStatus(safetyFfUncorrectableErrors[errorId].alarmName) == FALSE)
           {
               alarmClearSuccessful = TRUE;
           }
       }
       else if (safetyFfUncorrectableErrors[errorId].smuType == TYPE_SMU_STDBY)
       {
           if (clearBitSMUstdby(safetyFfUncorrectableErrors[errorId].alarmName) == FAIL)
           {
               result = FAIL;
           }
       }
    }

   /* Set ebcm smu status variable */
   if ((alarmClearSuccessful == TRUE) && (result == PASS))
   {
       ebcmStatus.smuStatus.smuSafetyFlipFlopTestAlarmFlagClearSts = PASS;
   }

   /* Lock SMU registers again*/
   IfxSmu_temporaryLockConfigRegisters();

   /* Disable all clocks */
   if (mtuWasEnabled == FALSE)
   {
       /* Clear EndInit */
       IfxScuWdt_clearCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());

       /* MTU clock disable */
       MTU_CLC.B.DISR = 0x1U;
       /*Set EndInit */
       IfxScuWdt_setCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());
   }

   if(iomWasEnabled == FALSE)
   {
       IfxIom_resetModule(&MODULE_IOM);
   }

   IfxScuWdt_clearCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());
   if(dmaWasEnabled == FALSE)
   {
       MODULE_DMA.CLC.B.DISR = 0x1U;
   }
   if(smuWasEnabled == FALSE)
   {
       MODULE_SMU.CLC.B.DISR = 0x1U;
   }
   IfxScuWdt_setCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());


   if (smuRegMonTestPassed == TRUE)
   {
       ebcmStatus.smuStatus.regMonitorTestSmu = PASS;
   }
   else
   {
       ebcmStatus.smuStatus.regMonitorTestSmu = FAIL;
   }
}


