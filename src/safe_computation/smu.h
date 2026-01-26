/******************************************************************************
 * @file    smu.h
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

#ifndef SAFE_COMPUTATION_SMU_H_
#define SAFE_COMPUTATION_SMU_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxSrc_reg.h"
#include "IfxSmu.h"
#include "IfxSmu_cfg.h"
#include "IfxSrc_cfg.h"
#include "IfxMtu_cfg.h"
#include "IfxCpu_Trap.h"

#if CPU_WHICH_RUN_SMU == 0
    #if defined(__TASKING__)
    #pragma section code    "text_cpu0"
    #pragma section farbss  "bss_cpu0"
    #pragma section fardata "data_cpu0"
    #pragma section farrom  "rodata_cpu0"
    #else
        #error "Only TASKING is supported, add pragmas for your compiler"
    #endif
#elif ((CPU_WHICH_RUN_SMU == 1) && (CPU_WHICH_RUN_SMU < IFXCPU_NUM_MODULES))
    #if defined(__TASKING__)
    #pragma section code    "text_cpu1"
    #pragma section farbss  "bss_cpu1"
    #pragma section fardata "data_cpu1"
    #pragma section farrom  "rodata_cpu1"
    #else
        #error "Only TASKING is supported, add pragmas for your compiler"
    #endif
#elif ((CPU_WHICH_RUN_SMU == 2) && (CPU_WHICH_RUN_SMU < IFXCPU_NUM_MODULES))
    #if defined(__TASKING__)
    #pragma section code    "text_cpu2"
    #pragma section farbss  "bss_cpu2"
    #pragma section fardata "data_cpu2"
    #pragma section farrom  "rodata_cpu2"
    #else
        #error "Only TASKING is supported, add pragmas for your compiler"
    #endif
#else
#error "Set CPU_WHICH_RUN_SMU to a valid value!"
#endif


/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define SOFT_SMU_ALM_SCHED              IfxSmu_Alarm_Software_Alarm1    /* Scheduler deadline violation alarm */
#define SOFT_SMU_ALM_DTS                IfxSmu_Alarm_Software_Alarm8    /* Die temperature sensor alarm */
#define SOFT_SMU_ALM_STM                IfxSmu_Alarm_Software_Alarm11   /* STM plausibility check alarm */
#define SOFT_SMU_ALM_NULL_PTR

#define USER_ALARM_NUMBER           40
#define AMOUNT_OF_SMU_ALARMS        (uint16)(IfxSmu_Alarm_XBAR_SOTA_SwapError + 1)
/* Just took last alarm as default, any alarm can be set here */
#define DEFAULT_ALARM               (uint16)(IfxSmu_Alarm_XBAR_SOTA_SwapError)
#define DEFAULT_ALARM_ACTION        IfxSmu_InternalAlarmAction_igcs0

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/



/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef enum
{
    SmuSR0       = 1,
    SmuSR1       = 2,
    SmuSR0SR1    = 3,
    SmuSR2       = 4,
    SmuSR0SR2    = 5,
    SmuSR1SR2    = 6,
    SmuSR0SR1SR2 = 7,
    SmuNAN       = 8,
} SmuSR;

typedef enum
{
    FAIL   = 0,
    PASS   = 1,
    NA     = 2
} SmuStatusType;

typedef enum
{
    notPending = 0,
    pending    = 1,
} AlarmStateSMU;

/* Holds the config of an alarm */
typedef struct
{
    IfxSmu_Alarm                alarm;
    IfxSmu_InternalAlarmAction  alarmReaction;
    boolean                     externalReactionEnabled;
    boolean                     triggerRecoveryTimer;
    void                        (*alarmDetectionCallback)(void);
} AlarmConfigStruct;


/* Bind an IGCS reaction with its config */
typedef struct {
    IfxSmu_InternalAlarmAction  igcs_id;
    SmuSR                      igcs_config;
} IGCSisrBindingStruct;


/* Store the state of an user configured alarm with its config (const) */
typedef struct {
    const AlarmConfigStruct*  alarmConfig;
    AlarmStateSMU             alarmState;
} RuntimeAlarmHandle;


/* Hold records of all the alarm raised */
typedef struct
{
    RuntimeAlarmHandle  *lastAlarmRaised[USER_ALARM_NUMBER];
    uint16              alarmCounter;
} SmuAlarmPendingType;

/* Store function execution status */
typedef struct
{
    SmuStatusType regMonitorTestSmu;
    SmuStatusType structInitSts;
    SmuStatusType unlockConfigRegisterSMU;
    SmuStatusType smuCoreAlarmConfigSts;
    SmuStatusType smuCoreAliveTestSts;
    SmuStatusType smuCoreAliveTestClearSts;
    SmuStatusType smuCoreKeysTestSts;
    SmuStatusType smuCoreKeysTestClearSts;
    SmuStatusType stdbySmuFspReactionEnableSts;
    SmuStatusType smuStdbyFSP0DrivingEnableSts;
    SmuStatusType smuStdbyFSP1DrivingEnableSts;
    SmuStatusType smuCoreInitSts;
    SmuStatusType smuCoreAlarmPESSetSts;
    SmuStatusType smuCoreFSPConfigSts;
    SmuStatusType smuCoreFSPReactionToAlarmConfigSts;
    SmuStatusType smuSafetyFlipFlopTriggerTestSts;
    SmuStatusType smuSafetyFlipFlopTestResultCheckSts;
    SmuStatusType smuSafetyFlipFlopTestAlarmFlagClearSts;
    SmuStatusType smuRecoveryTimerConfigSts;
    SmuStatusType smuCoreSWAlarmTriggerSts;
} SmuExecutionStatus;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void EbcmSfty_initSmu(void);

#endif /* SAFE_COMPUTATION_SMU_H_ */
