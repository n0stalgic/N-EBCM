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

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define SOFT_SMU_ALM_CFG_CHECK          IfxSmu_Alarm_Software_Alarm0
#define SOFT_SMU_ALM_ADC_BWD            IfxSmu_Alarm_Software_Alarm1    /* not used currently */
#define SOFT_SMU_ALM_ADC                IfxSmu_Alarm_Software_Alarm2    /* not used currently */
#define SOFT_SMU_ALM_GTM_ECKL           IfxSmu_Alarm_Software_Alarm3
#define SOFT_SMU_ALM_GTM_ISR_MON        IfxSmu_Alarm_Software_Alarm4
#define SOFT_SMU_ALM_SBST               IfxSmu_Alarm_Software_Alarm5
#define SOFT_SMU_ALM_QSPI_SAFE          IfxSmu_Alarm_Software_Alarm6
#define SOFT_SMU_ALM_DIGITAL_ACQ_ACT    IfxSmu_Alarm_Software_Alarm7
#define SOFT_SMU_ALM_DTS                IfxSmu_Alarm_Software_Alarm8
#define SOFT_SMU_ALM_DMA                IfxSmu_Alarm_Software_Alarm9
#define SOFT_SMU_ALM_CPU_MON            IfxSmu_Alarm_Software_Alarm10   /* Cpu monitor  */
#define SOFT_SMU_ALM_STM                IfxSmu_Alarm_Software_Alarm11   /* compare two stm ticks */
#define SOFT_SMU_ALM_PFLASH             IfxSmu_Alarm_Software_Alarm12
#define SOFT_SMU_ALM_TRAP               IfxSmu_Alarm_Software_Alarm13
#define SOFT_SMU_ALM_PORT_SMs           IfxSmu_Alarm_Software_Alarm14   /* not used currently */
#define SOFT_SMU_ALM_CLOCK_PLAUS        IfxSmu_Alarm_Software_Alarm15   /* not used currently */

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
    FAIL   = 0,
    PASS   = 1,
    NA     = 2
} SmuStatusType;

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
} smu_execution_status_t;
 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/


#endif /* SAFE_COMPUTATION_SMU_H_ */
