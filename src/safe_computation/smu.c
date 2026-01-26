/******************************************************************************
 * @file    smu.c
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
#include "Cpu/Irq/IfxCpu_Irq.h"
#include "smu.h"
#include "ebcm_main.h"
#include "ebcm_cfg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

#if 0
/*-------------------------------------------------------------------------------------------------------------------*/
/*                           CONFIGURABLE BY USER                                                                    */
/*-------------------------------------------------------------------------------------------------------------------*/
/*
 * The array globalAlarmConfig[] holds the configuration of every alarm configured by the user
 * For each alarm, you can:
 *  - configure the internal reaction (trigger interrupts, issue NMI, individual CPU reset or MCU reset)
 *  - enable the external reaction (FSP or PES depending of the PES config: see IfxSmu_configAlarmActionPES function)
 *  - enable the use of recovery timer (currently can only be triggered by alarms max)
 *  - configure a callback function that will be called when alarm is detected
 *
 * Notes:
 *  During program execution, alarms are stored in different arrays according to their internal reaction.
 *  Alarms that needs to be detected as fast as possible should be placed at the first position
 *  of their reaction group section in the globalAlarmConfig[] array.
 *
 *  The use of the recovery timer only makes sense if the internal action is an interrupt or NMI. However no
 *  hardware check is done, it is up to software to configure the SMU_core in the appropriate way.
 *  */
const AlarmConfigStruct globalAlarmConfig[USER_ALARM_NUMBER] =
{
    /*
     * Attention
     * Alarm message names are created in /outputPrintedSMU.js to show alarm message names in OneEye GUI.
     * */
    /*---------------------------------------------------------------------------------------------------------------*/
    /* IfxSmu_Alarm alarm                       IfxSmu_InternalAlarmAction         Enable      Trigger  Function to
                                                                                   external    Recovery call when alarm
                                                                                   reaction    Timer    is detected  */
    /*------------------------------------------------------------- IGCS0 -------------------------------------------*/
    {DEFAULT_ALARM,                              DEFAULT_ALARM_ACTION,               FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_ADC_BWD,                       IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_ADC,                           IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_GTM_ISR_MON,                   IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_SBST,                          IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_QSPI_SAFE,                     IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_DIGITAL_ACQ_ACT,               IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_DTS,                           IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_DMA,                           IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_CPU_MON,                       IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_STM,                           IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_PFLASH,                        IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_TRAP,                          IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_PORT_SMs,                      IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {SOFT_SMU_ALM_CLOCK_PLAUS,                   IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_DMA_DMASRI_EccError,           IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_IOM_Pin_MismatchIndication,    IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_EVR_Undervoltage_Alarm,        IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_XBAR_EDC_WritePhaseError,      IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_LMU_EDC_WritePhaseError,       IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_CPU2_Buslevel_MpuViolation,     IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,     NULL_PTR},
    {IfxSmu_Alarm_CPU0_PFI0_PFLASH0_ReadPathError, IfxSmu_InternalAlarmAction_igcs0,   FALSE,      FALSE,    NULL_PTR},

    /*------------------------------------------------------------- IGCS1 -------------------------------------------*/
    {IfxSmu_Alarm_CPU0_Lockstep_ComparatorError, IfxSmu_InternalAlarmAction_igcs1,   TRUE,       FALSE,      NULL_PTR},
    {IfxSmu_Alarm_CPU1_Lockstep_ComparatorError, IfxSmu_InternalAlarmAction_igcs1,   TRUE,       FALSE,      NULL_PTR},
    {IfxSmu_Alarm_CPU2_Lockstep_ComparatorError, IfxSmu_InternalAlarmAction_igcs1,   TRUE,       FALSE,      NULL_PTR},

    /*------------------------------------------------------------- IGCS2 -------------------------------------------*/
    {IfxSmu_Alarm_SMU_Error_PinFaultStateActivation, IfxSmu_InternalAlarmAction_igcs2,  FALSE,    FALSE,     NULL_PTR},
    {IfxSmu_Alarm_SCU_External_EmergencyStopSignalEvent, IfxSmu_InternalAlarmAction_igcs2, FALSE, FALSE,     NULL_PTR},
    {SOFT_SMU_ALM_GTM_ECKL,                       IfxSmu_InternalAlarmAction_igcs2,   FALSE,      FALSE,     NULL_PTR},
    {IfxSmu_Alarm_HSM_Undervoltage_Alarm,         IfxSmu_InternalAlarmAction_igcs2,   FALSE,      FALSE,     NULL_PTR},
    {IfxSmu_Alarm_FSI_PFlash_SingleBitError,      IfxSmu_InternalAlarmAction_igcs2,   FALSE,      FALSE,     NULL_PTR},
    {IfxSmu_Alarm_FSI_PFlash_DoubleBitError, IfxSmu_InternalAlarmAction_igcs2, FALSE, FALSE, &enableWlFailDetectPFLASH},
    {IfxSmu_Alarm_FSI_Multiple_BitErrorDetectionTrackingBufferFull, IfxSmu_InternalAlarmAction_igcs2,
                                                                                      FALSE,      FALSE,     NULL_PTR},
    {IfxSmu_Alarm_SMU_Access_EnableErrorDetected, IfxSmu_InternalAlarmAction_igcs2,   FALSE,      FALSE,     NULL_PTR},
    /*-------------------------------------------------------------- NMI --------------------------------------------*/
    /* Note: STHE: Watchdog alarm and Recovery timer is not exactly configured as stated in UM section 15.3.1.5.8
     * Watchdog Alarms -> Changes in Recovery Timer configuration needs to be done */
    {SMU_ALARM_WHICH_TRIGGERS_NMI,       IfxSmu_InternalAlarmAction_nmi, FALSE, TRUE,       &slkWatchdogAlarmHandling},

    /*------------------------------------------------------------- RESET --------------------------------------------*/
    {IfxSmu_Alarm_SMU_Timer0_TimeOut,             IfxSmu_InternalAlarmAction_reset,   FALSE,      FALSE,      NULL_PTR},
    {IfxSmu_Alarm_SMU_Timer1_TimeOut,             IfxSmu_InternalAlarmAction_reset,   FALSE,      FALSE,      NULL_PTR},

    /*------------------------- CPU RESET (SMU AGC register defines which CPUs are affected by the CPU reset) --------*/
    {IfxSmu_Alarm_SCU_CPU2_WatchdogTimeOut,         IfxSmu_InternalAlarmAction_igcs2, FALSE,      FALSE,      NULL_PTR},

    /*-------------------------------------------------------- Disabled alarms ---------------------------------------*/
    /* Used for SMU ISR test */
    {SOFT_SMU_ALM_CFG_CHECK,                     IfxSmu_InternalAlarmAction_disabled, FALSE,      FALSE,      NULL_PTR},

    {IfxSmu_Alarm_SCU_External_RequestUnitAlarm1, IfxSmu_InternalAlarmAction_disabled, FALSE,     FALSE,      NULL_PTR},
    {IfxSmu_Alarm_SPB_BusErrorEvent,              IfxSmu_InternalAlarmAction_igcs2, FALSE,     FALSE,      NULL_PTR}
    /*---------------------------------------------------------------------------------------------------------------*/
};

#endif

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void EbcmSfty_initSmu(void)
{

    /* Prevention of double SMU reset */
    IfxScuWdt_clearSafetyEndinit(IfxScuWdt_getSafetyWatchdogPassword());
    SCU_WDTSCON1.B.CLRIRF = 1;
    IfxScuWdt_setSafetyEndinit(IfxScuWdt_getSafetyWatchdogPassword());


}
