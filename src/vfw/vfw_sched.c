/******************************************************************************
 * @file    sched.c
 * @brief   Scheduler Implementation
 *
 * MIT License
 *
 * Copyright (c) 2026 n0stalgic
 *****************************************************************************/


/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "Ifx_reg.h"
#include "ebcm_cfg.h"
#include "sm_stm.h"
#include "ebcm_isr.h"
#include "ebcm_main.h"
#include "IfxGpt12.h"
#include "IfxCpu_Trap.h"
#include "vfw_checkpoint.h"
#include "vfw_ffi.h"
#include "vfw_init.h"
#include <string.h>
#include <vfw_sched.h>



/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define    VFW_CORE_ID_0_BITPOS    0
#define    VFW_CORE_ID_1_BITPOS    1
#define    VFW_CORE_ID_2_BITPOS    2

#define    VFW_SET_CORE_0          (1 << VFW_CORE_ID_0_BITPOS)
#define    VFW_SET_CORE_1          (1 << VFW_CORE_ID_1_BITPOS)
#define    VFW_SET_CORE_2          (1 << VFW_CORE_ID_2_BITPOS)

#define    VFW_CHECK_CORE(x, y)        (x & (1 << y))

#define GPT12_BASE_FREQ_100MHZ    (100000000UL)

#define GPT1_BLOCK_PRESCALER_16    16U
#define GPT12_INPUT_PRESCALER_8    8U
#define GPT12_INPUT_PRESCALER_16   16U
#define GPT12_INPUT_PRESCALER_64   64U
#define WCET_20_USEC_GPT12_RELOAD  2000U

#define USEC_PER_SEC               1000000UL
#define TICK_HZ                    (GPT12_BASE_FREQ_100MHZ / (GPT1_BLOCK_PRESCALER_16 * GPT12_INPUT_PRESCALER_64))
#define SCHED_P                    GPT1_BLOCK_PRESCALER_16 * GPT12_INPUT_PRESCALER_8
#define SCHED_DENOM                SCHED_P * 1000000ULL
#define CONVERT_USEC_TO_HZ(x)      ((USEC_PER_SEC / (x)))
#define WCET_20_USEC_IN_HZ         50000UL

#define TASK_DEADLINE_MARGIN_USEC  3U

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

#pragma section all "vfw_safe0"

volatile uint32 cpu0execTaskCounter;
volatile uint32 cpu1execTaskCounter;
volatile uint32 cpu2execTaskCounter;

#pragma section all restore


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/




#pragma section all "vfw_safe0"
static Task taskTable[] =
{
        [TASK_ID_C0_WDT] = { WDT_TASK_ID,   EbcmHw_svcWdt,     "EbcmHw_svcWdt",     1,    0,        5,   1,  0, TRUE, VFW_SET_CORE_0  },
        [TASK_ID_C0_LED] = { LED_TASK_ID,   EbcmHw_ledTask,    "EbcmHw_ledTask",    10,   0,        4,   10, 0, TRUE, VFW_SET_CORE_0 | VFW_SET_CORE_1  },

        /* ---------------------------- Sentinel ---------------------------- */
        [TASK_COUNT] = { 0xFF,    NULL,     "NULL_TASK",              0  ,  0,        0,    0, FALSE, 0xFFFFFFFFu }


};


volatile Task* core0_currentTask;
volatile Task* core1_currentTask;

#pragma section all restore

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(ebcmCore0SchIsr, CPU0_VECT_TABLE_ID, ISR_PRIORITY_OS_TICK);
IFX_INTERRUPT(ebcmCore1SchIsr, CPU1_VECT_TABLE_ID, ISR_PRIORITY_OS_TICK);
IFX_INTERRUPT(VFW_Gpt12_core0DeadlineTripwireIsr, CPU0_VECT_TABLE_ID, ISR_PRIORITY_GPT12_TIMER);
IFX_INTERRUPT(EbcmSch_Gpt12_core1DeadlineTripwireIsr, CPU1_VECT_TABLE_ID, ISR_PRIORITY_GPT12_TIMER);
//IFX_INTERRUPT(ebcmCore2SchIsr, 2, ISR_PRIORITY_OS_TICK);

void ebcmCore0SchIsr(void);
void ebcmCore1SchIsr(void);
void ebcmCore2SchIsr(void);
void ebcmGpt12DeadlineMonIsr(void);

static inline void runWithUnboundedExecDetectionCore0(uint32 reload, void (*func)(void));
static inline void runWithUnboundedExecDetectionCore1(uint32 reload, void (*func)(void));
static inline void updateTemporalProtectionTimerReg(uint32 reload);


/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

#pragma optimize 0
static inline boolean isCurrentCoreTaskReady(IfxCpu_ResourceCpu currCore)
{
    switch(currCore)
    {
        case IfxCpu_ResourceCpu_0: return (cpu0execTaskCounter > 0); break;
        case IfxCpu_ResourceCpu_1: return (cpu1execTaskCounter > 0); break;

        // maybe latch a software alarm here because something is very wrong.
        default:
        {
            /* SAFETY VIOLATION: Wrong CPU ID, future: trigger safe state */
            ;
        }
    }

    return FALSE;
}

static inline void updateTemporalProtectionTimer(uint32 reload)
{
    Ifx_CPU_TPS tpsValue;
    tpsValue.TIMER[0].U = __mfcr(CPU_TPS_TIMER0);
    tpsValue.TIMER[0].B.TIMER = reload;
    __mtcr(CPU_TPS_TIMER0, tpsValue.TIMER[0].U);
}

void VFW_runTasks(IfxCpu_ResourceCpu cpuId)
{
    VFW_GrantSafeMemAccess();

    /* Run hardware timer plausibility checks */
    stmPlausibilityCheck(cpuId);

    __disable();
    boolean isCurrentCoreReady = isCurrentCoreTaskReady(cpuId);
    __enable();

    if (isCurrentCoreReady == TRUE)
    {
        for (uint16 tId = 0; tId < TASK_COUNT; ++tId)
        {
            Task* targetTask = &taskTable[tId];
            if (targetTask->enabled && targetTask->func != NULL)
            {
                if (targetTask->countdown > 0U)
                {
                    targetTask->countdown--;
                }

                if (targetTask->countdown == 0U)
                {
                    targetTask->countdown  = targetTask->periodMs;

                    if (IfxCpu_ResourceCpu_0 == cpuId)
                    {
                        core0_currentTask = targetTask;
                    }

                    else if (IfxCpu_ResourceCpu_1 == cpuId)
                    {
                        core1_currentTask = targetTask;
                    }

                    if (VFW_CHECK_CORE(targetTask->coreIdMask, cpuId))
                    {
                        updateTemporalProtectionTimer(targetTask->deadlineTicks);

                        targetTask->func();

                        updateTemporalProtectionTimer(0);
                        VFW_IntegrityCheck();
                    }


                    core0_currentTask = &taskTable[TASK_COUNT];
                    core1_currentTask = &taskTable[TASK_COUNT];

                    }
                }
            }
        }

        /* in a less stringent scheduler setup, you could make the flag(s) an actual counter, run tasks and decrement it
         * until it hits zero to allow the scheduler to run until its caught up on overruns. however,
         * if the scheduler has overrun that badly, we consider ourselves in a non-safe state. if a task overruns,
         * put the whole system into a safe-state, no chance for scheduler recovery.
         */

        __disable();
        if (cpu0execTaskCounter > 0 && cpuId == IfxCpu_ResourceCpu_0)
        {
            cpu0execTaskCounter--;
        }

        if (cpu1execTaskCounter > 0 && cpuId == IfxCpu_ResourceCpu_1)
        {
            cpu1execTaskCounter--;
        }
        __enable();

        VFW_ReleaseSafeMemAccess();
}




#pragma endoptimize


void ebcmCore0SchIsr(void)
{
    /* Set next 1ms scheduler tick alarm */
    IfxStm_increaseCompare(&MODULE_STM0, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);

    if (cpu0execTaskCounter >= 1)
    {
        /* FRAME OVERRUN. trigger an NMI or software alarm and enter a safe state */
        ;
    }
    else
    {
        cpu0execTaskCounter++;
    }

}

void ebcmCore1SchIsr(void)
{
    /* Set next 1ms scheduler tick alarm */
    IfxStm_increaseCompare(&MODULE_STM1, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);

    if (cpu1execTaskCounter >= 1)
    {
        /* FRAME OVERRUN. trigger an NMI or software alarm and enter a safe state */
        ;
    }
    else
    {
        cpu1execTaskCounter++;
    }

}

void ebcmCore2SchIsr(void)
{
    /* Set next 1ms scheduler tick alarm */
    IfxStm_increaseCompare(&MODULE_STM2, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);

    if (cpu1execTaskCounter >= 1)
    {
        /* FRAME OVERRUN. trigger an NMI or software alarm and enter a safe state */
        ;
    }
    else
    {
        cpu2execTaskCounter++;
    }

}

void VFW_InitStm(EbcmStmCfg* ebcmStm, IfxCpu_ResourceCpu cpuIdx)
{
    if (VFW_IS_CPU0())
    {
        VFW_GrantSafeMemAccess();

        cpu0execTaskCounter = FALSE;
        cpu1execTaskCounter = FALSE;
        cpu2execTaskCounter = FALSE;

        /* Disable interrupts */
        IfxCpu_disableInterrupts();

        for (uint8 t_id = 0; t_id < TASK_COUNT; ++t_id)
        {
            Task *targetTask = &taskTable[t_id];

            /* Pre-calculate all task deadline temporal protection reloads */
            uint32  deadlineTicks    = (uint32) (ebcmInfo.cpuFreq * (targetTask->wcetUs * 1e-6f));
            targetTask->deadlineTicks = deadlineTicks;
        }

        VFW_ReleaseSafeMemAccess();

    }

    /* Initialize configuration-structure with defaults. */
    IfxStm_initCompareConfig(&ebcmStm->stmConfig);

    switch(cpuIdx)
    {
        case IfxCpu_ResourceCpu_0:
            ebcmStm->stmSfr = &MODULE_STM0;
            ebcmStm->stmConfig.typeOfService = IfxSrc_Tos_cpu0;
            ebcmStm->stmConfig.triggerPriority = ISR_PRIORITY_OS_TICK;
            break;

        case IfxCpu_ResourceCpu_1:
            ebcmStm->stmSfr = &MODULE_STM1;
            ebcmStm->stmConfig.typeOfService = IfxSrc_Tos_cpu1;
            ebcmStm->stmConfig.triggerPriority = ISR_PRIORITY_OS_TICK;
            break;

        case IfxCpu_ResourceCpu_2:
            ebcmStm->stmSfr = &MODULE_STM2;
            ebcmStm->stmConfig.typeOfService = IfxSrc_Tos_cpu2;
            ebcmStm->stmConfig.triggerPriority = ISR_PRIORITY_OS_TICK;
            break;

        default:
            while(1)
            {
                __disable();
            }

            break;

    }

    ebcmStm->stmConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    ebcmStm->stmConfig.ticks               = IfxStm_getTicksFromMilliseconds(ebcmStm->stmSfr, 1U);

    /* Suspend if debugger attaches */
    IfxStm_enableOcdsSuspend(ebcmStm->stmSfr);

    /* Now Initialize the comparator */
    IfxStm_initCompare(ebcmStm->stmSfr, &ebcmStm->stmConfig);

    /* Enable interrupts again */
    IfxCpu_restoreInterrupts(TRUE);

}

