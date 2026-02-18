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
#include <string.h>
#include <vfw_sched.h>



/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

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

#define CORE0_TASK_START  0U
#define CORE0_TASK_END    1U

#define CORE1_TASK_START  2U
#define CORE1_TASK_END    2U

#define CORE2_TASK_START  3U
#define CORE2_TASK_END    3U

#define EBCM_TASK_TABLE_END    4U

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



#if (CORE0_TASK_START > CORE0_TASK_END)
#error "Bad Core 0 scheduler task boundaries!"
#endif

#if !((CORE1_TASK_START <= CORE1_TASK_END) && (CORE1_TASK_START > CORE0_TASK_END))
#error "Bad Core 1 scheduler task boundaries!"
#endif

#if !((CORE2_TASK_START <= CORE2_TASK_END) && (CORE2_TASK_START > CORE1_TASK_END))
#error "Bad Core 2 scheduler task boundaries!"
#endif

#if (CORE2_TASK_END >= EBCM_TASK_TABLE_END)
#error "Scheduler task boundary exceeds taskTable length!"
#endif

#pragma section all "vfw_safe0"
static Task taskTable[] =
{
        /* ---------------------------- CORE0 TASKS START ---------------------------- */
        [TASK_ID_C0_WDT] = { WDT_TASK_ID,   EbcmHw_svcWdt,     "EbcmHw_svcWdt",     1,    0,        5,   1,  0, TRUE  },
        [TASK_ID_C0_LED] = { LED_TASK_ID,   EbcmHw_ledTask,    "EbcmHw_ledTask",    10,   0,        4,   10, 0, TRUE  },

        /* ---------------------------- CORE1 TASKS START ---------------------------- */

        /* ---------------------------- CORE2 TASKS START ---------------------------- */

        /* ---------------------------- Sentinel ---------------------------- */
        [TASK_COUNT] = { 0xFF,    NULL,     "NULL_TASK",              0  ,  0,        0,    0, FALSE }


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

static inline void runWithUnboundedExecDetectionCore0(uint16 reload, void (*func)(void));
static inline void runWithUnboundedExecDetectionCore1(uint16 reload, void (*func)(void));


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


static inline void runWithUnboundedExecDetectionCore0(uint16 reload, void (*func)(void))
{
    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
    IfxSrc_clearRequest(IfxGpt12_T3_getSrc(&MODULE_GPT120));

    /* Arm the WCET overrun detection timer */
    IfxGpt12_T3_setTimerValue(&MODULE_GPT120, reload);

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);

    (*func)();

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
}

static inline void runWithUnboundedExecDetectionCore1(uint16 reload, void (*func)(void))
{
    if (func == NULL)
    {

    }

    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
    IfxSrc_clearRequest(IfxGpt12_T2_getSrc(&MODULE_GPT120));


    /* Arm the WCET overrun detection timer */
    IfxGpt12_T2_setTimerValue(&MODULE_GPT120, reload);

    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);

    (*func)();

    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
}



void VFW_Gpt12_core0DeadlineTripwireIsr(void)
{

    /* Just toggle an LED for now. We'll set safe electrical outputs here eventually */
    IfxCpu_disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxPort_setPinState(&MODULE_P00, EBCM_LED2, IfxPort_State_low);

}

void EbcmSch_Gpt12_core1DeadlineTripwireIsr(void)
{

    /* do nothing for now on core 1. for now, may add a special LED pattern to indicate
     * WCET violation. for the final system, trigger safe outputs
     */

    IfxCpu_disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxPort_setPinState(&MODULE_P00, EBCM_LED2, IfxPort_State_low);


}

void VFW_runTasks(IfxCpu_ResourceCpu cpuId)
{
    VFW_GrantSafeMemAccess();

    /* Run hardware timer plausibility checks */
    stmPlausibilityCheck(cpuId);

    uint8 start, end;
    Ifx_STM* stm;

    __disable();
    boolean isCurrentCoreReady = isCurrentCoreTaskReady(cpuId);
    __enable();

    switch(cpuId)
    {
        case IfxCpu_ResourceCpu_0:
            start = CORE0_TASK_START;
            end   = CORE0_TASK_END;
            stm  = &MODULE_STM0;
            break;
        case IfxCpu_ResourceCpu_1:
            start = CORE1_TASK_START;
            end   = CORE1_TASK_END;
            stm   = &MODULE_STM1;
            break;
        case IfxCpu_ResourceCpu_2:
            start = CORE2_TASK_START;
            end   = CORE2_TASK_END;
            stm   = &MODULE_STM2;
            break;
        default:
            return; /* invalid core */
    }

    if (isCurrentCoreReady == TRUE)
    {
        for (uint16 tId = start; tId <= end; tId++)
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

                    if (cpuId == IfxCpu_ResourceCpu_0)
                    {
                        core0_currentTask = targetTask;
                        runWithUnboundedExecDetectionCore0(targetTask->deadlineGuardReload, targetTask->func);
                        VFW_IntegrityCheck();

                    }

                    else if (cpuId == IfxCpu_ResourceCpu_1)
                    {
                        core1_currentTask = targetTask;
                        runWithUnboundedExecDetectionCore1(targetTask->deadlineGuardReload, targetTask->func);
                        VFW_IntegrityCheck();
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
    }

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
    VFW_GrantSafeMemAccess();

    cpu0execTaskCounter = FALSE;
    cpu1execTaskCounter = FALSE;
    cpu2execTaskCounter = FALSE;

    /* Disable interrupts */
    IfxCpu_disableInterrupts();

    for (uint8 t_id = 0; t_id < EBCM_TASK_TABLE_END; ++t_id)
    {
        Task *targetTask = &taskTable[t_id];
        /* Pre-calculate all task deadline guardian timer reloads */
        uint64 deadlineUsec    = targetTask->wcetUs + TASK_DEADLINE_MARGIN_USEC;

        // SCHED_P * 1e6
        uint64 num = (uint64)deadlineUsec * (uint64)GPT12_BASE_FREQ_100MHZ;

        uint64 reload64 = (num + ((uint64) SCHED_DENOM) - 1ULL) / SCHED_DENOM;          // ceil(num/den)

        if (reload64 == 0ULL) reload64 = 1ULL;
        if (reload64 > 0xFFFFULL) reload64 = 0xFFFFULL;

        uint16 reload = (uint16)reload64;

        targetTask->deadlineGuardReload = reload;
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

    VFW_ReleaseSafeMemAccess();
}


void VFW_initGpt12_monitor(void)
{
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_16);

    /* Initialize core Timer T3 for core 0  */
    IfxGpt12_T3_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T3 to timer mode         */
    IfxGpt12_T3_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T3 count direction       */
    IfxGpt12_T3_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_8);  /* Set T3 input prescaler       */

    /* Initialize auxiliary Timer T2 for core 1  */
    IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T2 to timer mode         */
    IfxGpt12_T2_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T2 count direction       */
    IfxGpt12_T2_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_8);  /* Set T2 input prescaler       */

    /* Initialize the interrupt */
    volatile Ifx_SRC_SRCR *src0 = IfxGpt12_T3_getSrc(&MODULE_GPT120);               /* Get the interrupt address    */
    volatile Ifx_SRC_SRCR *src1 = IfxGpt12_T2_getSrc(&MODULE_GPT120);
    IfxSrc_init(src0, IfxSrc_Tos_cpu0 , ISR_PRIORITY_GPT12_TIMER);                   /* Initialize service request   */
    IfxSrc_init(src1, IfxSrc_Tos_cpu1 , ISR_PRIORITY_GPT12_TIMER);                   /* Initialize service request   */


    IfxSrc_enable(src0);                                                             /* Enable GPT12 interrupt       */
    IfxSrc_enable(src1);                                                             /* Enable GPT12 interrupt       */
}
