/******************************************************************************
 * @file    sched.c
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
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "Ifx_reg.h"
#include "ebcm_sched.h"
#include "ebcm_cfg.h"
#include "sm_stm.h"
#include "ebcm_isr.h"
#include "ebcm_main.h"
#include "IfxGpt12.h"
#include <string.h>


/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
uint32 cpu0_tick_count;
uint32 cpu1_tick_count;
uint32 cpu2_tick_count;


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
#error "Scheduler task boundary exceeds task_table length!"
#endif


static task_t task_table[] =
{
        /* ---------------------------- CORE0 TASKS START ---------------------------- */
        [0] = { WDT_TASK_ID,   ebcm_svc_wdt,         "ebcm_svc_wdt",           1,    0,        2,   1,   TRUE  },
        [1] = { LED_TASK_ID,   ebcm_led_task,        "ebcm_led",               250,  0,        20,   250, TRUE  },

        /* ---------------------------- CORE1 TASKS START ---------------------------- */

        /* ---------------------------- CORE2 TASKS START ---------------------------- */

        /* ---------------------------- Sentinel ---------------------------- */
        [EBCM_TASK_TABLE_END] = { 0xFF,    NULL,     "NULL_TASK",              0  ,  0,        0,    FALSE }


};



/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(ebcm_core0_sch_isr, 0, ISR_PRIORITY_OS_TICK);
IFX_INTERRUPT(ebcm_gpt12_deadline_mon_isr, 0, ISR_PRIORITY_GPT12_TIMER);
//IFX_INTERRUPT(ebcm_core1_sch_isr, 1, ISR_PRIORITY_OS_TICK);
//IFX_INTERRUPT(ebcm_core2_sch_isr, 2, ISR_PRIORITY_OS_TICK);

static inline void run_with_unbounded_exec_detection_core0(uint16 reload, void (*func)(void));
static inline void run_with_unbounded_exec_detection_core1(uint16 reload, void (*func)(void));


/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/


static inline void run_with_unbounded_exec_detection_core0(uint16 reload, void (*func)(void))
{
    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
    IfxSrc_clearRequest(IfxGpt12_T3_getSrc(&MODULE_GPT120));

    /* Arm the WCET overrun detection timer */
    IfxGpt12_T3_setTimerValue(&MODULE_GPT120, reload);

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);

    (*func)();

    IfxGpt12_T3_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
}

static inline void run_with_unbounded_exec_detection_core1(uint16 reload, void (*func)(void))
{
    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
    IfxSrc_clearRequest(IfxGpt12_T2_getSrc(&MODULE_GPT120));


    /* Arm the WCET overrun detection timer */
    IfxGpt12_T2_setTimerValue(&MODULE_GPT120, reload);

    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_start);

    (*func)();

    IfxGpt12_T2_run(&MODULE_GPT120, IfxGpt12_TimerRun_stop);
}


#pragma optimize 0
void ebcm_gpt12_deadline_mon_isr(void)
{
    /* Just toggle an LED for now. We'll set safe electrical outputs here eventually */
    IfxPort_setPinState(&MODULE_P00, EBCM_LED2_ABS_ACTIVE, IfxPort_State_toggled);
}

void ebcm_sch_run_tasks(IfxCpu_ResourceCpu cpu_id)
{

    /* Run hardware timer plausibility checks */
    stm_plausibility_chk( cpu_id );

    uint8 start, end;
    Ifx_STM* stm;

    switch(cpu_id)
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

    for (uint16 t_id = start; t_id <= end; t_id++)
    {
        task_t* target_task = &task_table[t_id];
        if (target_task->enabled && target_task->func != NULL)
        {
            if(target_task->countdown > 0U)
            {
                target_task->countdown--;
            }

            if (target_task->countdown == 0U)
            {
                target_task->countdown = target_task->period_ms;

                uint64 deadline_usec    = target_task->wcet_us + TASK_DEADLINE_MARGIN_USEC;

                uint64 P   = (uint64)GPT1_BLOCK_PRESCALER_16 * (uint64)GPT12_INPUT_PRESCALER_8;
                uint64 den = P * 1000000ULL;                         // P * 1e6
                uint64 num = (uint64)deadline_usec * (uint64)GPT12_BASE_FREQ_100MHZ;

                uint64 reload64 = (num + den - 1ULL) / den;          // ceil(num/den)

                if (reload64 == 0ULL) reload64 = 1ULL;
                if (reload64 > 0xFFFFULL) reload64 = 0xFFFFULL;

                uint16 reload = (uint16)reload64;


                if (cpu_id == IfxCpu_ResourceCpu_0)
                {
                    run_with_unbounded_exec_detection_core0(reload, target_task->func);
                }

                else if (cpu_id == IfxCpu_ResourceCpu_1)
                {
                    run_with_unbounded_exec_detection_core1(reload, target_task->func);
                }
            }
        }
    }
}

#pragma endoptimize


void ebcm_core0_sch_isr(void)
{
    /* Set next 1ms scheduler tick alarm */
    IfxStm_increaseCompare(&MODULE_STM0, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);

    /* Increment the ms value */
    cpu0_tick_count++;

    /* Enable the global interrupts of this CPU */
    IfxCpu_enableInterrupts();

    /* possibly latch alarm here if scheduler is called before system is fully init'd */


    /* Execute the main task scheduler */
    ebcm_sch_run_tasks((IfxCpu_ResourceCpu) IfxCpu_getCoreIndex());
}

void ebcm_core1_sch_isr(void)
{
    /* Set next 1ms scheduler tick alarm */
    IfxStm_increaseCompare(&MODULE_STM1, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);

    /* Increment the ms value */
    cpu1_tick_count++;

    /* Enable the global interrupts of this CPU */
    IfxCpu_enableInterrupts();

    /* possibly latch alarm here if scheduler is called before system is fully init'd */

    /* Execute the main task scheduler */
    ebcm_sch_run_tasks((IfxCpu_ResourceCpu) IfxCpu_getCoreIndex());
}

void ebcm_core2_sch_isr(void)
{
    /* Set next 1ms scheduler tick alarm */
    IfxStm_increaseCompare(&MODULE_STM2, IfxStm_Comparator_0, IFX_CFG_STM_TICKS_PER_MS);

    /* Increment the ms value */
    cpu2_tick_count++;

    /* Enable the global interrupts of this CPU */
    IfxCpu_enableInterrupts();

    /* possibly latch alarm here if scheduler is called before system is fully init'd */

    /* Execute the main task scheduler */
    ebcm_sch_run_tasks((IfxCpu_ResourceCpu) IfxCpu_getCoreIndex());
}

void ebcm_sch_init_stm(ebcm_stm_cfg* ebcm_stm, IfxCpu_ResourceCpu cpu_idx)
{

    cpu0_tick_count = 0;
    cpu1_tick_count = 0;
    cpu2_tick_count = 0;

    /* Disable interrupts */
    IfxCpu_disableInterrupts();

    /* Initialize configuration-structure with defaults. */
    IfxStm_initCompareConfig(&ebcm_stm->stm_config);

    switch(cpu_idx)
    {
        case IfxCpu_ResourceCpu_0:
            ebcm_stm->stm_sfr = &MODULE_STM0;
            ebcm_stm->stm_config.typeOfService = IfxSrc_Tos_cpu0;
            ebcm_stm->stm_config.triggerPriority = ISR_PRIORITY_OS_TICK;
            memset(&task_overrun_data_fifos[0], 0, sizeof(task_overrun_data_fifos[0]) / sizeof(deadline_violation_report));
            Ifx_Fifo_init(&task_overrun_data_fifos[0], OVERRUN_REPORT_BUF_SIZE, sizeof(deadline_violation_report));
            break;

        case IfxCpu_ResourceCpu_1:
            ebcm_stm->stm_sfr = &MODULE_STM1;
            ebcm_stm->stm_config.typeOfService = IfxSrc_Tos_cpu1;
            ebcm_stm->stm_config.triggerPriority = ISR_PRIORITY_OS_TICK;
            memset(&task_overrun_data_fifos[1], 0, sizeof(task_overrun_data_fifos[1]) / sizeof(deadline_violation_report));
            Ifx_Fifo_init(&task_overrun_data_fifos[1], OVERRUN_REPORT_BUF_SIZE, sizeof(deadline_violation_report));

            break;
        case IfxCpu_ResourceCpu_2:
            ebcm_stm->stm_sfr = &MODULE_STM2;
            ebcm_stm->stm_config.typeOfService = IfxSrc_Tos_cpu2;
            ebcm_stm->stm_config.triggerPriority = ISR_PRIORITY_OS_TICK;
            memset(&task_overrun_data_fifos[2], 0, sizeof(task_overrun_data_fifos[2]) / sizeof(deadline_violation_report));
            Ifx_Fifo_init(&task_overrun_data_fifos[2], OVERRUN_REPORT_BUF_SIZE, sizeof(deadline_violation_report));

            break;

        default:
            while(1)
            {
                __debug();
            }

            break;

    }

    ebcm_stm->stm_config.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    ebcm_stm->stm_config.ticks               = IfxStm_getTicksFromMilliseconds(ebcm_stm->stm_sfr, 1U);

    /* Suspend if debugger attaches */
    IfxStm_enableOcdsSuspend(ebcm_stm->stm_sfr);

    /* Now Initialize the comparator */
    IfxStm_initCompare(ebcm_stm->stm_sfr, &ebcm_stm->stm_config);

    /* Enable interrupts again */
    IfxCpu_restoreInterrupts(TRUE);
}


void ebcm_sch_init_gpt12_monitor(void)
{
    IfxGpt12_enableModule(&MODULE_GPT120);
    IfxGpt12_setGpt1BlockPrescaler(&MODULE_GPT120, IfxGpt12_Gpt1BlockPrescaler_16);

    /* Initialize core Timer T3 for core 0  */
    IfxGpt12_T3_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T3 to timer mode         */
    IfxGpt12_T3_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T3 count direction       */
    IfxGpt12_T3_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_16); /* Set T3 input prescaler       */

    /* Initialize auxiliary Timer T2 for core 1  */
    IfxGpt12_T2_setMode(&MODULE_GPT120, IfxGpt12_Mode_timer);                       /* Set T2 to timer mode         */
    IfxGpt12_T2_setTimerDirection(&MODULE_GPT120, IfxGpt12_TimerDirection_down);    /* Set T2 count direction       */
    IfxGpt12_T2_setTimerPrescaler(&MODULE_GPT120, IfxGpt12_TimerInputPrescaler_16); /* Set T2 input prescaler       */

    /* Initialize the interrupt */
    volatile Ifx_SRC_SRCR *src = IfxGpt12_T3_getSrc(&MODULE_GPT120);                /* Get the interrupt address    */
    IfxSrc_init(src, IfxSrc_Tos_cpu0 , ISR_PRIORITY_GPT12_TIMER);                   /* Initialize service request   */
    IfxSrc_enable(src);                                                             /* Enable GPT12 interrupt       */
}
