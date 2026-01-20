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

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(ebcm_core0_sch_isr, 0, ISR_PRIORITY_OS_TICK);
//IFX_INTERRUPT(ebcm_core1_sch_isr, 1, ISR_PRIORITY_OS_TICK);
//IFX_INTERRUPT(ebcm_core2_sch_isr, 2, ISR_PRIORITY_OS_TICK);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/* Run the tasks here. Each CPU will run tasks tagged with its affinity.
 * We won't disable interrupts here at all because the tick count reading
 * and writing is taking place in the same ISR.
 */
void ebcm_sch_run_tasks(IfxCpu_ResourceCpu cpu_id)
{

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

    for (uint8 t_id = start; t_id <= end; t_id++)
    {
        if (task_table[t_id].enabled && task_table[t_id].func != NULL)
        {
            if (--(task_table[t_id].countdown) == 0U)
            {
                uint64 start_ticks = IfxStm_get(stm);
                (*task_table[t_id].func)();
                uint64 end_ticks   = IfxStm_get(stm);
                uint64 elapsed_ticks    = end_ticks - start_ticks;
                uint64 elapsed_us       = (elapsed_ticks / IfxStm_getTicksFromMicroseconds(&MODULE_STM0, 1U));

                if ( elapsed_us  > task_table[t_id].wcet_us )
                {
                    /* Deadline violated, latch an alarm to the safety management unit.
                     * The SMU driver isn't complete so for now add to a FIFO
                     */

                    deadline_violation_report rep;
                    rep.task = task_table[t_id];
                    rep.start_ticks = start_ticks;
                    rep.end_ticks   = end_ticks;
                    rep.elapsed_us = elapsed_us;

                    // Ignore return for now as FIFO logging is temporary
                    Ifx_Fifo_write(&task_overrun_data_fifos[cpu_id], (void*) &rep, 1, 0);

                }

                task_table[t_id].countdown = task_table[t_id].period_ms;
            }
        }
    }
}


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
