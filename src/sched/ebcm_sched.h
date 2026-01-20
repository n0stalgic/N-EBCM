/******************************************************************************
 * @file    sched.h
 * @brief   Interface for scheduler
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

#ifndef INC_SCHED_H_
#define INC_SCHED_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include "Ifx_Types.h"
#include "IfxStm.h"
#include "ebcm_wdt.h"
#include "ebcm_led.h"
#include "Ifx_Fifo.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
    Ifx_STM             *stm_sfr;
    IfxStm_CompareConfig stm_config;
} ebcm_stm_cfg;



typedef struct
{
       uint8 tid;
       void (*func) (void);
       const char* func_name;
       const uint32 period_ms;
       const uint32 offset_ms;
       const uint32 wcet_us;
       uint32 countdown;
       uint8  enabled;


} task_t;

typedef struct
{
        task_t task;
        uint64 start_ticks;
        uint64 end_ticks;
        uint64 elapsed_us;

} deadline_violation_report;


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

#define OVERRUN_REPORT_BUF_SIZE  8U

extern Ifx_Fifo task_overrun_data_fifos[3U];


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/
#define WDT_TASK_ID       0U
#define LED_TASK_ID       1U
#define PLCHK_TASK_ID     2U


#define CORE0_TASK_START  0U
#define CORE0_TASK_END    1U

#define CORE1_TASK_START  2U
#define CORE1_TASK_END    2U

#define CORE2_TASK_START  3U
#define CORE2_TASK_END    3U

#define EBCM_TASK_TABLE_END    2U

__attribute__((unused)) static task_t task_table[] =
{
        /* ---------------------------- CORE0 TASKS START ---------------------------- */
        [0] = { WDT_TASK_ID,   ebcm_svc_wdt,         "ebcm_svc_wdt",           1,    0,        10,   1,   TRUE  },
        [1] = { LED_TASK_ID,   ebcm_led_task,        "ebcm_led",               250,  0,        10,   250, TRUE  },

        /* ---------------------------- CORE1 TASKS START ---------------------------- */

        /* ---------------------------- CORE2 TASKS START ---------------------------- */

        /* ---------------------------- Sentinel ---------------------------- */
        [EBCM_TASK_TABLE_END] = { 0xFF,    NULL,     "NULL_TASK",              0  ,  0,        0,    FALSE }


};

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void ebcm_sch_init_stm(ebcm_stm_cfg* ebcm_stm, IfxCpu_ResourceCpu cpu_idx);
void ebcm_sch_run_tasks(IfxCpu_ResourceCpu cpu_id);
void ebcm_core0_sch_isr(void);
void ebcm_core1_sch_isr(void);
void ebcm_core2_sch_isr(void);

#endif /* INC_SCHED_H_ */
