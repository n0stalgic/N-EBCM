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
    Ifx_STM* stmSfr;
    IfxStm_CompareConfig stmConfig;
} EbcmStmCfg;

typedef enum
{
    TASK_ID_C0_WDT = 0,
    TASK_ID_C0_LED,

    TASK_COUNT
} taskId;


typedef struct
{
       taskId tid;
       void (*func) (void);
       const char* funcName;
       const uint32 periodMs;
       const uint32 offsetMs;
       const uint32 wcetUs;
       uint32 countdown;
       uint16 deadlineGuardReload;
       uint8  enabled;


} Task;

typedef struct
{
        Task task;
        uint64 startTicks;
        uint64 endTicks;
        uint64 elapsedUs;

} DeadlineViolationReport;


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

#define OVERRUN_REPORT_BUF_SIZE  8U

extern Ifx_Fifo taskOverrunDataFifos[3U];


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/
#define WDT_TASK_ID       0U
#define LED_TASK_ID       1U
#define PLCHK_TASK_ID     2U

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void EbcmSch_initGpt12_monitor(void);
void EbcmSch_InitStm(EbcmStmCfg* ebcmStm, IfxCpu_ResourceCpu cpuIdx);
void EbcmSch_runTasks(IfxCpu_ResourceCpu cpuId);


#endif /* INC_SCHED_H_ */
