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

#include <inc/platform.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

static const task_t task_table[] = {
        // function           name              period_ms     offset     wcet_us     enabled
        { svc_wdt,           "svc_wdt",           1,              0,          10,      TRUE },
        { ecc_ram_scrub,     "ecc_ram_scrub",     100,            5,        1000,      TRUE },

};

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
       void (*func) (void);
       const char* func_name;
       const u32 period_ms;
       const u32 offset_ms;
       const u32 wcet_us;
       u8  enabled;

} task_t;
 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/


#endif /* INC_SCHED_H_ */
