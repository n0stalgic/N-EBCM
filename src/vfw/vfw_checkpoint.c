/******************************************************************************
 * @file    vfw_checkpoint.c
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
#include "vfw_checkpoint.h"
#include "IfxCpu.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/* probably put these VFW counters and flags in safe RAM... TBD */
volatile uint32 vfwCheckpoint_A = 0U;
volatile uint32 vfwCheckpoint_B = 0U;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

volatile boolean VFW_integrityCheckFailed = FALSE;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void VFW_Init(void)
{
    vfwCheckpoint_A = 0U;
    vfwCheckpoint_B = 0U;
}

void VFW_Precheck(VfwSignature sig)
{
    vfwCheckpoint_A += sig;
}


void VFW_Postcheck(VfwSignature sig)
{
    vfwCheckpoint_B -= sig;
}

boolean VFW_HasIntegrityCheckFailed(void)
{
    return VFW_integrityCheckFailed;
}

#pragma optimize 0
boolean  VFW_IntegrityCheck(void)
{
    if ((vfwCheckpoint_A + vfwCheckpoint_B) != 0)
    {
        /* SAFETY VIOLATION: Execution path error */
        IfxCpu_disableInterrupts();
        IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
        VFW_integrityCheckFailed = TRUE;
        return FALSE;

      //  __debug();
    }

    VFW_integrityCheckFailed = FALSE;

    return TRUE;
}

#pragma endoptimize
