/******************************************************************************
 * @file    vfw_mem.c
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
#include "vfw_mem.h"
#include "vfw_err.h"
#include <string.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define STACK_PAINT_VALUE_0 0xaa
#define STACK_PAINT_VALUE_1 0xbb
#define STACK_PAINT_VALUE_2 0xcc

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

extern uintptr_t __USTACK0;
extern uintptr_t __USTACK0_END;

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void Vfw_paintStack(IfxCpu_ResourceCpu cpu_id, uintptr_t stack_top, size_t size)
{
    switch (cpu_id)
    {
        case IfxCpu_ResourceCpu_0: memset((void *) stack_top, STACK_PAINT_VALUE_0, size); break;
        case IfxCpu_ResourceCpu_1: memset((void *) stack_top, STACK_PAINT_VALUE_1, size); break;
        case IfxCpu_ResourceCpu_2: memset((void *) stack_top, STACK_PAINT_VALUE_2, size); break;
        default:
            VFW_ERR_HALT_CPU_AND_SCHEDULER();
            break;
    }
}

uintptr_t Vfw_stackHighWatermark(IfxCpu_ResourceCpu cpu_id, uintptr_t stack_top, size_t size)
{
    uint8* stack     = (uint8 *) __USTACK0;
    uint8* stack_end = (uint8 *) __USTACK0_END;

    uint8 fill = STACK_PAINT_VALUE_0;

    switch (cpu_id)
    {
        case IfxCpu_ResourceCpu_0: fill = STACK_PAINT_VALUE_0; break;
        case IfxCpu_ResourceCpu_1: fill = STACK_PAINT_VALUE_1; break;
        case IfxCpu_ResourceCpu_2: fill = STACK_PAINT_VALUE_2; break;
        default:
            VFW_ERR_HALT_CPU_AND_SCHEDULER();
            break;
    }

    while (stack < stack_end && *stack == fill)
    {
        stack++;
    }

    return (uintptr_t) stack;
}


