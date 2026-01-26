/******************************************************************************
 * @file    trap_handle.h
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

#ifndef SAFE_COMPUTATION_TRAP_HANDLE_H_
#define SAFE_COMPUTATION_TRAP_HANDLE_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Cpu/Std/Ifx_Types.h"
#include "Cpu/Std/IfxCpu_Intrinsics.h"
#include "Ifx_Cfg.h"
#include "IfxCpu_Trap.h"
#include "IfxSmu_reg.h"
#include "IfxCpu_cfg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/


#define SV_FAILURE_DBG() \
                        do { \
                            __disable(); \
                            while (1) __debug(); \
                        } \
                        while (1)

#define SV_BAD_SCHEDULER_FUNC_DBG SV_FAILURE_DBG
#define SV_VFW_INTEGRITY_CHECK_FAILURE SV_FAILURE_DBG


/*define a hook for internal protection error traps*/
#define IFX_CFG_CPU_TRAP_IPE_HOOK(trapWatch)    ((void)internalProtectionHook(trapWatch))

/*define a hook for instruction error traps*/
#define IFX_CFG_CPU_TRAP_IE_HOOK(trapWatch)     ((void)instructionErrorsHook(trapWatch))

/*define a hook for bus error traps*/
#define IFX_CFG_CPU_TRAP_BE_HOOK(trapWatch)     ((void)busHook(trapWatch))

/*define a hook for non-maskable interrupt traps*/
#define IFX_CFG_CPU_TRAP_NMI_HOOK(trapWatch)    ((void)nonMaskableInterruptHook(trapWatch))

/*define how many context save areas can be dumped*/
#define CSA_CAPTURE_LIMIT   (20u)

/*define how many stacks can be dumped for upper context dumps*/
#define STACK_CAPTURE_LIMIT (10u)

/*define how many words should be captured from each stack*/
#define STACK_CAPTURE_SIZE  (8u)

#define CSA_UPPER_CONTEXT   (1)
#define CSA_LOWER_CONTEXT   (0)

#define IFX_CFG_CPU_TRAP_ASSERT_HOOK(trapWatch)     ((void)assertHook(trapWatch))

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct {
     Ifx_CPU_PCXI CSA_PCXI;
     Ifx_CPU_PSW CSA_PSW;
     Ifx_CPU_A CSA_A10;
     Ifx_CPU_A CSA_A11;
     Ifx_CPU_D CSA_D8;
     Ifx_CPU_D CSA_D9;
     Ifx_CPU_D CSA_D10;
     Ifx_CPU_D CSA_D11;
     Ifx_CPU_A CSA_A12;
     Ifx_CPU_A CSA_A13;
     Ifx_CPU_A CSA_A14;
     Ifx_CPU_A CSA_A15;
     Ifx_CPU_D CSA_D12;
     Ifx_CPU_D CSA_D13;
     Ifx_CPU_D CSA_D14;
     Ifx_CPU_D CSA_D15;
}Ifx_CSA_Upper;

typedef struct {
     Ifx_CPU_PCXI CSA_PCXI;
     Ifx_CPU_A CSA_A11;
     Ifx_CPU_A CSA_A2;
     Ifx_CPU_A CSA_A3;
     Ifx_CPU_D CSA_D0;
     Ifx_CPU_D CSA_D1;
     Ifx_CPU_D CSA_D2;
     Ifx_CPU_D CSA_D3;
     Ifx_CPU_A CSA_A4;
     Ifx_CPU_A CSA_A5;
     Ifx_CPU_A CSA_A6;
     Ifx_CPU_A CSA_A7;
     Ifx_CPU_D CSA_D4;
     Ifx_CPU_D CSA_D5;
     Ifx_CPU_D CSA_D6;
     Ifx_CPU_D CSA_D7;
}Ifx_CSA_Lower;

typedef union {
    Ifx_CSA_Upper UPPER;
    Ifx_CSA_Lower LOWER;
} Ifx_CSA;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

void csaCapture(void);
void piearPietrCapture(void);
void diearDietrCapture(void);
void datrDeaddCapture(void);
void agCapture(void);

void internalProtectionHook(IfxCpu_Trap trapInfo);
void instructionErrorsHook(IfxCpu_Trap trapInfo);
void busHook(IfxCpu_Trap trapInfo);
void assertHook(IfxCpu_Trap trapInfo);
void nonMaskableInterruptHook(IfxCpu_Trap trapInfo);


#endif /* SAFE_COMPUTATION_TRAP_HANDLE_H_ */
