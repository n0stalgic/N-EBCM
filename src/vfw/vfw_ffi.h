/******************************************************************************
 * @file    ebcm_mpu.h
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

#ifndef VFW_FFI_H_
#define VFW_FFI_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxPort.h"
#include "IfxCpu_reg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define DPR_GRANULARITY                 8                           /* Data Protection Range granularity in bytes   */
#define CPR_GRANULARITY                 32                          /* Code Protection Range granularity in bytes   */

/* Data Protection Ranges */
#define DATA_PROTECTION_RANGE_0         0
#define DATA_PROTECTION_RANGE_1         1
#define DATA_PROTECTION_RANGE_2         2
#define DATA_PROTECTION_RANGE_3         3
#define DATA_PROTECTION_RANGE_4         4
#define DATA_PROTECTION_RANGE_5         5
#define DATA_PROTECTION_RANGE_6         6
#define DATA_PROTECTION_RANGE_7         7
#define DATA_PROTECTION_RANGE_8         8
#define DATA_PROTECTION_RANGE_9         9
#define DATA_PROTECTION_RANGE_10        10
#define DATA_PROTECTION_RANGE_11        11
#define DATA_PROTECTION_RANGE_12        12
#define DATA_PROTECTION_RANGE_13        13
#define DATA_PROTECTION_RANGE_14        14
#define DATA_PROTECTION_RANGE_15        15
#define DATA_PROTECTION_RANGE_16        16
#define DATA_PROTECTION_RANGE_17        17
#define DATA_PROTECTION_RANGE_18        18
#define DATA_PROTECTION_RANGE_19        19
#define DATA_PROTECTION_RANGE_20        20
#define DATA_PROTECTION_RANGE_21        21
#define DATA_PROTECTION_RANGE_22        22
#define DATA_PROTECTION_RANGE_23        23

/* Code Protection Ranges */
#define CODE_PROTECTION_RANGE_0         0
#define CODE_PROTECTION_RANGE_1         1
#define CODE_PROTECTION_RANGE_2         2
#define CODE_PROTECTION_RANGE_3         3
#define CODE_PROTECTION_RANGE_4         4
#define CODE_PROTECTION_RANGE_5         5
#define CODE_PROTECTION_RANGE_6         6
#define CODE_PROTECTION_RANGE_7         7
#define CODE_PROTECTION_RANGE_8         8
#define CODE_PROTECTION_RANGE_9         9
#define CODE_PROTECTION_RANGE_10        10
#define CODE_PROTECTION_RANGE_11        11
#define CODE_PROTECTION_RANGE_12        12
#define CODE_PROTECTION_RANGE_13        13
#define CODE_PROTECTION_RANGE_14        14
#define CODE_PROTECTION_RANGE_15        15

/* Protection Sets */
#define PROTECTION_SET_0                0
#define PROTECTION_SET_1                1
#define PROTECTION_SET_2                2
#define PROTECTION_SET_3                3
#define PROTECTION_SET_4                4
#define PROTECTION_SET_5                5
#define PROTECTION_SET_6                6
#define PROTECTION_SET_7                7


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/* MPU control functions */
void VFW_enableMemProtection(void);
void VFW_defineDataProtectionRange(uint32 lowerBoundAddress, uint32 upperBoundAddress, uint8 range);
void VFW_defineCodeProtectionRange(uint32 lowerBoundAddress, uint32 upperBoundAddress, uint8 range);
void VFW_enableDataRead(uint8 protectionSet, uint8 range);
void VFW_enableDataWrite(uint8 protectionSet, uint8 range);
void VFW_enableCodeExecution(uint8 protectionSet, uint8 range);


/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/* Function to set the given Protection Set as active.
 * This function needs to be declared as inline because the Program Status Word (PSW) is one of the registers
 * automatically saved to the Context Save Area (CSA) when a function is called.
 * If this function was not declared as inline, the Upper Context (16 registers including the PSW) would be
 * automatically saved to the CSA and re-loaded when the function return, thus losing the change in the PSW.
 */

IFX_INLINE void set_active_protection_set(uint8 protectionSet)
{
    Ifx_CPU_PSW PSWRegisterValue;

    PSWRegisterValue.U = __mfcr(CPU_PSW);               /* Get the Program Status Word (PSW) register value         */
    PSWRegisterValue.B.PRS = protectionSet;             /* Set the PRS bitfield to enable the Protection Set        */
    __mtcr(CPU_PSW, PSWRegisterValue.U);                /* Set the Program Status Word (PSW) register               */
}


IFX_INLINE void VFW_GrantSafeMemAccess(void)
{
    set_active_protection_set(PROTECTION_SET_0);
}

IFX_INLINE void VFW_ReleaseSafeMemAccess(void)
{
    set_active_protection_set(PROTECTION_SET_1);
}

#endif /* VFW_FFI_H_ */
