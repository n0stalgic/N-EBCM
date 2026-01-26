/******************************************************************************
 * @file    vfw_checkpoint.h
 * @brief   Vital Framework checkpointing
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

#ifndef VFW_VFW_CHECKPOINT_H_
#define VFW_VFW_CHECKPOINT_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define VFW_CHECKPOINT_PRECHECK(signature)  (vfwCheckpoint_A += (signature))
#define VFW_CHECKPOINT_ENTRY(signature)     (vfwCheckpoint_A -= (signature))
#define VFW_CHECKPOINT_EXIT(signature)      (vfwCheckpoint_B += (signature))
#define VFW_CHECKPOINT_POSTCHECK(signature) (vfwCheckpoint_B -= (signature))

/* Task signatures for functions running in the scheduler */
#define VFW_NULL_SIGNATURE             0x00000000U
#define VFW_TASK_SIGNATURE_LED         0xD40B934CU
#define VFW_TASK_SIGNATURE_WDT         0x1C653C80U
#define VFW_TASK_SIGNATURE_2           0x747D2AEFU
#define VFW_TASK_SIGNATURE_3           0x1E1D6CB4U
#define VFW_TASK_SIGNATURE_4           0xE0175EE9U
#define VFW_TASK_SIGNATURE_5           0x07A1557BU
#define VFW_TASK_SIGNATURE_6           0xB0EF99CDU
#define VFW_TASK_SIGNATURE_7           0x2D63F2ACU
#define VFW_TASK_SIGNATURE_8           0x8D3AFF51U
#define VFW_TASK_SIGNATURE_9           0xD4B0D912U

/* Function signatures for individual functions needed for checkpointing.
 * These funcs may or may not be called by tasks running in the scheduler
 */
#define VFW_FUNC_SIGNATURE_LED_UPDATE  0x737AB5EFU



/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IFX_EXTERN volatile uint32 vfwCheckpoint_A;
IFX_EXTERN volatile uint32 vfwCheckpoint_B;

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef uint32 VfwSignature;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void VFW_Init(void);
void VFW_Precheck(VfwSignature sig);
void VFW_Postcheck(VfwSignature sig);
boolean VFW_IntegrityCheck(void);
boolean VFW_HasIntegrityCheckFailed(void);


#endif /* VFW_VFW_CHECKPOINT_H_ */
