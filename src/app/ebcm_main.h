/******************************************************************************
 * @file    ebcm_main.h
 * @brief   Interface for system wide ebcm defines
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

#ifndef INC_EBCM_MAIN_H_
#define INC_EBCM_MAIN_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include <safe_computation/smu.h>
#include "ssw.h"
#include "ebcm_dts.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
    float32 sysFreq;                /**< \brief Actual SPB frequency */
    float32 cpuFreq;                /**< \brief Actual CPU frequency */
    float32 pllFreq;                /**< \brief Actual PLL frequency */
    float32 stmFreq;                /**< \brief Actual STM frequency */
} EbcmSysInfo;

 /* status of EBCM */
typedef struct
{
    SswStatus          sswStatus;
    EbcmResetCode      resetCode;
    boolean            wakeupFromStby;
    SmuExecutionStatus smuStatus;
    boolean            unlockConfig;
    boolean            initComplete;
    DieTempStatusType  dieTempProfile;

} EbcmStatus;

IFX_EXTERN EbcmStatus ebcmStatus;


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void EbcmHw_initEbcm(EbcmSysInfo* ebcmInfo, IfxCpu_ResourceCpu cpuIdx);
void EbcmHw_initEbcmSafetyMechanisms(void);


#endif /* INC_EBCM_MAIN_H_ */
