/******************************************************************************
 * @file    ssw_fw_chk.h
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

#ifndef INC_SSW_MCU_FW_CHK_H_
#define INC_SSW_MCU_FW_CHK_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxMtu_cfg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define EBCM_MCU_FW_CHECK_MAX_RUNS       2


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
    uint16 eccdVal;
    uint16 fltStatusVal;
    uint16 errInfoVal;
} SshRegister;

typedef struct
{
    IfxMtu_MbistSel sshUnderTest;
    uint16 memoryType;
    uint16 inSelMask;
    SshRegister sshRegistersDef;
    SshRegister sshRegistersStb;
} MemoryTested;

typedef struct
{
    boolean mcuFwCheckSmu;
    boolean mcuFwCheckScuStem;
    boolean mcuFwCheckScuLclcon;
    boolean mcuFwCheckSsh;
} McuFwCheckStatus;

IFX_EXTERN McuFwCheckStatus mcuFwCheckStatus;

 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*
 * @brief The Safety Mechanism MCU_FW_CHECK is required for detecting failures, including both random hardware and
 * systematic hardware/software, which may have affected the correct termination of the firmware.
 */
void EbcmSsw_mcuFwCheck(void);

void EbcmSsw_clearAllSmuAlarms(void);


#endif /* INC_SSW_MCU_FW_CHK_H_ */
