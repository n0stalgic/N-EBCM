/******************************************************************************
 * @file    ssw.h
 * @brief   Interface for safe software startup
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

#ifndef INC_SSW_H_
#define INC_SSW_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxPms_reg.h"
#include "IfxScuRcu.h"
#include "IfxScu_reg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define PMSWSTAT2_WAKE_UP_FLAGS_MASK     0xFF

/**
 * @brief address to store power-on check metadata in SCR XRAM
 */
#define SSW_STATUS_DATA_ADDRESS (*(volatile SswRunCount*)PMS_XRAM)


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
    uint8           lbistAppReqCount;       /* Amount of LBIST requests (by application software) */
    uint8           lbistRuns;              /* Amount of LBIST executions */
    uint8           mcuFwCheckCount;        /* Amount of MCU_FW_CHECK executions*/
    uint8           mcuFwCheckFailCount;    /* Amount of MCU_FW_CHECK failures */
    Ifx_SCU_RSTSTAT RSTSTAT;                /* RSTSTAT register copy */

} SswRunCount;

typedef enum
{
    FAILED = 0,
    PASSED,
    TEST_NOT_EVAL
} SswTestStatus;

#define LAST_IFXSCURCU_RESETTYPE_ENUM IfxScuRcu_ResetType_undefined

typedef enum IfxScuRcu_ResetType
{
    EBCMResetType_coldpoweron = IfxScuRcu_ResetType_coldpoweron,      /* Cold Power On Reset */
    EBCMResetType_system      = IfxScuRcu_ResetType_system,           /* system Reset */
    EBCMResetType_application = IfxScuRcu_ResetType_application,      /* application reset */
    EBCMResetType_warmpoweron = IfxScuRcu_ResetType_warmpoweron,      /* Warm Power On Reset */
    EBCMResetType_debug       = IfxScuRcu_ResetType_debug,            /* debug reset */
    EBCMResetType_undefined   = IfxScuRcu_ResetType_undefined,        /* Undefined Reset */
    EBCMResetType_lbist       = LAST_IFXSCURCU_RESETTYPE_ENUM + 10    /* LBIST Reset, adding 10 to be sure
                                                                               enum value is not assigned. */
} EbcmReset;

typedef struct
{

    SswTestStatus lbistStatus;
    SswTestStatus monbistStatus;
    SswTestStatus mcuFwChkStatus;
    SswTestStatus mcuStartupStatus;
    SswTestStatus aliveAlarmStatus;
    SswTestStatus mbistStatus;
} SswStatus;

typedef struct
{
    boolean testOkFlag;
    boolean smuErrorFlag;
    boolean pmsErrorFlag;
} SswMonbistStatus;

IFX_EXTERN SswMonbistStatus monbistStatus;


typedef IfxScuRcu_ResetType EbcmResetType;
typedef IfxScuRcu_ResetCode EbcmResetCode;

IFX_EXTERN volatile SswRunCount* sswRunCount;

 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void EbcmSsw_runAppSwStartup(void);
void EbcmSsw_triggerWarmPorst(void);
void EbcmSsw_triggerSwReset(EbcmResetType resetType);
void EbcmSsw_lbist(void);
EbcmResetCode EbcmSsw_evalReset(void);

const char* EbcmSsw_getLbistResultStr(SswTestStatus status);

boolean EbcmSsw_lockstepInjectionTest(void);


#endif /* INC_SSW_H_ */
