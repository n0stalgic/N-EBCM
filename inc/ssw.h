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
#define SSW_STATUS_DATA_ADDRESS (*(volatile ssw_run_count_t* ) PMS_XRAM)


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
    uint8               lbist_app_req_count;    /* Amount of LBIST requests (by application software) */
    uint8               lbist_runs;             /* Amount of LBIST executions */
    uint8               mcu_fw_check_count;     /* Amount of MCU_FW_CHECK executions*/
    Ifx_SCU_RSTSTAT     RSTSTAT;                /* RSTSTAT register copy */

} ssw_run_count_t;

typedef enum
{
    FAILED = 0,
    PASSED,
    TEST_NOT_EVAL
} ssw_test_status;

typedef struct
{

        ssw_test_status    lbist_status;
        ssw_test_status    monbist_status;
        ssw_test_status    mcu_fw_chk_status;
        ssw_test_status    mcu_startup_status;
        ssw_test_status    alive_alarm_status;
        ssw_test_status    mbist_status;
} ssw_status_t;

typedef struct
{
    boolean test_ok_flag;
    boolean smu_error_flag;
    boolean pms_error_flag;
} ssw_monbist_status_t;

IFX_EXTERN ssw_monbist_status_t monbist_status;


typedef IfxScuRcu_ResetType ebcm_reset_type_t;
typedef IfxScuRcu_ResetCode ebcm_reset_code_t;

IFX_EXTERN volatile ssw_run_count_t* ssw_run_count;

 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void run_app_sw_startup(void);
void ebcm_trigger_warm_porst(void);
void ebcm_trigger_sw_reset(ebcm_reset_code_t resetType);
void ebcm_ssw_lbist(void);
ebcm_reset_code_t ebcm_eval_reset(void);

const char* ebcm_get_lbist_result_str(ssw_test_status status);

boolean ebcm_lockstep_injection_test(void);


#endif /* INC_SSW_H_ */
