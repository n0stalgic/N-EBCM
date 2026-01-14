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
    uint16   eccd_val;
    uint16   flt_status_val;
    uint16   err_info_val;
} ssh_register_t;

typedef struct
{
    IfxMtu_MbistSel  ssh_under_test;
    uint16           memory_type;
    uint16           in_sel_mask;
    ssh_register_t   ssh_registers_def;
    ssh_register_t   ssh_registers_stb;
} memory_tested_t;

typedef struct
{
    boolean mcu_fw_check_smu;
    boolean mcu_fw_check_scu_stem;
    boolean mcu_fw_check_scu_lclcon;
    boolean mcu_fw_check_ssh;
} mcu_fw_check_status_t;

IFX_EXTERN mcu_fw_check_status_t mcu_fw_check_status;

 
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
void ebcm_ssw_mcu_fw_check(void);


void ebcm_clear_all_smu_alarms(void);


#endif /* INC_SSW_MCU_FW_CHK_H_ */
