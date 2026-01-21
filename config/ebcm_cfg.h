/******************************************************************************
 * @file    {file_name}
 * @brief   Add brief here
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

#ifndef CONFIG_EBCM_CFG_H_
#define CONFIG_EBCM_CFG_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define EBCM_CFG_SSW_ENABLE_LBIST_BOOT            1
#define EBCM_CFG_SSW_ENABLE_LBIST_APPSW           1
#define EBCM_CFG_SSW_ENABLE_MONBIST               1
#define EBCM_CFG_SSW_ENABLE_MCU_FW_CHECK          1
#define EBCM_CFG_SSW_ENABLE_MCU_STARTUP           1
#define EBCM_CFG_SSW_ENABLE_ALIVE_ALARM_TEST      1
#define EBCM_CFG_SSW_ENABLE_REG_MONITOR_TEST      1
#define EBCM_CFG_SSW_ENABLE_MBIST                 1

/* Number of STM ticks per millisecond */
#define IFX_CFG_STM_TICKS_PER_MS                  100000
#define IFX_CFG_STM_TICKS_PER_US                  100

#define LED1_EBCM_ALIVE                           &MODULE_P00,5
#define LED2_ALRM_DETECTED                        &MODULE_P00,6

/* [°C] difference in the redundant die temperature as specified in the safety manual */
#define MAX_DIE_TEMP_DIFF                        9.0


#define ISR_PRORITY_SMU_ISR_0                    5
#define ISR_PRORITY_SMU_ISR_1                    6
#define ISR_PRORITY_SMU_ISR_2                    7
#define ISR_PRIORITY_GPT12_TIMER                 8
#define ISR_PRIORITY_OS_TICK                     9       /* Define the tick for the Application */
#define ISR_PRIORITY_FCE_ER                      13      /* Flexible CRC Engine */



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


#endif /* CONFIG_EBCM_CFG_H_ */
