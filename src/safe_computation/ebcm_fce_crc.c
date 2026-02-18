/******************************************************************************
 * @file    ebcm_fce_crc.c
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
#include "ebcm_fce_crc.h"
#include "ebcm_cfg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
boolean fceInitComplete = FALSE;


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(FCE_ERR_ISR, CPU0_VECT_TABLE_ID, ISR_PRIORITY_FCE_ER);

void FCE_ERR_ISR(void)
{
    /* report an error here, fault handling system not fully implemented yet */
}

void FCE_init(void)
{
    __disable();
    uint16         password = IfxScuWdt_getCpuWatchdogPassword();
    IfxScuWdt_clearCpuEndinit(password);

    /* Enable CRC engine */
    FCE_CLC.U = 0x0U;

    /* Enable kernel 1, XOR final value with 0xFFFFFFFF, with reflection */
    /* AUTOSAR CRC32P4 - 0xF4ACFB13 */
    FCE_CFG0.B.KERNEL = 0x1U;
    FCE_CFG0.B.XSEL   = 0x1U;
    FCE_CFG0.B.REFOUT = 0x1U;

    /* no check */
    FCE_CFG0.B.CCE = 0x00U;

    /* Set up CRC mismatch interrupt */
    FCE_CFG0.B.CMI    = 0x1U;

    /* Set a seed */
    FCE_CRC0.B.CRC = 0xFFFFFFFFU;

    /* Set up interrupts */
    SRC_FCE0.B.TOS  = 0x00U; /* CPU 0 services */
    SRC_FCE0.B.SRPN = ISR_PRIORITY_FCE_ER;
    SRC_FCE0.B.SRE  = 0x01U;

    IfxScuWdt_setCpuEndinit(password);
    __enable();

    fceInitComplete = TRUE;

}

uint32 FCE_calc_CRC32(const uint8* data, uint16 len, uint32 seed)
{
    if (!fceInitComplete)
    {
        return 0xFFFFFFFF;
    }
    uint32 result  = 0x0;

    FCE_LENGTH0.U = 0xFACECAFE;
    FCE_LENGTH0.U = len;
    FCE_CHECK0.U  = 0xFACECAFE;
    FCE_CHECK0.U  = 0x0;


    FCE_CRC0.U = seed;

    volatile Ifx_FCE_IN_IR* inputData = &FCE_IR0;

    for (uint16 i = 0; i < len; i++)
    {
        inputData->U = *(data++);
    }

    /* FCE engine needs 2 cycles after writing into the IR to read a stable result */
    result = FCE_RES0.U;
    result = FCE_RES0.U;

    return result;

}
