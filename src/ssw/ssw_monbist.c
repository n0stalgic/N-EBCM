/******************************************************************************
 * @file    ssw_monbist.c
 * @brief   MONBIST implementation
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


/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "ebcm_cfg.h"
#include "ebcm_main.h"
#include "ssw_monbist.h"
#include "IfxSmuStdby.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

#if (IFX_CFG_SSW_ENABLE_MONBIST == 1U || EBCM_CFG_SSW_ENABLE_MONBIST == 1U)

void EbcmSsw_monbist(void)
{
     uint32 timeout;

     /* Disable the write-protection for registers */
    IFX_CFG_SSW_CLEAR_SAFETY_ENDINIT();
    /* Enable SMU Standby */
    PMS_CMD_STDBY.U    = 0x40000001U;
    /* clears the TSTEN, TSTRUN, TSTDONE, TSTOK, SMUERR and PMSERR flags */
    PMS_MONBISTCTRL.U  = 0x40000002U;
    /* Clear the filter */
    PMS_MONFILT.U = 0x20000000U;
    /* Wait until register is updated */
    timeout = 0x1000U;
    while ((PMS_MONFILT.U != 0x20000000U) && (timeout--))
    {};
    /* Activate under voltage and over voltage alarms */
    PMS_MONCTRL.U = 0xa5a5a5U;
    /* Wait until register is updated */
    timeout = 0x1000U;
    while ((PMS_MONCTRL.U != 0xa5a5a5U) && (timeout--))
    {};
    /* corresponding Over-voltage and Under-voltage interrupts are disabled */
    PMS_PMSIEN.U &= ~0x00000FFFU;
    /* Fault Signal reaction on alarms are disabled */
    PMS_AGFSP_STDBY0.U = 0x40000000U;
    PMS_AGFSP_STDBY1.U = 0x40000000U;
    /* FSP0EN and FSP1EN configuration bits are cleared to avoid spurious Error pin activation */
    /* ASCE bit is set and respective alarms are cleared */
    PMS_CMD_STDBY.U |= 0x40000008U;
    PMS_AG_STDBY0.U = 0xFFF0U;
    PMS_CMD_STDBY.U |= 0x40000008U;
    PMS_AG_STDBY1.U = 0x1FFFFU;
    /* Reset the MONFILT register */
    PMS_MONFILT.U = 0x00000000U;
    /* Start MONBIST test */
    PMS_MONBISTCTRL.U = 0x40000001U;

    /* Wait until MONBIST execution is done */
    timeout = 0x1000U;
    while (((PMS_MONBISTSTAT.B.TSTRUN == 1) || (PMS_MONBISTSTAT.B.TSTDONE == 0)) && (timeout--))
    {};

    /* Disable SMU Standby */
    PMS_CMD_STDBY.U    = 0x40000000U;
    IFX_CFG_SSW_SET_SAFETY_ENDINIT();
#if (IFX_CFG_SSW_ENABLE_MONBIST == 1U || EBCM_CFG_SSW_ENABLE_MONBIST == 1U)
    Ifx_Ssw_jumpToFunction(&EbcmSsw_monbistCheck);
#else
    EbcmSsw_monbistCheck();

    /* Disable the write-protection for registers */
    IFX_CFG_SSW_CLEAR_SAFETY_ENDINIT();

    /* Clear SMU Standby alarms */
    PMS_CMD_STDBY.U |= 0x40000008U;
    PMS_AG_STDBY0.U = 0xFFF0U;
    PMS_CMD_STDBY.U |= 0x40000008U;
    PMS_AG_STDBY1.U = 0x1FFFFU;

    IFX_CFG_SSW_SET_SAFETY_ENDINIT();
#endif /* IFX_CFG_SSW_ENABLE_MONBIST == 1U && SLK_CFG_SSW_ENABLE_MONBIST == 0U */

    monbistStatus.testOkFlag = IfxSmuStdby_getSmuStdbyMonBistTestOkFlag();
    monbistStatus.smuErrorFlag = !IfxSmuStdby_getSmuStdbyMonBistSmuErrorFlag();
    monbistStatus.pmsErrorFlag = !IfxSmuStdby_getSmuStdbyMonBistPmsErrorFlag();
    if ((FALSE == monbistStatus.testOkFlag) ||
        (FALSE == monbistStatus.smuErrorFlag) ||
        (FALSE == monbistStatus.pmsErrorFlag))
    {
       ebcmStatus.sswStatus.monbistStatus = FAILED;
    }
    else
    {
        ebcmStatus.sswStatus.monbistStatus = PASSED;
    }
}

void EbcmSsw_monbistCheck(void)
{
    /* Check for MONBIST error state */
    if ((PMS_MONBISTSTAT.B.TSTOK == 0U) || (PMS_MONBISTSTAT.B.SMUERR == 1U) || (PMS_MONBISTSTAT.B.PMSERR == 1U))
    {
        __debug();
    }

#if IFX_CFG_SSW_ENABLE_MONBIST == 1U && SLK_CFG_SSW_ENABLE_MONBIST == 0U
    Ifx_Ssw_jumpBackToLink();
#endif /* IFX_CFG_SSW_ENABLE_MONBIST == 1U && SLK_CFG_SSW_ENABLE_MONBIST == 0U */
}
#endif

