/******************************************************************************
 * @file    ssw_lbist.c
 * @brief   Implementation for LBIST
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
#include "ssw.h"
#include "ebcm_main.h"
#include "ssw_lbist.h"
#include "IfxScuLbist.h"
#include "Ifx_Types.h"
#include "Ifx_Ssw_Infra.h"
#include "IfxScuLbist.h"
#include "IfxPort.h"
#include "IfxScuRcu.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define LBIST_MAX_TRIES       3


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
LbistStatus lbistStatus;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/


boolean EbcmSsw_scuLbistIsTermProperly(void);
boolean EbcmSsw_scuLbistIsTermPorst(void);

/**
 * @brief execute an LBIST and handle the result during SSW sequence
 */
void EbcmSsw_execLbist(void);

/**
 * @brief Evaluate LBIST result
 */
void EbcmSsw_evalLbist(void);

/**
 * @brief Check LBIST status
 */
void EbcmSsw_checkLbist(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/**
 * The RSTSTAT.B.LBTERM status bit indicates if LBIST execution was properly terminated
 *
 */
boolean EbcmSsw_scuLbistIsTermProperly(void)
{
    return (boolean)MODULE_SCU.RSTSTAT.B.LBTERM;
}

/**
 * The RSTSTAT.LBPORST status bit indicates if LBIST execution was terminated earlier due to a PORST assertion
 */
boolean EbcmSsw_scuLbistIsTermPorst(void)
{
    return (boolean)MODULE_SCU.RSTSTAT.B.LBPORST;
}
/**
 * @brief execute an LBIST and handle the result during SSW sequence
 */

void EbcmSsw_execLbist(void)
{

    /* Increment the counter that talies the LBIST requests via application SW.
     * Located in SCR XRAM memory to avoid modification by the LBIST.
     */
    sswRunCount->lbistAppReqCount++;

    /* Clear cold reset status */
    sswRunCount->RSTSTAT.U = MODULE_SCU.RSTSTAT.U;

    /* Reset the LBIST controller */
    IfxScuWdt_clearGlobalSafetyEndinitInline(IfxScuWdt_getGlobalSafetyEndinitPasswordInline());
    MODULE_SCU.LBISTCTRL0.B.LBISTRES = 1;
    IfxScuWdt_setGlobalSafetyEndinitInline(IfxScuWdt_getGlobalSafetyEndinitPasswordInline());

    /* Clear COLD PORST reason to preserve the data on the SCR XRAM after LBIST trigger i.e if not cleared,
     * then SCR XRAM will be initialized */
    IfxScuRcu_clearColdResetStatus();

    /* Trigger the BIST and wait for a warm reset */
    IfxScuLbist_triggerInline(&IfxScuLbist_defaultConfig);

    while(1)
    {
        __nop(); /* After triggering LBIST wait for warm reset */
    }

}

/*
 * Check the LBIST status
 * */
void EbcmSsw_checkLbist(void)
{
    /* check if LBIST was terminated by a PORST i.e. if not terminated then move forward with checks*/
    if (FALSE == EbcmSsw_scuLbistIsTermPorst())
    {
        lbistStatus.lbistNotTermByPorst = PASSED;

        /* check if LBIST was terminated Properly */
        if (TRUE == EbcmSsw_scuLbistIsTermProperly())
        {
            lbistStatus.lbistTermOk = PASSED;

            /* Check if LBIST was already executed */
            if (TRUE == IfxScuLbist_isDone())
            {
                lbistStatus.lbistTestDone = PASSED;
            }
            else
            {
                lbistStatus.lbistTestDone = FAILED;
            }
        }

        /* If LBIST was not terminated properly then check if cold power reset and trigger LBIST */
        else
        {
            lbistStatus.lbistTermOk = FAILED;
        }
    }

    /* If LBIST was early terminated because of PORST then check if cold-POR and trigger LBIST */
    else
    {
        lbistStatus.lbistNotTermByPorst = FAILED;
    }
}

/**
 * @brief Evaluate LBIST result
 */
void EbcmSsw_evalLbist(void)
{

    boolean lbistPassed;

    /* if LBIST correctly executed, then we can evaluate results */
    if (lbistStatus.lbistNotTermByPorst == PASSED &&
        lbistStatus.lbistTermOk == PASSED &&
        lbistStatus.lbistTestDone == PASSED)
    {

        lbistPassed = IfxScuLbist_evaluateResult(IfxScuLbist_defaultConfig.signature);

        /* unlock LBIST module registers as these are safety critical registers */
        IfxScuWdt_clearGlobalSafetyEndinitInline(IfxScuWdt_getGlobalSafetyEndinitPasswordInline());

        /* Reset LBIST moodule and wait for it to emerge from reset */
        MODULE_SCU.LBISTCTRL0.B.LBISTRES = 1;

        IfxScuWdt_setGlobalSafetyEndinitInline(IfxScuWdt_getGlobalSafetyEndinitPasswordInline());

        uint8 timeout = 0xFF;
        while ((MODULE_SCU.LBISTCTRL0.B.LBISTDONE == 1U) && (timeout > 0))
        {
            timeout--;
        }

        if (lbistPassed == TRUE)
        {
            ebcmStatus.sswStatus.lbistStatus = PASSED;
            lbistStatus.devNotDefect = PASSED;
        }
        else
        {
            if (sswRunCount->lbistRuns < LBIST_MAX_TRIES)
            {
                /* Trigger an LBIST */
                EbcmSsw_execLbist();
            }
            else
            {
                lbistStatus.devNotDefect = FAILED;

                // TODO: indicate brake module failure, system needs to be serviced
            }
        }
    }
    else
    {
        ebcmStatus.sswStatus.lbistStatus = FAILED;

        if (sswRunCount->lbistRuns < LBIST_MAX_TRIES)
        {
            EbcmSsw_execLbist();
        }
        else
        {
            /* Trigger LBIST */
            lbistStatus.devNotDefect = FAILED;

            // TODO: indicate brake module failure, system needs to be serviced
        }
    }

}

/**
 * @brief Execute LBIST and evaluate the results
 */
void EbcmSsw_lbist(void)
{
    lbistStatus.lbistNotTermByPorst = TEST_NOT_EVAL;
    lbistStatus.lbistTermOk = TEST_NOT_EVAL;
    lbistStatus.lbistTestDone = TEST_NOT_EVAL;
    lbistStatus.devNotDefect = TEST_NOT_EVAL;
    ebcmStatus.sswStatus.lbistStatus = TEST_NOT_EVAL;

    if (Ifx_Ssw_isColdPoweronReset() ||
          EbcmSsw_scuLbistIsTermProperly()  )
    {
        /* check LBIST status */
        EbcmSsw_checkLbist();

        /* Evaluate results */
        EbcmSsw_evalLbist();
    }
}
