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
LBIST_status_t LBIST_status;
ebcm_status_t ebcm_status;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/


boolean ebcm_scu_LBIST_is_term_properly(void);
boolean ebcm_scu_LBIST_is_term_PORST(void);

/**
 * @brief execute an LBIST and handle the result during SSW sequence
 */
void ssw_exec_LBIST(void);

/**
 * @brief Evaluate LBIST result
 */
void ssw_eval_LBIST(void);

/**
 * @brief Check LBIST status
 */
void ssw_check_LBIST(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

/**
 * The RSTSTAT.B.LBTERM status bit indicates if LBIST execution was properly terminated
 *
 */
boolean ebcm_scu_LBIST_is_term_properly(void)
{
    return (boolean)MODULE_SCU.RSTSTAT.B.LBTERM;
}

/**
 * The RSTSTAT.LBPORST status bit indicates if LBIST execution was terminated earlier due to a PORST assertion
 */
boolean ebcm_scu_LBIST_is_term_PORST(void)
{
    return (boolean)MODULE_SCU.RSTSTAT.B.LBPORST;
}
/**
 * @brief execute an LBIST and handle the result during SSW sequence
 */

void ssw_exec_LBIST(void)
{

    /* Increment the counter that talies the LBIST requests via application SW.
     * Located in SCR XRAM memory to avoid modification by the LBIST.
     */
    ssw_run_count->lbist_app_req_count++;

    /* Clear cold reset status */
    ssw_run_count->RSTSTAT.U = MODULE_SCU.RSTSTAT.U;

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
void ssw_check_LBIST(void)
{
    /* check if LBIST was terminated by a PORST i.e. if not terminated then move forward with checks*/
    if (FALSE == ebcm_scu_LBIST_is_term_PORST())
    {
        LBIST_status.LBIST_not_term_by_porst = PASSED;

        /* check if LBIST was terminated Properly */
        if (TRUE == ebcm_scu_LBIST_is_term_properly())
        {
            LBIST_status.LBIST_term_ok = PASSED;

            /* Check if LBIST was already executed */
            if (TRUE == IfxScuLbist_isDone())
            {
                LBIST_status.LBIST_test_done = PASSED;
            }
            else
            {
                LBIST_status.LBIST_test_done = FAILED;
            }
        }

        /* If LBIST was not terminated properly then check if cold power reset and trigger LBIST */
        else
        {
            LBIST_status.LBIST_term_ok = FAILED;
        }
    }

    /* If LBIST was early terminated because of PORST then check if cold-POR and trigger LBIST */
    else
    {
        LBIST_status.LBIST_not_term_by_porst = FAILED;
    }
}

/**
 * @brief Evaluate LBIST result
 */
void ssw_eval_LBIST(void)
{

    boolean LBIST_passed;

    /* if LBIST correctly executed, then we can evaluate results */
    if (LBIST_status.LBIST_not_term_by_porst == PASSED &&
            LBIST_status.LBIST_term_ok == PASSED &&
                LBIST_status.LBIST_test_done == PASSED)
    {

        LBIST_passed = IfxScuLbist_evaluateResult(IfxScuLbist_defaultConfig.signature);

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

        if (LBIST_passed == TRUE)
        {
            ebcm_status.ssw_status.lbist_status = PASSED;
            LBIST_status.dev_not_defect = PASSED;
        }
        else
        {
            if (ssw_run_count->lbist_runs < LBIST_MAX_TRIES)
            {
                /* Trigger an LBIST */
                ssw_exec_LBIST();
            }
            else
            {
                LBIST_status.dev_not_defect = FAILED;

                // TODO: indicate brake module failure, system needs to be serviced
            }
        }
    }
    else
    {
        ebcm_status.ssw_status.lbist_status = FAILED;

        if (ssw_run_count->lbist_runs < LBIST_MAX_TRIES)
        {
            ssw_exec_LBIST();
        }
        else
        {
            /* Trigger LBIST */
            LBIST_status.dev_not_defect = FAILED;

            // TODO: indicate brake module failure, system needs to be serviced
        }
    }

}

/**
 * @brief Execute LBIST and evaluate the results
 */
void ebcm_ssw_lbist(void)
{
    LBIST_status.LBIST_not_term_by_porst = TEST_NOT_EVAL;
    LBIST_status.LBIST_term_ok = TEST_NOT_EVAL;
    LBIST_status.LBIST_test_done = TEST_NOT_EVAL;
    LBIST_status.dev_not_defect = TEST_NOT_EVAL;
    ebcm_status.ssw_status.lbist_status = TEST_NOT_EVAL;

    if (Ifx_Ssw_isColdPoweronReset() ||
          ebcm_scu_LBIST_is_term_properly()  )
    {
        /* check LBIST status */
        ssw_check_LBIST();

        /* Evaluate results */
        ssw_eval_LBIST();
    }
}
