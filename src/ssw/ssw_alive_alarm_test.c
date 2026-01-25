/******************************************************************************
 * @file    ssw_alive_alarm_test.c
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

#include "ssw_alive_alarm_test.h"
#include "IfxSmu.h"
#include "IfxSmuStdby.h"
#include "ebcm_main.h"

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

void EbcmSsw_aliveAlarmTest(void)
{
    ebcmStatus.smuStatus.smuCoreAliveTestSts        = NA;
    ebcmStatus.smuStatus.smuCoreAliveTestClearSts   = NA;
    ebcmStatus.sswStatus.aliveAlarmStatus = TEST_NOT_EVAL;

    /* Only cold power on reset will reset the SMU.
     * For this part of test SMU needs to be in a standby mode.
     */
    if (ebcmStatus.resetCode.resetType == IfxScuRcu_ResetType_coldpoweron)
    {
        /* Start SMU Alive Test */
        IfxSmu_startAliveTest();

        /* Poll for command status (success) */
        uint8 timeout = 0xFF;
        while (SMU_STS.B.RES != 0U && timeout > 0)
        {
            timeout--;
        }

        /* Wait for ALM21[16] - SMU Alive Monitor Alarm */
        uint8 timeoutAlarmStatus = 0xFF;
        while(IfxSmuStdby_getSmuStdbyAlarmStatus(21, 16) != IfxSmuStdby_AlarmStatusFlag_faultExist && timeout > 0)
        {
            timeoutAlarmStatus--;
            __nop();
        }

        /* Set smuCoreAliveTestSts SMU status variable */
        ebcmStatus.smuStatus.smuCoreAliveTestSts = PASS;

        /* Stop alive test */
        IfxSmu_stopAliveTest();

        /* Clear the ALM21[16] which is triggered by the Core alive test */
        IfxSmuStdby_setSmuStdbyAlarmStatusFlag(21, 16, IfxSmuStdby_AlarmStatusFlag_faultExist);

        /* Poll for command status (success) */
        timeout = 0xFF;
        while (SMU_STS.B.RES != 0U && timeout > 0)
        {
            timeout--;
        }

        /* Check if alarm is cleared */
        if(IfxSmuStdby_getSmuStdbyAlarmStatus(21, 16) == IfxSmuStdby_AlarmStatusFlag_noFaultExist)
        {
            ebcmStatus.sswStatus.aliveAlarmStatus = PASS;

            /* Set smuCoreAliveTestClearSts SMU status variable */
            ebcmStatus.smuStatus.smuCoreAliveTestClearSts = PASS;
        }
        else
        {
            ebcmStatus.sswStatus.aliveAlarmStatus = FAILED;

            /* Set smuCoreAliveTestClearSts SMU status variable */
            ebcmStatus.smuStatus.smuCoreAliveTestClearSts = FAIL;
        }
    }
    else
    {

    }
}

