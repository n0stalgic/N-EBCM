/******************************************************************************
 * @file    smu_stdby_FSP.c
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
#include <safe_computation/smu_stdby_FSP.h>
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

/*
 * clear the status bit of stdby_alarm and check if the clear has really been executed
 */
SmuStatusType clearBitSMUstdby(AlarmStdbySMU stdbyAlarm)
{
    Ifx_PMS_AG_STDBY0 stdbyAG20;
    Ifx_PMS_AG_STDBY1 stdbyAG21;
    Ifx_PMS_CMD_STDBY stdbyCMD;

    boolean bool = FALSE;

    /* Clear either an alarm from group 20 or 21 */
    switch (stdbyAlarm / 32)
    {
        case 0 :
            IfxScuWdt_clearSafetyEndinitInline(IfxScuWdt_getSafetyWatchdogPasswordInline());

            /* Lift write protection for one write access
             Alarm status clear enabled */
            stdbyCMD.B.BITPROT = 1;
            stdbyCMD.B.ASCE = 1;

            /* Applying the configuration to prepare for the bit clear */
            PMS_CMD_STDBY.U = stdbyCMD.U;

            /* BITPROT read as zero, thus the zero here */
            stdbyCMD.B.BITPROT = 0;
            bool = (PMS_CMD_STDBY.U == stdbyCMD.U);

            /* Bit clear */
            stdbyAG20.U = PMS_AG20_STDBY.U;
            stdbyAG20.U |= 1 << (stdbyAlarm % 32);
            PMS_AG20_STDBY.U = stdbyAG20.U;

            /* If the clear has been successful the bit stdby_alarm%32 will be 0 */
            stdbyAG20.U &= ~(1 << (stdbyAlarm % 32));
            bool &= (PMS_AG20_STDBY.U == stdbyAG20.U);

            IfxScuWdt_setSafetyEndinitInline(IfxScuWdt_getSafetyWatchdogPasswordInline());
            break;

        case 1 :
            IfxScuWdt_clearSafetyEndinitInline(IfxScuWdt_getSafetyWatchdogPasswordInline());

            /* Llift write protection for one write access
             Alarm status clear enabled */
            stdbyCMD.B.BITPROT = 1;
            stdbyCMD.B.ASCE = 1;

            /* Applying the configuration to prepare for the bit clear */
            PMS_CMD_STDBY.U = stdbyCMD.U;

            /* BITPROT read as zero, thus the zero here */
            stdbyCMD.B.BITPROT = 0;
            bool = (PMS_CMD_STDBY.U == stdbyCMD.U);

            /* Bit clear */
            stdbyAG21.U = PMS_AG21_STDBY.U;
            stdbyAG21.U |= 1 << (stdbyAlarm % 32);
            PMS_AG21_STDBY.U = stdbyAG21.U;

            /* If the clear has been successful the bit stdby_alarm%32 will be 0 */
            stdbyAG21.U &= ~(1 << (stdbyAlarm % 32));
            bool &= (PMS_AG21_STDBY.U == stdbyAG21.U);
            IfxScuWdt_setSafetyEndinitInline(IfxScuWdt_getSafetyWatchdogPasswordInline());
            break;

        default:
            break;
    }

    if(bool)
    {
        return PASS;
    }
    else
    {
        return FAIL;
    }
}
