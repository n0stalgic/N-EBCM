/******************************************************************************
 * @file    smu_stdby_FSP.h
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

#ifndef SAFETY_SMU_STDBY_FSP_H_
#define SAFETY_SMU_STDBY_FSP_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "smu.h"
#include "IfxPms_reg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/* stdby alarm */
typedef enum
{
    smuStdbyAlarmEVRVDDOverVoltage = 4,   /* ALM20[4] : Safety Mechanism: Voltage Monitor Alarm: VDD
                                                   Over-voltage Alarm */
    smuStdbyAlarmEVRVDDPDOverVoltage = 5, /* ALM20[5] : Safety Mechanism: Voltage Monitor Alarm: VDDPD
                                                   Over-voltage Alarm */
    smuStdbyAlarmEVRVDDP3OverVoltage = 6, /* ALM20[6] : Safety Mechanism: Voltage Monitor Alarm: VDDP3
                                                   Over-voltage Alarm */
    smuStdbyAlarmEVRVDDPMOverVoltage = 7, /* ALM20[7] : Safety Mechanism: Voltage Monitor Alarm: VDDPM
                                                   Over-voltage Alarm */
    smuStdbyAlarmEVRVEXTOverVoltage = 8,  /* ALM20[8] : Safety Mechanism: Voltage Monitor Alarm: VEXT
                                                   Over-voltage Alarm */
    smuStdbyAlarmEVRVEVRSBOverVoltage = 9, /* ALM20[9] : Safety Mechanism: Voltage Monitor Alarm: VEVRSB
                                                    Over-voltage Alarm */
    smuStdbyAlarmEVRVDDUnderVoltage = 10,  /* ALM20[10] : Safety Mechanism: Voltage Monitor Alarm: VDD
                                                    Under-voltage Alarm */
    smuStdbyAlarmEVRVDDPDUnderVoltage = 11, /* ALM20[11] : Safety Mechanism: Voltage Monitor Alarm: VDDPD
                                                     Under-voltage Alarm */
    smuStdbyAlarmEVRVDDP3UnderVoltage = 12, /* ALM20[12] : Safety Mechanism: Voltage Monitor Alarm: VDDP3
                                                     Under-voltage Alarm */
    smuStdbyAlarmEVRVDDPMUnderVoltage = 13, /* ALM20[13] : Safety Mechanism: Voltage Monitor Alarm: VDDPM
                                                     Under-voltage Alarm */
    smuStdbyAlarmEVRVEXTUnderVoltage = 14,  /* ALM20[14] : Safety Mechanism: Voltage Monitor Alarm: VEXT
                                                     Under-voltage Alarm */
    smuStdbyAlarmEVRVEVRSBUnderVoltage = 15, /* ALM20[15] : Safety Mechanism: Voltage Monitor Alarm: VEVRSB
                                                      Under-voltage Alarm */

    smuStdbyAlarmHSMVDDCUnderVoltage = 32,   /* ALM21[0] : Safety Mechanism: Voltage Monitor Alarm: VDDC
                                                      Under-voltage Alarm */
    smuStdbyAlarmHSMVDDP3UnderVoltage = 33,  /* ALM21[1] : Safety Mechanism: Voltage Monitor Alarm: VDDP3
                                                      Under-voltage Alarm */
    smuStdbyAlarmHSMVEXTUnderVoltage = 34,   /* ALM21[2] : Safety Mechanism: Voltage Monitor Alarm: VEXT
                                                      Under-voltage Alarm */
    smuStdbyAlarmHSMVDDCOverVoltage = 35,    /* ALM21[3] : Safety Mechanism: Voltage Monitor Alarm: VDDC
                                                      Over-voltage Alarm */
    smuStdbyAlarmHSMVDDP3OverVoltage = 36,   /* ALM21[4] : Safety Mechanism: Voltage Monitor Alarm: VDDP3
                                                      Over-voltage Alarm */
    smuStdbyAlarmHSMVEXTOverVoltage = 37,    /* ALM21[5] : Safety Mechanism: Voltage Monitor Alarm: VEXT
                                                      Over-voltage Alarm */
    smuStdbyAlarmSafetyFlipFlopUncorrectableError = 39, /* ALM21[7] : Safety Mechanism: Safety Flip-flop Alarm:
                                                                  Safety flip-flop uncorrectable error detected */
    smuStdbyAlarmDTSTemperatureUnderflow = 40, /* ALM21[8] : Safety Mechanism: Die Temperature Sensor Alarm:
                                                       Temperature underflow */
    smuStdbyAlarmDTSTemperatureOverflow = 41,  /* ALM21[9] : Safety Mechanism: Die Temperature Sensor Alarm:
                                                       Temperature overflow */
    smuStdbyAlarmHSMAccessProtectionViolation = 42, /* ALM21[10] : Safety Mechanism: Register Access Protection
                                                             Alarm: Access Protection violation */
    smuStdbyAlarmEVREVRCShortToHigh = 43,   /* ALM21[11] : Safety Mechanism: Voltage Monitor Alarm: EVRC Short
                                                      to Low  Alarm */
    smuStdbyAlarmEVREVRCShortToLow = 44,    /* ALM21[12] : Safety Mechanism: Voltage Monitor Alarm: EVRC Short
                                                      to High Alarm */
    smuStdbyAlarmEVREV33ShortToLow = 45,    /* ALM21[13] : Safety Mechanism: Voltage Monitor Alarm: EV33 Short
                                                      to Low Alarm */
    smuStdbyAlarmEVREV33ShortToHigh = 46,   /* ALM21[14] : Safety Mechanism: Voltage Monitor Alarm: EV33 Short
                                                      to High Alarm */
    smuStdbyAlarmCCUPLLxfSPBAlive = 47,      /* ALM21[15] : Safety Mechanism: Voltage Monitor Safety Mechanism:
                                                      Clock Alive Monitor */
    /*  Alarm: PLLx/fSPB Alive Alarm (provided on fBACK clock with x = 0..2) */
    /*  Note: This alarm is also set if TCU related signals are activated in the */
    /*  CCU causing the clocks to fail */
    smuStdbyAlarmSMUCoreALive = 48 /* ALM21[16] : Safety Mechanism: SMU Alive Monitor Alarm: SMU Alive Alarm */

} AlarmStdbySMU;

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
SmuStatusType clearBitSMUstdby(AlarmStdbySMU stdbyAlarm);


#endif /* SAFETY_SMU_STDBY_FSP_H_ */
