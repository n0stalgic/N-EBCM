/******************************************************************************
 * @file    ebcm_pga460cmd.h
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

#ifndef HW_ISR_EBCM_PGA460MSG_H_
#define HW_ISR_EBCM_PGA460MSG_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define PGA460_DEFAULT_ADDR         (0U)
#define PGA460_MAX_CMD_DATA_BYTES   64U

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

#define

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
typedef enum _PGA460_CMD_TYPE
{
    PGA460_COMMAND_BURST_AND_LISTEN_PRESET_1 = 0,
    PGA460_COMMAND_BURST_AND_LISTEN_PRESET_2,
    PGA460_COMMAND_LISTEN_ONLY_PRESET_1,
    PGA460_COMMAND_LISTEN_ONLY_PRESET_2,
    PGA460_COMMAND_TEMPERATURE_NOISE_MEASUREMENT,
    PGA460_COMMAND_ULTRASONIC_MEASUREMENT_RESULT,
    PGA460_COMMAND_TEMPERATURE_NOISE_LEVEL_RESULT,
    PGA460_COMMAND_TRANSDUCER_ECHO_DATA_DUMP,
    PGA460_COMMAND_SYSTEM_DIAGNOSTICS,
    PGA460_COMMAND_REGISTER_READ,
    PGA460_COMMAND_REGISTER_WRITE,
    PGA460_COMMAND_EEPROM_BULK_READ,
    PGA460_COMMAND_EEPROM_BULK_WRITE,
    PGA460_COMMAND_TVG_BULK_READ,
    PGA460_COMMAND_TVG_BULK_WRITE,
    PGA460_COMMAND_THRESHOLD_BULK_READ,
    PGA460_COMMAND_THRESHOLD_BULK_WRITE,
    PGA460_COMMAND_BURST_LISTEN_PRESET_1_BCAST,
    PGA460_COMMAND_BURST_LISTEN_PRESET_2_BCAST,
    PGA460_COMMAND_LISTEN_ONLY_PRESET_1_BCAST,
    PGA460_COMMAND_LISTEN_ONLY_PRESET_2_BCAST,
    PGA460_COMMAND_TEMPERATURE_NOISE_LEVEL_MEASUREMENT_BCAST,
    PGA460_COMMAND_REGISTER_WRITE_BCAST,
    PGA460_COMMAND_EEPROM_BULK_WRITE_BCAST,
    PGA460_COMMAND_TVG_BULK_WRITE,
    PGA460_COMMAND_THRESHOLD_BULK_WRITE,
    RESERVED0,
    RESERVED1,
    RESERVED2,
    RESERVED3,
    RESERVED4,
    RESERVED5,
    PGA460_COMMAND_MAX_COMMANDS

} PGA460_CMD_TYPE;
/*********************************************************************************************************************/
/*-------------------------------------------------PGA460 Commands---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct _PGA460_CMD_Bits
{
        PGA460_CMD_TYPE CMD_ID:5;
        uint8 ADDR:3;
} PGA460_CMD_Bits;

__attribute__((packed)) typedef struct _PGA460_BL_P1_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA;
        uint8 CHKSUM;
} PGA460_BL_P1_CMD_Frame;

__attribute__((packed)) typedef struct _PGA460_BL_P2_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA;
        uint8 CHKSUM;
} PGA460_BL_P2_CMD_Frame;

__attribute__((packed)) typedef struct _PGA460_Listen_P1_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA;
        uint8 CHKSUM;
} PGA460_Listen_P1_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_Listen_P2_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA;
        uint8 CHKSUM;
} PGA460_Listen_P2_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_TempNoise_Measurement_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA;
        uint8 CHKSUM;
} PGA460_TempNoise_Measurement_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_Ultrasonic_Measurement_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_Ultrasonic_Measurement_CMD_Frame;

__attribute__((packed)) typedef struct _PGA460_TempNoise_Result_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_TempNoise_Result_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_Transducer_Echo_Data_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_Transducer_EchoData_CMD_Frame;

__attribute__((packed)) typedef struct _PGA460_System_Diagnostics_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_System_Diagnostics_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_Register_Read_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA;
        uint8 CHKSUM;
} PGA460_Register_Read_CMD_Frame;

__attribute__((packed)) typedef struct _PGA460_Register_Write_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA[2];
        uint8 CHKSUM;
} PGA460_Register_Write_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_EEPROM_Bulk_Read_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_EEPROM_Bulk_Read_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_EEPROM_Bulk_Write_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA[43];
        uint8 CHKSUM;
} PGA460_EEPROM_Bulk_Write_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_TVG_Bulk_Read_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_TVG_Bulk_Read_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_TVG_Bulk_Write_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA[7];
        uint8 CHKSUM;
} PGA460_TVG_Bulk_Write_CMD_Frame;

__attribute__((packed)) typedef struct _PGA460_Threshold_Bulk_Read_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 CHKSUM;
} PGA460_Threshold_Bulk_Read_CMD_Frame;


__attribute__((packed)) typedef struct _PGA460_Threshold_Bulk_Write_CMD_Frame
{
        uint8 SYNC;
        PGA460_CMD_Bits CMD;
        uint8 DATA[32];
        uint8 CHKSUM;
} PGA460_Threshold_Bulk_Write_CMD_Frame;



typedef union
{
        uint8 MSG_BYTES[PGA460_MAX_CMD_DATA_BYTES];
        PGA460_BL_P1_CMD_Frame BL_P1_CMD_MSG;
        PGA460_BL_P2_CMD_Frame BL_P2_CMD_MSG;
        PGA460_Listen_P1_CMD_Frame L_P1_CMD_MSG;
        PGA460_Listen_P2_CMD_Frame L_P2_CMD_MSG;
        PGA460_TempNoise_Measurement_CMD_Frame  TempNoise_Measurement_CMD_MSG;
        PGA460_Ultrasonic_Measurement_CMD_Frame Ultrasonic_Measurement_CMD_MSG;
        PGA460_TempNoise_Result_CMD_Frame       TempNoise_Result_CMD_MSG;
        PGA460_Transducer_EchoData_CMD_Frame    Transducer_EchoData_CMD_MSG;
        PGA460_System_Diagnostics_CMD_Frame     SystemDiagnostics_CMD_MSG;
        PGA460_Register_Read_CMD_Frame          RegisterRead_CMD_MSG;
        PGA460_Register_Write_CMD_Frame         RegisterWrite_CMD_MSG;
        PGA460_EEPROM_Bulk_Read_CMD_Frame       EEPROM_BulkRead_CMD_MSG;
        PGA460_EEPROM_Bulk_Write_CMD_Frame      EEPROM_BulkWrite_CMD_MSG;
        PGA460_TVG_Bulk_Read_CMD_Frame          TVG_BulkRead_CMD_MSG;
        PGA460_TVG_Bulk_Write_CMD_Frame         TVG_BulkWrite_CMD_MSG;
        PGA460_Threshold_Bulk_Read_CMD_Frame    Threshold_BulkRead_CMD_MSG;
        PGA460_Threshold_Bulk_Write_CMD_Frame   Threshold_BulkWrite_CMD_MSG;

} PGA460_Message;


/*********************************************************************************************************************/
/*-------------------------------------------------PGA460 Responses--------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct _PGA460_RESP_REG_READ
{
        uint8 REG_ADDR;
} PGA460_RESP_REG_READ;


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/


#endif /* HW_ISR_EBCM_PGA460MSG_H_ */
