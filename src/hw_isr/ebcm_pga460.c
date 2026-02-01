/******************************************************************************
 * @file    ebcm_pga460.c
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
#include "Ifx_Types.h"
#include "IfxAsclin.h"
#include "ebcm_pga460.h"
#include "ebcm_cfg.h"
#include "stdint.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define UART_BAUDRATE         115200                                     /* Baud rate in bit/s                   */

#define UART_PIN_RX           IfxAsclin1_RXA_P15_1_IN                    /* RX pin of the board                  */
#define UART_PIN_TX           IfxAsclin1_TX_P15_0_OUT                    /* TX pin of the board                  */

#define ASC_TX_BUFFER_SIZE             64
#define ASC_RX_BUFFER_SIZE             64

#define PGA460_INITIAL_EEPROM_BURN    (1U)

#define PGA460_SYNC_BYTE        0x55

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

IfxAsclin_Asc asc_1;
static uint8 pga460TxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 pga460RxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];


/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(asclin1_Tx_ISR, CPU2_VECT_TABLE_ID, ISR_PRIORITY_ASCLIN1_TX);
void asclin0_Tx_ISR(void)
{
    IfxAsclin_Asc_isrTransmit(&asc_1);
}

IFX_INTERRUPT(asclin1_Rx_ISR, CPU2_VECT_TABLE_ID, ISR_PRIORITY_ASCLIN1_RX);
void asclin1_Rx_ISR(void)
{
    IfxAsclin_Asc_isrReceive(&asc_1);
}

static uint8 getCmdDataLen(PGA460_CMD cmd)
{
    switch(cmd.B.CMD_ID)
    {
        case PGA460_COMMAND_ULTRASONIC_MEASUREMENT_RESULT:
        case PGA460_COMMAND_TEMPERATURE_NOISE_LEVEL_RESULT:
        case PGA460_COMMAND_TRANSDUCER_ECHO_DATA_DUMP:
        case PGA460_COMMAND_SYSTEM_DIAGNOSTICS:
        case PGA460_COMMAND_EEPROM_BULK_READ:
        case PGA460_COMMAND_TVG_BULK_READ:
        case PGA460_COMMAND_THRESHOLD_BULK_READ:
            return 0;
            break;
        case PGA460_COMMAND_BURST_AND_LISTEN_PRESET_1:
        case PGA460_COMMAND_BURST_AND_LISTEN_PRESET_2:
        case PGA460_COMMAND_LISTEN_ONLY_PRESET_1:
        case PGA460_COMMAND_LISTEN_ONLY_PRESET_2:
        case PGA460_COMMAND_TEMPERATURE_NOISE_MEASUREMENT:
        case PGA460_COMMAND_REGISTER_READ:
            return 1;
            break;
        case PGA460_COMMAND_REGISTER_WRITE:
            return 2;
            break;
        case PGA460_COMMAND_TVG_BULK_WRITE:
            return 7;
            break;
        case PGA460_COMMAND_THRESHOLD_BULK_WRITE:
            return 32;
            break;
        case PGA460_COMMAND_EEPROM_BULK_WRITE:
            return 43;
            break;
        default:
            return UINTMAX_C;

    }
}

static uint8 calcChecksum(PGA460_Message frame, size_t data_len)
{
    uint16 carry = 0;

    /* checksum the cmd + data bytes */
    uint8  ckSumLen = data_len + 1;

#define DATA_OFFSET  1U

    for (uint8 i = DATA_OFFSET; i < ckSumLen; ++i)
    {

#undef DATA_OFFSET
        if ((frame.MSG_BYTES[i] + carry) < carry)
        {
            carry = carry + frame.MSG_BYTES[i] + 1;
        }
        else
        {
            carry = carry + frame.MSG_BYTES[i];
        }

        if (carry > 0xFF)
        {
          carry = carry - 255;
        }
     }


    carry = (~carry & 0x00FF);
    return carry;
}



void PGA460_InitInterface(void)
{
    /* Initialize an instance of IfxAsclin_Asc_Config with default values */
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, UART_PIN_TX.module);

    /* 115200 8N2 */
    ascConfig.baudrate.baudrate = UART_BAUDRATE;
    ascConfig.frame.stopBit = IfxAsclin_StopBit_2;
    ascConfig.frame.dataLength = IfxAsclin_DataLength_8;
    ascConfig.frame.parityBit = FALSE;

    /* ISR priorities and interrupt target */
    ascConfig.interrupt.txPriority = ISR_PRIORITY_ASCLIN1_TX;
    ascConfig.interrupt.rxPriority = ISR_PRIORITY_ASCLIN1_RX;
    ascConfig.interrupt.typeOfService = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

    /* FIFO configuration */
    ascConfig.txBuffer = &pga460TxBuffer;
    ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConfig.rxBuffer = &pga460RxBuffer;
    ascConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;

    /* Port pins configuration */
    const IfxAsclin_Asc_Pins pins =
    {
        NULL_PTR,       IfxPort_InputMode_pullUp,                   /* CTS pin not used                         */
        &UART_PIN_RX,   IfxPort_InputMode_pullUp,                   /* RX pin                                   */
        NULL_PTR,       IfxPort_OutputMode_pushPull,                /* RTS pin not used                         */
        &UART_PIN_TX,   IfxPort_OutputMode_pushPull,                /* TX pin                                   */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConfig.pins = &pins;

    IfxAsclin_Asc_initModule(&asc_1, &ascConfig);                       /* Initialize module with above parameters  */
}


void    PGA460_RegisterRead(uint8 reg_addr)
{
    PGA460_Message PGA460Msg;

    size_t count = sizeof(PGA460Msg);

    PGA460Msg.RegisterRead_CMD_MSG.SYNC       = PGA460_SYNC_BYTE;
    PGA460Msg.RegisterRead_CMD_MSG.CMD.CMD_ID = PGA460_COMMAND_REGISTER_READ;
    PGA460Msg.RegisterRead_CMD_MSG.CMD.ADDR   = PGA460_DEFAULT_ADDR;
    PGA460Msg.RegisterRead_CMD_MSG.DATA       = reg_addr;
    PGA460Msg.RegisterRead_CMD_MSG.CHKSUM     = calcChecksum(PGA460Msg, sizeof(PGA460Msg.RegisterRead_CMD_MSG.DATA));

    memmove(&PGA460Msg, pga460TxBuffer, count);

    IfxAsclin_Asc_write(&asc_1, pga460TxBuffer, &count, TIME_INFINITE);

}

uint8    PGA460_RegisterWrite(uint8 addr, uint8 data)
{

}

void     PGA460_initThresholds(void)
{

}

boolean  PGA460_burnEEPROM()
{

}

