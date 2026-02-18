/******************************************************************************
 * @file    vcom.c
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
#include "IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"
#include "IfxStdIf_DPipe.h"
#include <stdio.h>
#include <stdarg.h>
#include "ebcm_cfg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define SERIAL_BAUDRATE         115200                                      /* Baud rate in bit/s                   */

#define SERIAL_PIN_RX           IfxAsclin0_RXA_P14_1_IN                     /* RX pin of the board                  */
#define SERIAL_PIN_TX           IfxAsclin0_TX_P14_0_OUT                     /* TX pin of the board                  */


#define ASC_TX_BUFFER_SIZE      64                                          /* Definition of the TX buffer size     */
#define ASC_RX_BUFFER_SIZE      128                                         /* Definition of the RX buffer size     */

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxAsclin_Asc g_asc;                                                /* Declaration of the ASC handle        */
IfxStdIf_DPipe g_ascStandardInterface;                                    /* Standard interface object            */

/* The transfer buffers allocate memory for the data itself and for FIFO runtime variables.
 * 8 more bytes have to be added to ensure a proper circular buffer handling independent from
 * the address to which the buffers have been located.
 */
uint8 g_ascTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];             /* Declaration of the TX FIFO parameters */
uint8 g_ascRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];             /* Declaration of the RX FIFO parameters */

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(asclin0_Tx_ISR, 0, ISR_PRIORITY_ASCLIN0_TX);                   /* Adding the Interrupt Service Routine */
IFX_INTERRUPT(asclin0_Rx_ISR, 0, ISR_PRIORITY_ASCLIN0_RX);
IFX_INTERRUPT(asclin0_Err_ISR, 0, ISR_PRIORITY_ASCLIN0_ER);

void asclin0_Tx_ISR(void)
{
    IfxStdIf_DPipe_onTransmit(&g_ascStandardInterface);
}

void asclin0_Rx_ISR(void)
{
    IfxStdIf_DPipe_onReceive(&g_ascStandardInterface);
}

void asclin0_Err_ISR(void)
{
    IfxStdIf_DPipe_onError(&g_ascStandardInterface);
}

void init_UART(void)
{
    /* Initialize an instance of IfxAsclin_Asc_Config with default values */
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, SERIAL_PIN_TX.module);

    /* Set the desired baud rate */
    ascConfig.baudrate.baudrate = SERIAL_BAUDRATE;

    /* ISR priorities and interrupt target */
    ascConfig.interrupt.rxPriority = (uint8) ISR_PRIORITY_ASCLIN0_RX;
    ascConfig.interrupt.txPriority = (uint8) ISR_PRIORITY_ASCLIN0_TX;
    ascConfig.interrupt.erPriority = (uint8) ISR_PRIORITY_ASCLIN0_ER;
    ascConfig.interrupt.typeOfService = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

    /* FIFO configuration */
    ascConfig.txBuffer = &g_ascTxBuffer;
    ascConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
    ascConfig.rxBuffer = &g_ascRxBuffer;
    ascConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;

    /* Port pins configuration */
    const IfxAsclin_Asc_Pins pins =
    {
        NULL_PTR,         IfxPort_InputMode_pullUp,                     /* CTS pin not used                         */
        &SERIAL_PIN_RX,   IfxPort_InputMode_pullUp,                     /* RX pin not used                          */
        NULL_PTR,         IfxPort_OutputMode_pushPull,                  /* RTS pin not used                         */
        &SERIAL_PIN_TX,   IfxPort_OutputMode_pushPull,                  /* TX pin                                   */
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };
    ascConfig.pins = &pins;

    IfxAsclin_Asc_initModule(&g_asc, &ascConfig);                       /* Initialize module with above parameters  */
    IfxStdIf_DPipe_ascInit(&g_ascStandardInterface, &g_asc);
}

void vcom_Print(const char *format, ...)
{
    char buffer[128];
    va_list args;
    Ifx_SizeT count;

    va_start(args, format);
    count = (Ifx_SizeT)vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    IfxStdIf_DPipe_write(&g_ascStandardInterface, (uint8 *)buffer, &count, TIME_INFINITE);
}

void send_UART_message(void)
{
    ;
}
