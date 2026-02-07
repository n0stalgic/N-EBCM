/******************************************************************************
 * @file    ebcm_pga460.c
 * @brief   Driver for PGA460 Ultrasonic DSP
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
#include "IfxAsclin_Asc.h"
#include "IfxCpu_Irq.h"
#include "IfxDma_Dma.h"
#include "IfxSrc.h"
#include "ebcm_pga460.h"
#include "ebcm_cfg.h"
#include "stdint.h"
#include "IfxCpu.h"
#include "string.h"


/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/* DMA macros */

/* DMA channel must be equal to ISR priority */
#define DMA_TX_CHANNEL     ISR_PRIORITY_ASCLIN1_TX
#define DMA_RX_CHANNEL     ISR_PRIORITY_ASCLIN1_RX

#define NUM_DMA_LL_NODES    (2U)

/* UART macros */

#define UART_BAUDRATE         115200                                     /* Baud rate in bit/s                   */

#define UART_PIN_RX           IfxAsclin1_RXA_P15_1_IN                    /* RX pin of the board                  */
#define UART_PIN_TX           IfxAsclin1_TX_P15_0_OUT                    /* TX pin of the board                  */

#define ASC_TX_BUFFER_SIZE             64
#define ASC_RX_BUFFER_SIZE             256

/* PGA460 specific macros */
#define PGA460_ADDR                   0x00
#define PGA460_INITIAL_EEPROM_BURN    (1U)

#define PGA460_SYNC_BYTE        0x55

#define OBJECTS_TO_DETECT       0x1

#define COMMAND_ADDR_POS        5U
#define COMMAND_CMD_POS         0U
#define COMMAND_CMD_MASK        0x1F
#define COMMAND_ADDR_MASK       0x7

#define COMMAND_METADATA_BYTE_COUNT         0x3
#define RESP_METADATA_BYTE_COUNT            0x2

#define CMD_BL_P1_DATA_LEN                  0x1
#define CMD_BL_P2_DATA_LEN                  0x1
#define CMD_L_P1_DATA_LEN                   0x1
#define CMD_L_P2_DATA_LEN                   0x1
#define CMD_TEMP_NOISE_M_DATA_LEN           0x1
#define CMD_ULTRASONIC_M_DATA_LEN           0x0
#define CMD_TMP_NOISE_R_DATA_LEN            0x0
#define CMD_TRANSDUCER_ECHO_DUMP_DATA_LEN   0x0
#define CMD_SYSTEM_DIAG_DATA_LEN            0x0
#define CMD_REGISTER_READ_DATA_LEN          0x1
#define CMD_REGISTER_WRITE_DATA_LEN         0x2
#define CMD_EEPROM_BULK_READ_DATA_LEN       0x0
#define CMD_EEPROM_BULK_WRITE_DATA_LEN      0x2B  /* 43 bytes */
#define CMD_TVG_BULK_READ_DATA_LEN          0x0
#define CMD_TVG_BULK_WRITE_DATA_LEN         0x7
#define CMD_THR_BULK_READ_DATA_LEN          0x0
#define CMD_THR_BULK_WRITE_DATA_LEN         0x20


#define RESP_ULTRASONIC_M_DATA_LEN          (0x4*OBJECTS_TO_DETECT)
#define RESP_TMP_NOISE_R_DATA_LEN           0x2
#define RESP_TRANSDUCER_ECHO_DUMP_DATA_LEN  0x80  /* 128 bytes */
#define RESP_SYSTEM_DIAG_DATA_LEN           0x2
#define RESP_REGISTER_READ_DATA_LEN         0x1
#define RESP_EEPROM_BULK_READ_DATA_LEN      0x2B  /* 43 bytes  */
#define RESP_TVG_BULK_READ_DATA_LEN         0x7
#define RESP_THR_BULK_READ_DATA_LEN         0x20

#define THRESHOLD_P1_L1_L2_INIT_VALUE       152U
#define THRESHOLD_P1_L3_INIT_VALUE          64U
#define THRESHOLD_P1_L4_INIT_VALUE          56U
#define THRESHOLD_P1_L5_L6_INIT_VALUE       32U
#define THRESHOLD_P1_L7_L8_INIT_VALUE       24U
#define THRESHOLD_P1_L9_INIT_VALUE          40U
#define THRESHOLD_P1_L10_INIT_VALUE         48U
#define THRESHOLD_P1_L11_INIT_VALUE         52U
#define THRESHOLD_P1_L12_INIT_VALUE         60
#define THRESHOLD_P1_LEVEL_OFFSET           0U


#define TVG_INITIAL_GAIN_DB                 58.5

#define PACK_COMMAND_BYTE(cmd, addr) (((cmd & COMMAND_CMD_MASK) << COMMAND_CMD_POS) \
        | (addr & COMMAND_ADDR_MASK) << COMMAND_ADDR_POS)

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

const float32 TVG_INIT_GAIN_DB = 58.5f;
const float32 TVG_GAIN1_DB     = 58.5f;
const float32 TVG_GAIN2_DB     = 59.5f;
const float32 TVG_GAIN3_DB     = 67.5f;
const float32 TVG_GAIN4_DB     = 71.5f;
const float32 TVG_GAIN5_DB     = 77.5f;

typedef enum _PGA460_PresetType
{
    PGA460_NEAR_PROFILE = 0,
    PGA460_FAR_PROFILE,
    PGA460_MAX_PRESET
} PGA460_PresetType;

typedef enum _PGA460_AbsoluteThresholdGainTiming
{
    PGA460_THR_TVG_100_USEC = 0,
    PGA460_THR_TVG_200_USEC,
    PGA460_THR_TVG_300_USEC,
    PGA460_THR_TVG_400_USEC,
    PGA460_THR_TVG_600_USEC,
    PGA460_THR_TVG_800_USEC,
    PGA460_THR_TVG_1000_USEC,
    PGA460_THR_TVG_1200_USEC,
    PGA460_THR_TVG_1400_USEC,
    PGA460_THR_TVG_2000_USEC,
    PGA460_THR_TVG_2400_USEC,
    PGA460_THR_TVG_3200_USEC,
    PGA460_THR_TVG_4000_USEC,
    PGA460_THR_TVG_5200_USEC,
    PGA460_THR_TVG_6400_USEC,
    PGA460_THR_TVG_8000_USEC,
    PGA460_THR_TVG_MAX_TIMING
} PGA460_AbsoluteThresholdGainTiming;


typedef enum _PGA460_AFEGainRangeSel
{
    PGA460_AFE_GAIN_RANGE_58_90_dB = 0,
    PGA460_AFE_GAIN_RANGE_52_84_dB,
    PGA460_AFE_GAIN_RANGE_46_78_dB,
    PGA460_AFE_GAIN_RANGE_32_64_dB,
    PGA460_AFE_GAIN_RANGE_MAX
} PGA460_AFEGainRangeSel;

typedef enum _PGA460_CmdType
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
    PGA460_COMMAND_TVG_BULK_WRITE_BCAST,
    PGA460_COMMAND_THRESHOLD_BULK_WRITE_BCAST,
    RESERVED0,
    RESERVED1,
    RESERVED2,
    RESERVED3,
    RESERVED4,
    RESERVED5,
    PGA460_COMMAND_MAX_COMMANDS

} PGA460_CmdType;

typedef enum _PGA460_cmd_st
{
    ST_IDLE,
    ST_CMD_IN_FLIGHT,
    ST_CMD_FINISHED
} PGA460_comm_st;

IfxAsclin_Asc asc_1;

uint8 pga460TxBuffer[ASC_TX_BUFFER_SIZE];
uint8 pga460RxBuffer[ASC_RX_BUFFER_SIZE];

volatile boolean resp_recvd = FALSE;
volatile uint8  tx_bytes = 0;
volatile PGA460_comm_st pga460CommState;

IfxDma_Dma dma;

/* A "lazy" shadow of the PGA460 device.
 * Used to provide an easy way to store internal PGA460 configs.
 * This will not mirror the device exactly, but can be used to access
 * device registers in a friendly way, and refer to last written/read configs if needed
 */
volatile PGA460_Reg pga460;
/* Allocate TCS blocks 32-byte aligned */

/* Initialization TX TCS block */
IFX_ALIGN(256) static uint32 __tcs0[6];

/* Initialization RX TCS block */
IFX_ALIGN(256) static uint32 __tcs1[6];

/* Normal operation TX TCS block */
IFX_ALIGN(256) static uint32 __tcs0[6];

/* Normal operation RX TCS block */
IFX_ALIGN(256) static uint32 __tcs1[6];

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/



/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static void initDMA(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(DMA_TX_ISR, CPU2_VECT_TABLE_ID, DMA_TX_CHANNEL);
void DMA_TX_ISR(void)
{
    tx_bytes++;
    resp_recvd = FALSE;
}

IFX_INTERRUPT(DMA_RX_ISR, CPU2_VECT_TABLE_ID, DMA_RX_CHANNEL);
void DMA_RX_ISR(void)
{
    if (DMA_CHCSR015.B.TCOUNT == 0)
    {
        pga460CommState = ST_CMD_FINISHED;
    }
}

static uint8 calcChecksum(const uint8* frame, size_t data_len)
{
    uint16 carry = 0;

    for (size_t i = 1; i <= data_len; ++i)
    {

        if ((frame[i] + carry) < carry)
        {
            carry = carry + frame[i] + 1;
        }
        else
        {
            carry = carry + frame[i];
        }

        if (carry > 0xFF)
        {
          carry = carry - 255;
        }
     }

    return (uint8) (~carry & 0x00FF);
}



void PGA460_InitInterface(void)
{

    /* Initialize an instance of IfxAsclin_Asc_Config with default values */
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, &MODULE_ASCLIN1);

    /* 115200 8N2 */
    ascConfig.baudrate.baudrate = UART_BAUDRATE;
    ascConfig.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;
    ascConfig.frame.stopBit = IfxAsclin_StopBit_2;
    ascConfig.frame.dataLength = IfxAsclin_DataLength_8;
    ascConfig.frame.parityBit = FALSE;

    /* ISR priorities and interrupt target */
    ascConfig.interrupt.txPriority = ISR_PRIORITY_ASCLIN1_TX;
    ascConfig.interrupt.rxPriority = ISR_PRIORITY_ASCLIN1_RX;
    ascConfig.interrupt.typeOfService = IfxCpu_Irq_getTos(IfxCpu_getCoreIndex());

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

    /* change service from CPU2 to DMA */
    volatile Ifx_SRC_SRCR *src_rx;
    volatile Ifx_SRC_SRCR *src_tx;
    src_rx = IfxAsclin_getSrcPointerRx(ascConfig.asclin);
    src_tx = IfxAsclin_getSrcPointerTx(ascConfig.asclin);

    /* Assign DMA as service provider when ASCLIN1_RX is triggered */
    IfxSrc_init(src_rx, IfxSrc_Tos_dma, ISR_PRIORITY_ASCLIN1_RX);


    IfxSrc_init(src_tx, IfxSrc_Tos_dma, ISR_PRIORITY_ASCLIN1_TX);

    IfxAsclin_enableRxFifoFillLevelFlag(ascConfig.asclin, TRUE);
    IfxAsclin_enableTxFifoFillLevelFlag(ascConfig.asclin, TRUE);
    IfxSrc_enable(src_rx);
    IfxSrc_enable(src_tx);

    initDMA();

}

void PGA460_InitDevice(void)
{
    PGA460_InitInterface();




}

static void dma_init_tcs(void)
{

    __tcs0[0] = 0x00; /* RDCRC */
    __tcs0[1] = 0x00; /* SDCRC */
    __tcs0[2] = (uint32) pga460TxBuffer; /* SADR */
    __tcs0[3] = (uint32) &asc_1.asclin->TXDATA.U; /* DADR */
    __tcs0[4] = 0x0C200088;      /* ADICR: SMF = 0x2, INCD=0x1, DCBE=0x1, INCS=0x1 */
    __tcs0[5] = 0x00080001;      /* CHCFGR: TREL=0x1, RROAT=0x0 PRSEL=0x0 (HW req)  */


    __tcs1[0] = 0x00; /* RDCRC */
    __tcs1[1] = 0x00; /* SDCRC */
    __tcs1[2] = (uint32) &asc_1.asclin->RXDATA.U;; /* SADR */
    __tcs1[3] = (uint32) pga460RxBuffer;           /* DADR */
    __tcs1[4] = 0xC100088;       /* ADICR: DMF = 0x0, INCD=0x1, INCS=0x1, SCBE=0x1 */
    __tcs1[5] = 0x00000005;      /* CHCFGR: TREL=0x5 (init value), RROAT=0x0 PRSEL=0x0  */

    SRC_DMACH16.U       = 0x00001C10; /* SRC_DMACH16: SRPN=0x10 (ISR Prio #), SRE=0x1, TOS=0x3 (CPU2) */
    SRC_DMACH15.U       = 0x00001C0F; /* SRC_DMACH16: SRPN=0x0F (ISR Prio #), SRE=0x1, TOS=0x3 (CPU2) */


}

static void dma_load_cmd_tcs(size_t cmd_len)
{
    __tcs0[5] &= ~(0x3FFF);
    __tcs0[5] |= (cmd_len & 0x3FFF); /* CHCFGR: TREL=cmd_len */

    DMA_RDCRCR016.U     = __tcs0[0];
    DMA_SDCRCR016.U     = __tcs0[1];
    DMA_SADR016.U       = __tcs0[2];
    DMA_DADR016.U       = __tcs0[3];
    DMA_ADICR016.U      = __tcs0[4];
    DMA_CHCFGR016.U     = __tcs0[5];

    DMA_TSR016.B.ECH = 0x1;
    DMA_TSR015.B.ECH = 0x1;

}

static void asclin_start_tx(void)
{
    /* Initiate the transmission by using the transmit FIFO level flag */
    ASCLIN1_FLAGSSET.B.TFLS = 1;
    pga460CommState = ST_CMD_IN_FLIGHT;
}

static void dma_load_resp_tcs(size_t resp_len)
{
    __tcs1[5] &= ~(0x3FFF);
    __tcs1[5] |= (resp_len & 0x3FFF);

    DMA_RDCRCR015.U     = __tcs1[0];
    DMA_SDCRCR015.U     = __tcs1[1];
    DMA_SADR015.U       = __tcs1[2];
    DMA_DADR015.U       = __tcs1[3];
    DMA_ADICR015.U      = __tcs1[4];
    DMA_CHCFGR015.U     = __tcs1[5];

    DMA_TSR015.B.ECH = 0x1;
}

/** \brief Initialize EBCM DMA
 *  The controller uses DMA in linked list mode to interface with an ultrasonic sensor via ASCLIN1.
 *  We will use a fixed source address (ASCLIN1 RX FIFO) and an incremented
 *  destination address
 *
 *
 */
static void initDMA(void)
{
    __UNLOCK_CPU_SAFETY_WD();
    DMA_CLC.U = 0x00000000;
    __LOCK_CPU_SAFETY_WD();

    /* Initialize an instance of IfxDma_Dma_Config with default values */
    dma_init_tcs();

}

static inline size_t pack_RegisterReadFrame(uint8* buffer, uint8 pga_addr, uint8 reg_addr)
{
    if (buffer != NULL_PTR)
    {
        buffer[0] = PGA460_SYNC_BYTE;
        buffer[1] = PACK_COMMAND_BYTE(PGA460_COMMAND_REGISTER_READ, pga_addr);
        buffer[2] = reg_addr;
        buffer[3] = calcChecksum(buffer, 2); // cmd + 1 data byte

        return 4;
    }

    return -1;
}

static inline size_t pack_RegisterWriteFrame(uint8* buffer, uint8 pga_addr, uint8 reg_addr, uint8 data)
{
    if (buffer != NULL_PTR)
    {
        buffer[0] = PGA460_SYNC_BYTE;
        buffer[1] = PACK_COMMAND_BYTE(PGA460_COMMAND_REGISTER_WRITE, pga_addr);
        buffer[2] = reg_addr;
        buffer[3] = data;
        buffer[4] = calcChecksum(buffer, 3); // cmd + 2 data byte

        return 5;
    }

    return -1;
}

static inline size_t pack_EEPROMBulkReadFrame(uint8* buffer, uint8 pga_addr)
{
    if (buffer != NULL_PTR)
    {
        buffer[0] = PGA460_SYNC_BYTE;
        buffer[1] = PACK_COMMAND_BYTE(PGA460_COMMAND_EEPROM_BULK_READ, pga_addr);

        return 2;
    }

    return -1;
}

#if 0
static inline size_t unpack_EEPROMBulkReadData(uint8* buffer)
{
    if (buffer != NULL_PTR)
    {
        ;
    }
}

#endif

void PGA460_ProcessFrame(void)
{
    // do some processing here, like setting internal shadow registers. do nothing for now
    if (pga460CommState == ST_CMD_FINISHED)
    {
        pga460CommState = ST_IDLE;
    }
}

boolean PGA460_isAvailable(void)
{
    return (pga460CommState == ST_IDLE);
}

boolean PGA460_isDataReady(void)
{
    return (pga460CommState == ST_CMD_FINISHED);
}

void    PGA460_RegisterRead(uint8 reg_addr)
{

    if (DMA_TSR016.B.CH || DMA_TSR015.B.CH)
    {
        return;
    }

    /* +-- sync--+--command--+ data--+--CRC */
    uint8 buf[4] = {0};
    const Ifx_SizeT msgLen = 4;
    const Ifx_SizeT respLen = RESP_METADATA_BYTE_COUNT + RESP_REGISTER_READ_DATA_LEN; /* diag data + data len + chksum */
    pack_RegisterReadFrame(buf, PGA460_ADDR, reg_addr);

    memmove(pga460TxBuffer, buf, msgLen);

    /* we expect a response, so load DMACH015 RX and DMACH016 TX */
    dma_load_resp_tcs(respLen);
    dma_load_cmd_tcs(msgLen);


    /* Initiate the transmission by using the transmit FIFO level flag */
    asclin_start_tx();

}

void    PGA460_RegisterWrite(uint8 reg_addr, uint8 data)
{

    if (DMA_TSR016.B.CH || DMA_TSR015.B.CH)
    {
        return;
    }

    /* +-- sync--+--command--+--data(2)--+--CRC */
    uint8 buf[5] = {0};
    Ifx_SizeT msgLen = 5;
    pack_RegisterWriteFrame(buf, PGA460_ADDR, reg_addr, data);

    memmove(pga460TxBuffer, buf, msgLen);

    /* we don't expect a response, so disable DMA daisy chain */
    dma_load_cmd_tcs(msgLen);

    asclin_start_tx();
}


void    PGA460_EEPROMRead(void)
{

    /* extra protection, give up if DMA is busy */
    if (DMA_TSR016.B.CH || DMA_TSR015.B.CH)
    {
        return;
    }

    /* +-- sync--+--command--+ data(0)--+--CRC */
    uint8 buf[2] = {0};
    const Ifx_SizeT msgLen = 2;
    const Ifx_SizeT respLen = RESP_EEPROM_BULK_READ_DATA_LEN + RESP_METADATA_BYTE_COUNT; /* CRC + diag data */
    pack_EEPROMBulkReadFrame(buf, PGA460_ADDR);

    memmove(pga460TxBuffer, buf, msgLen);

    dma_load_cmd_tcs(msgLen);
    dma_load_resp_tcs(respLen);

    asclin_start_tx();
}

void PGA460_InitDecoupling(void)
{

    pga460.DECPL_TEMP.B.AFE_GAIN_RNG = PGA460_AFE_GAIN_RANGE_52_84_dB;
    pga460.DECPL_TEMP.B.LPM_EN       = 0x00;
}

void     PGA460_InitTVGs(void)
{

    pga460.TVGAIN0.B.TVG_T0 = PGA460_THR_TVG_600_USEC;
    pga460.TVGAIN0.B.TVG_T1 = PGA460_THR_TVG_600_USEC;
    pga460.TVGAIN1.B.TVG_T2 = PGA460_THR_TVG_600_USEC;
    pga460.TVGAIN1.B.TVG_T3 = PGA460_THR_TVG_600_USEC;
    pga460.TVGAIN2.B.TVG_T4 = PGA460_THR_TVG_600_USEC;
    pga460.TVGAIN2.B.TVG_T5 = PGA460_THR_TVG_600_USEC;


}

void     PGA460_initThresholds(void)
{
    pga460.P1_THR_0.B.TH_P1_T1  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_0.B.TH_P1_T2  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_1.B.TH_P1_T3  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_1.B.TH_P1_T4  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_2.B.TH_P1_T5  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_2.B.TH_P1_T6  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_3.B.TH_P1_T7  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_3.B.TH_P1_T8  = PGA460_THR_TVG_600_USEC;
    pga460.P1_THR_4.B.TH_P1_T9  = PGA460_THR_TVG_800_USEC;
    pga460.P1_THR_4.B.TH_P1_T10 = PGA460_THR_TVG_800_USEC;
    pga460.P1_THR_5.B.TH_P1_T11 = PGA460_THR_TVG_800_USEC;
    pga460.P1_THR_5.B.TH_P1_T12 = PGA460_THR_TVG_800_USEC;
    pga460.P1_THR_6.U  = THRESHOLD_P1_L1_L2_INIT_VALUE;
    pga460.P1_THR_7.U  = THRESHOLD_P1_L3_INIT_VALUE;
    pga460.P1_THR_8.U  = THRESHOLD_P1_L4_INIT_VALUE;
    pga460.P1_THR_9.U  = THRESHOLD_P1_L5_L6_INIT_VALUE;
    pga460.P1_THR_10.U  = THRESHOLD_P1_L7_L8_INIT_VALUE;
    pga460.P1_THR_11.B.TH_P1_L9 = THRESHOLD_P1_L9_INIT_VALUE;
    pga460.P1_THR_12.B.TH_P1_L10 = THRESHOLD_P1_L12_INIT_VALUE;

}

boolean  PGA460_burnEEPROM()
{
    return FALSE;
}

