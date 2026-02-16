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
#include "_Utilities/Ifx_Assert.h"


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

#define COMMAND_METADATA_BYTE_COUNT         0x2 // sync + CRC
#define RESP_METADATA_BYTE_COUNT            0x2

#define PGA460_CMD_DATA_LEN    ASC_TX_BUFFER_SIZE - COMMAND_METADATA_BYTE_COUNT

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

#define BURST_FREQUENCY_EQUATION_PARAM      0x8Fu
#define SECONDARY_DECOUPLE_TIME             0xFu
#define BURST_PULSE_COUNT_P1                0x4u
#define CURRENT_LIMIT_TERM_P1_MA            0x7u
#define LPF_CUTOFF_FREQ_TERM                0x1u
#define DEGLITCH_TIME                       0x8u
#define BURST_PULSE_DEADTIME                0x0u
#define SATURATION_DIAG_THRESHOLD           0x1u
#define FREQUENCY_DIAG_ABS_ERR_THRESHOLD    0x1u

#define PACK_COMMAND_BYTE(cmd, addr) (((cmd & COMMAND_CMD_MASK) << COMMAND_CMD_POS) \
        | (addr & COMMAND_ADDR_MASK) << COMMAND_ADDR_POS)

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/


/* The following TVGAINx values result in the above floating point gains, refer to PGA460 datasheet for formulas */
#define TVG_INIT_GAIN_DB             0U   // with 52-84dB range, init gain is 52.5 dB;
#define TVG_GAIN1_DB                 2U   // 53.5 dB
#define TVG_GAIN2_DB                 6U   // 55.5
#define TVG_GAIN3_DB                 18U  // 61.5
#define TVG_GAIN4_DB                 26U  // 65.5
#define TVG_GAIN5_DB                 38U  // 71.5


#define BANDPASS_FILTER_WIDTH_FACTOR 1U     // Bandwidth = 2 * (BPF_BW + 1) [kHz]




/** \brief Default PGA460 Configuration
 *  This configuration uses the hardcoded values that were previously defined as macros.
 *  Users can modify these values or create their own configuration structures.
 */
const PGA460_Config PGA460_DefaultConfig = {

    .afeGainRange        = PGA460_AFE_GAIN_RANGE_52_84_dB,
    .lowPowerMode        = PGA460_LPM_Disabled,
    .decoupleTimeTempSel = PGA460_TimeDecouple,
    .decoupleTime        = SECONDARY_DECOUPLE_TIME,

    .tvgT0 = PGA460_THR_TVG_600_USEC,
    .tvgT1 = PGA460_THR_TVG_600_USEC,
    .tvgT2 = PGA460_THR_TVG_600_USEC,
    .tvgT3 = PGA460_THR_TVG_600_USEC,
    .tvgT4 = PGA460_THR_TVG_600_USEC,
    .tvgT5 = PGA460_THR_TVG_600_USEC,
    .tvgG1 = TVG_GAIN1_DB,
    .tvgG2 = TVG_GAIN2_DB,
    .tvgG3 = TVG_GAIN3_DB,
    .tvgG4 = TVG_GAIN4_DB,
    .tvgG5 = TVG_GAIN5_DB,
    .freqShift = 0x0,

    .initGain     = TVG_INIT_GAIN_DB,
    .bpfBandwidth = BANDPASS_FILTER_WIDTH_FACTOR,

    .burstFrequency      = BURST_FREQUENCY_EQUATION_PARAM,
    .burstPulseCountP1   = BURST_PULSE_COUNT_P1,
    .currentLimitType     = PGA460_CurrentLimitEnabled,
    .currentLimitP1      = CURRENT_LIMIT_TERM_P1_MA,
    .lpfCutoffFreq       = LPF_CUTOFF_FREQ_TERM,
    .recordTimeLengthP1  = 0x0,
    .deglitchTime        = DEGLITCH_TIME,
    .burstPulseDeadtime  = BURST_PULSE_DEADTIME,

    .LR_StartingDigitalGainCtrl = PGA460_StartingDigitalGainLR_TH9,
    .LR_DigitalGainCtrl         = PGA460_LRDigitalGain_8x,
    .SR_DigitalGainCtrl         = PGA460_SRDigitalGain_1x,

    .saturationDiagThreshold    = SATURATION_DIAG_THRESHOLD,
    .frequencyDiagAbsErrTimeThreshold = FREQUENCY_DIAG_ABS_ERR_THRESHOLD,
    .P1_NLS = PGA460_P1_NLS_Disabled,

    .Scale_K = PGA460_NonlinearScaling_1_5,
    .Scale_N = PGA460_NonlinearScalingStart_TH9,
    .noiseLevel = 0x0,

    .thrT1  = PGA460_THR_TVG_600_USEC,
    .thrT2  = PGA460_THR_TVG_600_USEC,
    .thrT3  = PGA460_THR_TVG_600_USEC,
    .thrT4  = PGA460_THR_TVG_600_USEC,
    .thrT5  = PGA460_THR_TVG_600_USEC,
    .thrT6  = PGA460_THR_TVG_600_USEC,
    .thrT7  = PGA460_THR_TVG_600_USEC,
    .thrT8  = PGA460_THR_TVG_600_USEC,
    .thrT9  = PGA460_THR_TVG_800_USEC,
    .thrT10 = PGA460_THR_TVG_800_USEC,
    .thrT11 = PGA460_THR_TVG_800_USEC,
    .thrT12 = PGA460_THR_TVG_800_USEC,
    .thrL1L2 = THRESHOLD_P1_L1_L2_INIT_VALUE,
    .thrL3   = THRESHOLD_P1_L3_INIT_VALUE,
    .thrL4   = THRESHOLD_P1_L4_INIT_VALUE,
    .thrL5L6 = THRESHOLD_P1_L5_L6_INIT_VALUE,
    .thrL7L8 = THRESHOLD_P1_L7_L8_INIT_VALUE,
    .thrL9   = THRESHOLD_P1_L9_INIT_VALUE,
    .thrL10  = THRESHOLD_P1_L12_INIT_VALUE
};

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
    ST_IN_FLIGHT,
    ST_AWAIT_RESPONSE,
    ST_CMD_DONE,
    ST_CMD_ERR
} PGA460_comm_st;

typedef struct _PGA460_CommandInfo
{
    uint8 cmdDataLen;
    uint8 respDataLen;
    boolean expectsResponse;
} PGA460_CommandInfo;

typedef struct _PGA460_CommandParams
{
    PGA460_CmdType cmdType;
    uint8 pga_addr;
    uint8 data[PGA460_CMD_DATA_LEN];
    uint8 dataLen;
} PGA460_CommandParams;

IfxAsclin_Asc asc_1;

/* place in non-cacheable CPU2 LMU region */
__at(0xb0020000) uint8 pga460TxBuffer[ASC_TX_BUFFER_SIZE];
__at(0xb0020300) uint8 pga460RxBuffer[ASC_RX_BUFFER_SIZE];

volatile boolean resp_recvd = FALSE;
volatile uint8  tx_bytes = 0;
volatile PGA460_comm_st pga460CommState;

boolean EEPROMBulkReadRdy = FALSE;;

IfxDma_Dma dma;

volatile boolean TxComplete = FALSE;

/* A "lazy" shadow of the PGA460 device.
 * Used to provide an easy way to store internal PGA460 configs.
 * This will not mirror the device exactly, but can be used to access
 * device registers in a friendly way, and refer to last written/read configs if needed
 */
volatile PGA460_Reg pga460shadow;
volatile PGA460_EEPROM pga460_eeprom;
/* Allocate TCS blocks 32-byte aligned */

/* Initialization TX TCS block */
IFX_ALIGN(256) static uint32 __tcs0[6];

/* Initialization RX TCS block */
IFX_ALIGN(256) static uint32 __tcs1[6];

/* Normal operation TX TCS block */
__attribute__((unused)) IFX_ALIGN(256) static uint32 __tcs2[6];

/* Normal operation RX TCS block */
__attribute__((unused)) IFX_ALIGN(256) static uint32 __tcs3[6];

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

volatile PGA460_CmdType activeCommand;

static const PGA460_CommandInfo commandTable[PGA460_COMMAND_MAX_COMMANDS] = {
    [PGA460_COMMAND_BURST_AND_LISTEN_PRESET_1] = {
        .cmdDataLen = CMD_BL_P1_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_BURST_AND_LISTEN_PRESET_2] = {
        .cmdDataLen = CMD_BL_P2_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_LISTEN_ONLY_PRESET_1] = {
        .cmdDataLen = CMD_L_P1_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_LISTEN_ONLY_PRESET_2] = {
        .cmdDataLen = CMD_L_P2_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_TEMPERATURE_NOISE_MEASUREMENT] = {
        .cmdDataLen = CMD_TEMP_NOISE_M_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_ULTRASONIC_MEASUREMENT_RESULT] = {
        .cmdDataLen = CMD_ULTRASONIC_M_DATA_LEN,
        .respDataLen = RESP_ULTRASONIC_M_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_TEMPERATURE_NOISE_LEVEL_RESULT] = {
        .cmdDataLen = CMD_TMP_NOISE_R_DATA_LEN,
        .respDataLen = RESP_TMP_NOISE_R_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_TRANSDUCER_ECHO_DATA_DUMP] = {
        .cmdDataLen = CMD_TRANSDUCER_ECHO_DUMP_DATA_LEN,
        .respDataLen = RESP_TRANSDUCER_ECHO_DUMP_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_SYSTEM_DIAGNOSTICS] = {
        .cmdDataLen = CMD_SYSTEM_DIAG_DATA_LEN,
        .respDataLen = RESP_SYSTEM_DIAG_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_REGISTER_READ] = {
        .cmdDataLen = CMD_REGISTER_READ_DATA_LEN,
        .respDataLen = RESP_REGISTER_READ_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_REGISTER_WRITE] = {
        .cmdDataLen = CMD_REGISTER_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_EEPROM_BULK_READ] = {
        .cmdDataLen = CMD_EEPROM_BULK_READ_DATA_LEN,
        .respDataLen = RESP_EEPROM_BULK_READ_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_EEPROM_BULK_WRITE] = {
        .cmdDataLen = CMD_EEPROM_BULK_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_TVG_BULK_READ] = {
        .cmdDataLen = CMD_TVG_BULK_READ_DATA_LEN,
        .respDataLen = RESP_TVG_BULK_READ_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_TVG_BULK_WRITE] = {
        .cmdDataLen = CMD_TVG_BULK_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_THRESHOLD_BULK_READ] = {
        .cmdDataLen = CMD_THR_BULK_READ_DATA_LEN,
        .respDataLen = RESP_THR_BULK_READ_DATA_LEN,
        .expectsResponse = TRUE
    },
    [PGA460_COMMAND_THRESHOLD_BULK_WRITE] = {
        .cmdDataLen = CMD_THR_BULK_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_BURST_LISTEN_PRESET_1_BCAST] = {
        .cmdDataLen = CMD_BL_P1_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_BURST_LISTEN_PRESET_2_BCAST] = {
        .cmdDataLen = CMD_BL_P2_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_LISTEN_ONLY_PRESET_1_BCAST] = {
        .cmdDataLen = CMD_L_P1_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_LISTEN_ONLY_PRESET_2_BCAST] = {
        .cmdDataLen = CMD_L_P2_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_TEMPERATURE_NOISE_LEVEL_MEASUREMENT_BCAST] = {
        .cmdDataLen = CMD_TEMP_NOISE_M_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_REGISTER_WRITE_BCAST] = {
        .cmdDataLen = CMD_REGISTER_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_EEPROM_BULK_WRITE_BCAST] = {
        .cmdDataLen = CMD_EEPROM_BULK_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_TVG_BULK_WRITE_BCAST] = {
        .cmdDataLen = CMD_TVG_BULK_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    },
    [PGA460_COMMAND_THRESHOLD_BULK_WRITE_BCAST] = {
        .cmdDataLen = CMD_THR_BULK_WRITE_DATA_LEN,
        .respDataLen = 0,
        .expectsResponse = FALSE
    }
};



/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
static void initDMA(void);
static void unpack_EEPROMBulkRead(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(DMA_TX_ISR, CPU2_VECT_TABLE_ID, DMA_TX_CHANNEL);
void DMA_TX_ISR(void)
{
    TxComplete = TRUE;
    if (activeCommand < PGA460_COMMAND_MAX_COMMANDS && commandTable[activeCommand].expectsResponse)
    {
        pga460CommState = ST_AWAIT_RESPONSE;
    }
    else
    {
        pga460CommState = ST_CMD_DONE;
        activeCommand = PGA460_COMMAND_MAX_COMMANDS;
    }
}

IFX_INTERRUPT(DMA_RX_ISR, CPU2_VECT_TABLE_ID, DMA_RX_CHANNEL);
void DMA_RX_ISR(void)
{
    if (DMA_CHCSR015.B.TCOUNT == 0)
    {
        pga460CommState = ST_CMD_DONE;
        activeCommand = PGA460_COMMAND_MAX_COMMANDS;
    }
}


static void dma_init_tcs(void)
{

    __tcs0[0] = 0x00; /* RDCRC */
    __tcs0[1] = 0x00; /* SDCRC */
    __tcs0[2] = (uint32) pga460TxBuffer; /* SADR */
    __tcs0[3] = (uint32) &asc_1.asclin->TXDATA.U; /* DADR */
    __tcs0[4] = 0x08200088;      /* ADICR: SMF = 0x2, INCD=0x1, DCBE=0x1, INCS=0x1 */
    __tcs0[5] = 0x0000000A;      /* CHCFGR: TREL=0x1, RROAT=0x0 PRSEL=0x0 (HW req)  */


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

__attribute__((unused)) static void dma_init_ultrasonic_tcs(void)
{
    return;
}

__attribute__((unused)) static void dma_load_ultrasonic_tcs(size_t cmd_len)
{
    return;
}

static void asclin_start_tx(void)
{
    /* Initiate the transmission by using the transmit FIFO level flag */
    ASCLIN1_FLAGSSET.B.TFLS = 1;
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

    dma_init_tcs();

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

    /* Initialize PGA460 with default configuration */
    PGA460_UltrasonicInit(&PGA460_DefaultConfig);
}

void PGA460_UltrasonicInit(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    /* Apply all configuration settings to the device shadow registers */
    PGA460_InitDecoupling(config);
    PGA460_InitTVGs(config);
    PGA460_SetInitialGain(config);
    PGA460_SetBurstFrequency(config);
    PGA460_SetNumberBurstPulsesP1(config);
    PGA460_SetCurrentLimitP1(config);
    PGA460_SetLPFCutoffFrequency(config);
    PGA460_SetRecordTimeLengthP1(config);
    PGA460_SetDeglitchTime(config);
    PGA460_InitThresholds(config);
    PGA460_SetNonlinearScalingP1(config);
    PGA460_SetFrequencyDiagErrThreshold(config);
    PGA460_SetSaturationDiagThreshold(config);
    PGA460_SetDSPScaling(config);

    PGA460_TimeVaryingGainBulkWriteBlocking(5);

    PGA460_RegisterWriteBlocking(PGA460_REG_INIT_GAIN_OFFSET, pga460shadow.INIT_GAIN.U, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_FREQUENCY_OFFSET, pga460shadow.FREQUENCY.U, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_DEADTIME_OFFSET,  pga460shadow.DEADTIME.U, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_PULSE_P1_OFFSET,  pga460shadow.PULSE_P1.U, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_CURR_LIM_P1_OFFSET,  pga460shadow.CURR_LIM_P1.U, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_REC_LENGTH_OFFSET,   pga460shadow.REC_LENGTH.B.P1_REC, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_SAT_FDIAG_TH_OFFSET, pga460shadow.SAT_FDIAG_TH.U, 2);
    PGA460_RegisterWriteBlocking(PGA460_REG_DSP_SCALE_OFFSET,    pga460shadow.DSP_SCALE.U, 2);

}



static size_t PGA460_PackCommand(uint8* buffer, const PGA460_CommandParams* params)
{
    if (buffer == NULL_PTR || params == NULL_PTR)
    {
        return 0;
    }

    if (params->cmdType >= PGA460_COMMAND_MAX_COMMANDS)
    {
        return 0;
    }

    const PGA460_CommandInfo* cmdInfo = &commandTable[params->cmdType];

    if (params->dataLen != cmdInfo->cmdDataLen)
    {
        return 0;
    }

    buffer[0] = PGA460_SYNC_BYTE;
    buffer[1] = PACK_COMMAND_BYTE(params->cmdType, params->pga_addr);

    if (params->dataLen > 0)
    {
        memmove(&buffer[2], params->data, params->dataLen);
    }

    size_t frameLen = 2 + params->dataLen;
    buffer[frameLen] = calcChecksum(buffer, frameLen - 1);

    return frameLen + 1;
}

static boolean PGA460_SendCommandAsync(const PGA460_CommandParams* params)
{
    if (params == NULL_PTR)
    {
        return FALSE;
    }

    /* protect the DMA while it's transferring */
    while(DMA_TSR016.B.HTRE != 0);

    if (params->cmdType >= PGA460_COMMAND_MAX_COMMANDS)
    {
        return FALSE;
    }

    const PGA460_CommandInfo* cmdInfo = &commandTable[params->cmdType];

    size_t msgLen = PGA460_PackCommand(pga460TxBuffer, params);
    if (msgLen == 0)
    {
        return FALSE;
    }

    if (cmdInfo->expectsResponse)
    {
        size_t respLen = RESP_METADATA_BYTE_COUNT + cmdInfo->respDataLen;
        dma_load_resp_tcs(respLen);
    }

    dma_load_cmd_tcs(msgLen);
    __disable();
    pga460CommState = ST_IN_FLIGHT;
    activeCommand = params->cmdType;
    __enable();
    asclin_start_tx();

    return TRUE;
}

static boolean PGA460_SendCommandBlocking(const PGA460_CommandParams* params, uint64 timeoutMs)
{
    boolean rc = FALSE;

    if (params == NULL_PTR)
    {
        return FALSE;
    }

    if (params->cmdType >= PGA460_COMMAND_MAX_COMMANDS)
    {
        return FALSE;
    }

    const PGA460_CommandInfo* cmdInfo = &commandTable[params->cmdType];

    size_t msgLen = PGA460_PackCommand(pga460TxBuffer, params);
    if (msgLen == 0)
    {
        return FALSE;
    }

    if (cmdInfo->expectsResponse)
    {
        size_t respLen = RESP_METADATA_BYTE_COUNT + cmdInfo->respDataLen;
        dma_load_resp_tcs(respLen);
    }

    /* protect the DMA while it's transferring */
    while(DMA_TSR016.B.HTRE && !DMA_CHCSR016.B.ICH);

    dma_load_cmd_tcs(msgLen);

    TxComplete = FALSE;
    __disable();
    pga460CommState = ST_IN_FLIGHT;
    activeCommand = params->cmdType;
    __enable();
    asclin_start_tx();

    uint64 start = IfxStm_get(&MODULE_STM2);

    if (cmdInfo->expectsResponse)
    {
        while (!TxComplete || pga460CommState == ST_IN_FLIGHT)
        {
            uint64 now = IfxStm_get(&MODULE_STM2);
            uint64 diff = now - start;
            if (diff >= timeoutMs * IFX_CFG_STM_TICKS_PER_MS)
            {
                break;
            }
        }

        rc = ((TxComplete) && (pga460CommState == ST_CMD_DONE));

        if (!rc)
        {
            pga460CommState = ST_CMD_ERR;
        }
        else
        {
            pga460CommState = ST_IDLE;
        }
    }
    else
    {
        while (pga460CommState != ST_CMD_DONE)
        {
            uint64 now = IfxStm_get(&MODULE_STM2);
            uint64 diff = now - start;
            if (diff >= timeoutMs * IFX_CFG_STM_TICKS_PER_MS)
            {
                break;
            }
        }

        rc = TxComplete;
    }

    TxComplete = FALSE;

    return rc;
}

void PGA460_ProcessFrame(void)
{
    if (pga460CommState == ST_CMD_DONE)
    {
        pga460CommState = ST_IDLE;
    }
}

boolean PGA460_isAvailable(void)
{
    return (pga460CommState == ST_IDLE);
}

void   PGA460_TimeVaryingGainBulkWriteBlocking(uint64 timeoutMs)
{
    PGA460_CommandParams params = {
            .cmdType = PGA460_COMMAND_TVG_BULK_WRITE,
            .pga_addr = PGA460_ADDR,
            .dataLen = 7
    };

    params.data[0] = pga460shadow.TVGAIN0.U;
    params.data[1] = pga460shadow.TVGAIN1.U;
    params.data[2] = pga460shadow.TVGAIN2.U;
    params.data[3] = pga460shadow.TVGAIN3.U;
    params.data[4] = pga460shadow.TVGAIN4.U;
    params.data[5] = pga460shadow.TVGAIN5.U;
    params.data[6] = pga460shadow.TVGAIN6.U;

    PGA460_SendCommandBlocking(&params, timeoutMs);

}

void PGA460_RegisterReadAsync(uint8 reg_addr)
{
    PGA460_CommandParams params = {
        .cmdType = PGA460_COMMAND_REGISTER_READ,
        .pga_addr = PGA460_ADDR,
        .dataLen = 1
    };
    params.data[0] = reg_addr;

    PGA460_SendCommandAsync(&params);
}

void PGA460_RegisterWriteAsync(uint8 reg_addr, uint8 data)
{
    PGA460_CommandParams params = {
        .cmdType = PGA460_COMMAND_REGISTER_WRITE,
        .pga_addr = PGA460_ADDR,
        .dataLen = 2
    };
    params.data[0] = reg_addr;
    params.data[1] = data;

    PGA460_SendCommandAsync(&params);
}


boolean PGA460_RegisterReadBlocking(uint8 reg_addr, uint64 timeoutMs)
{
    PGA460_CommandParams params = {
        .cmdType = PGA460_COMMAND_REGISTER_READ,
        .pga_addr = PGA460_ADDR,
        .dataLen = 1
    };
    params.data[0] = reg_addr;

    return PGA460_SendCommandBlocking(&params, timeoutMs);
}

boolean PGA460_RegisterWriteBlocking(uint8 reg_addr, uint8 data, uint64 timeoutMs)
{
    PGA460_CommandParams params = {
        .cmdType = PGA460_COMMAND_REGISTER_WRITE,
        .pga_addr = PGA460_ADDR,
        .dataLen = 2
    };
    params.data[0] = reg_addr;
    params.data[1] = data;

    return PGA460_SendCommandBlocking(&params, timeoutMs);
}


void PGA460_EEPROMRead(uint64 timeoutMs)
{
    PGA460_CommandParams params = {
        .cmdType = PGA460_COMMAND_EEPROM_BULK_READ,
        .pga_addr = PGA460_ADDR,
        .dataLen = 0
    };

    PGA460_SendCommandBlocking(&params, timeoutMs);
}


void PGA460_BurstListenP1(void)
{
    PGA460_CommandParams params = {
        .cmdType = PGA460_COMMAND_BURST_AND_LISTEN_PRESET_1,
        .pga_addr = PGA460_ADDR,
        .dataLen = 1
    };
    params.data[0] = OBJECTS_TO_DETECT;

    PGA460_SendCommandAsync(&params);
}


void PGA460_InitDecoupling(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.DECPL_TEMP.B.AFE_GAIN_RNG   = config->afeGainRange;
    pga460shadow.DECPL_TEMP.B.LPM_EN         = config->lowPowerMode;
    pga460shadow.DECPL_TEMP.B.DECPL_TEMP_SEL = config->decoupleTimeTempSel;
    pga460shadow.DECPL_TEMP.B.DECPL_T        = config->decoupleTime;
}

void     PGA460_InitTVGs(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.TVGAIN0.B.TVG_T0 = config->tvgT0;
    pga460shadow.TVGAIN0.B.TVG_T1 = config->tvgT1;
    pga460shadow.TVGAIN1.B.TVG_T2 = config->tvgT2;
    pga460shadow.TVGAIN1.B.TVG_T3 = config->tvgT3;
    pga460shadow.TVGAIN2.B.TVG_T4 = config->tvgT4;
    pga460shadow.TVGAIN2.B.TVG_T5 = config->tvgT5;
    pga460shadow.TVGAIN3.B.TVG_G1 = config->tvgG1;
    pga460shadow.TVGAIN3.B.TVG_G2 = (config->tvgG2 & 0x3);
    pga460shadow.TVGAIN4.B.TVG_G2 = (config->tvgG2 & 0xF);
    pga460shadow.TVGAIN4.B.TVG_G3 = (config->tvgG3 & 0xF);
    pga460shadow.TVGAIN5.B.TVG_G3 = (config->tvgG3 & 0x3);
    pga460shadow.TVGAIN5.B.TVG_G4 = config->tvgG4;
    pga460shadow.TVGAIN6.B.TVG_G5 = config->tvgG5;
    pga460shadow.TVGAIN6.B.FREQ_SHIFT = config->freqShift;

}

void    PGA460_SetInitialGain(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.INIT_GAIN.B.GAIN_INIT = config->initGain;
    pga460shadow.INIT_GAIN.B.BPF_BW    = config->bpfBandwidth;
}

void    PGA460_SetBurstFrequency(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.FREQUENCY.B.FREQUENCY = config->burstFrequency;
}

void    PGA460_SetNumberBurstPulsesP1(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.PULSE_P1.B.P1_PULSE = config->burstPulseCountP1;
}

void    PGA460_SetCurrentLimitP1(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.CURR_LIM_P1.B.CURR_LIM1 = config->currentLimitP1;
}

void    PGA460_SetLPFCutoffFrequency(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.CURR_LIM_P2.B.LPF_CO = config->lpfCutoffFreq;
}

void    PGA460_SetRecordTimeLengthP1(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.REC_LENGTH.B.P1_REC = config->recordTimeLengthP1;
}

void     PGA460_SetDeglitchTime(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.DEADTIME.B.THR_CMP_DEGLTCH = config->deglitchTime;
    pga460shadow.DEADTIME.B.PULSE_DT = config->burstPulseDeadtime;
}

void    PGA460_SetNonlinearScalingP1(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.SAT_FDIAG_TH.B.P1_NLS_EN = config->P1_NLS;
}

void    PGA460_SetSaturationDiagThreshold(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.SAT_FDIAG_TH.B.SAT_TH = config->saturationDiagThreshold;
}

void PGA460_SetFrequencyDiagErrThreshold(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.SAT_FDIAG_TH.B.FDIAG_ERR_TH = config->frequencyDiagAbsErrTimeThreshold;
}

void PGA460_SetDSPScaling(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.DSP_SCALE.B.NOISE_LVL = config->noiseLevel;
    pga460shadow.DSP_SCALE.B.SCALE_K   = config->Scale_K;
    pga460shadow.DSP_SCALE.B.SCALE_N   = config->Scale_N;
}

void   PGA460_SetDigitalGainsP1(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.P1_GAIN_CTRL.B.P1_DIG_GAIN_LR = config->LR_DigitalGainCtrl;
}

void    PGA460_InitThresholds(const PGA460_Config* config)
{
    IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, config != NULL_PTR);

    pga460shadow.P1_THR_0.B.TH_P1_T1   = config->thrT1;
    pga460shadow.P1_THR_0.B.TH_P1_T2   = config->thrT2;
    pga460shadow.P1_THR_1.B.TH_P1_T3   = config->thrT3;
    pga460shadow.P1_THR_1.B.TH_P1_T4   = config->thrT4;
    pga460shadow.P1_THR_2.B.TH_P1_T5   = config->thrT5;
    pga460shadow.P1_THR_2.B.TH_P1_T6   = config->thrT6;
    pga460shadow.P1_THR_3.B.TH_P1_T7   = config->thrT7;
    pga460shadow.P1_THR_3.B.TH_P1_T8   = config->thrT8;
    pga460shadow.P1_THR_4.B.TH_P1_T9   = config->thrT9;
    pga460shadow.P1_THR_4.B.TH_P1_T10  = config->thrT10;
    pga460shadow.P1_THR_5.B.TH_P1_T11  = config->thrT11;
    pga460shadow.P1_THR_5.B.TH_P1_T12  = config->thrT12;
    pga460shadow.P1_THR_6.U            = config->thrL1L2;
    pga460shadow.P1_THR_7.U            = config->thrL3;
    pga460shadow.P1_THR_8.U            = config->thrL4;
    pga460shadow.P1_THR_9.U            = config->thrL5L6;
    pga460shadow.P1_THR_10.U           = config->thrL7L8;
    pga460shadow.P1_THR_11.B.TH_P1_L9  = config->thrL9;
    pga460shadow.P1_THR_12.B.TH_P1_L10 = config->thrL10;

}

boolean  PGA460_burnEEPROM()
{
    return FALSE;
}

