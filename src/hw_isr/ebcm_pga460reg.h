/******************************************************************************
 * @file    ebcm_pga460.h
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

#ifndef HW_ISR_EBCM_PGA460REG_H_
#define HW_ISR_EBCM_PGA460REG_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/


/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define PGA460_DEF_USER_DATA(x) \
    typedef struct _PGA460_USER_DATA##x_Bits \
    { \
        PGA460_UReg_8Bit USER_##x:8; \
    } PGA460_USER_DATA##x_Bits; \
    \
    typedef union \
    { \
        PGA460_UReg_8Bit U; \
        PGA460_USER_DATA##x_Bits B; \
    } PGA_USER_DATA_##x



#define PGA460_DEF_USER_DATA_REG(x) \

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/**********************************************************************************************************************/
/*-------------------------------------------------Type definitions---------------------------------------------------*/
/**********************************************************************************************************************/

typedef unsigned char   PGA460_UReg_8Bit;
typedef unsigned short  PGA460_UReg_16Bit;
typedef unsigned int    PGA460_UReg_32Bit;
typedef signed char     PGA460_SReg_8Bit;
typedef signed short    PGA460_SReg_16Bit;
typedef signed int      PGA460_SReg_32Bit;

/**< \brief USER_DATA0 register */
PGA460_DEF_USER_DATA(0);

/**< \brief USER_DATA1 register */
PGA460_DEF_USER_DATA(1);

/**< \brief USER_DATA2 register */
PGA460_DEF_USER_DATA(2);

/**< \brief USER_DATA3 register */
PGA460_DEF_USER_DATA(3);

/**< \brief USER_DATA4 register */
PGA460_DEF_USER_DATA(4);

/**< \brief USER_DATA5 register */
PGA460_DEF_USER_DATA(5);

/**< \brief USER_DATA6 register */
PGA460_DEF_USER_DATA(6);

/**< \brief USER_DATA7 register */
PGA460_DEF_USER_DATA(7);

/**< \brief USER_DATA8 register */
PGA460_DEF_USER_DATA(8);

/**< \brief USER_DATA9 register */
PGA460_DEF_USER_DATA(9);

/**< \brief USER_DATA10 register */
PGA460_DEF_USER_DATA(10);

/**< \brief USER_DATA11 register */
PGA460_DEF_USER_DATA(11);

/**< \brief USER_DATA12 register */
PGA460_DEF_USER_DATA(12);

/**< \brief USER_DATA13 register */
PGA460_DEF_USER_DATA(13);

/**< \brief USER_DATA14 register */
PGA460_DEF_USER_DATA(14);

/**< \brief USER_DATA15 register */
PGA460_DEF_USER_DATA(15);

/**< \brief USER_DATA16 register */
PGA460_DEF_USER_DATA(16);

/**< \brief USER_DATA17 register */
PGA460_DEF_USER_DATA(17);

/**< \brief USER_DATA18 register */
PGA460_DEF_USER_DATA(18);

/**< \brief USER_DATA19 register */
PGA460_DEF_USER_DATA(19);


typedef struct _PGA460_USER_DATA_Bits
{
        PGA460_UReg_8Bit USER:8;
} PGA460_USER_DATA_Bits;

/** \brief TVGAIN0 Register   */
typedef struct _PGA460_TVGAIN0_Bits
{
        PGA460_UReg_8Bit     TVG_T1:4;      /**< \brief [0:3] TV gain T1 */
        PGA460_UReg_8Bit     TVG_T0:4;      /**< \brief [4:7] TV gain T0 */

} PGA460_TVGAIN0_Bits;

/** \brief TVGAIN1 Register   */
typedef struct _PGA460_TVGAIN1_Bits
{
        PGA460_UReg_8Bit     TVG_T3:4;      /**< \brief [0:3] TV gain T3 */
        PGA460_UReg_8Bit     TVG_T2:4;      /**< \brief [4:7] TV gain T2 */

} PGA460_TVGAIN1_Bits;


/** \brief TVGAIN2 Register   */
typedef struct _PGA460_TVGAIN2_Bits
{
        PGA460_UReg_8Bit     TVG_T5:4;      /**< \brief [0:3] TV gain T5 */
        PGA460_UReg_8Bit     TVG_T4:4;      /**< \brief [4:7] TV gain T4 */

} PGA460_TVGAIN2_Bits;


/** \brief TVGAIN3 Register   */
typedef struct _PGA460_TVGAIN3_Bits
{
        PGA460_UReg_8Bit     TVG_G2:2;      /**< \brief [0:1] TV gain G2 */
        PGA460_UReg_8Bit     TVG_G1:6;      /**< \brief [2:7] TV gain G1 */

} PGA460_TVGAIN3_Bits;


/** \brief TVGAIN4 Register   */
typedef struct _PGA460_TVGAIN4_Bits
{
        PGA460_UReg_8Bit     TVG_G3:4;      /**< \brief [0:3] TV gain G3 */
        PGA460_UReg_8Bit     TVG_G2:4;      /**< \brief [4:7] TV gain G2 */

} PGA460_TVGAIN4_Bits;


/** \brief TVGAIN5 Register   */
typedef struct _PGA460_TVGAIN5_Bits
{
        PGA460_UReg_8Bit     TVG_G4:6;      /**< \brief [0:5] TV gain G4 */
        PGA460_UReg_8Bit     TVG_G3:2;      /**< \brief [6:7] TV gain G3 */

} PGA460_TVGAIN5_Bits;

/** \brief TVGAIN6 Register   */
typedef struct _PGA460_TVGAIN6_Bits
{
        PGA460_UReg_8Bit     FREQ_SHIFT:1;  /**< \brief [0] Burst frequency range shift  */
        PGA460_UReg_8Bit     RSV:1;         /**< \brief [1]   \Reserved */
        PGA460_UReg_8Bit     TVG_G5:6;      /**< \brief [2:7] TV gain G5 */

} PGA460_TVGAIN6_Bits;

/** \brief INIT_GAIN Register   */
typedef struct _PGA460_INIT_GAIN_Bits
{
        PGA460_UReg_8Bit    GAIN_INIT:6;   /**< \brief [0:5] Initial AFE Gain. See formula in PGA460 datasheet */
        PGA460_UReg_8Bit    BPF_BW:2;      /**< \brief [6:7] Digital bandpass filter bandwidth. See formula in PGA460 datasheet */
} PGA460_INIT_GAIN_Bits;

/** \brief FREQUENCY Register   */
typedef struct _PGA460_FREQUENCY_Bits
{
        PGA460_UReg_8Bit    FREQUENCY:8;   /**< \brief [0:7] Burst frequency equation parameter */
} PGA460_FREQUENCY_Bits;

/** \brief DEADTIME Register */
typedef struct _PGA460_DEADTIME_Bits
{
        PGA460_UReg_8Bit   PULSE_DT:4;          /**< \brief [0:3] Burst Pulse Dead-Time */
        PGA460_UReg_8Bit   THR_CMP_DEGLTCH:4;   /**< \brief [4:7] Threshold level comparator de-glitch period */
} PGA460_DEADTIME_Bits;

/** \brief PULSE_P1 Register */
typedef struct _PGA460_PULSE_P1_Bits
{
        PGA460_UReg_8Bit   P1_PULSE:5;      /**< \brief [0:4] Number of burst pulses for P1 */
        PGA460_UReg_8Bit   IO_DIS:1;        /**< \brief [5] Disable I/O pin transceiver, 0=enabled, 1=disabled IFF IO_IF_SEL=0 */
        PGA460_UReg_8Bit   UART_DIAG:1;     /**< \brief [6] UART diagnostic page selection, 0=UART diag, 1=System diag */
        PGA460_UReg_8Bit   IO_IF_SEL:1;     /**< \brief [7] Interface selection, 0=TBI, 1=UART */
} PGA460_PULSE_P1_Bits;


/** \brief PULSE_P2 Register */
typedef struct _PGA460_PULSE_P2_Bits
{
        PGA460_UReg_8Bit   P2_PULSE:4;    /**< \brief [0:3] Number of burst pulses for Preset2 */
        PGA460_UReg_8Bit   UART_ADDR:4;   /**< \brief [4:7] UART interface address */
} PGA460_PULSE_P2_Bits;

/** \brief CURR_LIM_P1 Register */
typedef struct _PGA460_CURR_LIM_P1_Bits
{
        PGA460_UReg_8Bit   CURR_LIM1:4;    /**< \brief [0:5] Number of burst pulses for Preset2 */
        PGA460_UReg_8Bit   UNUSED:1;       /**< \brief [6] Unused */
        PGA460_UReg_8Bit   DIS_CL:1;       /**< \brief [7] UART interface address */
} PGA460_CURR_LIM_P1_Bits;

/** \brief CURR_LIM_P2 Register */
typedef struct _PGA460_CURR_LIM_P1_Bits
{
        PGA460_UReg_8Bit  CURR_LIM2:6;     /**< \brief [0:5] Driver current limit for Preset2 */
        PGA460_UReg_8Bit  LPF_CO:2;        /**< \brief [6:7] Lowpass filter cutoff frequency */

} PGA460_CURR_LIM_P2_Bits;

/** \brief REC_LENGTH Register */
typedef struct _PGA460_REC_LENGTH_Bits
{
        PGA460_UReg_8Bit    P2_REC:4;     /**< \brief [0:3] Preset2 record time length */
        PGA460_UReg_8Bit    P1_REC:4;     /**< \brief [4:7] Preset1 record time length */
} PGA460_REC_LENGTH_Bits;

/** \brief FREQ_DIAG Register */
typedef struct _PGA460_FREQ_DIAG_Bits
{
        PGA460_UReg_8Bit   FDIAG_START:4;   /**< \brief [0:3] Frequency diagnostic start time */
        PGA460_UReg_8Bit   FDIAG_LEN:4;     /**< \brief [4:7] Frequency diagnostic window length */
} PGA460_FREQ_DIAG_Bits;

/** \brief SAT_FDIAG_TH Register */
typedef struct _PGA460_SAT_FDIAG_TH_Bits
{
        PGA460_UReg_8Bit   P1_NLS_EN:1;         /**< \brief [0] Preset1 Non-linear scaling */
        PGA460_UReg_8Bit   SAT_TH:4;            /**< \brief [1:4] Saturation diagnostic threshold level */
        PGA460_UReg_8Bit   FDIAG_ERR_TH:3;      /**< \brief [5:7] Frequency diagnostic absolute error time threshold */
} PGA460_SAT_FDIAG_TH_Bits;

/** \brief FVOLT_DEC Register */
typedef struct _PGA460_FVOLT_DEC_Bits
{
        PGA460_UReg_8Bit FVOLT_ERR_TH:3;        /**< \brief [0:2] Voltage diagnostic error threshold */
        PGA460_UReg_8Bit LPM_TMR:2;             /**< \brief [3:4] Low power mode enter time */
        PGA460_UReg_8Bit VPWR_OV_TH:2;          /**< \brief [5:6] VPWR over voltage threshold select */
        PGA460_UReg_8Bit P2_NLS_EN:1;           /**< \brief [7]   Enable Preset2 non-linear scaling */
} PGA460_FVOLT_DEC_Bits;

/** \brief DECPL_TEMP Register */
typedef struct _PGA460_DECPL_TEMP_Bits
{
        PGA460_UReg_8Bit DECPL_T:4;             /**< \brief [0:3] Secondary decouple time / temperature decouple */
        PGA460_UReg_8Bit DECPL_TEMP_SEL:1;      /**< \brief [4]   Decouple Time / Temperature Select */
        PGA460_UReg_8Bit LPM_EN:1;              /**< \brief [5]   Low Power mode enable */
        PGA460_UReg_8Bit AFE_GAIN_RNG:2;        /**< \brief [6:7] AFE gain range selection code */
} PGA460_DECPL_TEMP_Bits;

/** \brief DSP_SCALE Register */
typedef struct _PGA4640_DSP_SCALE_Bits
{
        PGA460_UReg_8Bit SCALE_N:2;             /**< \brief [0:1] Select starting threhold level point from which non-linear gain is applied */
        PGA460_UReg_8Bit SCALE_K:1;             /**< \brief [2]   Non-linear scaling exponent selection */
        PGA460_UReg_8Bit NOISE_LVL:5;           /**< \brief [3:7] Noist level configuration */
} PGA460_DSP_SCALE_Bits;

/** \brief TEMP_TRIM Register */
typedef struct _PGA460_TEMP_TRIM_Bits
{
        PGA460_UReg_8Bit TEMP_OFF:4;            /**< \brief [0:3] Temperature scaling offset */
        PGA460_SReg_8Bit TEMP_GAIN:4;           /**< \brief [4:7] Temperature scaling gain */
} PGA460_TEMP_TRIM_Bits;

/** \brief P1_GAIN_CTRL Register */
typedef struct _PGA460_P1_GAIN_CTRL_Bits
{
        PGA460_UReg_8Bit P1_DIG_GAIN_SR:3;           /**< \brief [0:3] Preset1 digital short range gain */
        PGA460_UReg_8Bit P1_DIG_GAIN_LR:3;           /**< \brief [4:7] Preset1 digital long range gain   */
        PGA460_UReg_8Bit P1_DIG_GAIN_LR_ST:2;        /**< \brief [6:7] Starting Preset1 threshold */
} PGA460_P1_GAIN_CTRL_Bits;

/** \brief P2_GAIN_CTRL Register */
typedef struct _PGA460_P2_GAIN_CTRL_Bits
{
        PGA460_UReg_8Bit P2_DIG_GAIN_SR:3;           /**< \brief [0:3] Preset2 digital short range gain */
        PGA460_UReg_8Bit P2_DIG_GAIN_LR:3;           /**< \brief [4:7] Preset2 digital long range gain   */
        PGA460_UReg_8Bit P2_DIG_GAIN_LR_ST:2;        /**< \brief [6:7] Starting Preset2 threshold */
} PGA460_P2_GAIN_CTRL_Bits;

/** \brief EE_CRC Register */
typedef struct _PGA460_EE_CRC_Bits
{
        PGA460_UReg_8Bit EE_CRC:8;                  /**< \brief [0:7] User EEPROM space data CRC value */
} PGA460_EE_CRC_Bits;

/** \brief EE_CTRL Register */
typedef struct _PGA460_EE_CTRL_Bits
{
        PGA460_UReg_8Bit EE_PRGM:1;                  /**< \brief [0] EEPROM program trigger, 0=disabled, 1=program data to EEPROM */
        PGA460_UReg_8Bit EE_RLOAD:1;                 /**< \brief [1] EEPROM reload trigger, 0=disabled, 1=reload data from EEPROM */
        PGA460_UReg_8Bit EE_PRGM_OK:1;               /**< \brief [2] EEPROM programming status: 0=program failure, 1=programming successful */
        PGA460_UReg_8Bit EE_UNLCK:4;                 /**< \brief [3:6] EEPROM program enable unlock passcode register */
        PGA460_UReg_8Bit DATADUMP_EN:1;              /**< \brief [7] Data Dump Enable bit, 0=disable, 1=enabled */
} PGA460_EE_CTRL_Bits;

/** \brief BPF_A2_MSB Register */
typedef struct _PGA460_BPF_A2_MSB_Bits
{
        PGA460_UReg_8Bit BPF_A2_MSB:8;        /**< \brief Bandpass filter A2 coefficient MSB */
} PGA460_BPF_A2_MSB_Bits;

/** \brief BPF_A2_LSB Register */
typedef struct _PGA460_BPF_A2_LSB_Bits
{
        PGA460_UReg_8Bit BPF_A2_LSB:8;        /**< \brief Bandpass filter A2 coefficient LSB */
} PGA460_BPF_A2_LSB_Bits;

/** \brief BPF_A3_MSB Register */
typedef struct _PGA460_BPF_A3_MSB_Bits
{
        PGA460_UReg_8Bit BPF_A3_MSB:8;        /**< \brief Bandpass filter A3 coefficient MSB */
} PGA460_BPF_A3_MSB_Bits;

/** \brief BPF_A3_LSB Register */
typedef struct _PGA460_BPF_A3_LSB_Bits
{
        PGA460_UReg_8Bit BPF_A3_LSB:8;        /**< \brief Bandpass filter A3 coefficient LSB */
} PGA460_BPF_A3_LSB_Bits;

/** \brief BPF_B1_MSB Register */
typedef struct _PGA460_BPF_B1_MSB_Bits
{
        PGA460_UReg_8Bit BPF_B1_MSB:8;        /**< \brief Bandpass filter B1 coefficient MSB */
} PGA460_BPF_B1_MSB_Bits;

/** \brief BPF_B1_LSB Register */
typedef struct _PGA460_BPF_B1_LSB_Bits
{
        PGA460_UReg_8Bit BPF_B1_LSB:8;        /**< \brief Bandpass filter B1 coefficient LSB */
} PGA460_BPF_B1_LSB_Bits;

/** \brief LPF_A2_MSB Register */
typedef struct _PGA460_LPF_A2_MSB_Bits
{
        PGA460_UReg_8Bit  LPF_A2_MSB:8;        /**< \brief Lowpass filter A2 coefficient MSB */
} PGA460_LPF_A2_MSB_Bits;

/** \brief LPF_A2_LSB Register */
typedef struct _PGA460_LPF_A2_LSB_Bits
{
        PGA460_UReg_8Bit  LPF_A2_LSB:8;        /**< \brief Lowpass filter A2 coefficient LSB */
} PGA460_LPF_A2_LSB_Bits;

/** \brief LPF_B1_MSB Register */
typedef struct _PGA460_LPF_B1_MSB_Bits
{
        PGA460_UReg_8Bit  LPF_B1_MSB:8;        /**< \brief Lowpass filter B1 coefficient MSB */
} PGA460_LPF_B1_MSB_Bits;

/** \brief LPF_B1_LSB Register */
typedef struct _PGA460_LPF_B1_LSB_Bits
{
        PGA460_UReg_8Bit  LPF_B1_LSB:8;        /**< \brief Lowpass filter B1 coefficient LSB */
} PGA460_LPF_B1_LSB_Bits;

/** \brief TEST_MUX Register */
typedef struct _PGA460_TEST_MUX_Bits
{
        PGA460_UReg_8Bit DP_MUX:3;          /**< \brief [0:2] Data path multiplexer source select */
        PGA460_UReg_8Bit SAMPLE_SEL:1;      /**< \brief [3]   Data path sample select */
        PGA460_UReg_8Bit RESERVED:1;        /**< \brief [4]   RESERVED */
        PGA460_UReg_8Bit TEST_MUX:3;        /**< \brief [5:7] Multiplexer output on the TEST Pin */
} PGA460_TEST_MUX_Bits;

/** \brief DEV_STAT0 Register */
typedef struct _PGA460_DEV_STAT0_Bits
{
        PGA460_UReg_8Bit TRIM_CRC_ERR:1;    /**< \brief [0] Trim EEPROM space data CRC error status */
        PGA460_UReg_8Bit EE_CRC_ERR:1;      /**< \brief [1] User EEPROM space data CRC error status */
        PGA460_UReg_8Bit THR_CRC_ERR:1;     /**< \brief [2] Threshold map config register data CRC error status */
        PGA460_UReg_8Bit CMW_WU_ERR:1;      /**< \brief [3] Wakeup error status */
        PGA460_UReg_8Bit OPT_ID:2;          /**< \brief [4:5] Device Option Identification */
        PGA460_UReg_8Bit REV_ID:2;          /**< \brief [6:7] Device Revision Identification */
} PGA460_DEV_STAT0_Bits;

/** \brief DEV_STAT1 Register */
typedef struct _PGA460_DEV_STAT1_Bits
{
        PGA460_UReg_8Bit VPWR_UV:1;         /**< \brief [0] VPWR pin under voltage status */
        PGA460_UReg_8Bit VPWR_OV:1;         /**< \brief [1] VPWR pin over voltage status */
        PGA460_UReg_8Bit AVDD_UV:1;         /**< \brief [2] AVDD pin under voltage status */
        PGA460_UReg_8Bit AVDD_OV:1;         /**< \brief [3] AVDD pin over voltage status */
        PGA460_UReg_8Bit IOREG_UV:1;        /**< \brief [4] IOREG pin under voltage status */
        PGA460_UReg_8Bit IOREG_OV:1;        /**< \brief [5] IOREG pin over voltage status */
        PGA460_UReg_8Bit TSD_PROT:1;        /**< \brief [6] Thermal shut-down protection status */
        PGA460_UReg_8Bit RESERVED:1;        /**< \brief [7] RESERVED */
} PGA460_DEV_STAT1_Bits;

/** \brief P1_THR_0 Register */
typedef struct _PGA460_P1_THR_0_Bits
{
        PGA460_UReg_8Bit TH_P1_T2:4;        /**< \brief [0:3] Preset1 Threshold T2 delta time */
        PGA460_UReg_8Bit TH_P1_T1:4;        /**< \brief [4:7] Preset1 Threshold T1 delta time */
} PGA460_P1_THR_0_Bits;

/** \brief P1_THR_1 Register */
typedef struct _PGA460_P1_THR_1_Bits
{
        PGA460_UReg_8Bit TH_P1_T4:4;        /**< \brief [0:3] Preset1 Threshold T4 delta time */
        PGA460_UReg_8Bit TH_P1_T3:4;        /**< \brief [4:7] Preset1 Threshold T3 delta time */
} PGA460_P1_THR_1_Bits;

/** \brief P1_THR_2 Register */
typedef struct _PGA460_P1_THR_2_Bits
{
        PGA460_UReg_8Bit TH_P1_T6:4;        /**< \brief [0:3] Preset1 Threshold T6 delta time */
        PGA460_UReg_8Bit TH_P1_T5:4;        /**< \brief [4:7] Preset1 Threshold T5 delta time */
} PGA460_P1_THR_2_Bits;

/** \brief P1_THR_3 Register */
typedef struct _PGA460_P1_THR_3_Bits
{
        PGA460_UReg_8Bit TH_P1_T8:4;        /**< \brief [0:3] Preset1 Threshold T8 delta time */
        PGA460_UReg_8Bit TH_P1_T7:4;        /**< \brief [4:7] Preset1 Threshold T7 delta time */
} PGA460_P1_THR_3_Bits;

/** \brief P1_THR_4 Register */
typedef struct _PGA460_P1_THR_4_Bits
{
        PGA460_UReg_8Bit TH_P1_T10:4;        /**< \brief [0:3] Preset1 Threshold T10 delta time */
        PGA460_UReg_8Bit TH_P1_T9:4;         /**< \brief [4:7] Preset1 Threshold T9 delta time */
} PGA460_P1_THR_4_Bits;

/** \brief P1_THR_5 Register */
typedef struct _PGA460_P1_THR_5_Bits
{
        PGA460_UReg_8Bit TH_P1_T12:4;        /**< \brief [0:3] Preset1 Threshold T12 delta time */
        PGA460_UReg_8Bit TH_P1_T11:4;        /**< \brief [4:7] Preset1 Threshold T11 delta time */
} PGA460_P1_THR_5_Bits;

/** \brief P1_THR_6 Register */
typedef struct _PGA460_P1_THR_6_Bits
{
        PGA460_UReg_8Bit TH_P1_L2:3;        /**< \brief [0:2] Preset1 Threshold L2 delta time (Bit4 to Bit2) */
        PGA460_UReg_8Bit TH_P1_L1:5;        /**< \brief [3:7] Preset1 Threshold L1 delta time */
} PGA460_P1_THR_6_Bits;

/** \brief P1_THR_7 Register */
typedef struct _PGA460_P1_THR_7_Bits
{
        PGA460_UReg_8Bit TH_P1_L4:1;        /**< \brief [0] Preset1 Threshold L4 (Bit4)   */
        PGA460_UReg_8Bit TH_P1_L3:5;        /**< \brief [1:5] Preset1 Threshold L3        */
        PGA460_UReg_8Bit TH_P1_L2:2;        /**< \brief [6:7] Preset1 Threshold L2 (Bit1 to Bit0)  */

} PGA460_P1_THR_7_Bits;

/** \brief P1_THR_8 Register */
typedef struct _PGA460_P1_THR_8_Bits
{
        PGA460_UReg_8Bit TH_P1_L5:4;        /**< \brief [0:3] Preset1 Threshold L5 delta time (Bit4 to Bit1) */
        PGA460_UReg_8Bit TH_P1_L4:4;        /**< \brief [4:7] Preset1 Threshold L4 delta time (Bit3 to Bit0) */
} PGA460_P1_THR_8_Bits;

/** \brief P1_THR_9 Register */
typedef struct _PGA460_P1_THR_9_Bits
{
        PGA460_UReg_8Bit TH_P1_L7:2;        /**< \brief [0] Preset1 Threshold L7 (Bit4 to Bit3)   */
        PGA460_UReg_8Bit TH_P1_L6:5;        /**< \brief [1:5] Preset1 Threshold L6         */
        PGA460_UReg_8Bit TH_P1_L5:1;        /**< \brief [6:7] Preset1 Threshold L5 (Bit0)  */

} PGA460_P1_THR_9_Bits;

/** \brief P1_THR_10 Register */
typedef struct _PGA460_P1_THR_10_Bits
{
        PGA460_UReg_8Bit TH_P1_L8:5;        /**< \brief [0:4] Preset1 Threshold L8 delta time  */
        PGA460_UReg_8Bit TH_P1_L7:3;        /**< \brief [5:7] Preset1 Threshold L7 delta time (Bit2 to Bit0) */
} PGA460_P1_THR_10_Bits;

/** \brief P1_THR_11 Register */
typedef struct _PGA460_P1_THR_11_Bits
{
        PGA460_UReg_8Bit TH_P1_L9:8;        /**< \brief Threshold L9 */
} PGA460_P1_THR_11_Bits;

/** \brief P1_THR_12 Register */
typedef struct _PGA460_P1_THR_12_Bits
{
        PGA460_UReg_8Bit TH_P1_L10:8;        /**< \brief Threshold L10 */
} PGA460_P1_THR_12_Bits;

/** \brief P1_THR_13 Register */
typedef struct _PGA460_P1_THR_13_Bits
{
        PGA460_UReg_8Bit TH_P1_L11:8;        /**< \brief Threshold L13 */
} PGA460_P1_THR_13_Bits;

/** \brief P1_THR_14 Register */
typedef struct _PGA460_P1_THR_14_Bits
{
        PGA460_UReg_8Bit TH_P1_L12:8;        /**< \brief Threshold L14 */
} PGA460_P1_THR_14_Bits;

/** \brief P1_THR_15 Register */
typedef struct _PGA460_P1_THR_15_Bits
{
        PGA460_UReg_8Bit TH_P1_OFF:4;        /**< \brief P1 Threshold offset */
        PGA460_UReg_8Bit RESERVED:4;         /**< \brief RESERVED            */
} PGA460_P1_THR_15_Bits;


/** \brief P2_THR_0 Register */
typedef struct _PGA460_P2_THR_0_Bits
{
        PGA460_UReg_8Bit TH_P2_T2:4;        /**< \brief [0:3] Preset2 Threshold T2 delta time */
        PGA460_UReg_8Bit TH_P2_T1:4;        /**< \brief [4:7] Preset2 Threshold T1 delta time */
} PGA460_P2_THR_0_Bits;

/** \brief P2_THR_1 Register */
typedef struct _PGA460_P2_THR_1_Bits
{
        PGA460_UReg_8Bit TH_P2_T4:4;        /**< \brief [0:3] Preset2 Threshold T4 delta time */
        PGA460_UReg_8Bit TH_P2_T3:4;        /**< \brief [4:7] Preset2 Threshold T3 delta time */
} PGA460_P2_THR_1_Bits;

/** \brief P2_THR_2 Register */
typedef struct _PGA460_P2_THR_2_Bits
{
        PGA460_UReg_8Bit TH_P2_T6:4;        /**< \brief [0:3] Preset2 Threshold T6 delta time */
        PGA460_UReg_8Bit TH_P2_T5:4;        /**< \brief [4:7] Preset2 Threshold T5 delta time */
} PGA460_P2_THR_2_Bits;

/** \brief P2_THR_3 Register */
typedef struct _PGA460_P2_THR_3_Bits
{
        PGA460_UReg_8Bit TH_P2_T8:4;        /**< \brief [0:3] Preset2 Threshold T8 delta time */
        PGA460_UReg_8Bit TH_P2_T7:4;        /**< \brief [4:7] Preset2 Threshold T7 delta time */
} PGA460_P2_THR_3_Bits;

/** \brief P2_THR_4 Register */
typedef struct _PGA460_P2_THR_4_Bits
{
        PGA460_UReg_8Bit TH_P2_T10:4;        /**< \brief [0:3] Preset2 Threshold T10 delta time */
        PGA460_UReg_8Bit TH_P2_T9:4;         /**< \brief [4:7] Preset2 Threshold T9 delta time */
} PGA460_P2_THR_4_Bits;

/** \brief P2_THR_5 Register */
typedef struct _PGA460_P2_THR_5_Bits
{
        PGA460_UReg_8Bit TH_P2_T12:4;        /**< \brief [0:3] Preset2 Threshold T12 delta time */
        PGA460_UReg_8Bit TH_P2_T11:4;        /**< \brief [4:7] Preset2 Threshold T11 delta time */
} PGA460_P2_THR_5_Bits;

/** \brief P2_THR_6 Register */
typedef struct _PGA460_P2_THR_6_Bits
{
        PGA460_UReg_8Bit TH_P2_L2:3;        /**< \brief [0:2] Preset2 Threshold L2 delta time (Bit4 to Bit2) */
        PGA460_UReg_8Bit TH_P2_L1:5;        /**< \brief [3:7] Preset2 Threshold L1 delta time */
} PGA460_P2_THR_6_Bits;

/** \brief P2_THR_7 Register */
typedef struct _PGA460_P2_THR_7_Bits
{
        PGA460_UReg_8Bit TH_P2_L4:1;        /**< \brief [0] Preset2 Threshold L4 (Bit4)   */
        PGA460_UReg_8Bit TH_P2_L3:5;        /**< \brief [1:5] Preset2 Threshold L3        */
        PGA460_UReg_8Bit TH_P2_L2:2;        /**< \brief [6:7] Preset2 Threshold L2 (Bit1 to Bit0)  */

} PGA460_P2_THR_7_Bits;

/** \brief P2_THR_8 Register */
typedef struct _PGA460_P2_THR_8_Bits
{
        PGA460_UReg_8Bit TH_P2_L5:4;        /**< \brief [0:3] Preset2 Threshold L5 delta time (Bit4 to Bit1) */
        PGA460_UReg_8Bit TH_P2_L4:4;        /**< \brief [4:7] Preset2 Threshold L4 delta time (Bit3 to Bit0) */
} PGA460_P2_THR_8_Bits;

/** \brief P2_THR_9 Register */
typedef struct _PGA460_P2_THR_9_Bits
{
        PGA460_UReg_8Bit TH_P2_L7:2;        /**< \brief [0] Preset2 Threshold L7 (Bit4 to Bit3)   */
        PGA460_UReg_8Bit TH_P2_L6:5;        /**< \brief [1:5] Preset2 Threshold L6         */
        PGA460_UReg_8Bit TH_P2_L5:1;        /**< \brief [6:7] Preset2 Threshold L5 (Bit0)  */

} PGA460_P2_THR_9_Bits;

/** \brief P2_THR_10 Register */
typedef struct _PGA460_P2_THR_10_Bits
{
        PGA460_UReg_8Bit TH_P2_L8:5;        /**< \brief [0:4] Preset2 Threshold L8 delta time  */
        PGA460_UReg_8Bit TH_P2_L7:3;        /**< \brief [5:7] Preset2 Threshold L7 delta time (Bit2 to Bit0) */
} PGA460_P2_THR_10_Bits;

/** \brief P2_THR_11 Register */
typedef struct _PGA460_P2_THR_11_Bits
{
        PGA460_UReg_8Bit TH_P2_L9:8;        /**< \brief Threshold L9 */
} PGA460_P2_THR_11_Bits;

/** \brief P2_THR_12 Register */
typedef struct _PGA460_P2_THR_12_Bits
{
        PGA460_UReg_8Bit TH_P2_L10:8;        /**< \brief Threshold L10 */
} PGA460_P2_THR_12_Bits;

/** \brief P2_THR_13 Register */
typedef struct _PGA460_P2_THR_13_Bits
{
        PGA460_UReg_8Bit TH_P2_L11:8;        /**< \brief Threshold L13 */
} PGA460_P2_THR_13_Bits;

/** \brief P2_THR_14 Register */
typedef struct _PGA460_P2_THR_14_Bits
{
        PGA460_UReg_8Bit TH_P2_L12:8;        /**< \brief Threshold L14 */
} PGA460_P2_THR_14_Bits;

/** \brief P2_THR_15 Register */
typedef struct _PGA460_P2_THR_15_Bits
{
        PGA460_UReg_8Bit TH_P2_OFF:4;        /**< \brief P2 Threshold offset */
        PGA460_UReg_8Bit RESERVED:4;         /**< \brief RESERVED            */
} PGA460_P2_THR_15_Bits;


/** \brief THR_CRC Register */
typedef struct _PGA460_THR_CRC_Bits
{
        PGA460_UReg_8Bit THR_CRC:8;        /**< \brief Threshold map configuration registers data CRC value. Read only */
} PGA460_THR_CRC_Bits;

/** \brief TVGAIN0 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN0_Bits  B;
} PGA460_TVGAIN0;

/** \brief TVGAIN1 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN1_Bits  B;
} PGA460_TVGAIN1;

/** \brief TVGAIN2 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN2_Bits  B;
} PGA460_TVGAIN2;

/** \brief TVGAIN3 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN3_Bits  B;
} PGA460_TVGAIN3;

/** \brief TVGAIN4 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN4_Bits  B;
} PGA460_TVGAIN4;

/** \brief TVGAIN5 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN5_Bits  B;
} PGA460_TVGAIN5;

/** \brief TVGAIN6 Register   */
typedef union
{
        PGA460_UReg_8Bit     U;
        PGA460_TVGAIN6_Bits  B;
} PGA460_TVGAIN6;

/** \brief INIT_GAIN Register   */
typedef union
{
        PGA460_UReg_8Bit      U;
        PGA460_INIT_GAIN_Bits B;
} PGA460_INIT_GAIN;

/** \brief FREQUENCY Register   */
typedef union
{
        PGA460_UReg_8Bit      U;
        PGA460_FREQUENCY_Bits B;
} PGA460_FREQUENCY;

/** \brief DEADTIME Register   */
typedef union
{
        PGA460_UReg_8Bit      U;
        PGA460_DEADTIME_Bits  B;
} PGA460_DEADTIME;


/** \brief PULSE_P1 Register   */
typedef union
{
        PGA460_UReg_8Bit      U;
        PGA460_PULSE_P1_Bits  B;
} PGA460_PULSE_P1;


/** \brief PULSE_P2 Register   */
typedef union
{
        PGA460_UReg_8Bit      U;
        PGA460_PULSE_P2_Bits  B;
} PGA460_PULSE_P2;

/** \brief CURR_LIM_P1 Register   */
typedef union
{
        PGA460_UReg_8Bit         U;
        PGA460_CURR_LIM_P1_Bits  B;
} PGA460_CURR_LIM_P1;

/** \brief CURR_LIM_P2 Register   */
typedef union
{
        PGA460_UReg_8Bit         U;
        PGA460_CURR_LIM_P2_Bits  B;
} PGA460_CURR_LIM_P2;

/** \brief REC_LENGTH Register   */
typedef union
{
        PGA460_UReg_8Bit         U;
        PGA460_REC_LENGTH_Bits   B;
} PGA460_REC_LENGTH;

/** \brief FREQ_DIAG Register   */
typedef union
{
        PGA460_UReg_8Bit         U;
        PGA460_FREQ_DIAG_Bits    B;
} PGA460_FREQ_DIAG;


/** \brief SAT_FDIAG_TH Register   */
typedef union
{
        PGA460_UReg_8Bit           U;
        PGA460_SAT_FDIAG_TH_Bits   B;
} PGA460_SAT_FDIAG_TH;

/** \brief FVOLT_DEC Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_FVOLT_DEC_Bits   B;
} PGA460_FVOLT_DEC;


/** \brief DECPL_TEMP Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_DECPL_TEMP_Bits   B;
} PGA460_DECPL_TEMP;

/** \brief DSP_SCALE Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_DSP_SCALE_Bits   B;
} PGA460_DSP_SCALE;

/** \brief TEMP_TRIM Register   */
typedef union
{
        PGA460_SReg_8Bit        I;
        PGA460_TEMP_TRIM_Bits   B;
} PGA460_TEMP_TRIM;


/** \brief P1_GAIN_CTRL Register   */
typedef union
{
        PGA460_UReg_8Bit           U;
        PGA460_P1_GAIN_CTRL_Bits   B;
} PGA460_P1_GAIN_CTRL;

/** \brief P2_GAIN_CTRL Register   */
typedef union
{
        PGA460_UReg_8Bit           U;
        PGA460_P2_GAIN_CTRL_Bits   B;
} PGA460_P2_GAIN_CTRL;

/** \brief P2_EE_CRC Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_EE_CRC_Bits      B;
} PGA460_EE_CRC;

/** \brief EE_CNTRL Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_EE_CRC_Bits      B;
} PGA460_EE_CNTRL;

/** \brief BPF_A2_MSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_BPF_A2_MSB_Bits  B;
} PGA460_BPF_A2_MSB;

/** \brief BPF_A2_LSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_BPF_A2_LSB_Bits  B;
} PGA460_BPF_A2_LSB;

/** \brief BPF_A3_MSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_BPF_A3_MSB_Bits  B;
} PGA460_BPF_A3_MSB;

/** \brief BPF_A3_LSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_BPF_A3_LSB_Bits  B;
} PGA460_BPF_A3_LSB;

/** \brief BPF_B1_MSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_BPF_B1_MSB_Bits  B;
} PGA460_BPF_B1_MSB;

/** \brief BPF_B1_LSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_BPF_B1_LSB_Bits  B;
} PGA460_BPF_B1_LSB;

/** \brief BPF_A2_MSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_LPF_A2_MSB_Bits  B;
} PGA460_LPF_A2_MSB;

/** \brief LPF_A2_LSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_LPF_A2_LSB_Bits  B;
} PGA460_LPF_A2_LSB;

/** \brief LPF_B1_MSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_LPF_B1_MSB_Bits  B;
} PGA460_LPF_B1_MSB;

/** \brief LPF_B1_LSB Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_LPF_B1_LSB_Bits  B;
} PGA460_LPF_B1_LSB;

/** \brief TEST_MUX Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_TEST_MUX_Bits    B;
} PGA460_TEST_MUX;

/** \brief DEV_STAT0 Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_DEV_STAT0_Bits   B;
} PGA460_DEV_STAT0;

/** \brief DEV_STAT1 Register   */
typedef union
{
        PGA460_UReg_8Bit        U;
        PGA460_DEV_STAT0_Bits   B;
} PGA460_DEV_STAT1;


#define PGA460_DEF_P1_THR_REG(n) \
    typedef union \
    { \
            PGA460_UReg_8Bit        U; \
            PGA460_P1_THR_##n_Bits  B; \
    } PGA460_P1_THR_##n

#define PGA460_DEF_P2_THR_REG(n) \
    typedef union \
    { \
            PGA460_UReg_8Bit        U; \
            PGA460_P2_THR_##n_Bits  B; \
    } PGA460_P1_THR_##n


PGA460_DEF_P1_THR_REG(0);
PGA460_DEF_P1_THR_REG(1);
PGA460_DEF_P1_THR_REG(2);
PGA460_DEF_P1_THR_REG(3);
PGA460_DEF_P1_THR_REG(4);
PGA460_DEF_P1_THR_REG(5);
PGA460_DEF_P1_THR_REG(6);
PGA460_DEF_P1_THR_REG(7);
PGA460_DEF_P1_THR_REG(8);
PGA460_DEF_P1_THR_REG(9);
PGA460_DEF_P1_THR_REG(10);
PGA460_DEF_P1_THR_REG(11);
PGA460_DEF_P1_THR_REG(12);
PGA460_DEF_P1_THR_REG(13);
PGA460_DEF_P1_THR_REG(14);
PGA460_DEF_P1_THR_REG(15);

PGA460_DEF_P2_THR_REG(0);
PGA460_DEF_P2_THR_REG(1);
PGA460_DEF_P2_THR_REG(2);
PGA460_DEF_P2_THR_REG(3);
PGA460_DEF_P2_THR_REG(4);
PGA460_DEF_P2_THR_REG(5);
PGA460_DEF_P2_THR_REG(6);
PGA460_DEF_P2_THR_REG(7);
PGA460_DEF_P2_THR_REG(8);
PGA460_DEF_P2_THR_REG(9);
PGA460_DEF_P2_THR_REG(10);
PGA460_DEF_P2_THR_REG(11);
PGA460_DEF_P2_THR_REG(12);
PGA460_DEF_P2_THR_REG(13);
PGA460_DEF_P2_THR_REG(14);
PGA460_DEF_P2_THR_REG(15);

typedef union
{
        PGA460_UReg_8Bit      U;
        PGA460_THR_CRC_Bits   B;
} PGA460_THR_CRC;

__attribute__((packed)) typedef volatile struct _PGA460_Reg
{

        PGA460_UReg_8Bit        FREQUENCY;


} PGA460_Reg;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
uint8 pga460_registerRead(uint8 addr);
uint8 pga460_registerWrite(uint8 addr, uint8 data);
void  pga460_initThresholds(void);


#endif /* HW_ISR_EBCM_PGA460REG_H_ */
