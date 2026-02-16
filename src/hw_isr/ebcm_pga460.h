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

#ifndef HW_ISR_EBCM_PGA460_H_
#define HW_ISR_EBCM_PGA460_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include <ebcm_pga460msg.h>
#include "ebcm_pga460reg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define NUM_USERDATA_REGS       20u

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

/** \brief PGA460 Preset Profile Selection */
typedef enum _PGA460_PresetType
{
    PGA460_NEAR_PROFILE = 0,
    PGA460_FAR_PROFILE,
    PGA460_MAX_PRESET
} PGA460_PresetType;

/** \brief Low Power Mode Configuration */
typedef enum _PGA460_LowPowerModeType
{
    PGA460_LPM_Disabled = 0,
    PGA460_LPM_Enabled
} PGA460_LowPowerModeType;

/** \brief Preset1 Non-linear scaling enable/disable */
typedef enum _PGA460_P1_NonLinearScalingType
{
    PGA460_P1_NLS_Disabled = 0,
    PGA460_P1_NLS_Enabled
} PGA460_P1_NonLinearScalingType;

/** \brief Disable current limiting */
typedef enum _PGA460_CurrentLimitDisabledType
{
    PGA460_CurrentLimitEnabled = 0,
    PGA460_CurrentLimitDisabled
} PGA460_CurrentLimitDisabledType;

/** \brief Decouple Time/Temperature Selection */
typedef enum _PGA460_DecoupleTimeTemperatureSel
{
    PGA460_TimeDecouple = 0,
    PGA460_TemperatureDecouple
} PGA460_DecoupleTimeTemperatureSel;

/** \brief Starting Preset1 threshold from which long range digital gain is applied */
typedef enum _PGA460_StartingLRDigitalGainControl
{
    PGA460_StartingDigitalGainLR_TH9 = 0,
    PGA460_StartingDigitalGainLR_TH10,
    PGA460_StartingDigitalGainLR_TH11,
    PGA460_StartingDigitalGainLR_TH12
} PGA460_LRStartingDigitalGainCtrl;

/** \brief Preset1 digital long range gain applied from selected long range threshold level
 *         point to the end of the record period applied to thresholds set by P1_DIG_GAIN_LR_ST */
typedef enum _PGA460_LRDigitalGainControl
{
    PGA460_LRDigitalGain_1x = 0,
    PGA460_LRDigitalGain_2x,
    PGA460_LRDigitalGain_4x,
    PGA460_LRDigitalGain_8x,
    PGA460_LRDigitalGain_16,
    PGA460_LRDigitalGain_32
} PGA460_LRDigitalGainCtrl;

/** \brief Preset1 digital short range gain applied from time zero to the
 *         start of the selected LR threshold point level */
typedef enum _PGA460_SRDigitalGainControl
{
    PGA460_SRDigitalGain_1x = 0,
    PGA460_SRDigitalGain_2x,
    PGA460_SRDigitalGain_4x,
    PGA460_SRDigitalGain_8x,
    PGA460_SRDigitalGain_16x,
    PGA460_SRDigitalGain_32x
} PGA460_SRDigitalGainCtrl;

typedef enum _PGA460_NonlinearScalingExponentSel
{
    PGA460_NonlinearScaling_1_5 = 0,
    PGA460_NonlinearScaling_2_0

} PGA460_NonlinearScalingExponentSel;

typedef enum _PGA460_NonlinearScalingStartingThr
{
    PGA460_NonlinearScalingStart_TH9 = 0,
    PGA460_NonlinearScalingStart_TH10,
    PGA460_NonlinearScalingStart_TH11,
    PGA460_NonlinearScalingStart_TH12

} PGA460_NonlinearScalingStartingThr;

/** \brief Threshold and TVG Timing Values */
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

/** \brief AFE Gain Range Selection */
typedef enum _PGA460_AFEGainRangeSel
{
    PGA460_AFE_GAIN_RANGE_58_90_dB = 0,
    PGA460_AFE_GAIN_RANGE_52_84_dB,
    PGA460_AFE_GAIN_RANGE_46_78_dB,
    PGA460_AFE_GAIN_RANGE_32_64_dB,
    PGA460_AFE_GAIN_RANGE_MAX
} PGA460_AFEGainRangeSel;

/** \brief PGA460 Configuration Structure
 *
 *  This structure contains all configurable parameters for the PGA460 device.
 *  Users can create custom configurations by copying PGA460_DefaultConfig
 *  and modifying the desired fields.
 */
typedef struct _PGA460_Config
{
    /* USERDATA registers */
    uint8                               userData1;
    uint8                               userData2;
    uint8                               userData3;
    uint8                               userData4;
    uint8                               userData5;
    uint8                               userData6;
    uint8                               userData7;
    uint8                               userData8;
    uint8                               userData9;
    uint8                               userData10;
    uint8                               userData11;
    uint8                               userData12;
    uint8                               userData13;
    uint8                               userData14;
    uint8                               userData15;
    uint8                               userData16;
    uint8                               userData17;
    uint8                               userData18;
    uint8                               userData19;
    uint8                               userData20;

    /* TVG Configuration */
    PGA460_AbsoluteThresholdGainTiming tvgT0;  /**< TVG time segment T0 */
    PGA460_AbsoluteThresholdGainTiming tvgT1;  /**< TVG time segment T1 */
    PGA460_AbsoluteThresholdGainTiming tvgT2;  /**< TVG time segment T2 */
    PGA460_AbsoluteThresholdGainTiming tvgT3;  /**< TVG time segment T3 */
    PGA460_AbsoluteThresholdGainTiming tvgT4;  /**< TVG time segment T4 */
    PGA460_AbsoluteThresholdGainTiming tvgT5;  /**< TVG time segment T5 */
    PGA460_AbsoluteThresholdGainTiming tvgT6; /**<  TVG time segment T6 */
    uint8 tvgG1;
    uint8 tvgG2;
    uint8 tvgG3;
    uint8 tvgG4;
    uint8 tvgG5;
    uint8 freqShift;


    /* Initial Gain Configuration */
    uint8                       initGain;      /**< Initial AFE gain value */
    uint8                       bpfBandwidth;  /**< Bandpass filter bandwidth */

    /* Burst Configuration */
    uint8                       burstFrequency;      /**< Burst frequency parameter */

    /* Deadtime configuration */
    uint8                       deglitchTime;        /**< Threshold comparator deglitch time */
    uint8                       burstPulseDeadtime;  /**< Burst pulse dead-time */

    /* P1 Pulse Configuration */
    uint8                       burstPulseCountP1;   /**< Number of burst pulses for P1 */

    /* Current limit P1 Configuration */
    PGA460_CurrentLimitDisabledType     currentLimitType;
    uint8                       currentLimitP1;      /**< Current limit for P1 */


    /* Decoupling Configuration */
    PGA460_AFEGainRangeSel              afeGainRange;       /**< AFE gain range selection */
    PGA460_LowPowerModeType             lowPowerMode;       /**< Low power mode enable */
    PGA460_DecoupleTimeTemperatureSel   decoupleTimeTempSel;/**< Time or temperature decouple */
    uint8                               decoupleTime;       /**< Secondary decouple time */


    uint8                       lpfCutoffFreq;       /**< Lowpass filter cutoff frequency */
    uint8                       recordTimeLengthP1;  /**< Record time length for P1 */



    /* Digital gain control */
    PGA460_LRStartingDigitalGainCtrl     LR_StartingDigitalGainCtrl;
    PGA460_LRDigitalGainCtrl             LR_DigitalGainCtrl;
    PGA460_SRDigitalGainCtrl             SR_DigitalGainCtrl;

    /* Threshold Configuration (Preset 1) */
    PGA460_AbsoluteThresholdGainTiming thrT1;   /**< Threshold time T1 */
    PGA460_AbsoluteThresholdGainTiming thrT2;   /**< Threshold time T2 */
    PGA460_AbsoluteThresholdGainTiming thrT3;   /**< Threshold time T3 */
    PGA460_AbsoluteThresholdGainTiming thrT4;   /**< Threshold time T4 */
    PGA460_AbsoluteThresholdGainTiming thrT5;   /**< Threshold time T5 */
    PGA460_AbsoluteThresholdGainTiming thrT6;   /**< Threshold time T6 */
    PGA460_AbsoluteThresholdGainTiming thrT7;   /**< Threshold time T7 */
    PGA460_AbsoluteThresholdGainTiming thrT8;   /**< Threshold time T8 */
    PGA460_AbsoluteThresholdGainTiming thrT9;   /**< Threshold time T9 */
    PGA460_AbsoluteThresholdGainTiming thrT10;  /**< Threshold time T10 */
    PGA460_AbsoluteThresholdGainTiming thrT11;  /**< Threshold time T11 */
    PGA460_AbsoluteThresholdGainTiming thrT12;  /**< Threshold time T12 */
    uint8                              thrL1L2; /**< Threshold levels L1/L2 */
    uint8                              thrL3;   /**< Threshold level L3 */
    uint8                              thrL4;   /**< Threshold level L4 */
    uint8                              thrL5L6; /**< Threshold levels L5/L6 */
    uint8                              thrL7L8; /**< Threshold levels L7/L8 */
    uint8                              thrL9;   /**< Threshold level L9 */
    uint8                              thrL10;  /**< Threshold level L10 */

    uint8                              frequencyDiagAbsErrTimeThreshold;
    uint8                              saturationDiagThreshold;
    PGA460_P1_NonLinearScalingType     P1_NLS;

    uint8                                    noiseLevel;
    PGA460_NonlinearScalingStartingThr       Scale_N;
    PGA460_NonlinearScalingExponentSel       Scale_K;



} PGA460_Config;

/** \brief Default PGA460 Configuration
 *
 *  This constant provides a default configuration with commonly used values.
 *  Copy this structure and modify fields as needed for custom configurations.
 */
extern const PGA460_Config PGA460_DefaultConfig;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/** \brief Initialize PGA460 device with default configuration
 *
 *  This function initializes the PGA460 interface and applies the default
 *  configuration to all registers.
 */
void     PGA460_InitDevice(void);

/** \brief Initialize PGA460 with custom configuration
 *
 *  This function applies a custom configuration to all PGA460 registers.
 *  The interface must be initialized first using PGA460_InitInterface().
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 *
 *  Example usage:
 *  \code
 *  PGA460_Config myConfig = PGA460_DefaultConfig;
 *  myConfig.burstFrequency = 0x90u;  // Customize frequency
 *  PGA460_InitInterface();
 *  PGA460_InitWithConfig(&myConfig);
 *  \endcode
 */
void     PGA460_UltrasonicInit(const PGA460_Config* config);

/** \brief Initialize DMA for PGA460 communication */
void     PGA460_InitDMA(void);

/** \brief Initialize Time Varying Gain (TVG) settings
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_InitTVGs(const PGA460_Config* config);

/** \brief Initialize decoupling and AFE gain range settings
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_InitDecoupling(const PGA460_Config* config);

/** \brief Initialize threshold map for Preset 1
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_InitThresholds(const PGA460_Config* config);

/** \brief Write Time Varying Gains */
void   PGA460_TimeVaryingGainBulkWriteBlocking(uint64 timeoutMs);


/** \brief Process received frame from PGA460 */
void     PGA460_ProcessFrame(void);

/** \brief Initialize UART interface for PGA460 communication */
void     PGA460_InitInterface(void);

/** \brief Read entire EEPROM contents from PGA460 */
void    PGA460_EEPROMRead(uint64 timeoutMs);

/** \brief Read a single register from PGA460
 *
 *  \param reg_addr Register address to read
 */
void     PGA460_RegisterReadAsync(uint8 reg_addr);

/** \brief Write a single register to PGA460
 *
 *  \param reg_addr Register address to write
 *  \param data     Data value to write
 */
void     PGA460_RegisterWriteAsync(uint8 reg_addr, uint8 data);

/** \brief Read a single register from PGA460
 *
 *  \param reg_addr Register address to read
 */
boolean     PGA460_RegisterReadBlocking(uint8 reg_addr, uint64 timeout);

/** \brief Write a single register to PGA460
 *
 *  \param reg_addr Register address to write
 *  \param data     Data value to write
 */
boolean     PGA460_RegisterWriteBlocking(uint8 reg_addr, uint8 data, uint64 timeout);

/** \brief Set initial AFE gain and bandpass filter bandwidth
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetInitialGain(const PGA460_Config* config);

/** \brief Set burst frequency parameter
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetBurstFrequency(const PGA460_Config* config);

/** \brief Set number of burst pulses for Preset 1
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetNumberBurstPulsesP1(const PGA460_Config* config);

/** \brief Set current limit for Preset 1
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetCurrentLimitP1(const PGA460_Config* config);

/** \brief Set lowpass filter cutoff frequency
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetLPFCutoffFrequency(const PGA460_Config* config);

/** \brief Set record time length for Preset 1
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetRecordTimeLengthP1(const PGA460_Config* config);

/** \brief Set threshold comparator deglitch time
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void     PGA460_SetDeglitchTime(const PGA460_Config* config);

/** \brief Enable/Disable non-linear scaling for Preset1
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void    PGA460_SetNonlinearScalingP1(const PGA460_Config* config);

/** \brief Set Digital Gains for P1
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void   PGA460_SetDigitalGainsP1(const PGA460_Config* config);

/** \brief Set Saturation diagnostic threshold level
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void    PGA460_SetSaturationDiagThreshold(const PGA460_Config* config);

/** \brief Set freqneyc diagnostic time diagnostic threshold level
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void PGA460_SetFrequencyDiagErrThreshold(const PGA460_Config* config);

/** \brief Set DSP scaling and noise level
 *
 *  \param config Pointer to configuration structure. If NULL, function returns without action.
 */
void PGA460_SetDSPScaling(const PGA460_Config* config);

/** \brief Trigger ultrasonic burst measurements
 *
 */
void PGA460_BurstListenP1(void);


/** \brief Check if PGA460 is available for new commands
 *
 *  \return TRUE if available, FALSE if busy
 */
boolean  PGA460_isAvailable(void);

/** \brief Check if data is ready from PGA460
 *
 *  \return TRUE if data ready, FALSE otherwise
 */
boolean  PGA460_isDataReady(void);

/** \brief Burn current configuration to EEPROM
 *
 *  \return TRUE if successful, FALSE otherwise
 */
boolean  PGA460_BurnEEPROM();


#endif /* HW_ISR_EBCM_PGA460_H_ */
