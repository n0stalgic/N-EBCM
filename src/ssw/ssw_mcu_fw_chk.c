/******************************************************************************
 * @file    ssw_fw_chk.c
 * @brief   Checks operation of MCU FW boot rom
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

#include "ebcm_main.h"
#include "ssw.h"
#include "ssw_mcu_fw_chk.h"
#include "ssw_mcu_fw_chk_tables.h"
#include "IfxMtu.h"
#include "IfxMtu_cfg.h"
#include "IfxSmu.h"
#include "IfxDmu_Reg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

volatile Ifx_SMU_AG ebcmSmuAlarmStatus[IFXSMU_NUM_ALARM_GROUPS];
McuFwCheckStatus mcuFwCheckStatus;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

boolean EbcmSsw_fwCheckSmuStMemLclcon(const FwCheckStruct* fwCheckTable, const int structSize,
                                  const EbcmResetType resetType, FwCheckVerificationStruct* fwCheckVerification);


IfxMtu_MbistSel EbcmSsw_fwCheckSsh(const EbcmResetType resetType);
IfxMtu_MbistSel EbcmSsw_fwCheckSshRegisters(const MemoryTested* sshTable, int tableSize);
boolean EbcmSsw_fwCheckEvalRamInit(uint16 memoryMask);
boolean EbcmSsw_fwCheckEvalLmuInit(uint16 memoryMask);
void EbcmSsw_fwCheckClrSmuAlarms(const FwCheckStruct* fwCheckTable, const int tableSize);
void EbcmSsw_fwCheckClearSsh(const EbcmResetType resetType);
void EbcmSsw_fwCheckRetriggerCheck(const EbcmResetType resetType);
void EbcmSsw_clearFaultStatusAndEccDetectionFsiram(void);
void EbcmSsw_fwCheckClearResetStatus(const EbcmResetType resetType);
void EbcmSsw_fwCheckClearAppAndSysStatus(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void EbcmSsw_mcuFwCheck(void)
{
    mcuFwCheckStatus.mcuFwCheckSmu = FALSE;
    mcuFwCheckStatus.mcuFwCheckScuStem = FALSE;
    mcuFwCheckStatus.mcuFwCheckScuLclcon = FALSE;
    mcuFwCheckStatus.mcuFwCheckSsh = FALSE;

    /* Enable MTU module if not yet enabled
     * MTU is required especially for the check SSH because MTU controls SSH instances */
    boolean mtuWasEnabled = IfxMtu_isModuleEnabled();
    if (mtuWasEnabled == FALSE)
    {
        /* Enable MTU module */
        IfxMtu_enableModule();
    }
    /* Take a snapshot of the SMU alarm registers before executing the FW check */
    for (uint8 alarmReg = 0; alarmReg < IFXSMU_NUM_ALARM_GROUPS; alarmReg++)
    {
        ebcmSmuAlarmStatus[alarmReg].U = MODULE_SMU.AG[alarmReg].U;
    }
    /* Increment the firmware check execution counter */
    /* Read SMU alarm register values and compare with expected ones (listed in Appendix A of the Safety Manual) */
    /* Note: depending on the device and reset type different register values are expected */

    if (TRUE
            == EbcmSsw_fwCheckSmuStMemLclcon(fwCheckSMUTC37A, fwCheckSMUTC37ASize, ebcmStatus.resetCode.resetType,
                                         fwCheckVerificationSMU))
    {
        mcuFwCheckStatus.mcuFwCheckSmu = TRUE;
    }

    if (TRUE
            == EbcmSsw_fwCheckSmuStMemLclcon(fwCheckSTMEMTC37A, fwCheckSTMEMTC37ASize,
                                         ebcmStatus.resetCode.resetType, fwCheckVerificationSTMEM))
    {
        mcuFwCheckStatus.mcuFwCheckScuStem = TRUE;
    }

    if (TRUE
            == EbcmSsw_fwCheckSmuStMemLclcon(fwCheckLCLCONTC37A, fwCheckLCLCONTC37ASize,
                                         ebcmStatus.resetCode.resetType, fwCheckVerificationLCLCON))
    {
        mcuFwCheckStatus.mcuFwCheckScuLclcon = TRUE;
    }

    if (IfxMtu_MbistSel_none == EbcmSsw_fwCheckSsh(ebcmStatus.resetCode.resetType))
    {
        mcuFwCheckStatus.mcuFwCheckSsh = TRUE;
    }

    /* Verify if all checks have PASSED */
    if ((mcuFwCheckStatus.mcuFwCheckSmu &&
         mcuFwCheckStatus.mcuFwCheckScuStem &&
         mcuFwCheckStatus.mcuFwCheckScuLclcon &&
         mcuFwCheckStatus.mcuFwCheckSsh))
    {
        ebcmStatus.sswStatus.mcuFwChkStatus = PASSED;

        /* If all registers and SMU alarm registers have reported the expected values .. */
        /* clear the content of the registers mentioned in the Appendix table */
        EbcmSsw_fwCheckClearSsh(ebcmStatus.resetCode.resetType);
        /* clear the SMU alarms SMU_AG0..11 */
        EbcmSsw_fwCheckClrSmuAlarms(fwCheckSMUTC37A, fwCheckSMUTC37ASize);
        /* clear the corresponding reset status bits in RSTSTAT register */
        EbcmSsw_fwCheckClearResetStatus(ebcmStatus.resetCode.resetType);
    }

    else
    {
        sswRunCount->mcuFwCheckFailCount++;
        ebcmStatus.sswStatus.mcuFwChkStatus = FAILED;
        /* If FW check has FAILED during its first execution trigger the check again */
        if (sswRunCount->mcuFwCheckFailCount < EBCM_MCU_FW_CHECK_MAX_RUNS)
        {
            /* clear the corresponding reset status bits in RSTSTAT register */
             EbcmSsw_fwCheckClearResetStatus(ebcmStatus.resetCode.resetType);
            /* Debugger issue : this line needs to be commented during debug state */
            EbcmSsw_fwCheckRetriggerCheck(ebcmStatus.resetCode.resetType);
        }
    }

    /* Disable MTU module if it was disabled */
    if(mtuWasEnabled == FALSE)
    {
        /* Disable again */
        IfxScuWdt_clearCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());
        MTU_CLC.B.DISR = 1;
        IfxScuWdt_setCpuEndinit(IfxScuWdt_getCpuWatchdogPassword());
    }

}

/* This function is comparing the actual register values of the registers listed in Safety Manual Appendix A with the expected ones.
 * The expected values are depending on the device type and also the reset type.
 * const FwCheckStruct *fwCheckTable => pointer to a structure of type FwCheckStruct. The table consists of a pointer to the address of the register which should be verified and accordingly the expected register values for each reset type.
 * const int tableSize => amount of entries in the fwCheckTable
 * const ebcm_reset_type_t resetType => type of the reset
 * FwCheckVerificationStruct *fwCheckVerification => pointer to a structure where the test result will be written into. Can be observed via debugger in case the FW check has FAILED
 * */
boolean EbcmSsw_fwCheckSmuStMemLclcon(const FwCheckStruct* fwCheckTable,
                                  const int tableSize,
                                  const EbcmResetType resetType,
                                  FwCheckVerificationStruct* fwCheckVerification)
{
    boolean fwcheckHasPassed = TRUE;
    uint32 registerValue;
    uint32 expectedRegisterValue;

    /* A pointer to the corresponding FwCheckRegisterCheckStruct inside the fwCheckTable.
     * The corresponding structure of interest depends on the reset type. */
    const FwCheckRegisterCheckStruct* ptrRegisterCheckStruct;

    /* Iterate through the fwCheckTable */
    for(uint8 i = 0; i < tableSize; i++)
    {
        /* Set the pointer to the structure containing the expected register values */
        switch(resetType)
        {
            case IfxScuRcu_ResetType_coldpoweron:
                ptrRegisterCheckStruct = &fwCheckTable[i].coldPORST;
                break;
            case IfxScuRcu_ResetType_warmpoweron:
                ptrRegisterCheckStruct = &fwCheckTable[i].warmPORST;
                break;
            case IfxScuRcu_ResetType_system:
                ptrRegisterCheckStruct = &fwCheckTable[i].systemReset;
                break;
            case IfxScuRcu_ResetType_application:
                ptrRegisterCheckStruct = &fwCheckTable[i].applicationReset;
                break;
            default:
                __debug();
                break;
        }

        /* Read the value of the register which is checked during this iteration */
        registerValue          =   *(volatile uint32 *)fwCheckTable[i].regUnderTest;
        /* Mask the register value in case there are any exceptions. (Refer to the Appendix A of the Safety Manual) */
        registerValue          &=  ptrRegisterCheckStruct->mask;
        /* Read the expected register value */
        expectedRegisterValue =   ptrRegisterCheckStruct->expectedRegVal;

        /* Compare the register value with the expected value and write both the test result and the actual
         * register value into the verification structure. */
        fwCheckVerification[i].regVal = registerValue;
        fwCheckVerification[i].testHasPassed = (registerValue == expectedRegisterValue) ? TRUE : FALSE;
        /* Set fwcheckHasPassed to FALSE in case test has FAILED for any register during the iteration. */
        if(!fwCheckVerification[i].testHasPassed)
        {
            fwcheckHasPassed = FALSE;
        }
    }

    /* Return result after iteration and comparison of all registers listed in the fwCheckTable */
    return fwcheckHasPassed;
}

/*
 * Check the SSH registers and compare with the expected values depending on the reset type
 * */
IfxMtu_MbistSel EbcmSsw_fwCheckSsh(const EbcmResetType resetType)
{
    IfxMtu_MbistSel fwcheckSshResult;
    switch(resetType)
    {
        case IfxScuRcu_ResetType_coldpoweron:
            fwcheckSshResult = EbcmSsw_fwCheckSshRegisters(coldPorstSSHTC37A, coldPorstSSHTC37ASize);
            break;
        case IfxScuRcu_ResetType_warmpoweron:
            fwcheckSshResult = EbcmSsw_fwCheckSshRegisters(warmPorstSSHTC37A, warmPorstSSHTC37ASize);
            break;
        case IfxScuRcu_ResetType_system:
            fwcheckSshResult = EbcmSsw_fwCheckSshRegisters(systemSSHTC37A, systemSSHTC37ASize);
            break;
        case IfxScuRcu_ResetType_application:
            fwcheckSshResult = EbcmSsw_fwCheckSshRegisters(applicationSSHTC37A, applicationSSHTC37ASize);
            break;
        default:
            __debug();
            break;
    }
    return fwcheckSshResult;
}

/*
 * Clear SSH register
 * */
void EbcmSsw_fwCheckClearSsh(const EbcmResetType resetType)
{
    /* Get pointer to specific table and set variable about the size of this table */
    const MemoryTested* sshTable;
    int tableSize;

    switch(resetType)
    {
        case IfxScuRcu_ResetType_coldpoweron:
                sshTable    = coldPorstSSHTC37A;
                tableSize  = coldPorstSSHTC37ASize;
            break;
        case IfxScuRcu_ResetType_warmpoweron:
                sshTable    = warmPorstSSHTC37A;
                tableSize  = warmPorstSSHTC37ASize;
            break;
        case IfxScuRcu_ResetType_system:
                sshTable    = systemSSHTC37A;
                tableSize  = systemSSHTC37ASize;
            break;
        case IfxScuRcu_ResetType_application:
                sshTable    = applicationSSHTC37A;
                tableSize  = applicationSSHTC37ASize;
            break;
        default:
            __debug();
            break;
    }

    /* Now iterate through the table and clear the ECCD, FAULTSTS and ERRINFO register values */
    IfxMtu_MbistSel mbistSel;
    Ifx_MTU_MC *mc;
    int a;
    uint16 password = IfxScuWdt_getSafetyWatchdogPassword();

    for (a = 0; a < tableSize ; a++ )
    {
       mbistSel = sshTable[a].sshUnderTest;

       mc = &MODULE_MTU.MC[mbistSel];
       mc->ECCD.U = 0x0;

       IfxScuWdt_clearSafetyEndinit(password);

       mc->FAULTSTS.U = 0x0;
       mc->ECCD.B.TRC = 1;  /* This will clear ERRINFO */

       IfxScuWdt_setSafetyEndinit(password);
   }

}

/*
 * Clear reset application and system reset register
 * */
void EbcmSsw_fwCheckClearAppAndSysStatus(void)
{
    uint16         password;
    password = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_clearSafetyEndinitInline(password);

    MODULE_SCU.RSTCON.B.SW = 0;
    IfxScuWdt_setSafetyEndinit(password);
}

/*
 * Clear reset status
 * */
void EbcmSsw_fwCheckClearResetStatus(const EbcmResetType resetType)
{
    switch(resetType)
    {
        case IfxScuRcu_ResetType_coldpoweron:
            IfxScuRcu_clearColdResetStatus();
            break;
        case IfxScuRcu_ResetType_warmpoweron:
            IfxScuRcu_clearColdResetStatus();
            break;
        case IfxScuRcu_ResetType_system:
        case IfxScuRcu_ResetType_application:
            EbcmSsw_fwCheckClearAppAndSysStatus();
            break;

        default:
            __debug();
            break;
    }
}

/*
 * This is the implementation of Erratum from Errata sheet
 * Erratum: [SMU_TC.H012]
 * The SMU alarms ALM7[1] and ALM7[0] are set intentionally after PORST and system reset and shall be
 * cleared by the application SW (cf. SM:SYS:MCU_FW_CHECK in Safety Manual) Also, in order to
 * clear the SMU alarms ALM7[1] and ALM7[0], it is necessary to clear the alarms within this MC40.
 * */
void EbcmSsw_clearFaultStatusAndEccDetectionFsiram(void)
{
    uint16 password = IfxScuWdt_getSafetyWatchdogPassword();
    uint16 *ptrEccd = (uint16 *)(0xF0063810);      /* MCi_ECCD and i = 40 */
    *ptrEccd = 0;

    IfxScuWdt_clearSafetyEndinit(password);
    uint16 *ptrFaultsts = (uint16 *)(0xF00638F0);  /* MCi_FAULTSTS and i = 40*/
    *ptrFaultsts = 0;
    IfxScuWdt_setSafetyEndinit(password);
}

/*
 * Check if Smu alarms were cleared
 * */
void EbcmSsw_fwCheckClrSmuAlarms(const FwCheckStruct* fwCheckTable, const int tableSize)
{
    uint16 passwd = IfxScuWdt_getSafetyWatchdogPassword();
    EbcmSsw_clearFaultStatusAndEccDetectionFsiram();
    IfxScuWdt_clearSafetyEndinit(passwd);

    /* Iterate through all SMU alarm status registers listed in the fwCheckTable and clear them */
    for(uint8 i = 0; i < tableSize; i++)
    {
        MODULE_SMU.CMD.U = IfxSmu_Command_alarmStatusClear;
        *(volatile uint32 *)fwCheckTable[i].regUnderTest = (uint32) 0xFFFFFFFF;
    }

    IfxScuWdt_setSafetyEndinit(passwd);
}

/*
 * Clear SSH and trigger a specific reset depending on incoming reset
 * */
void EbcmSsw_fwCheckRetriggerCheck(const EbcmResetType resetType)
{
    /* Initiate the specific reset to trigger an FW check. Note: the reset should be the same type as it was before this FW check execution */
    switch(resetType)
    {
        case IfxScuRcu_ResetType_coldpoweron:
            /* Trigger Cold PORST */
            /* No TLF for this board to trigger cold PORST */
            break;
        case IfxScuRcu_ResetType_warmpoweron:
            EbcmSsw_fwCheckClearSsh(resetType);
            /* Trigger Warm PORST */
            EbcmSsw_triggerWarmPorst();
            break;
        case IfxScuRcu_ResetType_system:
            EbcmSsw_fwCheckClearSsh(resetType);
            EbcmSsw_triggerSwReset(IfxScuRcu_ResetType_system);
            break;
        case IfxScuRcu_ResetType_application:
            EbcmSsw_fwCheckClearSsh(resetType);
            EbcmSsw_triggerSwReset(IfxScuRcu_ResetType_application);
            break;
        default:
            __debug();
            break;
    }
}

/*
 * This function is comparing the SSH registers of all RAMs with their expected values
 * */
IfxMtu_MbistSel EbcmSsw_fwCheckSshRegisters(const MemoryTested* sshTable, int tableSize)
{
    IfxMtu_MbistSel mbistSel;
    Ifx_MTU_MC *mc;
    int a;
    volatile uint16 faultstsExpectedValue, eccdExpectedValue, errinfoExpectedValue;

    /* Iterate through all entries in the sshTable */
    for (a = 0; a < tableSize ; a++ )
    {
        /* Get SSH which is tested during this iteration */
        mbistSel = sshTable[a].sshUnderTest;
        /* Get pointer to MC object of tested SSH */
        mc = &MODULE_MTU.MC[mbistSel];

        /* Evaluate which register values are expected for FAULTSTS, ECCD and ERRINFO registers. */
        /* Check which memory type it is and evaluate if the RAM is initialized or not. Depending
         * on this different FAULTSTS values are expected for CPU and LMU memories. */
        if (CPU_MEM_TYPE == sshTable[a].memoryType)
        {
            if (TRUE == EbcmSsw_fwCheckEvalRamInit(sshTable[a].inSelMask))
            {
                faultstsExpectedValue = 0x9;
            }
            else
            {
                faultstsExpectedValue = 0x1;
            }
        }
        else if (LMU_MEM_TYPE == sshTable[a].memoryType)
        {
            if (TRUE == EbcmSsw_fwCheckEvalLmuInit(sshTable[a].inSelMask))
            {
                faultstsExpectedValue = 0x9;
            }
            else
            {
                faultstsExpectedValue = 0x1;
            }
        }
        else
        {
           faultstsExpectedValue = sshTable[a].sshRegistersDef.fltStatusVal;
        }

        eccdExpectedValue     = sshTable[a].sshRegistersDef.eccdVal;
        errinfoExpectedValue  = sshTable[a].sshRegistersDef.errInfoVal;

           /* Exception: If AURIX woke up from standby, overwrite expected values with the dedicated standby values. */
           if (ebcmStatus.wakeupFromStby)
           {
               faultstsExpectedValue = sshTable[a].sshRegistersStb.fltStatusVal;
               eccdExpectedValue     = sshTable[a].sshRegistersStb.eccdVal;
               errinfoExpectedValue  = sshTable[a].sshRegistersStb.errInfoVal;
           }
        {

            /* Finally compare register values of selected memory with the expected ones, if any mismatch is
             * detected return the name of the selected memory and stop the function execution. */
            if (mc->ECCD.U != eccdExpectedValue ||
                mc->FAULTSTS.U != faultstsExpectedValue ||
                mc->ERRINFO[0].U != errinfoExpectedValue)
            {
               return (mbistSel);
            }
        }
        /* EXCEPTION TEST */
    }
    return (IfxMtu_MbistSel_none);
}

/*
 * Verify if CPU memory is initialized
 * */
boolean EbcmSsw_fwCheckEvalRamInit(uint16 memoryMask)
{
    if ( RAM_INIT_AT_COLD_WARM == DMU_HF_PROCONRAM.B.RAMIN ||
         RAM_INIT_AT_COLD_ONLY == DMU_HF_PROCONRAM.B.RAMIN)
    {
        if (((DMU_HF_PROCONRAM.B.RAMINSEL) & (uint16) (memoryMask)) == 0)
        {
            return (TRUE);
        }
        else
        {
            return (FALSE);
        }
    }
    else
    {
        return (FALSE);
    }
}

/*
 * Verify if LMU memory is initialized
 * */
boolean EbcmSsw_fwCheckEvalLmuInit(uint16 memoryMask)
{
    if ( RAM_INIT_AT_COLD_WARM == DMU_HF_PROCONRAM.B.RAMIN ||
         RAM_INIT_AT_COLD_ONLY == DMU_HF_PROCONRAM.B.RAMIN)
    {
        if (((DMU_HF_PROCONRAM.B.LMUINSEL) & (uint16) (memoryMask)) == 0)
        {
            return (TRUE);
        }
        else
        {
            return (FALSE);
        }
    }
    else
    {
        return (FALSE);
    }
}

/*
 * This function is to clear all SMU alarms
 * */
void EbcmSsw_clearAllSmuAlarms(void)
{
    EbcmSsw_fwCheckClrSmuAlarms(fwCheckSMUTC37A, fwCheckSMUTC37ASize);
}
