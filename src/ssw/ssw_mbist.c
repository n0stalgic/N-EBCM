/******************************************************************************
 * @file    ssw_mbist.c
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
#include "IfxMtu.h"
#include "Ifx_Cfg_Ssw.h"
#include "IfxDma_reg.h"
#include "ebcm_main.h"
#include "ebcm_cfg.h"
#include "ssw_mbist.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
MbistStatus g_MbistStatus;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
IFX_EXTERN const IfxMtu_MbistConfig *const mbistGangConfig[];
void slkClearMbistSshRegisters(void);

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
#if EBCM_CFG_SSW_ENABLE_MBIST
/*
 * SM:VMT:MBIST
 * */
void ebcm_ssw_mbist(void)
{
    ebcm_status.ssw_status.mbist_status = TEST_NOT_EVAL;
    /* Note:
     * - "For peripherals SRAMs, it shall be ensured that the module does not access the
     *    SRAM under test" (Refer to UM Non-Destructive Test (NDT) in MTU section) */

    /* Preparations for MBIST */
    /* Disable all other CPUs, but of course not the one which is executing the function */
    IfxCpu_ResourceCpu coreIndex  = IfxCpu_getCoreIndex();
    if(coreIndex != IfxCpu_ResourceCpu_0)
    {
        IfxCpu_setCoreMode(&MODULE_CPU0, IfxCpu_CoreMode_idle);
    }
#if (IFXCPU_NUM_MODULES > 1)
    if(coreIndex != IfxCpu_ResourceCpu_1)
    {
        IfxCpu_setCoreMode(&MODULE_CPU1, IfxCpu_CoreMode_idle);
    }
#endif
#if (IFXCPU_NUM_MODULES > 2)
    if(coreIndex != IfxCpu_ResourceCpu_2)
    {
        IfxCpu_setCoreMode(&MODULE_CPU2, IfxCpu_CoreMode_idle);
    }
#endif

    /* Disable CPU caches */
    IfxCpu_setDataCache    (0);
    IfxCpu_setProgramCache (0);

    /* If DMA master is enabled disable it */
    boolean dmaWasEnabled = FALSE;
    if(MODULE_DMA.CLC.B.DISS == 0)
    {
        dmaWasEnabled = TRUE;
        uint16 passwd = IfxScuWdt_getCpuWatchdogPassword();
        IfxScuWdt_clearCpuEndinit(passwd);
        MODULE_DMA.CLC.B.DISR = 1;
        IfxScuWdt_setCpuEndinit(passwd);
    }

    /* MBIST Tests and evaluation */
    boolean nBistError = TRUE;
    /* Disable the Xram clear in IfxMtu_cfg.c because
     * XRAM is used in LBIST to store the execution number */
    nBistError = IfxMtu_runMbistAll(mbistGangConfig);

    /* show correctly in OneEye GUI*/
    g_MbistStatus.noMbistError = !nBistError;

    /* check if there was any error */
    if (nBistError == FALSE)
    {
        ebcm_status.ssw_status.mbist_status = PASSED;
    }
    else
    {
        ebcm_status.ssw_status.mbist_status = FAILED;
    }

    /* Clear all ECCD and FAULTSTS registers of the tested memory */
    slkClearMbistSshRegisters();

    /* Enable DMA module again if it got disabled before */
    if(dmaWasEnabled)
    {
        uint16 passwd = IfxScuWdt_getCpuWatchdogPassword();
        IfxScuWdt_clearCpuEndinit(passwd);
        MODULE_DMA.CLC.B.DISR = 0;
        IfxScuWdt_setCpuEndinit(passwd);
    }

    /* Enable CPU caches */
    IfxCpu_setDataCache    (1);
    IfxCpu_setProgramCache (1);

    /* Enable all other CPUs, but not the one which is executing the function (of course it is also not disabled) */
    if(coreIndex != IfxCpu_ResourceCpu_0)
    {
        IfxCpu_setCoreMode(&MODULE_CPU0, IfxCpu_CoreMode_run);
    }
#if (IFXCPU_NUM_MODULES > 1)
    if(coreIndex != IfxCpu_ResourceCpu_1)
    {
        IfxCpu_setCoreMode(&MODULE_CPU1, IfxCpu_CoreMode_run);
    }
#endif
#if (IFXCPU_NUM_MODULES > 2)
    if(coreIndex != IfxCpu_ResourceCpu_2)
    {
        IfxCpu_setCoreMode(&MODULE_CPU2, IfxCpu_CoreMode_run);
    }
#endif
}
#endif /* SLK_CFG_SSW_ENABLE_MBIST */

/*
 * Clear all ECCD and FAULTSTS registers of the memory on which MBIST was executed.
 * Algorithm is based on a combination of the two functions
 * boolean IfxMtu_runMbistAll(const IfxMtu_MbistConfig *const mbistConfig[]) and
 * void slkFwCheckClearSSH(SlkResetType resetType)
 * */
void slkClearMbistSshRegisters(void)
{
    IfxMtu_MbistSel mbistSel;
    Ifx_MTU_MC *mc;
    sint32 count;
    uint16 password;

    unsigned int gangConfigCount = 0;
    while (mbistGangConfig[gangConfigCount] != (void *)0)
    {
        for (count = 0; count < mbistGangConfig[gangConfigCount]->numOfSshConfigurations; count++)
        {
            mbistSel = mbistGangConfig[gangConfigCount]->sshConfigurations[count].sshSel;

            mc = &MODULE_MTU.MC[mbistSel];

            /* Clear error flags of tested memory */
            mc->ECCD.U = 0x0;

            password = IfxScuWdt_getSafetyWatchdogPassword();
            IfxScuWdt_clearSafetyEndinit(password);

            mc->FAULTSTS.U = 0x0;

            IfxScuWdt_setSafetyEndinit(password);
        }
        gangConfigCount++;
    }
}
