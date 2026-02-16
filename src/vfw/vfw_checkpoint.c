/******************************************************************************
 * @file    vfw_checkpoint.c
 * @brief   Vital Framework Implementation
 *
 * MIT License
 *
 * Copyright (c) 2026 n0stalgic
 *****************************************************************************/

#include <ebcm_vcom.h>
#include "vfw_checkpoint.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxStm.h"
#include "vfw_ffi.h"
#include "IfxAsclin_Asc.h"
#include "Ifx_Shell.h"
#include "Ifx_Console.h"
#include "IfxPort.h"
#include "string.h"
#include "stdio.h"
#include "ebcm_fce_crc.h"
#include "vfw_utils.h"


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
IfxAsclin_Asc g_asc;

/* Integrity Accumulator */
volatile uint32 VFW_integrity_accumulator __attribute__ ((__align(DPR_GRANULARITY))) = 0U;
volatile boolean VFW_integrityCheckFailed __attribute__ ((__align(DPR_GRANULARITY))) = FALSE;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/* Vital Framework checkpoint and profiling database 8-byte aligned for MPU protection */
static VfwCheckpoint* vfwRegistry[VFW_MAX_CHECKPOINTS] __attribute__ ((__align(DPR_GRANULARITY)));
static uint16 vfwRegistryCount __attribute__ ((__align(DPR_GRANULARITY))) = 0U;


/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

static void VFW_ProtectionInit(void)
{
    extern __far uint8 _lc_gb_vfw_safe0;
    extern __far uint8 _lc_ge_vfw_safe0;

    /* Set 1 : The entire MCU is readable and writable */
    VFW_defineDataProtectionRange(0x00000000, 0xFFFFFFFF, DATA_PROTECTION_RANGE_0);
    VFW_enableDataRead(PROTECTION_SET_1, DATA_PROTECTION_RANGE_0);
    VFW_enableDataWrite(PROTECTION_SET_1, DATA_PROTECTION_RANGE_0);

    uint8 *__VFW_SAFE0 = &_lc_gb_vfw_safe0;
    uint8 *__VFW_SAFE0_END = &_lc_ge_vfw_safe0;

    /* Set 2 : Vital framework safe region (DPR 15 restricted) */
    VFW_defineDataProtectionRange((uint32) __VFW_SAFE0, (uint32) __VFW_SAFE0_END , DATA_PROTECTION_RANGE_15);

    /* PRS 1: Read only - Allow application to read vital framework data */
    VFW_enableDataRead(PROTECTION_SET_1, DATA_PROTECTION_RANGE_15);

    /* PRS 0 (VFW) : R/W - Only vital framework can modify data in vfw_safe0 memory region */
    VFW_enableDataRead(PROTECTION_SET_0, DATA_PROTECTION_RANGE_15);
    VFW_enableDataWrite(PROTECTION_SET_0, DATA_PROTECTION_RANGE_15);

    /* Safe region can read/write to non-safe regions */
    VFW_enableDataRead(PROTECTION_SET_0, DATA_PROTECTION_RANGE_0);
    VFW_enableDataWrite(PROTECTION_SET_0, DATA_PROTECTION_RANGE_0);

    /* Allow us to execute code in both safe and non-safe memory protection sets */
    VFW_defineCodeProtectionRange(0x00000000, 0xFFFFFFFF, CODE_PROTECTION_RANGE_0);
    VFW_enableCodeExecution(PROTECTION_SET_0, CODE_PROTECTION_RANGE_0);
    VFW_enableCodeExecution(PROTECTION_SET_1, CODE_PROTECTION_RANGE_0);

    set_active_protection_set(PROTECTION_SET_0);
    VFW_enableMemProtection();
}

void VFW_Init(void)
{
    VFW_ProtectionInit();

    /* Switch to special memory access for VFW updates */
    set_active_protection_set(PROTECTION_SET_0);


    VFW_integrity_accumulator = 0U;
    VFW_integrityCheckFailed = FALSE;

    /* Clear registry? No, registry persists across frames, but we might want to clear it on boot.
     * Since this is usually called at startup, it's fine.
     * Note: Does not clear user objects, only the registry pointers, so this can be re-populated
     */
    for (uint16 i = 0; i < VFW_MAX_CHECKPOINTS; i++)
    {
        vfwRegistry[i] = NULL_PTR;
    }
    vfwRegistryCount = 0;

    /* Drop back to non-safe access */
    set_active_protection_set(PROTECTION_SET_1);

}

static uint32 VFW_generateSignature(const char* str)
{
    IFX_ASSERT (IFX_VERBOSE_LEVEL_ERROR, strlen(str) <= VFW_CHECKPOINT_NAME_MAX_LEN);

    uint32 crc = VFW_crc32((const uint8* ) str, strlen(str));

    return crc;
}

void VFW_CreateCheckpoint(VfwCheckpoint* cp, const char* name)
{
    /* Switch to special memory access for VFW updates */
    set_active_protection_set(PROTECTION_SET_0);

    IFX_ASSERT (IFX_VERBOSE_LEVEL_ERROR, cp != NULL_PTR);
    IFX_ASSERT (IFX_VERBOSE_LEVEL_ERROR, vfwRegistryCount >= VFW_MAX_CHECKPOINTS);

    /* Initialize Object */
    cp->name = name;
    cp->signature = VFW_generateSignature(name);
    cp->id = vfwRegistryCount;
    cp->callCount = 0;
    cp->maxTimeTicks = 0;
    cp->lastTimeTicks = 0;
    cp->isRegistered = TRUE;

    /* Add to Database */
    vfwRegistry[vfwRegistryCount] = cp;
    vfwRegistryCount++;

    /* Drop back to background access */
    set_active_protection_set(PROTECTION_SET_1);
}

void VFW_CheckpointEntry(VfwCheckpoint* cp)
{
    /* Switch to special memory access for VFW updates */
    set_active_protection_set(PROTECTION_SET_0);

    if (cp && cp->isRegistered)
    {
        /* 1. Update Integrity State (Add Signature) */
        VFW_integrity_accumulator += cp->signature;

        /* 2. Start Profiling Timer */
        /* Use STM0 as common timebase */
        cp->startTimeTicks = IfxStm_get(&MODULE_STM0);
    }

    /* Drop back to background access */
    set_active_protection_set(PROTECTION_SET_1);
}

void VFW_CheckpointExit(VfwCheckpoint* cp)
{
    /* Switch to special memory access for VFW updates */
    set_active_protection_set(PROTECTION_SET_0);

    if (cp && cp->isRegistered)
    {
        /* 1. Stop Profiling Timer */
        uint64 endTime = IfxStm_get(&MODULE_STM0);
        uint64 delta = endTime - cp->startTimeTicks; /* Handles overflow naturally */

        /* 2. Update Stats */
        cp->lastTimeTicks = delta;
        if (delta > cp->maxTimeTicks)
        {
            cp->maxTimeTicks = delta;
        }
        cp->callCount++;

        /* 3. Update Integrity State (Subtract Signature) */
        VFW_integrity_accumulator -= cp->signature;
    }

    /* Drop back to background access */
    set_active_protection_set(PROTECTION_SET_1);
}

boolean VFW_HasIntegrityCheckFailed(void)
{
    return VFW_integrityCheckFailed;
}

#pragma optimize 0
boolean VFW_IntegrityCheck(void)
{
    /* Switch to special memory access for Vital Framework updates */
    set_active_protection_set(PROTECTION_SET_0);

    if (VFW_integrity_accumulator != 0U)
    {
        /* SAFETY VIOLATION: Execution path error (imbalanced calls) */
        IfxCpu_disableInterrupts();
        IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
        VFW_integrityCheckFailed = TRUE;

        return FALSE;
    }

    VFW_integrityCheckFailed = FALSE;

    /* Drop back to non-safe access */
    set_active_protection_set(PROTECTION_SET_1);

    return TRUE;
}
#pragma endoptimize
