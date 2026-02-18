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

#pragma section all "vfw_safe0"

volatile uint32 VFW_integrity_accumulator = 0U;
volatile boolean VFW_integrityCheckFailed = FALSE;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

static VfwCheckpoint* vfwRegistry[VFW_MAX_CHECKPOINTS];
static uint16 vfwRegistryCount = 0U;


#pragma section all

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void VFW_initRegistry(void)
{
    VFW_GrantSafeMemAccess();

    VFW_integrity_accumulator = 0U;
    VFW_integrityCheckFailed = FALSE;

    for (uint16 i = 0; i < VFW_MAX_CHECKPOINTS; i++)
    {
        vfwRegistry[i] = NULL_PTR;
    }

    vfwRegistryCount = 0;

    VFW_ReleaseSafeMemAccess();
}

static uint32 VFW_generateSignature(const char* str)
{
    IFX_ASSERT (IFX_VERBOSE_LEVEL_ERROR, strlen(str) <= VFW_CHECKPOINT_NAME_MAX_LEN);

    uint32 crc = VFW_crc32((const uint8* ) str, strlen(str));

    return crc;
}

void VFW_CreateCheckpoint(VfwCheckpoint* cp, const char* name)
{


    IFX_ASSERT (IFX_VERBOSE_LEVEL_ERROR, cp != NULL_PTR);
    IFX_ASSERT (IFX_VERBOSE_LEVEL_ERROR, vfwRegistryCount <= VFW_MAX_CHECKPOINTS);

    /* Initialize Object */
    cp->name = name;
    cp->signature = VFW_generateSignature(name);
    cp->id = vfwRegistryCount;
    cp->callCount = 0;
    cp->maxTimeTicks = 0;
    cp->lastTimeTicks = 0;
    cp->isRegistered = TRUE;

    VFW_GrantSafeMemAccess();

    vfwRegistry[vfwRegistryCount] = cp;
    vfwRegistryCount++;

    VFW_ReleaseSafeMemAccess();
}

void VFW_CheckpointEntry(VfwCheckpoint* cp)
{

    if (cp && cp->isRegistered)
    {
        VFW_GrantSafeMemAccess();


        VFW_integrity_accumulator += cp->signature;

        cp->startTimeTicks = IfxStm_get(&MODULE_STM0);

        VFW_ReleaseSafeMemAccess();

    }

}

void VFW_CheckpointExit(VfwCheckpoint* cp)
{

    if (cp && cp->isRegistered)
    {
        uint64 endTime = IfxStm_get(&MODULE_STM0);
        uint64 delta = endTime - cp->startTimeTicks;

        cp->lastTimeTicks = delta;
        if (delta > cp->maxTimeTicks)
        {
            cp->maxTimeTicks = delta;
        }
        cp->callCount++;

        VFW_GrantSafeMemAccess();

        VFW_integrity_accumulator -= cp->signature;

        VFW_ReleaseSafeMemAccess();

    }

}

boolean VFW_HasIntegrityCheckFailed(void)
{
    return VFW_integrityCheckFailed;
}

#pragma optimize 0

boolean VFW_IntegrityCheck(void)
{
    VFW_GrantSafeMemAccess();

    if (VFW_integrity_accumulator != 0U)
    {
        /* SAFETY VIOLATION: Execution path error (imbalanced calls) */
        IfxCpu_disableInterrupts();
        IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
        VFW_integrityCheckFailed = TRUE;

        return FALSE;
    }

    VFW_integrityCheckFailed = FALSE;

    VFW_ReleaseSafeMemAccess();

    return TRUE;
}
#pragma endoptimize


