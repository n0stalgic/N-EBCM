/******************************************************************************
 * @file    vfw_registry.h
 * @brief   VFW Runtime Registry Definitions
 *
 * MIT License
 *
 * Copyright (c) 2026 n0stalgic
 *****************************************************************************/

#ifndef VFW_VFW_REGISTRY_H_
#define VFW_VFW_REGISTRY_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define VFW_MAX_CHECKPOINTS 64U

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * @brief Checkpoint Object
 * Allocated by the user (module), registered with VFW.
 */
typedef struct
{
    /* Metadata (Database) */
    const char* name;           /**< Human readable name (Flash) */
    uint32      signature;      /**< Auto-generated hash */
    uint16      id;             /**< Registry Index */
    boolean     isRegistered;   /**< Registration State */

    /* Profiling Stats */
    uint32      callCount;      /**< Number of times executed */
    uint64      maxTimeTicks;   /**< Peak execution time */
    uint64      lastTimeTicks;  /**< Last execution time */

    /* Runtime State */
    uint64      startTimeTicks; /**< Temp storage for start time */

} VfwCheckpoint;

#endif /* VFW_VFW_REGISTRY_H_ */
