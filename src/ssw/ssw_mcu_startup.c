/******************************************************************************
 * @file    ssw_mcu_startup.c
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
#include "ebcm_main.h"
#include "ssw_mcu_startup.h"
#include "IfxFce_reg.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define MCU_STARTUP_EXPECTED_CRC_00      0x0691B3AE

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
typedef struct
{
    volatile Ifx_UReg_32Bit* regUnderTest;
    uint32 mask;
} McuStartupType;

const McuStartupType mcuStartupCheck[] =
{
    /* regUnderTest,                mask        */
    { &DMU_HP_PROCONP00.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP01.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP02.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP03.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP04.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP04.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP05.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP10.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP11.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP12.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP13.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP14.U,       0xFFFFFFFFU     },
    { &DMU_HP_PROCONP15.U,       0xFFFFFFFFU     },
    { &DMU_HF_PROCONPF.U,        0x80000000U     },
    { &DMU_HF_PROCONUSR.U,       0xFFFFFFFFU     },
    { &DMU_HF_PROCONDF.U,        0xFFFFFFFFU     },
    { &DMU_HF_PROCONRAM.U,       0xFFFFFFFFU     },
    { &DMU_HF_PROCONDBG.U,       0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SP_PROCONHSM.U,       0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SF_PROCONUSR.U,       0xFFFFFFFFU     },
    { &DMU_SP_PROCONHSMCBS.U,    0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SP_PROCONHSMCX0.U,    0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SP_PROCONHSMCX1.U,    0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SP_PROCONHSMCOTP0.U,  0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SP_PROCONHSMCOTP1.U,  0xFFFFFFFFU     }, /* only if in use */
    { &DMU_SP_PROCONHSMCFG.U,    0xFFFFFFFFU     }, /* only if in use */
    { &DMU_HP_PROCONOTP00.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP01.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP02.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP03.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP04.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP05.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP10.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP11.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP12.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP13.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP14.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONOTP15.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP00.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP01.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP02.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP03.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP04.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP05.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP10.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP11.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP12.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP13.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP14.U,     0xFFFFFFFFU     },
    { &DMU_HP_PROCONWOP15.U,     0xFFFFFFFFU     },
    { &DMU_HF_PROCONTP.U,        0xFFFFFFFFU     },
    { &SCU_STSTAT.U,             0x000000FFU     }, /*Exception for TC33x and TC32 -> PMSWSTAT.HWCFGEVR.U */
};

const int mcuStartupCheckSize = sizeof(mcuStartupCheck) / sizeof(McuStartupType);


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

void EbcmSsw_mcuStartup(void)
{
    /* global variable to check is the CRC calculation is matching as expected */
    ebcmStatus.sswStatus.mcuStartupStatus = TEST_NOT_EVAL;

    /* Set crc_value to initial seed value */
    uint32 initialSeed00 = 0x00000000;
    uint32 crcValueXor00 = initialSeed00;

    /* And start to calculate the CRC value for all register values */
    for (uint8 crcElement = 0; crcElement < mcuStartupCheckSize; crcElement++)
    {   /* CRC Calculation with TriCore built in CRC instruction */
        uint32 currentRegValue = *(volatile uint32*)mcuStartupCheck[crcElement].regUnderTest;
        crcValueXor00 = __crc32(crcValueXor00, (currentRegValue & mcuStartupCheck[crcElement].mask));
    }

    if(crcValueXor00 == MCU_STARTUP_EXPECTED_CRC_00)
    {
        ebcmStatus.sswStatus.mcuStartupStatus = PASSED;
    }

    /* If value is not as expected appropriate reaction shall be taken */
    else
    {
        ebcmStatus.sswStatus.mcuStartupStatus = FAILED;
    }
}
