/******************************************************************************
 * @file    ssw_lbist.h
 * @brief   Interface for LBIST
 *
 * MIT License
 *
 * Copyright (c) 2025 n0stalgic
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

#ifndef INC_SSW_LBIST_H_
#define INC_SSW_LBIST_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "Ifx_Types.h"
#include "ssw.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

typedef struct
{
        ssw_test_status LBIST_not_term_by_porst;
        ssw_test_status LBIST_term_ok;
        ssw_test_status LBIST_test_done;
        ssw_test_status dev_not_defect;
} LBIST_status_t;

IFX_EXTERN LBIST_status_t LBIST_status;

 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

void ebcm_ssw_lbist(void);

#endif /* INC_SSW_LBIST_H_ */
