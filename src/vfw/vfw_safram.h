/******************************************************************************
 * @file    vfw_safram.h
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

#ifndef VFW_VFW_SAFRAM_H_
#define VFW_VFW_SAFRAM_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

#define VFW_SAFRAM_DEF_BYTE_NEAR(_var) __attribute__((section("dsram0_safe_brk"))) __near uint8 _var
#define VFW_SAFRAM_DEF_BYTE_NEAR_CMPL(_var) __attribute((section("dsram0_safe_brk_cmpl"))) __near uint8 _var
#define VFW_SAFRAM_DEF_WORD_NEAR(_var) __attribute__((section("dsram0_safe_brk"))) __near uint16 _var
#define VFW_SAFRAM_DEF_WORD_NEAR_CMPL(_var) __attribute((section("dsram0_safe_brk_cmpl"))) __near uint16 _var
#define VFW_SAFRAM_DEF_DWORD_NEAR(_var) __attribute__((section("dsram0_safe_brk"))) __near uint32 _var
#define VFW_SAFRAM_DEF_DWORD_NEAR_CMPL(_var) __attribute((section("dsram0_safe_brk_cmpl"))) __near uint32 _var
#define VFW_SAFRAM_DEF_FLOAT32_NEAR(_var) __attribute__((section("dsram0_safe_brk"))) __near float32 _var
#define VFW_SAFRAM_DEF_FLOAT32_NEAR_CMPL(_var) __attribute((section("dsram0_safe_brk_cmpl"))) __near float32 _var

#define VFW_SAFRAM_COMPLMNT(_value)  (~(_value))

#define VFW_SAFRAM_SET(_var, _value) \
            do { \
              (_var) = (_value); \
              (_var##_cmpl) = (typeof(_var))VFW_SAFRAM_COMPLMNT(_value); \
            } while (0)

#define VFW_SAFRAM_GET(_var, _errorcb) \
            ({ \
                typeof(_var) _val = (_var); \
                typeof(_var) _cmpl = (_var##_cmpl); \
                if (_val != (typeof(_var))VFW_SAFRAM_COMPLMNT(_cmpl)) { \
                    _errorcb(); \
                } \
                _val; \
            })

#define VFW_SAFRAM_GET_ATOMIC(_var, _errorcb) \
            ({ \
                __disable(); \
                typeof(_var) _val = (_var); \
                typeof(_var) _cmpl = (_var##_cmpl); \
                __enable(); \
                if (_val != (typeof(_var))VFW_SAFRAM_COMPLMNT(_cmpl)) { \
                    _errorcb(); \
                } \
                _val; \
            })


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/


#endif /* VFW_VFW_SAFRAM_H_ */
