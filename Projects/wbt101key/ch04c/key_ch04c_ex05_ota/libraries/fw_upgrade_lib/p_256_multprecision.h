/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 *
 * Multiprecision library functions.
 *
 */

#ifndef P_256_MULTPRECISION_H
#define P_256_MULTPRECISION_H

#include "p_256_types.h"

/* Type definitions */

#define DWORD_BITS      32
#define DWORD_BYTES     4
#define DWORD_BITS_SHIFT 5

#define KEY_LENGTH_DWORDS_P192 6
#define KEY_LENGTH_DWORDS_P256 8
/* Arithmetic Operations */


#ifdef __cplusplus
extern "C" {
#endif

int MP_CMP(DWORD *a, DWORD *b, UINT32 keyLength);
int MP_isZero(DWORD *a, UINT32 keyLength);
void MP_Init(DWORD *c, UINT32 keyLength);
void MP_Copy(DWORD *c, DWORD *a, UINT32 keyLength);
UINT32 MP_DWORDBits (DWORD a);
UINT32 MP_MostSignDWORDs(DWORD *a, UINT32 keyLength);
UINT32 MP_MostSignBits(DWORD *a, UINT32 keyLength);
void MP_InvMod(DWORD *aminus, DWORD *a, UINT32 keyLength);

DWORD MP_Add(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);           // c=a+b
void MP_AddMod(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);
DWORD MP_Sub(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);           // c=a-b
void MP_SubMod(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);
void MP_RShift(DWORD * c, DWORD * a, UINT32 keyLength);                 // c=a>>1, return carrier
void MP_LShiftMod(DWORD * c, DWORD * a, UINT32 keyLength);              // c=a<<b, return carrier
DWORD MP_LShift(DWORD * c, DWORD * a, UINT32 keyLength);                // c=a<<b, return carrier
void MP_Mult(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);           // c=a*b
void MP_MersennsMultMod(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);
void MP_MersennsSquaMod(DWORD *c, DWORD *a, UINT32 keyLength);
DWORD MP_LShift(DWORD * c, DWORD * a, UINT32 keyLength);
void MP_Mult(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength);
void MP_FastMod(DWORD *c, DWORD *a);
void MP_FastMod_P256(DWORD *c, DWORD *a);

#ifdef __cplusplus
}
#endif



#endif /* P_256_MULTPRECISION_H */




