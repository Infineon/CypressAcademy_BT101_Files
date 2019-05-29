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
 * Multiprecision library definitions.
 *
 */

#ifndef MULTPRECISION_H
#define MULTPRECISION_H

#include "p_256_types.h"

//#define ARM_ASM

#define DWORD_BITS 		        32
#define DWORD_BYTES 	        4
#define DWORD_BITS_SHIFT        5

#define KEY_LENGTH_BITS			256
#define KEY_LENGTH_DWORDS   	(KEY_LENGTH_BITS/DWORD_BITS)
#define KEY_LENGTH_BYTES		(KEY_LENGTH_DWORDS*DWORD_BYTES)


/* return 1 if a > b, return -1 if a < b, return 0 if a == b */
int MP_CMP(DWORD *a, DWORD *b);

/* return 1 if a is zero, return 0 otherwise */
int MP_isZero(DWORD *a);

/* set c = 0 */
void MP_Init(DWORD *c);

/* assign c = a */
void MP_Copy(DWORD *c, DWORD *a);

/* return most significant bit postion */
UINT32 MP_MostSignBits(DWORD *a);

/* compute aminus = u ^ -1 mod modulus */
void MP_InvMod(DWORD *aminus, DWORD *u, const DWORD* modulus);

/* c = a + b */
DWORD MP_Add(DWORD *c, DWORD *a, DWORD *b);

/* c = a + b mod p */
void MP_AddMod(DWORD *c, DWORD *a, DWORD *b);

/* c = a - b */
DWORD MP_Sub(DWORD *c, DWORD *a, DWORD *b);

/* c = a - b mod p */
void MP_SubMod(DWORD *c, DWORD *a, DWORD *b);

/* c = a >> 1 */
void MP_RShift(DWORD * c, DWORD * a);

/* c = a << 1 */
DWORD MP_LShift(DWORD * c, DWORD * a);

/* c = a << 1  mod a */
void MP_LShiftMod(DWORD * c, DWORD * a);

/* c = a * b mod p */
void MP_MersennsMultMod(DWORD *c, DWORD *a, DWORD *b);

/* c = a * a mod p */
void MP_MersennsSquaMod(DWORD *c, DWORD *a);

/* c = a * b */
void MP_Mult(DWORD *c, DWORD *a, DWORD *b);

/* c = a * b * r^-1 mod n */
void MP_MultMont(DWORD *c, DWORD *a, DWORD *b);


#endif





