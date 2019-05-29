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
 * Eleptic Curve Point definition
 *
 */

#ifndef P_256_ECC_PP_H
#define P_256_ECC_PP_H

#include "p_256_multprecision.h"

#define width 4

struct _point
{
    DWORD x[KEY_LENGTH_DWORDS_P256];
    DWORD y[KEY_LENGTH_DWORDS_P256];
    DWORD z[KEY_LENGTH_DWORDS_P256];
};
typedef struct _point Point;

typedef struct
{
    // curve's coefficients
    DWORD a[KEY_LENGTH_DWORDS_P256];
    DWORD b[KEY_LENGTH_DWORDS_P256];

    //whether a is -3
    int a_minus3;

    // prime modulus
    DWORD p[KEY_LENGTH_DWORDS_P256];

    // Omega, p = 2^m -omega
    DWORD omega[KEY_LENGTH_DWORDS_P256];

    // base point, a point on E of order r
    Point G;

}EC;

#ifdef __cplusplus
extern "C" {
#endif

extern EC curve;
extern EC curve_p256;

void ECC_PM_B_NAF(Point *q, Point *p, DWORD *n, UINT32 keyLength);

#define ECC_PM(q, p, n, keyLength)  ECC_PM_B_NAF(q, p, n, keyLength)

void p_256_init_curve(UINT32 keyLength);
void InitPoint(Point *q);

void ECC_Double(Point *q, Point *p, UINT32 keyLength);
void CopyPoint(Point *q, Point *p);
BOOL32 ecdsa_verify_(unsigned char* digest, unsigned char* signature, Point* key);

#ifdef __cplusplus
}
#endif

#endif /* P_256_ECC_PP_H */
