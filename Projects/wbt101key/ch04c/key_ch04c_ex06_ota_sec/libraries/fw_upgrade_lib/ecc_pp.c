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
 * ECDSA signature verification procedure.
 *
 */
#include "wiced_bt_trace.h"
#include "p_256_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"
#include <stdio.h>
#include <string.h>

#include "wiced_memory.h"

int nd;

#define KEY_LENGTH_BITS             256
#define KEY_LENGTH_DWORDS           (KEY_LENGTH_BITS / DWORD_BITS)
#define KEY_LENGTH_BYTES            (KEY_LENGTH_DWORDS * DWORD_BYTES)

/* Macro for endianess swap */
#define BE_SWAP(buf, index) \
    ((buf[index+0] << 24) | \
     (buf[index+1] << 16) | \
     (buf[index+2] << 8) | \
     (buf[index+3] << 0))

DWORD curve_n[KEY_LENGTH_DWORDS] = { 0xFC632551, 0xF3B9CAC2, 0xA7179E84, 0xBCE6FAAD, 0xFFFFFFFF, 0xFFFFFFFF, 0x0, 0xFFFFFFFF };

#define modp ((DWORD *)curve_p256.p)
#define modn ((DWORD *)(curve_n))


UINT32 nprime[KEY_LENGTH_DWORDS] = { 0xEE00BC4F, 0xCCD1C8AA, 0x7D74D2E4, 0x48C94408, 0xC588C6F6, 0x50FE77EC, 0xA9D6281C, 0x60D06633};

UINT32 rr[KEY_LENGTH_DWORDS] = { 0xBE79EEA2, 0x83244C95, 0x49BD6FA6, 0x4699799C, 0x2B6BEC59, 0x2845B239, 0xF3D95620, 0x66E12D94 }; 

#if ((OTA_CHIP == 20703) || (WICEDX == TRUE))
void CopyPoint(Point *q, Point *p)
{
    memcpy(q, p, sizeof(Point));
}
#endif

// q=q+p,     p is affine point
void ECC_Add_(Point *r, Point *p, Point *q, UINT32 keyLength)
{
    DWORD *x1, *x2, *x3, *y1, *y2, *y3, *z1, *z3;
#if 0
    DWORD t1[KEY_LENGTH_DWORDS], t2[KEY_LENGTH_DWORDS];
    DWORD k[KEY_LENGTH_DWORDS];
    DWORD s[KEY_LENGTH_DWORDS];
#else
    DWORD *t1 = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *t2 = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *k = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *s = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
#endif
    x1 = p->x; y1 = p->y; z1 = p->z;
    x2 = q->x; y2 = q->y;
    x3 = r->x; y3 = r->y; z3 = r->z;

    // if P=infinity, return q
    if (MP_isZero(z1, keyLength))
    {
        CopyPoint(r, q);
        goto end;
        return;
    }

    MP_MersennsSquaMod(t1, z1, keyLength);				// t1=z1^2
    MP_MersennsMultMod(t2, z1, t1, keyLength);			// t2=t1*z1
    MP_MersennsMultMod(t1, x2, t1, keyLength);			// t1=t1*x2
    MP_MersennsMultMod(t2, y2, t2, keyLength);			// t2=t2*y2

    MP_SubMod(t1, t1, x1, keyLength);				// t1=t1-x1
    MP_SubMod(t2, t2, y1, keyLength);				// t2=t2-y1


    if (MP_isZero(t1, keyLength))
    {
        if (MP_isZero(t2, keyLength))
        {
            ECC_Double(r, q, keyLength);
            goto end;
            return;
        }
        else
        {
            MP_Init(z3, keyLength);
            goto end;
            return;				// return infinity
        }
    }

    MP_MersennsMultMod(z3, z1, t1, keyLength);					// z3=z1*t1
    MP_MersennsSquaMod(s, t1, keyLength);						// t3=t1^2
    MP_MersennsMultMod(k, s, t1, keyLength);					// t4=t3*t1
    MP_MersennsMultMod(s, s, x1, keyLength);					// t3=t3*x1
    MP_LShiftMod(t1, s, keyLength);					// t1=2*t3
    MP_MersennsSquaMod(x3, t2, keyLength);						// x3=t2^2
    MP_SubMod(x3, x3, t1, keyLength);						// x3=x3-t1
    MP_SubMod(x3, x3, k, keyLength);						// x3=x3-t4
    MP_SubMod(s, s, x3, keyLength);						// t3=t3-x3
    MP_MersennsMultMod(s, s, t2, keyLength);					// t3=t3*t2
    MP_MersennsMultMod(k, k, y1, keyLength);					// t4=t4*t1
    MP_SubMod(y3, s, k, keyLength);
end:
    wiced_bt_free_buffer(t1);
    wiced_bt_free_buffer(t2);
    wiced_bt_free_buffer(k);
    wiced_bt_free_buffer(s);
}

// c=a*b; c must have a buffer of 2*Key_LENGTH_DWORDS, c != a != b
void MP_Mult(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength)
{
    UINT32 i, j;
    DWORD W, U, V;

    U = V = W=0;
    MP_Init(c, keyLength);

    //assume little endian right now
    for(i=0; i<keyLength; i++)
    {
        U = 0;
        for(j=0; j<keyLength; j++)
        {
            {
                UINT64 result;
                result = ((UINT64)a[i]) * ((UINT64)b[j]);
                W = result >> 32;
            }

            V=a[i]*b[j];

            V=V+U;
            U=(V<U);
            U+=W;

            V = V+c[i+j];
            U += (V<c[i+j]);

            c[i+j] = V;
        }
        c[i+keyLength]=U;
    }
}

DWORD MP_LAdd(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength)
{
    int i;
    DWORD carrier;
    DWORD temp;

    carrier = 0;
    for (i = 0; i<keyLength * 2; i++)
    {
        temp = a[i] + carrier;
        carrier = (temp<carrier);
        temp += b[i];
        carrier |= (temp<b[i]);
        c[i] = temp;
    }

    return carrier;
}


// Montgomery reduction
void MP_MontReduction(DWORD *q, DWORD* c, UINT32 keyLength)
{
    BOOL32 carry = 0;
#if 0
    DWORD a[KEY_LENGTH_DWORDS * 2];
    DWORD y[KEY_LENGTH_DWORDS * 2];
#else
    DWORD *a = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD) * 2);
    DWORD *y = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD) * 2);
    if (!a || !y)
    {
    	WICED_BT_TRACE("no mem\n");
    	return;
    }
#endif
    // q = c mod r
    memcpy(q, c, KEY_LENGTH_BYTES);

    // y = (c mod r) * nprime
    MP_Mult(y, q, (DWORD*)nprime, keyLength);

    // q = ((c mod r ) * nprime) mod r
    memcpy(q, y, KEY_LENGTH_BYTES);

    // y = q * n
    MP_Mult(y, q, modn, keyLength);

    // a = c + q * n
    if (MP_LAdd(a, c, y, keyLength))
        carry = 1;

    // q = (c + qn) / r
    memcpy(q, a + KEY_LENGTH_DWORDS, KEY_LENGTH_BYTES);

    // if q > n then q -= n
    if (carry || MP_CMP(q, modn, keyLength) > 0)
        MP_Sub(q, q, modn, keyLength);

    wiced_bt_free_buffer(a);
    wiced_bt_free_buffer(y);
}

void MP_InvMod_(DWORD *aminus, DWORD *u, const DWORD* modulus, UINT32 keyLength)
{
#if 0
    DWORD v[KEY_LENGTH_DWORDS];
    DWORD A[KEY_LENGTH_DWORDS + 1], C[KEY_LENGTH_DWORDS + 1];
#else
    DWORD *v = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *A = (DWORD *)wiced_bt_get_buffer((KEY_LENGTH_DWORDS + 1) * sizeof(DWORD));
    DWORD *C = (DWORD *)wiced_bt_get_buffer((KEY_LENGTH_DWORDS + 1) * sizeof(DWORD));
    if (!v || !A || !C)
    {
    	WICED_BT_TRACE("no mem\n");
    	return;
    }
#endif
    MP_Copy(v, (DWORD*)modulus, keyLength);
    MP_Init(A, keyLength);
    MP_Init(C, keyLength);
    A[0] = 1;

    while (!MP_isZero(u, keyLength))
    {
        while (!(u[0] & 0x01))					// u is even
        {
            MP_RShift(u, u, keyLength);
            if (!(A[0] & 0x01))					// A is even
                MP_RShift(A, A, keyLength);
            else
            {
                A[KEY_LENGTH_DWORDS] = MP_Add(A, A, (DWORD*)modulus, keyLength);	// A =A+p
                MP_RShift(A, A, keyLength);
                A[KEY_LENGTH_DWORDS - 1] |= (A[KEY_LENGTH_DWORDS] << 31);
            }
        }

        while (!(v[0] & 0x01))					// v is even
        {
            MP_RShift(v, v, keyLength);
            if (!(C[0] & 0x01))					// C is even
                MP_RShift(C, C, keyLength);
            else
            {
                C[KEY_LENGTH_DWORDS] = MP_Add(C, C, (DWORD*)modulus, keyLength);	// C =C+p
                MP_RShift(C, C, keyLength);
                C[KEY_LENGTH_DWORDS - 1] |= (C[KEY_LENGTH_DWORDS] << 31);
            }
        }

        if (MP_CMP(u, v, keyLength) >= 0)
        {
            MP_Sub(u, u, v, keyLength);
            if (MP_Sub(A, A, C, keyLength))
                MP_Add(A, A, (DWORD*)modulus, keyLength);
        }
        else
        {
            MP_Sub(v, v, u, keyLength);
            if (MP_Sub(C, C, A, keyLength))
                MP_Add(C, C, (DWORD*)modulus, keyLength);
        }
    }

    if (MP_CMP(C, (DWORD*)modulus, keyLength) >= 0)
    {
        MP_Sub(aminus, C, (DWORD*)modulus, keyLength);
    }
    else
    {
        MP_Copy(aminus, C, keyLength);
    }
    wiced_bt_free_buffer(v);
    wiced_bt_free_buffer(A);
    wiced_bt_free_buffer(C);
}

void MP_MultMont(DWORD *c, DWORD *a, DWORD *b, UINT32 keyLength)
{
    DWORD cc[2 * KEY_LENGTH_DWORDS];
    MP_Mult(cc, a, b, keyLength);
    MP_MontReduction(c, cc, keyLength);
}

// ECDSA Verify
//819A1 rom has ecdsa_verify, but that does not works. Probably stack overflow causing device restart
BOOL32 ecdsa_verify_(unsigned char* digest, unsigned char* signature, Point* key)
{
    UINT32 i;
#if 0
    Point p1, p2;
    DWORD e[KEY_LENGTH_DWORDS];
    DWORD r[KEY_LENGTH_DWORDS];
    DWORD s[KEY_LENGTH_DWORDS];

    DWORD u1[KEY_LENGTH_DWORDS];
    DWORD u2[KEY_LENGTH_DWORDS];

    DWORD tmp1[KEY_LENGTH_DWORDS];
    DWORD tmp2[KEY_LENGTH_DWORDS];
#else
    Point *p1 = (Point *)wiced_bt_get_buffer(sizeof(Point));
    Point *p2 = (Point *)wiced_bt_get_buffer(sizeof(Point));
    DWORD *e = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *r = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *s = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *u1 = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *u2 = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *tmp1 = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));
    DWORD *tmp2 = (DWORD *)wiced_bt_get_buffer(KEY_LENGTH_DWORDS * sizeof(DWORD));

    if (!p1 || !p2 || !e || !r || !s || !u1 || !u2 || !tmp1 || !tmp2)
    {
    	WICED_BT_TRACE("no mem\n");
    	return FALSE;
    }
#endif
    // swap input data endianess
    for (i = 0; i < KEY_LENGTH_DWORDS; i++)
   {
       // import digest to long integer
        e[KEY_LENGTH_DWORDS - 1 - i] = BE_SWAP(digest, 4 * i);

       // import signature to long integer
        r[KEY_LENGTH_DWORDS - 1 - i] = BE_SWAP(signature, 4 * i);
        s[KEY_LENGTH_DWORDS - 1 - i] = BE_SWAP(signature, KEY_LENGTH_BYTES + 4 * i);
   }

   // compute s' = s ^ -1 mod n
   MP_InvMod_(tmp1, s, modn, KEY_LENGTH_DWORDS);
    // MP_InvMod(tmp1, s, KEY_LENGTH_DWORDS);

   // convert s' to montgomery domain
   MP_MultMont(tmp2, tmp1, (DWORD*)rr, KEY_LENGTH_DWORDS);

   // convert e to montgomery domain
   MP_MultMont(tmp1, e, (DWORD*)rr, KEY_LENGTH_DWORDS);

   // compute u1 = e * s' mod n
   MP_MultMont(u1, tmp1, tmp2, KEY_LENGTH_DWORDS);

   // convert r to montgomery domain
   MP_MultMont(tmp1, r, (DWORD*)rr, KEY_LENGTH_DWORDS);

   // compute u2 = r * s' (mod n)
   MP_MultMont(u2, tmp1, tmp2, KEY_LENGTH_DWORDS);

   // set tmp1 = 1
   MP_Init(tmp1, KEY_LENGTH_DWORDS);
   tmp1[0] = 1;

   // convert u1 to normal domain
   MP_MultMont(u1, u1, tmp1, KEY_LENGTH_DWORDS);

   // convert u2 to normal domain
   MP_MultMont(u2, u2, tmp1, KEY_LENGTH_DWORDS);

   // compute (x,y) = u1G + u2QA
    if (key)
   {
        // if public key is given, using legacy method
#if 0
        ECC_PM(&p1, &(curve_p256.G), u1, KEY_LENGTH_DWORDS);
#else
        ECC_PM(p1, &(curve_p256.G), u1, KEY_LENGTH_DWORDS);
#endif
#ifdef OTA_UPGRADE_DEBUG
        WICED_BT_TRACE("ECC_PM1\n");
#if 0
        dump_hex(&p1, 96);
#else
        dump_hex(p1, 96);
#endif
#endif
#if 0
        ECC_PM(&p2, key, u2, KEY_LENGTH_DWORDS);
#else
        ECC_PM(p2, key, u2, KEY_LENGTH_DWORDS);
#endif
#ifdef OTA_UPGRADE_DEBUG
        WICED_BT_TRACE("ECC_PM2\n");
#if 0
        dump_hex(&p2, 96);
#else
        dump_hex(p2, 96);
#endif
#endif
#if 0
        ECC_Add_(&p1, &p1, &p2, KEY_LENGTH_DWORDS);
#else
        ECC_Add_(p1, p1, p2, KEY_LENGTH_DWORDS);
#endif
#ifdef OTA_UPGRADE_DEBUG
        WICED_BT_TRACE("ECC_Add\n");
#if 0
        dump_hex(&p1, 96);
#else
        dump_hex(p1, 96);
#endif
#endif
       // convert point to affine domain
#if 0
       MP_InvMod_(tmp1, p1.z, modp, KEY_LENGTH_DWORDS);
       MP_MersennsSquaMod(p1.z, tmp1, KEY_LENGTH_DWORDS);
       MP_MersennsMultMod(p1.x, p1.x, p1.z, KEY_LENGTH_DWORDS);
#else
       MP_InvMod_(tmp1, p1->z, modp, KEY_LENGTH_DWORDS);
       MP_MersennsSquaMod(p1->z, tmp1, KEY_LENGTH_DWORDS);
       MP_MersennsMultMod(p1->x, p1->x, p1->z, KEY_LENGTH_DWORDS);
#endif
#ifdef OTA_UPGRADE_DEBUG
       WICED_BT_TRACE("MP_Mult\n");
#if 0
        dump_hex(&p1, 96);
#else
        dump_hex(p1, 96);
#endif
#endif
   }
#if ECC_SHAMIR_SUPPORTED
   else
   {
       // if public key is not given, using pre-computed method
       ecdsa_fp_shamir(&p1, u1, u2);
   }
#endif

   // w = r (mod n)
    if (MP_CMP(r, modp, KEY_LENGTH_DWORDS) >= 0)
       MP_Sub(r, r, modp, KEY_LENGTH_DWORDS);

   // verify r == x ?
#if 0
    i = (memcmp(r, p1.x, KEY_LENGTH_BYTES) == 0);
#else
    i = (memcmp(r, p1->x, KEY_LENGTH_BYTES) == 0);

    wiced_bt_free_buffer(p1);
    wiced_bt_free_buffer(p2);
    wiced_bt_free_buffer(e);
    wiced_bt_free_buffer(r);
    wiced_bt_free_buffer(s);
    wiced_bt_free_buffer(u1);
    wiced_bt_free_buffer(u2);
    wiced_bt_free_buffer(tmp1);
    wiced_bt_free_buffer(tmp2);
#endif
    return i;
}

