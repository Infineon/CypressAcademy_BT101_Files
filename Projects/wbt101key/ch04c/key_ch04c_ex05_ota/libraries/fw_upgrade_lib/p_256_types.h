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
 * Basic type definitions for the ECC operations
 *
 */
#ifndef P_256_TYPES_H
#define P_256_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Basic Types
//

#if defined(WIN32)
typedef char                CHAR;
typedef short               SHORT;
typedef long                LONG;
#if 0
typedef unsigned char       UINT8;
typedef signed   char       INT8;
typedef unsigned short      UINT16;
typedef signed   short      INT16;
typedef unsigned int        UINT32;
typedef signed   int        INT32;
typedef unsigned __int64    UINT64;
#endif
typedef signed   __int64    INT64;

typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

typedef unsigned char       uint8;
typedef signed   char       int8;
typedef unsigned short      uint16;
typedef signed   short      int16;
typedef unsigned long       uint32;
typedef signed   long       int32;

#else

#if 1
typedef unsigned char       UINT8;
typedef signed   char       INT8;
typedef unsigned short      UINT16;
typedef signed   short      INT16;

#if OTA_CHIP == 20703
typedef unsigned long        UINT32;
typedef signed   long        INT32;
#else
typedef unsigned int        UINT32;
typedef signed   int        INT32;
#endif

typedef unsigned long long  UINT64;
#endif
typedef signed long long    INT64;

typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

typedef unsigned char       uint8;
typedef signed   char       int8;
typedef unsigned short      uint16;
typedef signed   short      int16;
typedef unsigned long       uint32;
typedef signed   long       int32;

#endif // win32

// Machine dependent data types, for processor efficiency
typedef unsigned int MBOOL;
typedef unsigned int MU8;
typedef unsigned int MU16;
typedef unsigned int MU32;
typedef int MI8;
typedef int MI16;
typedef int MI32;


// This definition is for backward compatibility
typedef UINT32          BOOL32;
typedef UINT8            BOOL8;

#ifndef NULL
#define NULL 0
#endif


//
// Rounding macros
//
#define ROUND2(n)       (((n) + 1) & ~1)
#define ROUND4(n)       (((n) + 3) & ~3)
#define ROUND8(n)       (((n) + 7) & ~7)

//
// inline macro
//

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
//
// #define FOO FREESODA
// printf("%s    %s\n", STRINGIFY(FOO), TOSTRING(FOO));
//
// The code above will generate the following output:
//        FOO    FREESODA

//
// Assume we are compiled under Little Endian system.
#define BE_SWAP_16(var)  (( UINT16)( ((var) << 8) | (((var) >> 8) & 0xff) ) )
#define LE_SWAP_16(var)  (var)
#define BE_SWAP_32(var)  (     ((var & 0x000000FF)<<24)  \
                           |   ((var & 0x0000FF00)<<8)   \
                           |   ((var & 0x00FF0000)>>8)   \
                           |  ((var & 0xFF000000)>>24)   \
                         )
#define LE_SWAP_32(var)    (var)

#ifndef MAX
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#ifdef __cplusplus
}
#endif

/*
*******************************************************************************
*
* $HISTORY:$
*
*******************************************************************************
*/
#endif /* P_256_TYPES_H */
