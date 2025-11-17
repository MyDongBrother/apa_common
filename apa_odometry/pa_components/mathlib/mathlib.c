/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: ap_psd.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#define COMPONENT_MTL

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
// Component header include strategy:
// ==================================
// <swcpa_include.h>
//       I--> <appl_mathlib.h>
//                 I--> <mathlib_api.h>

// #include <pa_components/base_type/global_include.h>
#include "pa_components/mathlib/mathlib.h"
#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>

#include <string.h> //for memcpy

#if (LS_MTL_TESTHARNESS != SW_OFF)
#include <mathlib_testharness.h>
#endif

/*--------------------------------------------------------------------------*/
/*- Check if Compiler Switches (utilised in this file) are defined         -*/
/*--------------------------------------------------------------------------*/
#ifndef SW_ON
#error ('global switch SW_ON is not defined')
#endif
#ifndef SW_OFF
#error ('global switch SW_OFF is not defined')
#endif
#ifndef LS_MTL_TESTHARNESS
#error ('switch LS_MTL_TESTHARNESS is not defined')
#endif

#ifndef GS_MTL_USE_ASM
#error ('switch GS_MTL_USE_ASM is not defined')
#endif

#ifndef GS_MTL_TRIGONOMETRIC_FUNC
#error ('switch GS_MTL_TRIGONOMETRIC_FUNC is not defined')
#endif

#ifndef GS_MTL_FILTER_FUNC
#error ('switch GS_MTL_FILTER_FUNC is not defined')
#endif

#ifndef GS_MTL_USE_ERROR_HOOK
#error ('switch GS_MTL_USE_ERROR_HOOK is not defined')
#endif

#ifndef GS_PP_DEBUG
#error ('switch GS_PP_DEBUG is not defined')
#endif

/*--------------------------------------------------------------------------*/
/*- local symbolic constants                                               -*/
/*--------------------------------------------------------------------------*/
/*Definition of minimum value for SInt32, since QA-C provides error
  messages when constant GD_MIN_SI32 is used*/
#ifndef md_mtl_MIN_si32
#define md_mtl_MIN_si32 ((SInt32)(0 - GD_MAX_SI32 - 1))
#endif

/*--------------------------------------------------------------------------*/
/*- local macros                                                           -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local data types                                                       -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- local data                                                             -*/
/*--------------------------------------------------------------------------*/
#if (GS_PP_DEBUG != SW_OFF)
static UInt8 m_mtl_ErrorCurCounter_ui8  = 0;
static UInt8 m_mtl_ErrorPrevCounter_ui8 = 0;
#endif

#if (GS_MTL_USE_ASM != SW_ON)
/* look-up table for fast sqrt functions */
static const UInt32 m_mtl_Sqrt_LookUp_pui32[256] = {
    0,   16,  22,  27,  32,  35,  39,  42,  45,  48,  50,  53,  55,  57,  59,  61,
    64,  65,  67,  69,  71,  73,  75,  76,  78,  80,  81,  83,  84,  86,  87,  89,
    90,  91,  93,  94,  96,  97,  98,  99,  101, 102, 103, 104, 106, 107, 108, 109,
    110, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126,
    128, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142,
    143, 144, 144, 145, 146, 147, 148, 149, 150, 150, 151, 152, 153, 154, 155, 155,
    156, 157, 158, 159, 160, 160, 161, 162, 163, 163, 164, 165, 166, 167, 167, 168,
    169, 170, 170, 171, 172, 173, 173, 174, 175, 176, 176, 177, 178, 178, 179, 180,
    181, 181, 182, 183, 183, 184, 185, 185, 186, 187, 187, 188, 189, 189, 190, 191,
    192, 192, 193, 193, 194, 195, 195, 196, 197, 197, 198, 199, 199, 200, 201, 201,
    202, 203, 203, 204, 204, 205, 206, 206, 207, 208, 208, 209, 209, 210, 211, 211,
    212, 212, 213, 214, 214, 215, 215, 216, 217, 217, 218, 218, 219, 219, 220, 221,
    221, 222, 222, 223, 224, 224, 225, 225, 226, 226, 227, 227, 228, 229, 229, 230,
    230, 231, 231, 232, 232, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238,
    239, 240, 240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 247,
    247, 248, 248, 249, 249, 250, 250, 251, 251, 252, 252, 253, 253, 254, 254, 255};

#endif //(GS_MTL_USE_ASM != SW_ON)
/*--------------------------------------------------------------------------*/
/*- prototypes of local functions                                          -*/
/*--------------------------------------------------------------------------*/

/* Adjust result of g_mtl_SqrtExactFast_u32_ui16.
   Use inline option to speed-up the execution.
*/

#if (GS_MTL_USE_ASM != SW_ON)
INL_OPT static UInt16 m_mtl_Sqrt_Adjustment_ui16(UInt32 f_x_ui32, UInt32 f_xn_ui32);
#endif //(GS_MTL_USE_ASM != SW_ON)

/*--------------------------------------------------------------------------*/
/*- global data objects                                                    -*/
/*--------------------------------------------------------------------------*/

// tables with bitmasks

// mask with one bit set for every bit position
const UInt8 gc_mtl_Mask8bit_pui8[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

// mask with one bit cleared for every bit position
const UInt8 gc_mtl_Maskinv8bit_pui8[8] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};

// mask from above left shifted by one bit position
const UInt8 gc_mtl_Mask8bitShl_pui8[8] = {0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x00};

// mask from above right shifted by one bit position
const UInt8 gc_mtl_Mask8bitShr_pui8[8] = {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};

// mask with two bit set for every bit position
const UInt8 gc_mtl_Mask8bitDouble_pui8[7] = {0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0};

// mask with one bit set for every 2 bit positions
const UInt8 gc_mtl_Mask8bitspecial_pui8[6] = {0x01, 0x02, 0x04, 0x10, 0x20, 0x40};

// mask with one bit set for every bit position, reverse bit order
const UInt8 gc_mtl_Mask8bitRev_pui8[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

// mask with one bit cleared for every bit position, reverse bit order
const UInt8 gc_mtl_Maskinv8bitRev_pui8[8] = {0x7F, 0xBF, 0xDF, 0xEF,
                                             0xF7, 0xFB, 0xFD, 0xFE};

// mask with one bit set for every bit position
const UInt16 gc_mtl_Mask16bit_pui16[16] = {0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020,
                                           0x0040, 0x0080, 0x0100, 0x0200, 0x0400, 0x0800,
                                           0x1000, 0x2000, 0x4000, 0x8000};

// mask with two bit set for every bit position
const UInt16 gc_mtl_Mask16bitDouble_pui16[15] = {0x0003, 0x0006, 0x000C, 0x0018, 0x0030,
                                                 0x0060, 0x00C0, 0x0180, 0x0300, 0x0600,
                                                 0x0C00, 0x1800, 0x3000, 0x6000, 0xC000};

/*--------------------------------------------------------------------------*/
/*- definition of local functions                                          -*/
/*--------------------------------------------------------------------------*/

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name:
|     Purpose:
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results:
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/

/*--------------------------------------------------------------------------*/
/*- public functions                                                       -*/
/*--------------------------------------------------------------------------*/

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: g_mtl_GetError_bl
|     Purpose: - informs about invalid result in previous mathlib functions
|              - each call of g_mtl_GetError_bl supplies the error indicator and
|                resets the error indicator for subsequent calls of mathlib
functions
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: none
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: error indicator
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
void g_mtl_HexToStr_vd(UInt8 *pbDest, UInt8 *pbSrc, int nLen)
{
    UInt8 ddl, ddh;
    UInt8 i;

    for (i = 0; i < nLen; i++)
    {
        ddh = 48 + pbSrc[i] / 16;
        ddl = 48 + pbSrc[i] % 16;
        if (ddh > 57)
        {
            ddh = ddh + 7;
        }
        if (ddl > 57)
        {
            ddl = ddl + 7;
        }
        pbDest[i * 2]     = ddh;
        pbDest[i * 2 + 1] = ddl;
    }

    pbDest[nLen * 2] = '\0';
}

Boolean g_mtl_GetError_bl(void)
{
    Boolean l_result_bl = FALSE;

#if (GS_PP_DEBUG != SW_OFF)

    // error occured
    if (m_mtl_ErrorCurCounter_ui8 != m_mtl_ErrorPrevCounter_ui8)
    {
        // update previous counter
        m_mtl_ErrorPrevCounter_ui8 = m_mtl_ErrorCurCounter_ui8;

        // indicate error
        l_result_bl = TRUE;
    }
#endif

    // indicate no error
    return l_result_bl;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: g_mathlibInit_vd
|     Purpose: Called once during start of AUTOSAR SW-Component PA
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: none
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: none
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
void g_mathlibInit_vd(void) { return; }

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s8_Add_s8_ui8
|     Purpose: x_si8 + y_si8 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: f_X_si8, f_Y_si8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_s8_Add_s8_si8(SInt8 f_X_si8, SInt8 f_Y_si8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_si8 + (SInt16)f_Y_si8);

    // limit to lower bound
    if (l_Result_si16 < ((SInt16)GD_MIN_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MIN_SI8);
    }
    // limit to upper bound
    else if (l_Result_si16 > ((SInt16)GD_MAX_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MAX_SI8);
    }
    else
    { // QAC
    }

    return ((SInt8)l_Result_si16);
} // mtl_s8_Add_s8_ui8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s8_Add_u8_ui8
|     Purpose: x_si8 + y_ui8 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si8, f_Y_ui8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name:
fig1lr*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_s8_Add_u8_si8(SInt8 f_X_si8, UInt8 f_Y_ui8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_si8 + (SInt16)f_Y_ui8);

    // underflow impossible --> no underflow treatment
    // limit to upper bound in case of overflow
    if (l_Result_si16 > ((SInt16)GD_MAX_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MAX_SI8);
    }

    return ((SInt8)l_Result_si16);
}
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u8_Add_u8_ui8
|     Purpose: x_ui8 + y_ui8 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui8, f_Y_ui8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_u8_Add_u8_ui8(UInt8 f_X_ui8, UInt8 f_Y_ui8)
{
    UInt16 l_Result_ui16;

    l_Result_ui16 = (UInt16)((UInt16)f_X_ui8 + (UInt16)f_Y_ui8);

    // underflow impossible --> no underflow treatment
    // limit to upper bound in case of overflow
    if (l_Result_ui16 > ((UInt16)GD_MAX_UI8))
    {
        l_Result_ui16 = ((UInt16)GD_MAX_UI8);
    }

    return ((UInt8)l_Result_ui16);
} // mtl_u8_Add_u8_ui8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u8_Add_s8_ui8
|     Purpose: x_ui8 + y_si8 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui8, f_Y_si8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_u8_Add_s8_ui8(UInt8 f_X_ui8, SInt8 f_Y_si8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_ui8 + (SInt16)f_Y_si8);

    // limit to lower bound
    if (l_Result_si16 < ((SInt16)GD_MIN_UI8))
    {
        l_Result_si16 = ((SInt16)GD_MIN_UI8);
    }
    else
    {
        // limit to upper bound
        if (l_Result_si16 > ((SInt16)GD_MAX_UI8))
        {
            l_Result_si16 = ((SInt16)GD_MAX_UI8);
        }
    }

    return ((UInt8)l_Result_si16);
} // mtl_u8_Add_u8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Add_s16_si16
|     Purpose: x_si16 + y_si16 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Add_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_si16 + (SInt32)f_Y_si16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_SI16);
    }
    // limit to upper bound
    else if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }
    else
    { // QAC
    }

    return ((SInt16)l_Result_si32);
} // mtl_s16_Add_s16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Add_u16_si16
|     Purpose: x_si16 + y_ui16 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Add_u16_si16(SInt16 f_X_si16, UInt16 f_Y_ui16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_si16 + (SInt32)f_Y_ui16);

    // underflow impossible --> no underflow treatment
    // limit to upper bound
    if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }
    return ((SInt16)l_Result_si32);
} // mtl_s16_Add_u16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Add_u16_ui16
|     Purpose: x_ui16 + y_ui16 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_u16_Add_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16)
{
    UInt32 l_Result_ui32;

    l_Result_ui32 = ((UInt32)f_X_ui16 + (UInt32)f_Y_ui16);

    // underflow impossible --> no underflow treatment
    // limit to upper bound
    if (l_Result_ui32 > ((UInt32)GD_MAX_UI16))
    {
        l_Result_ui32 = ((UInt32)GD_MAX_UI16);
    }

    return ((UInt16)l_Result_ui32);
} // mtl_u16_Add_u16_ui16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s32_Add_s32_si32
|     Purpose: x_si32 + y_si32 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si32, f_Y_si32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_s32_Add_s32_si32(SInt32 f_X_si32, SInt32 f_Y_si32)
{
    UInt8 l_Sign_ui8 = ((UInt8)0);
    UInt32 l_X_ui32;
    UInt32 l_Y_ui32;
    SInt32 l_Result_si32;
    UInt32 l_Value_ui32 = ((UInt32)0);

    // get sign of operands, conversion to unsigned values
    if (f_X_si32 < 0L)
    {
        l_Sign_ui8 += ((UInt8)1);
    }
    l_X_ui32 = ((UInt32)g_mtl_Abs_mac(f_X_si32));

    if (f_Y_si32 < 0L)
    {
        l_Sign_ui8 += ((UInt8)1);
    }
    l_Y_ui32 = ((UInt32)g_mtl_Abs_mac(f_Y_si32));

    // both operands positive
    if (l_Sign_ui8 == ((UInt8)0))
    {
        l_Value_ui32 = g_mtl_u32_Add_u32_ui32(l_X_ui32, l_Y_ui32);
        // indicate positive result
        l_Result_si32 = 0L;
    }
    // both operands negative
    else if (l_Sign_ui8 == ((UInt8)2))
    {
        l_Value_ui32 = g_mtl_u32_Add_u32_ui32(l_X_ui32, l_Y_ui32);
        // indicate negative result
        l_Result_si32 = -1L;
    }
    // one operand positive and one negative
    else
    {
        if (f_X_si32 < 0L)
        {
            l_Result_si32 = g_mtl_s32_Add_u32_si32(f_X_si32, l_Y_ui32);
        }
        else
        {
            l_Result_si32 = g_mtl_s32_Add_u32_si32(f_Y_si32, l_X_ui32);
        }
    }

    // check boundaries
    // for l_Sign_ui8=1 the boundaries are checked automatically
    if (l_Sign_ui8 != ((UInt8)1))
    {
        // positive result
        if (l_Result_si32 == 0L)
        {
            // upper boundary
            if (l_Value_ui32 > ((UInt32)GD_MAX_SI32))
            {
                l_Result_si32 = GD_MAX_SI32;
            }
            else
            {
                l_Result_si32 = ((SInt32)l_Value_ui32);
            }
        }
        // negative result
        else
        {
            // lower boundary
            if (l_Value_ui32 > ((UInt32)GD_MAX_SI32 + (UInt32)1))
            {
                l_Result_si32 = md_mtl_MIN_si32;
            }
            else
            {
                l_Result_si32 = -((SInt32)l_Value_ui32);
            }
        }
    }

    return l_Result_si32;
} // mtl_s32_Add_s32_si32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s32_Add_u32_si32
|     Purpose: x_si32 + y_ui32 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si32, f_Y_ui32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_s32_Add_u32_si32(SInt32 f_X_si32, UInt32 f_Y_ui32)
{
    SInt32 l_Result_si32;
    UInt32 l_Value_ui32;

    // signed operand positive -> addition
    if (f_X_si32 >= 0L)
    {
        l_Value_ui32 = g_mtl_u32_Add_u32_ui32(((UInt32)f_X_si32), f_Y_ui32);

        // positive result indicator
        l_Result_si32 = 1L;
    }
    // signed operand negative -> subtraction
    else
    {
        // conversion of negative value into valid positive value
        l_Value_ui32 = ((UInt32)(-f_X_si32));

        // subtraction
        if (f_Y_ui32 >= l_Value_ui32)
        {
            l_Value_ui32 = f_Y_ui32 - l_Value_ui32;

            // positive result indicator
            l_Result_si32 = 1L;
        }
        else
        {
            l_Value_ui32 = l_Value_ui32 - f_Y_ui32;

            // negative result indicator
            l_Result_si32 = -1L;
        }
    }

    // check boundaries
    // positive result
    if (l_Result_si32 >= 0L)
    {
        // upper boundary
        if (l_Value_ui32 > ((UInt32)GD_MAX_SI32))
        {
            l_Result_si32 = GD_MAX_SI32;
        }
        else
        {
            l_Result_si32 = ((SInt32)l_Value_ui32);
        }
    }
    // negative result
    else
    {
        // lower boundary
        if (l_Value_ui32 > ((UInt32)GD_MAX_SI32 + (UInt32)1))
        {
            l_Result_si32 = md_mtl_MIN_si32;
        }
        else
        {
            l_Result_si32 = -((SInt32)l_Value_ui32);
        }
    }

    return l_Result_si32;
} // mtl_s32_Add_u32_si32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u32_Add_u32_ui32
|     Purpose: x_ui32 + y_ui32 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui32, f_Y_ui32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sum
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt32 g_mtl_u32_Add_u32_ui32(UInt32 f_X_ui32, UInt32 f_Y_ui32)
{
    UInt8 l_Carry_ui8 = ((UInt8)0);
    UInt32 l_Result_ui32;

    // clear highest bit -> no overflow after addition
    // determine carry bits with respect to bit 30
    if (g_mtl_TstBit_u32_bl(f_X_ui32, 31) != FALSE)
    {
        // clear highest bit
        g_mtl_ClrBit_u32_mac(f_X_ui32, 31);
        // increase carry bits
        l_Carry_ui8 += 1;
    }

    if (g_mtl_TstBit_u32_bl(f_Y_ui32, 31) != FALSE)
    {
        // clear highest bit
        g_mtl_ClrBit_u32_mac(f_Y_ui32, 31);
        // increase carry bits
        l_Carry_ui8 += 1;
    }

    // addition without overflow
    l_Result_ui32 = f_X_ui32 + f_Y_ui32;

    // 2 carry bits -> overflow
    if (l_Carry_ui8 > 1)
    {
        l_Result_ui32 = GD_MAX_UI32;
    }
    // 1 carry bit
    else if (l_Carry_ui8 > ((UInt8)0))
    {
        // bit 31 already  set -> overflow
        if (g_mtl_TstBit_u32_bl(l_Result_ui32, 31) != FALSE)
        {
            l_Result_ui32 = GD_MAX_UI32;
        }
        // bit 31 not set -> add carry bit to result
        else
        {
            g_mtl_SetBit_u32_mac(l_Result_ui32, 31);
        }
    }
    else
    { // QAC
    }

    // return result
    return l_Result_ui32;
}
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s8_Sub_s8_si8
|     Purpose: x_si8 - y_si8 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si8, f_Y_si8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_s8_Sub_s8_si8(SInt8 f_X_si8, SInt8 f_Y_si8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_si8 - (SInt16)f_Y_si8);

    // limit to lower bound
    if (l_Result_si16 < ((SInt16)GD_MIN_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MIN_SI8);
    }
    // limit to upper bound
    else if (l_Result_si16 > ((SInt16)GD_MAX_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MAX_SI8);
    }
    else
    { // QAC
    }

    return ((SInt8)l_Result_si16);
} // mtl_s8_Sub_s8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s8_Sub_u8_si8
|     Purpose: x_si8 - y_ui8 with under treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si8, f_Y_ui8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_s8_Sub_u8_si8(SInt8 f_X_si8, UInt8 f_Y_ui8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_si8 - (SInt16)f_Y_ui8);

    // limit to lower bound
    if (l_Result_si16 < ((SInt16)GD_MIN_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MIN_SI8);
    }
    // overflow impossible --> no overflow treatment

    return ((SInt8)l_Result_si16);
} // mtl_s8_Sub_u8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u8_Sub_u8_si8
|     Purpose: x_ui8 - y_ui8 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui8, f_Y_ui8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_u8_Sub_u8_si8(UInt8 f_X_ui8, UInt8 f_Y_ui8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_ui8 - (SInt16)f_Y_ui8);

    // limit to lower bound
    if (l_Result_si16 < ((SInt16)GD_MIN_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MIN_SI8);
    }
    // limit to upper bound
    else if (l_Result_si16 > ((SInt16)GD_MAX_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MAX_SI8);
    }
    else
    { // QAC
    }

    return ((SInt8)l_Result_si16);
} // mtl_u8_Sub_u8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u8_Sub_s8_si8
|     Purpose: x_ui8 - y_si8 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui8, f_Y_si8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_u8_Sub_s8_si8(UInt8 f_X_ui8, SInt8 f_Y_si8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_ui8 - (SInt16)f_Y_si8);

    // limit to upper bound
    if (l_Result_si16 > ((SInt16)GD_MAX_SI8))
    {
        l_Result_si16 = ((SInt16)GD_MAX_SI8);
    }
    // underflow impossible --> no underflow treatment

    return ((SInt8)l_Result_si16);
} // mtl_u8_Sub_s8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u8_Sub_u8_ui8
|     Purpose: x_ui8 - y_ui8 with underflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui8, f_Y_ui8 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_u8_Sub_u8_ui8(UInt8 f_X_ui8, UInt8 f_Y_ui8)
{
    SInt16 l_Result_si16;

    l_Result_si16 = (SInt16)((SInt16)f_X_ui8 - (SInt16)f_Y_ui8);

    // limit to lower bound
    if (l_Result_si16 < 0)
    {
        l_Result_si16 = ((SInt16)GD_MIN_UI8);
    }
    // overflow impossible --> no overflow treatment

    return ((UInt8)l_Result_si16);
} // mtl_u8_Sub_u8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Sub_s16_si16
|     Purpose: x_si16 - y_si16 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Sub_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_si16 - (SInt32)f_Y_si16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_SI16);
    }
    // limit to upper bound
    else if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }
    else
    { // QAC
    }

    return ((SInt16)l_Result_si32);
} // mtl_s16_Sub_s16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Sub_u16_si16
|     Purpose: x_si16 - y_ui16 with underflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Sub_u16_si16(SInt16 f_X_si16, UInt16 f_Y_ui16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_si16 - (SInt32)f_Y_ui16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_SI16);
    }
    // overflow impossible --> no overflow treatment

    return ((SInt16)l_Result_si32);
} // mtl_s16_Sub_u16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Sub_u16_si16
|     Purpose: x_ui16 - y_ui16 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_u16_Sub_u16_si16(UInt16 f_X_ui16, UInt16 f_Y_ui16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_ui16 - (SInt32)f_Y_ui16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_SI16);
    }
    // limit to upper bound
    else if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }
    else
    { // QAC
    }

    return ((SInt16)l_Result_si32);
} // mtl_u16_Sub_u16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Sub_s16_si16
|     Purpose: x_ui16 - y_si16 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_u16_Sub_s16_si16(UInt16 f_X_ui16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_ui16 - (SInt32)f_Y_si16);

    // underflow impossible --> no underflow treatment
    // limit to upper bound
    if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }

    return ((SInt16)l_Result_si32);
} // mtl_u16_Sub_s16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Sub_u16_ui16
|     Purpose: x_ui16 - y_ui16 with underflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_u16_Sub_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_ui16 - (SInt32)f_Y_ui16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_UI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_UI16);
    }
    // overflow impossible --> no overflow treatment

    return ((UInt16)l_Result_si32);
}
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s32_Sub_s32_si32
|     Purpose: x_si32 - y_si32 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si32, f_Y_si32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_s32_Sub_s32_si32(SInt32 f_X_si32, SInt32 f_Y_si32)
{
    SInt32 l_Result_si32;

    // x - y = x + (-y)

    if (f_Y_si32 > md_mtl_MIN_si32)
    {
        l_Result_si32 = g_mtl_s32_Add_s32_si32(f_X_si32, (-f_Y_si32));
    }
    // -GD_MIN_SI32 = GD_MAX_SI32 + 1 lead to overflow -> use GD_MAX_SI32 and
    // additional increment by 1 if possible
    else
    {
        l_Result_si32 = g_mtl_s32_Add_s32_si32(f_X_si32, GD_MAX_SI32);

        if (l_Result_si32 < GD_MAX_SI32)
        {
            l_Result_si32++;
        }
    }

    return l_Result_si32;
} // mtl_s32_Sub_s32_si32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s32_Sub_u32_si32
|     Purpose: x_si32 - y_ui32 with underflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si32, f_Y_ui32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_s32_Sub_u32_si32(SInt32 f_X_si32, UInt32 f_Y_ui32)
{
    SInt32 l_Result_si32;
    UInt32 l_Value_ui32;

    // subtraction
    if (f_X_si32 >= 0)
    {
        // check subtraction order
        if (((UInt32)f_X_si32) >= f_Y_ui32)
        {
            l_Result_si32 = (SInt32)(((UInt32)f_X_si32) - f_Y_ui32);
        }
        else
        {
            l_Value_ui32 = f_Y_ui32 - ((UInt32)f_X_si32);

            // check for underflow, GD_MIN_SI32 = - ( GD_MAX_SI32 + 1 )
            if (l_Value_ui32 <= ((UInt32)(GD_MAX_SI32) + (UInt32)(1)))
            {
                l_Result_si32 = -((SInt32)l_Value_ui32);
            }
            else
            {
                l_Result_si32 = md_mtl_MIN_si32;
            }
        }
    }
    // addition of 2 negative values
    else
    {
        l_Value_ui32 = g_mtl_u32_Add_u32_ui32((UInt32)(-f_X_si32), f_Y_ui32);

        // check for underflow, GD_MIN_SI32 = - ( GD_MAX_SI32 + 1 )
        if (l_Value_ui32 <= ((UInt32)(GD_MAX_SI32) + (UInt32)(1)))
        {
            l_Result_si32 = -((SInt32)l_Value_ui32);
        }
        else
        {
            l_Result_si32 = md_mtl_MIN_si32;
        }
    }

    return l_Result_si32;
} // mtl_s32_Sub_u32_si32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u32_Sub_u32_si32
|     Purpose: x_ui32 - y_ui32 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui32, f_Y_ui32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_u32_Sub_u32_si32(UInt32 f_X_ui32, UInt32 f_Y_ui32)
{
    SInt32 l_Result_si32;
    UInt32 l_Value_ui32;

    // check subtraction order
    if (f_X_ui32 >= f_Y_ui32)
    {
        l_Value_ui32 = f_X_ui32 - f_Y_ui32;

        // check for overflow
        if (l_Value_ui32 > ((UInt32)GD_MAX_SI32))
        {
            l_Result_si32 = GD_MAX_SI32;
        }
        else
        {
            l_Result_si32 = ((SInt32)l_Value_ui32);
        }
    }
    else
    {
        l_Value_ui32 = f_Y_ui32 - f_X_ui32;

        // check for underflow
        if (l_Value_ui32 > ((UInt32)GD_MAX_SI32))
        {
            l_Result_si32 = md_mtl_MIN_si32;
        }
        else
        {
            l_Result_si32 = -((SInt32)l_Value_ui32);
        }
    }

    return l_Result_si32;
} // mtl_u32_Sub_u32_si32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u32_Sub_s32_si32
|     Purpose: x_ui32 - y_si32 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui32, f_Y_si32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: difference
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: dvs3bmh
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_u32_Sub_s32_si32(UInt32 f_X_ui32, SInt32 f_Y_si32)
{
    SInt32 l_Result_si32;
    UInt32 l_Value_ui32;

    // subtraction
    if (f_Y_si32 >= 0)
    {
        // subtraction order
        if (f_X_ui32 >= ((UInt32)f_Y_si32))
        {
            l_Value_ui32 = f_X_ui32 - ((UInt32)f_Y_si32);

            // overflow protection
            if (l_Value_ui32 <= ((UInt32)GD_MAX_SI32))
            {
                l_Result_si32 = (SInt32)l_Value_ui32;
            }
            else
            {
                l_Result_si32 = GD_MAX_SI32;
            }
        }
        else
        {
            // under-/overflow impossible
            l_Result_si32 = (SInt32)f_X_ui32 - f_Y_si32;
        }
    }
    // addition
    else
    {
        if (f_Y_si32 == md_mtl_MIN_si32)
        {
            f_Y_si32      = md_mtl_MIN_si32 + 1;
            l_Result_si32 = g_mtl_s32_Add_u32_si32((-f_Y_si32), f_X_ui32);
        }
        else
        {
            l_Result_si32 = g_mtl_s32_Add_u32_si32((-f_Y_si32), f_X_ui32);
        }
    }

    return l_Result_si32;
} // mtl_u32_Sub_s32_si32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Mul_s16_si16
|     Purpose: x_si16 * y_si16 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: product
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Mul_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_si16 * (SInt32)f_Y_si16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_SI16);
    }
    // limit to upper bound
    else if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }
    else
    { // QAC
    }

    return ((SInt16)l_Result_si32);
} // mtl_s16_Mul_s16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Mul_u16_si16
|     Purpose: x_si16 * y_ui16 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: product
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Mul_u16_si16(SInt16 f_X_si16, UInt16 f_Y_ui16)
{
    SInt32 l_Result_si32;

    l_Result_si32 = ((SInt32)f_X_si16 * (SInt32)f_Y_ui16);

    // limit to lower bound
    if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MIN_SI16);
    }
    // limit to upper bound
    else if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Result_si32 = ((SInt32)GD_MAX_SI16);
    }
    else
    { // QAC
    }

    return ((SInt16)l_Result_si32);
} // mtl_s16_Mul_u16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Mul_u16_ui16
|     Purpose: x_ui16 * y_ui16 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: product
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_u16_Mul_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16)
{
    UInt32 l_Result_ui32;

    l_Result_ui32 = ((UInt32)f_X_ui16 * (UInt32)f_Y_ui16);

    // underflow impossible --> no underflow treatment
    // limit to upper bound
    if (l_Result_ui32 > ((UInt32)GD_MAX_UI16))
    {
        l_Result_ui32 = ((UInt32)GD_MAX_UI16);
    }

    return ((UInt16)l_Result_ui32);
} // g_mtl_u16_Mul_u16_ui16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Div_s16_si8
|     Purpose: x_si16 / y_si16 with rounding and under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_s16_Div_s16_si8(SInt16 f_X_si16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    // division by 0
    if (f_Y_si16 == 0)
    {
#if (GS_PP_DEBUG != SW_OFF)
        // indicate error
        m_mtl_ErrorCurCounter_ui8++;
#endif

        if (f_X_si16 >= 0)
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI8);
        }
        else
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI8);
        }
    }
    else
    {
        // rounding
        if (f_X_si16 >= 0)
        {
            l_Result_si32 = (((SInt32)f_X_si16) + ((SInt32)g_mtl_Abs_mac(f_Y_si16 / 2))) /
                            ((SInt32)f_Y_si16);
        }
        else
        {
            l_Result_si32 = (((SInt32)f_X_si16) - ((SInt32)g_mtl_Abs_mac(f_Y_si16 / 2))) /
                            ((SInt32)f_Y_si16);
        }

        // limit to lower bound
        if (l_Result_si32 < ((SInt32)GD_MIN_SI8))
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI8);
        }
        // limit to upper bound
        else if (l_Result_si32 > ((SInt32)GD_MAX_SI8))
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI8);
        }
        else
        { // QAC
        }
    }
    return ((SInt8)l_Result_si32);
} // mtl_s16_Div_s16_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: g_mtl_s16_Div_s16_si16
|     Purpose: x_si16 / y_si16 with rounding and under/overflow treatment
|              When y_si16 = 0, depending on the sign of x_si16
|                 result = 32767 , if x_si16 is >=0
|                 result = -32768 , if x_si16 is < 0
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient l_Result_si16
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: dvs3bmh
*****************************************************************************/
SInt16 g_mtl_s16_Div_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    // division by 0
    if (f_Y_si16 == 0)
    {
        if (f_X_si16 >= 0)
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI16);
        }
        else
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI16);
        }
    }
    else
    {
        // rounding. Round half up method is been used
        if (f_X_si16 >= 0)
        {
            l_Result_si32 = (((SInt32)f_X_si16) + ((SInt32)g_mtl_Abs_mac(f_Y_si16 / 2))) /
                            ((SInt32)f_Y_si16);
        }
        else
        {
            l_Result_si32 = (((SInt32)f_X_si16) - ((SInt32)g_mtl_Abs_mac(f_Y_si16 / 2))) /
                            ((SInt32)f_Y_si16);
        }

        // overflow handling- Just for one case, when f_X_si16 = -32768 and f_Y_si16
        // = -1 l_Result_si32 = 32768, which is out of range for SInt16 type, So
        // return 32767 in this case.
        if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI16);
        }
    }
    return ((SInt16)l_Result_si32);
} // g_mtl_s16_Div_s16_si16

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Div_u16_si8
|     Purpose: x_si16 / y_ui16 with rounding and under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_s16_Div_u16_si8(SInt16 f_X_si16, UInt16 f_Y_ui16)
{
    SInt32 l_Result_si32;

    // division by 0
    if (f_Y_ui16 == 0)
    {
#if (GS_PP_DEBUG != SW_OFF)
        // indicate error
        m_mtl_ErrorCurCounter_ui8++;
#endif

        if (f_X_si16 >= 0)
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI8);
        }
        else
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI8);
        }
    }
    else
    {
        // rounding
        if (f_X_si16 >= 0)
        {
            l_Result_si32 =
                (((SInt32)f_X_si16) + ((SInt32)f_Y_ui16 / 2)) / ((SInt32)f_Y_ui16);
        }
        else
        {
            l_Result_si32 =
                (((SInt32)f_X_si16) - ((SInt32)f_Y_ui16 / 2)) / ((SInt32)f_Y_ui16);
        }

        // limit to lower bound
        if (l_Result_si32 < ((SInt32)GD_MIN_SI8))
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI8);
        }
        // limit to upper bound
        else if (l_Result_si32 > ((SInt32)GD_MAX_SI8))
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI8);
        }
        else
        { // QAC
        }
    }
    return ((SInt8)l_Result_si32);
} // mtl_s16_Div_u16_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Div_u16_ui8
|     Purpose: x_ui16 / y_ui16 with rounding and overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_u16_Div_u16_ui8(UInt16 f_X_ui16, UInt16 f_Y_ui16)
{
    UInt32 l_Result_ui32;

    // division by 0
    if (f_Y_ui16 == 0)
    {
#if (GS_PP_DEBUG != SW_OFF)
        // indicate error
        m_mtl_ErrorCurCounter_ui8++;
#endif

        l_Result_ui32 = ((UInt32)GD_MAX_UI8);
    }
    else
    {
        l_Result_ui32 =
            (((UInt32)f_X_ui16) + ((UInt32)f_Y_ui16 / 2)) / ((UInt32)f_Y_ui16);

        // limit to upper bound
        if (l_Result_ui32 > ((UInt32)GD_MAX_UI8))
        {
            l_Result_ui32 = ((UInt32)GD_MAX_UI8);
        }
    }
    return ((UInt8)l_Result_ui32);
} // mtl_u16_Div_u16_ui8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Div_s16_si8
|     Purpose: x_ui16 / y_si16 with rounding and under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_u16_Div_s16_si8(UInt16 f_X_ui16, SInt16 f_Y_si16)
{
    SInt32 l_Result_si32;

    // division by 0
    if (f_Y_si16 == 0)
    {
#if (GS_PP_DEBUG != SW_OFF)
        // indicate error
        m_mtl_ErrorCurCounter_ui8++;
#endif

        l_Result_si32 = ((SInt32)GD_MAX_SI8);
    }
    else
    {
        l_Result_si32 = (((SInt32)f_X_ui16) + ((SInt32)g_mtl_Abs_mac(f_Y_si16 / 2))) /
                        ((SInt32)f_Y_si16);

        // limit to lower bound
        if (l_Result_si32 < ((SInt32)GD_MIN_SI8))
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI8);
        }
        // limit to upper bound
        else if (l_Result_si32 > ((SInt32)GD_MAX_SI8))
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI8);
        }
        else
        { // QAC
        }
    }
    return ((SInt8)l_Result_si32);
} // mtl_u16_Div_s16_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: g_mtl_s32_Div_s32_si32
|     Purpose: x_si32 / y_si32 with rounding
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si32, f_Y_si32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
SInt32 g_mtl_s32_Div_s32_si32(SInt32 f_X_si32, SInt32 f_Y_si32)
{
    SInt32 l_Y2_si32;
    SInt32 l_Result_si32;

    // division by 0
    if (f_Y_si32 == 0)
    {
#if (GS_PP_DEBUG != SW_OFF)
        // indicate error
        m_mtl_ErrorCurCounter_ui8++;
#endif

        if (f_X_si32 >= 0)
        {
            l_Result_si32 = GD_MAX_SI32;
        }
        else
        {
            l_Result_si32 = md_mtl_MIN_si32;
        }
    }
    else
    {
        // y / 2
        l_Y2_si32 = f_Y_si32 / 2;

        // rounding for x >= 0
        if (f_X_si32 >= 0)
        {
            // rounding possible: x + abs(y/2) wihtin SInt32 range
            if (f_X_si32 <= (GD_MAX_SI32 - ((SInt32)g_mtl_Abs_mac(l_Y2_si32))))
            {
                l_Result_si32 =
                    (f_X_si32 + ((SInt32)g_mtl_Abs_mac(l_Y2_si32))) / f_Y_si32;
            }
            // rounding not possible: x + abs(y/2) outside SInt32 range
            else
            {
                l_Result_si32 = f_X_si32 / f_Y_si32;
            }
        }
        // rounding for x < 0
        else
        {
            // rounding possible: -x - abs(y/2) wihtin SInt32 range
            if (f_X_si32 > (md_mtl_MIN_si32 + ((SInt32)g_mtl_Abs_mac(l_Y2_si32))))
            {
                l_Result_si32 =
                    (f_X_si32 - ((SInt32)g_mtl_Abs_mac(l_Y2_si32))) / f_Y_si32;
            }
            // rounding not possible: -x - abs(y/2) outside SInt32 range
            else
            {
                l_Result_si32 = f_X_si32 / f_Y_si32;
            }
        }
    }
    return l_Result_si32;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u32_Div_u32_ui32
|     Purpose: x_si32 / y_si32 with rounding
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui32, f_Y_ui32 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: quotient
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
UInt32 g_mtl_u32_Div_u32_ui32(UInt32 f_X_ui32, UInt32 f_Y_ui32)
{
    UInt32 l_Y2_ui32;
    UInt32 l_Result_ui32;

    // division by 0
    if (f_Y_ui32 == 0)
    {
#if (GS_PP_DEBUG != SW_OFF)
        // indicate error
        m_mtl_ErrorCurCounter_ui8++;
#endif

        l_Result_ui32 = GD_MAX_UI32;
    }
    else
    {
        // y / 2
        l_Y2_ui32 = f_Y_ui32 / 2;

        // rounding possible: x + y/2 wihtin UInt32 range
        if (f_X_ui32 <= ((UInt32)(GD_MAX_UI32 - l_Y2_ui32)))
        {
            l_Result_ui32 = (f_X_ui32 + l_Y2_ui32) / f_Y_ui32;
        }
        // rounding not possible: x + y/2 outside SInt32 range
        else
        {
            l_Result_ui32 = f_X_ui32 / f_Y_ui32;
        }
    }
    return l_Result_ui32;
} // mtl_u32_Div_u32_ui32

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_u16_Mul_u16_Div_u16_ui16
|     Purpose: (x_ui16 * y_ui16) / z_ui16 with overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16, f_Y_ui16, f_Z_ui16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: (x*y)/z
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_u16_Mul_u16_Div_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16, UInt16 f_Z_ui16)
{
    UInt32 l_Result_ui32;

    // division by 0
    if (f_Z_ui16 == 0)
    {
        l_Result_ui32 = ((UInt32)GD_MAX_UI16);
    }
    else
    {
        l_Result_ui32 = (UInt32)(((((UInt32)f_X_ui16) * ((UInt32)f_Y_ui16)) +
                                  ((UInt32)f_Z_ui16 / 2)) /
                                 ((UInt32)f_Z_ui16));

        // limit to upper bound
        if (l_Result_ui32 > ((UInt32)GD_MAX_UI16))
        {
            l_Result_ui32 = ((UInt32)GD_MAX_UI16);
        }
    }
    return ((UInt16)l_Result_ui32);
} // mtl_u16_Mul_u16_Div_u16_ui16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_s16_Mul_s16_Div_s16_si16
|     Purpose: (x_si16 * y_si16) / z_si16 with under/overflow treatment
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16, f_Y_si16, f_Z_si16 operands
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: (x*y)/z
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_s16_Mul_s16_Div_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16, SInt16 f_Z_si16)
{
    SInt32 l_Result_si32;

    // division by 0
    if (f_Z_si16 == 0)
    {
        if (((f_X_si16 >= 0) && (f_Y_si16 >= 0)) || ((f_X_si16 < 0) && (f_Y_si16 < 0)))
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI16);
        }
        else
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI16);
        }
    }
    else
    {
        l_Result_si32 = (SInt32)(((((SInt32)f_X_si16) * ((SInt32)f_Y_si16))));

        // rounding depended on the sign
        if (g_mtl_Signum_mac(l_Result_si32) == g_mtl_Signum_mac(f_Z_si16))
        {
            l_Result_si32 += ((SInt32)f_Z_si16 / 2);
        }
        else
        {
            l_Result_si32 -= ((SInt32)f_Z_si16 / 2);
        }

        // division
        l_Result_si32 /= ((SInt32)f_Z_si16);

        // limit to lower bound
        if (l_Result_si32 < ((SInt32)GD_MIN_SI16))
        {
            l_Result_si32 = ((SInt32)GD_MIN_SI16);
        }
        // limit to upper bound
        else if (l_Result_si32 > ((SInt32)GD_MAX_SI16))
        {
            l_Result_si32 = ((SInt32)GD_MAX_SI16);
        }
        else
        { // QAC
        }
    }
    return ((SInt16)l_Result_si32);
} // mtl_s16_Mul_s16_Div_s16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Min_u8_Max_u8_ui8
|     Purpose: x is kept inside minimum and maximum boundaries
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui8 argument
|               f_Min_ui8, f_Max_ui8: lower and upper boundaries
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: x inside boundaries
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_Min_u8_Max_u8_ui8(UInt8 f_X_ui8, UInt8 f_Min_ui8, UInt8 f_Max_ui8)
{
    // check lower boundary
    if (f_X_ui8 < f_Min_ui8)
    {
        f_X_ui8 = f_Min_ui8;
    }
    else
    {
        // check upper boundary
        if (f_X_ui8 > f_Max_ui8)
        {
            f_X_ui8 = f_Max_ui8;
        }
    }

    return f_X_ui8;
} // mtl_Min_u8_Max_u8_ui8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Min_s8_Max_s8_si8
|     Purpose: x is kept inside minimum and maximum boundaries
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si8 argument
|               f_Min_si8, f_Max_si8: lower and upper boundaries
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: x inside boundaries
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt8 g_mtl_Min_s8_Max_s8_si8(SInt8 f_X_si8, SInt8 f_Min_si8, SInt8 f_Max_si8)
{
    // check lower boundary
    if (f_X_si8 < f_Min_si8)
    {
        f_X_si8 = f_Min_si8;
    }
    else
    {
        // check upper boundary
        if (f_X_si8 > f_Max_si8)
        {
            f_X_si8 = f_Max_si8;
        }
    }

    return f_X_si8;
} // mtl_Min_s8_Max_s8_si8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Min_u16_Max_u16_ui16
|     Purpose: x is kept inside minimum and maximum boundaries
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_ui16 argument
|               f_Min_ui16, f_Max_ui16: lower and upper boundaries
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: x inside boundaries
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_Min_u16_Max_u16_ui16(UInt16 f_X_ui16, UInt16 f_Min_ui16, UInt16 f_Max_ui16)
{
    // check lower boundary
    if (f_X_ui16 < f_Min_ui16)
    {
        f_X_ui16 = f_Min_ui16;
    }
    else
    {
        // check upper boundary
        if (f_X_ui16 > f_Max_ui16)
        {
            f_X_ui16 = f_Max_ui16;
        }
    }

    return f_X_ui16;
} // mtl_Min_u16_Max_u16_ui16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Min_s16_Max_s16_si16
|     Purpose: x is kept inside minimum and maximum boundaries
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si16 argument
|               f_Min_si16, f_Max_si16: lower and upper boundaries
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: x inside boundaries
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_Min_s16_Max_s16_si16(SInt16 f_X_si16, SInt16 f_Min_si16, SInt16 f_Max_si16)
{
    // check lower boundary
    if (f_X_si16 < f_Min_si16)
    {
        f_X_si16 = f_Min_si16;
    }
    else
    {
        // check upper boundary
        if (f_X_si16 > f_Max_si16)
        {
            f_X_si16 = f_Max_si16;
        }
    }

    return f_X_si16;
} // mtl_Min_s16_Max_s16_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Min_s32_Max_s32_si16
|     Purpose: x is kept inside minimum and maximum boundaries
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:  f_X_si32 argument
|               f_Min_si32, f_Max_si32: lower and upper boundaries
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: x inside boundaries
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: fig1lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt32 g_mtl_Min_s32_Max_s32_si32(SInt32 f_X_si32, SInt32 f_Min_si32, SInt32 f_Max_si32)
{
    // check lower boundary
    if (f_X_si32 < f_Min_si32)
    {
        f_X_si32 = f_Min_si32;
    }
    else
    {
        // check upper boundary
        if (f_X_si32 > f_Max_si32)
        {
            f_X_si32 = f_Max_si32;
        }
    }

    return f_X_si32;
} // mtl_Min_s32_Max_s32_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Sqrt_u16_ui8
|     Purpose: Calculates the square root from an integer value UInt16 by
gradual |              bitwise approximation
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value - input value
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: radix = square root of <value>
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_Sqrt_u16_ui8(UInt16 f_X_ui16)
{
    // result of calculation
    UInt8 l_Radix_ui8;
    // temporary result of calculation
    UInt8 l_NewRadix_ui8;
    // digital value added to radix
    UInt8 l_Digit_ui8;

    l_Radix_ui8 = ((UInt8)0);

    // start with highest bit set
    l_Digit_ui8 = (1 << 7);

    do
    {
        l_NewRadix_ui8 = (UInt8)(l_Radix_ui8 + l_Digit_ui8);

        // approximation from higher values
        if ((UInt16)(l_NewRadix_ui8 * l_NewRadix_ui8) <= f_X_ui16)
        {
            l_Radix_ui8 = l_NewRadix_ui8;
        }

        l_Digit_ui8 >>= 1;
    } while (l_Digit_ui8 != 0);

    // round result for better approximation
    if (((UInt16)l_Radix_ui8) <
        ((UInt16)(f_X_ui16 - (UInt16)(l_Radix_ui8 * l_Radix_ui8))))
    {
        if (l_Radix_ui8 < GD_MAX_UI8)
        {
            l_Radix_ui8++;
        }
    }

    return l_Radix_ui8;
} // mtl_Sqrt_u16_ui8
#endif // #if (GS_MTL_USE_ASM != SW_ON)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: m_mtl_Sqrt_Adjustment_ui16
|     Purpose: Adjust result of g_mtl_SqrtExactFast_u32_ui16.
|              Use inline option to speed-up the execution.
|
| need to test:
|   if  |xn * xn - x|  >  |x - (xn-1) * (xn-1)|  then xn-1 is more accurate
|   if  |xn * xn - x|  >  |(xn+1) * (xn+1) - x|  then xn+1 is more accurate
| or, for all cases except x == 0:
|   if  |xn * xn - x|  >  x - xn * xn + 2 * xn - 1 then xn-1 is more accurate
|   if  |xn * xn - x|  >  xn * xn + 2 * xn + 1 - x then xn+1 is more accurate
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: f_x_ui32, f_xn_ui32
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: adjusted square root
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   fig1lr, src2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)

INL_OPT static UInt16 m_mtl_Sqrt_Adjustment_ui16(UInt32 f_x_ui32, UInt32 f_xn_ui32)
{
    UInt32 l_xn2_ui32;
    UInt32 l_twice_xn_ui32;
    SInt32 l_comparator0_si32;
    SInt32 l_comparator1_si32;
    SInt32 l_comparator2_si32;

    l_xn2_ui32 = f_xn_ui32 * f_xn_ui32;

    // |xn * xn - x|
    l_comparator0_si32 = (SInt32)(l_xn2_ui32 - f_x_ui32);

    if (l_comparator0_si32 < 0)
    {
        l_comparator0_si32 = -l_comparator0_si32;
    }

    l_twice_xn_ui32 = f_xn_ui32 << 1;

    // |x - (xn-1) * (xn-1)|
    l_comparator1_si32 = (SInt32)(((f_x_ui32 - l_xn2_ui32) + l_twice_xn_ui32) - 1);

    if (l_comparator1_si32 < 0)
    {
        // need to correct for x == 0 case?
        l_comparator1_si32 = -l_comparator1_si32; // only gets here when x == 0
    }

    // |(xn+1) * (xn+1) - x|
    l_comparator2_si32 = (SInt32)(((l_xn2_ui32 + l_twice_xn_ui32) + 1) - f_x_ui32);

/* Following QAC warnings shall be suppressed in order to keep the algorithm
   fast. QAC(4,3440) Result of ++ or -- operator used in expression. QAC(2,3446)
   The result operands of this conditional operator generate side effects.
    QAC(4,2006) 'm_mtl_Sqrt_Adjustment_ui16()' has more than one 'return' path.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3440, 3446, 2006
#endif

    if (l_comparator0_si32 > l_comparator1_si32)
    {
        return (UInt16)((l_comparator1_si32 > l_comparator2_si32) ? ++f_xn_ui32
                                                                  : --f_xn_ui32);
    }

    return (UInt16)((l_comparator0_si32 > l_comparator2_si32) ? ++f_xn_ui32 : f_xn_ui32);
}

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3440, 3446, 2006
#endif

#endif //(GS_MTL_USE_ASM != SW_ON)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Sqrt_u32_ui16
|     Purpose: Calculates the square root from an integer value UInt32.
|              It uses look-up table m_mtl_Sqrt_LookUp_pui32.
|              This implementation is exact (correct rounding) and fast.
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value - input value
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: integer square root of <value>
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   fig1lr, src2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
/* Following QAC warnings shall be suppressed in order to keep the algorithm
   fast. QAC(4,2006) 'g_mtl_Sqrt_u32_ui16()' has more than one 'return' path.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 2006
#endif

UInt16 g_mtl_Sqrt_u32_ui16(UInt32 f_X_ui32)
{
    UInt32 l_xn_ui32;

    if (f_X_ui32 >= 0x10000)
    {
        if (f_X_ui32 >= 0x1000000)
        {
            if (f_X_ui32 >= 0x10000000)
            {
                if (f_X_ui32 >= 0x40000000)
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 24] << 8;
                }
                else
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 22] << 7;
                }
            }
            else
            {
                if (f_X_ui32 >= 0x4000000)
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 20] << 6;
                }
                else
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 18] << 5;
                }
            }

            l_xn_ui32 = (l_xn_ui32 + 1 + (f_X_ui32 / l_xn_ui32)) >> 1;
            l_xn_ui32 = (l_xn_ui32 + 1 + (f_X_ui32 / l_xn_ui32)) >> 1;

            // overflow handling
            if (l_xn_ui32 > 65535)
            {
                return GD_MAX_UI16;
            }
            else
            {
                return m_mtl_Sqrt_Adjustment_ui16(f_X_ui32, l_xn_ui32);
            }
        }
        else
        {
            if (f_X_ui32 >= 0x100000)
            {
                if (f_X_ui32 >= 0x400000)
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 16] << 4;
                }
                else
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 14] << 3;
                }
            }
            else
            {
                if (f_X_ui32 >= 0x40000)
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 12] << 2;
                }
                else
                {
                    l_xn_ui32 = m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 10] << 1;
                }
            }

            l_xn_ui32 = (l_xn_ui32 + 1 + (f_X_ui32 / l_xn_ui32)) >> 1;

            return m_mtl_Sqrt_Adjustment_ui16(f_X_ui32, l_xn_ui32);
        }
    }
    else
    {
        if (f_X_ui32 >= 0x100)
        {
            if (f_X_ui32 >= 0x1000)
            {
                if (f_X_ui32 >= 0x4000)
                {
                    l_xn_ui32 = (m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 8]) + 1;
                }
                else
                {
                    l_xn_ui32 = (m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 6] >> 1) + 1;
                }
            }
            else
            {
                if (f_X_ui32 >= 0x400)
                {
                    l_xn_ui32 = (m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 4] >> 2) + 1;
                }
                else
                {
                    l_xn_ui32 = (m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 2] >> 3) + 1;
                }
            }

            return m_mtl_Sqrt_Adjustment_ui16(f_X_ui32, l_xn_ui32);
        }
        else
        {
            return m_mtl_Sqrt_Adjustment_ui16(f_X_ui32,
                                              m_mtl_Sqrt_LookUp_pui32[f_X_ui32] >> 4);
        }
    }
} /* g_mtl_Sqrt_u32_ui16 */

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 2006
#endif

#endif //(GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: g_mtl_Sqrt_VeryFastLessAccurate_u32_ui16
|     Purpose: Calculates the square root from an integer value UInt32.
|              It uses look-up table m_mtl_Sqrt_LookUp_pui32.
|              Please note- There is no rounding, hence faster than the
|              integer square root algorithm. The result is not accurate.
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value - input value
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: integer square root of <value>
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   dvs3bmh
*****************************************************************************/

#if (GS_MTL_USE_ASM != SW_ON)
// to keep the algorithms fast allow the prohibited ;)
#ifdef QAC_MSG_OFF
// QAC(4,3440) Result of ++ or -- operator used in expression.
#pragma PRQA_MESSAGES_OFF 3440
// QAC(2,3446) The result operands of this conditional operator generate side
#pragma PRQA_MESSAGES_OFF 3446
// QAC(4,2006) 'g_mtl_Sqrt_u32_ui16()' has more than one 'return' path.
#pragma PRQA_MESSAGES_OFF 2006
#endif

UInt16 g_mtl_Sqrt_VeryFastLessAccurate_u32_ui16(UInt32 f_X_ui32)
{
    if (f_X_ui32 >= 0x10000)
    {
        if (f_X_ui32 >= 0x1000000)
        {
            if (f_X_ui32 >= 0x10000000)
            {
                if (f_X_ui32 >= 0x40000000)
                {
                    return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 24] << 8));
                }
                else
                {
                    return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 22] << 7));
                }
            }
            else if (f_X_ui32 >= 0x4000000)
            {
                return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 20] << 6));
            }
            else
            {
                return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 18] << 5));
            }
        }
        else if (f_X_ui32 >= 0x100000)
        {
            if (f_X_ui32 >= 0x400000)
            {
                return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 16] << 4));
            }
            else
            {
                return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 14] << 3));
            }
        }
        else if (f_X_ui32 >= 0x40000)
        {
            return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 12] << 2));
        }
        else
        {
            return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 10] << 1));
        }
    }
    else if (f_X_ui32 >= 0x100)
    {
        if (f_X_ui32 >= 0x1000)
        {
            if (f_X_ui32 >= 0x4000)
            {
                return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 8]));
            }
            else
            {
                return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 6] >> 1));
            }
        }
        else if (f_X_ui32 >= 0x400)
        {
            return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 4] >> 2));
        }
        else
        {
            return (UInt16)((m_mtl_Sqrt_LookUp_pui32[f_X_ui32 >> 2] >> 3));
        }
    }
    else
    {
        return (UInt16)(m_mtl_Sqrt_LookUp_pui32[f_X_ui32] >> 4);
    }
}

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3440
#pragma PRQA_MESSAGES_ON 3446
#pragma PRQA_MESSAGES_ON 2006
#endif

#endif //(GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Square_u8_ui8
|     Purpose: square of UInt8 with overflow protection
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value - input value
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: x^2 limited to UInt8 range
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt8 g_mtl_Square_u8_ui8(UInt8 f_X_ui8)
{
    UInt16 l_Square_ui16;
    UInt8 l_Result_ui8;

    l_Square_ui16 = (UInt16)(f_X_ui8 * f_X_ui8);

    if (l_Square_ui16 > ((UInt16)GD_MAX_UI8))
    {
        l_Result_ui8 = GD_MAX_UI8;
    }
    else
    {
        l_Result_ui8 = ((UInt8)l_Square_ui16);
    }

    return l_Result_ui8;
} // mtl_Square_u8_ui8
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ReducePrecision_si16
|     Purpose: precision reduction by right shifting including rounding
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value to be rounded
|              number of bit shifts
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: rounded value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
SInt16 g_mtl_ReducePrecision_si16(SInt16 f_X_si16, UInt8 f_BitNum_ui8)
{
    UInt16 f_X_ui16;

    // reduction has to be peformed
    if (f_BitNum_ui8 > ((UInt8)0))
    {
        // negative value
        if (f_X_si16 < 0L)
        {
            // -x is existing
            if (f_X_si16 > GD_MIN_SI16)
            {
                f_X_si16 = (-f_X_si16);
            }
            // -x is not existing
            else
            {
                f_X_si16 = GD_MAX_SI16;
            }

            f_X_si16 =
                g_mtl_s16_Add_u16_si16(f_X_si16, ((UInt16)(1 << (f_BitNum_ui8 - 1))));

            // type cast to UInt16 for right shift
            // the represented value does not change since f_X_si16 were made positive
            f_X_ui16 = ((UInt16)f_X_si16);
            f_X_si16 = (SInt16)(f_X_ui16 >> f_BitNum_ui8);

            f_X_si16 = (-f_X_si16);
        }
        // positive value
        else
        {
            f_X_si16 =
                g_mtl_s16_Add_u16_si16(f_X_si16, ((UInt16)(1 << (f_BitNum_ui8 - 1))));

            // type cast to UInt16 for right shift
            // the represented value does not change since f_X_si16 is positive
            f_X_ui16 = ((UInt16)f_X_si16);
            f_X_si16 = (SInt16)(f_X_ui16 >> f_BitNum_ui8);
        }
    }

    return f_X_si16;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ReducePrecision_si32
|     Purpose: precision reduction by right shifting including rounding
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value to be rounded
|              number of bit shifts
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: rounded value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
SInt32 g_mtl_ReducePrecision_si32(SInt32 f_X_si32, UInt8 f_BitNum_ui8)
{
    UInt32 f_X_ui32;

    // reduction has to be peformed
    if (f_BitNum_ui8 > ((UInt8)0))
    {
        // negative value
        if (f_X_si32 < 0L)
        {
            // -x is existing
            if (f_X_si32 > md_mtl_MIN_si32)
            {
                f_X_si32 = (-f_X_si32);
            }
            else
            {
                f_X_si32 = GD_MAX_SI32;
            }
            f_X_si32 =
                g_mtl_s32_Add_u32_si32(f_X_si32, ((UInt32)(1 << (f_BitNum_ui8 - 1))));

            // type cast to UInt32 for right shift
            // the represented value does not change since f_X_si32 is positive
            f_X_ui32 = ((UInt32)f_X_si32);
            f_X_si32 = (SInt32)(f_X_ui32 >> f_BitNum_ui8);
            f_X_si32 = (-f_X_si32);
        }
        // positive value
        else
        {
            f_X_si32 =
                g_mtl_s32_Add_u32_si32(f_X_si32, ((UInt32)(1 << (f_BitNum_ui8 - 1))));

            // type cast to UInt32 for right shift
            // the represented value does not change since f_X_si32 is positive
            f_X_ui32 = ((UInt32)f_X_si32);
            f_X_si32 = (SInt32)(f_X_ui32 >> f_BitNum_ui8);
        }
    }

    return f_X_si32;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ReducePrecision_ui16
|     Purpose: precision reduction by right shifting including rounding
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value to be rounded
|              number of bit shifts
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: rounded value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
UInt16 g_mtl_ReducePrecision_ui16(UInt16 f_X_ui16, UInt8 f_BitNum_ui8)
{
    // reduction has to be peformed
    if (f_BitNum_ui8 > ((UInt8)0))
    {
        f_X_ui16 = g_mtl_u16_Add_u16_ui16(f_X_ui16, ((UInt16)(1 << (f_BitNum_ui8 - 1))));
        f_X_ui16 = (f_X_ui16 >> f_BitNum_ui8);
    }

    return f_X_ui16;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ReducePrecision_ui32
|     Purpose: precision reduction by right shifting including rounding
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: value to be rounded
|              number of bit shifts
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: rounded value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
UInt32 g_mtl_ReducePrecision_ui32(UInt32 f_X_ui32, UInt8 f_BitNum_ui8)
{
    // reduction has to be peformed
    if (f_BitNum_ui8 > ((UInt8)0))
    {
        f_X_ui32 = g_mtl_u32_Add_u32_ui32(f_X_ui32, ((UInt32)(1 << (f_BitNum_ui8 - 1))));
        f_X_ui32 = (f_X_ui32 >> f_BitNum_ui8);
    }

    return f_X_ui32;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ReadChangeEndian_ui16
|     Purpose: read a little/big endian value and return the corresponding
big/little endian value
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: buffer address
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: endian converted UInt16 value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_ReadChangeEndian_ui16(const void *f_RefIn_pvd)
{
    UInt16 l_Result_ui16;
    UInt8 l_Output_pui8[2];
    UInt8 l_Input_pui8[2];

    (void)memcpy(l_Input_pui8, f_RefIn_pvd, 2);

    // reverse order of input bytes
    l_Output_pui8[0] = l_Input_pui8[1];
    l_Output_pui8[1] = l_Input_pui8[0];

    (void)memcpy(&l_Result_ui16, l_Output_pui8, 2);

    return l_Result_ui16;
} // mtl_ReadChangeEndian_ui16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ReadChangeEndian_ui32
|     Purpose: read a little/big endian value and return the corresponding
big/little endian value
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: buffer address
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: endian converted UInt32 value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt32 g_mtl_ReadChangeEndian_ui32(const void *f_RefIn_pvd)
{
    UInt32 l_Result_ui32;
    UInt8 l_Output_pui8[4];
    UInt8 l_Input_pui8[4];

    (void)memcpy(l_Input_pui8, f_RefIn_pvd, 4);

    // reverse order of input bytes
    l_Output_pui8[0] = l_Input_pui8[3];
    l_Output_pui8[1] = l_Input_pui8[2];
    l_Output_pui8[2] = l_Input_pui8[1];
    l_Output_pui8[3] = l_Input_pui8[0];

    (void)memcpy(&l_Result_ui32, l_Output_pui8, 4);

    return l_Result_ui32;
} // mtl_ReadChangeEndian_ui32
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_WriteChangeEndian_u16_vd
|     Purpose: write a little/big endian value with the corresponding big/little
endian value as input
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: buffer address
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: endian converted UInt16 value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
void g_mtl_WriteChangeEndian_u16_vd(void *f_RefOut_pvd, UInt16 f_Value_ui16)
{
    UInt8 l_Output_pui8[2];
    UInt8 l_Input_pui8[2];

    (void)memcpy(l_Input_pui8, &f_Value_ui16, 2);

    // reverse order of input bytes
    l_Output_pui8[0] = l_Input_pui8[1];
    l_Output_pui8[1] = l_Input_pui8[0];

    (void)memcpy(f_RefOut_pvd, l_Output_pui8, 2);

    return;
} // mtl_WriteChangeEndian_u16_vd
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_WriteChangeEndian_u32_vd
|     Purpose: write a little/big endian value with the corresponding big/little
endian value as input
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: buffer address
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: endian converted UInt32 value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
void g_mtl_WriteChangeEndian_u32_vd(void *f_RefOut_pvd, UInt32 f_Value_ui32)
{
    UInt8 l_Output_pui8[4];
    UInt8 l_Input_pui8[4];

    (void)memcpy(l_Input_pui8, &f_Value_ui32, 4);

    // reverse order of input bytes
    l_Output_pui8[0] = l_Input_pui8[3];
    l_Output_pui8[1] = l_Input_pui8[2];
    l_Output_pui8[2] = l_Input_pui8[1];
    l_Output_pui8[3] = l_Input_pui8[0];

    (void)memcpy(f_RefOut_pvd, l_Output_pui8, 4);

    return;
} // mtl_WriteChangeEndian_u32_vd
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: g_mtlLinInt_si16
|     Purpose: linear interpolation
|
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:
|             f_Y_psi16:                array of input result values to be
interpolated |             f_Xmin_si16, f_Xmax_si16: corresponding x axis limits
|             f_StepBitsX_ui8:          step size between 2 x axis values
|             f_X_si16:                 argument for which y has to be
interpolated
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: interpolated value
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|        Name: wij2lr
*****************************************************************************/
SInt16 g_mtl_LinIntPow2Step_si16(const SInt16 *f_Y_psi16, UInt8 f_LenY_ui8,
                                 SInt16 f_Xmin_si16, SInt16 f_Xmax_si16,
                                 UInt8 f_StepBitsX_ui8, SInt16 f_X_si16)
{
    UInt8 l_Index_ui8;
    UInt16 l_Value_ui16;
    SInt16 l_Value_si16;
    UInt32 l_Value_ui32;

    // check for left boundary
    if (f_X_si16 <= f_Xmin_si16)
    {
        l_Value_si16 = f_Y_psi16[0];
    }
    // check for right boundary
    else if (f_X_si16 >= f_Xmax_si16)
    {
        l_Value_si16 = (f_Y_psi16[f_LenY_ui8 - 1]);
    }
    else
    {
        // determine left index for y array
        l_Value_ui16 = ((UInt16)(f_X_si16 - f_Xmin_si16));
        l_Index_ui8  = (UInt8)(l_Value_ui16 >> f_StepBitsX_ui8);

        // check if index is valid
        if (l_Index_ui8 >= (f_LenY_ui8 - 1))
        {
            l_Value_si16 = (f_Y_psi16[f_LenY_ui8 - 1]);
        }
        else
        {
            // deltaX = x - x(index)
            l_Value_ui16 -= (((UInt16)(l_Index_ui8)) * ((UInt16)(1 << f_StepBitsX_ui8)));

            // deltaY = slope * deltaX
            if (f_Y_psi16[l_Index_ui8 + 1] >= f_Y_psi16[l_Index_ui8])
            {
                l_Value_ui32 = ((UInt32)(((UInt32)(f_Y_psi16[l_Index_ui8 + 1] -
                                                   f_Y_psi16[l_Index_ui8])) *
                                         l_Value_ui16 /* +
                 ((UInt32)(1<<f_StepBitsX_ui8)/2)*/
                                         )) >>
                               f_StepBitsX_ui8;
                l_Value_si16 = ((SInt16)l_Value_ui32);
            }
            else
            {
                l_Value_ui32 = ((UInt32)(((UInt32)(f_Y_psi16[l_Index_ui8] -
                                                   f_Y_psi16[l_Index_ui8 + 1])) *
                                         l_Value_ui16 /* +
                 ((UInt32)(1<<f_StepBitsX_ui8)/2)*/
                                         )) >>
                               f_StepBitsX_ui8;
                l_Value_si16 = -((SInt16)l_Value_ui32);
            }

            // result of interpolation
            l_Value_si16 = f_Y_psi16[l_Index_ui8] + l_Value_si16;
        }
    }
    // return result of interpolation
    return l_Value_si16;
} // mtl_LinIntPow2Step_si16

// --------------------------------------------------
// trigonometric functions
// --------------------------------------------------
#if (GS_MTL_TRIGONOMETRIC_FUNC != SW_OFF)

// angle step size for sin table: 2^8
#define md_MTL_SIN_ANGLE_STEPBITS 8
// lenth of sine table: 26
#define md_MTL_SIN_TABLE_SIZE ((gd_MTL_PI_2_ui16 / (1 << md_MTL_SIN_ANGLE_STEPBITS)) + 1)
// sine table: angle [0,pi/2], sine range [0, 2^14]
static const SInt16 mc_mtl_SinTab_psi16[md_MTL_SIN_TABLE_SIZE] = {
    0,     1023,  2043,  3054,  4053,  5037,  6001,  6942,  7855,
    8738,  9586,  10397, 11168, 11895, 12575, 13207, 13787, 14313,
    14783, 15195, 15548, 15840, 16071, 16239, 16343, 16384};

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Sin_si16
|     Purpose: determine sin(phi)
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: phi: angle in rad
|              scale: 2^12
|              range: [-2.546*pi,2.546*pi] -> [-(2^15+1), 2^15]
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: sin(phi)
|              scale: 2^14
|              range: [-1,1] -> [-2^14, 2^14]
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
SInt16 g_mtl_Sin_si16(SInt16 f_Phi_si16)
{
    // indicator for sign of the result
    UInt8 l_Sign_ui8 = ((UInt8)0);

    SInt16 l_Sine_si16;

    // guarantee the existence of a positive angle for the lowest possible
    // negative input angle the small error is neglected
    if (f_Phi_si16 == GD_MIN_SI16)
    {
        f_Phi_si16 = GD_MIN_SI16 + ((SInt16)1);
    }

    // make angle positive
    if (f_Phi_si16 < 0)
    {
        f_Phi_si16 = (-f_Phi_si16);

        // update sign indicator
        l_Sign_ui8 += 1;
    }

    // check vor valid interpolation range [0,pi/2]
    // [2pi,2.546pi] -> [0,0.546pi]
    if (f_Phi_si16 > ((SInt16)gd_MTL_2_PI_ui16))
    {
        f_Phi_si16 -= ((SInt16)gd_MTL_2_PI_ui16);
    }
    // [pi,2pi[ -> [0,pi[
    else if (f_Phi_si16 > ((SInt16)gd_MTL_PI_ui16))
    {
        f_Phi_si16 -= ((SInt16)gd_MTL_PI_ui16);

        // update sign indicator
        l_Sign_ui8 += 1;
    }
    else
    { // QAC
    }

    // check vor valid interpolation range [0,pi/2]
    if (f_Phi_si16 > ((SInt16)gd_MTL_PI_2_ui16))
    {
        f_Phi_si16 -= ((SInt16)gd_MTL_PI_2_ui16);
        f_Phi_si16 = ((SInt16)gd_MTL_PI_2_ui16) - f_Phi_si16;
    }

    // interpolate sine
    l_Sine_si16 = g_mtl_LinIntPow2Step_si16(mc_mtl_SinTab_psi16, md_MTL_SIN_TABLE_SIZE,
                                            ((SInt16)0), ((SInt16)gd_MTL_PI_2_ui16),
                                            md_MTL_SIN_ANGLE_STEPBITS, f_Phi_si16);

    // adapt sign
    if (g_mtl_OddValue_bl(l_Sign_ui8) != 0)
    {
        l_Sine_si16 = (-l_Sine_si16);
    }

    return l_Sine_si16;
} // mtl_Sin_si16

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Cos_si16
|     Purpose: determine cos(phi)
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: phi: angle in rad
|              scale: 2^12
|              range: [-2.546*pi,2.546*pi] -> [-2^15, 2^15-1]
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: cos(phi)
|              scale: 2^14
|              range: [-1,1] -> [-2^14, 2^14]
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
SInt16 g_mtl_Cos_si16(SInt16 f_Phi_si16)
{
    SInt32 l_Value_si32;
    SInt16 l_Value_si16;
    SInt16 l_Result_si16;

    // cos(Phi) = sin(Phi+Pi/2)

    // add pi/2
    l_Value_si32 = (SInt32)(((SInt32)f_Phi_si16) + ((SInt32)gd_MTL_PI_2_ui16));

    // underflow impossible --> no underflow treatment
    // overflow treatment
    if (l_Value_si32 > ((SInt32)GD_MAX_SI16))
    {
        l_Value_si32 -= ((SInt32)gd_MTL_2_PI_ui16);
    }

    l_Value_si16 = ((SInt16)l_Value_si32);

    // perform sin
    l_Result_si16 = g_mtl_Sin_si16(l_Value_si16);

    return l_Result_si16;
} // mtl_Cos_si16

// angle step size for sin table: 2^7
#define md_MTL_TAN_ANGLE_STEPBITS 7
// length of sine table: 42
#define md_MTL_TAN_TABLE_SIZE     42
#define md_MTL_TAN_MAX_ANGLE_si16 (SInt16)(((md_MTL_TAN_TABLE_SIZE - 1) * 8192) / 32)
// tangent table: angle [0, 41/32], tangent range [0, 3.36 * 2^13]
static const SInt16 mc_mtl_TanTab_psi16[md_MTL_TAN_TABLE_SIZE] = {
    0,     256,   513,   770,   1029,  1291,  1554,  1821,  2092,  2367,  2647,
    2932,  3225,  3524,  3832,  4148,  4475,  4814,  5165,  5530,  5910,  6309,
    6727,  7167,  7632,  8124,  8649,  9209,  9809,  10457, 11158, 11922, 12758,
    13680, 14704, 15849, 17142, 18616, 20315, 22300, 24654, 27497};

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Tan_si16
|     Purpose: determine tan(phi)
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: phi: angle in rad
|              scale: 2^12
|              range: [0, 41/32] -> [0, 41/32*2^12]
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: tan(phi)
|              scale: 2^13
|              range: [0, 3.36] -> [0, 3.36 * 2^13]
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
SInt16 g_mtl_Tan_si16(SInt16 f_Phi_si16)
{
    SInt16 l_Tangent_si16;
    Boolean l_Flag_bl = 0;

    // map input angle in positive range
    if (f_Phi_si16 < 0)
    {
        if (f_Phi_si16 == GD_MIN_SI16)
        {
            f_Phi_si16 = GD_MAX_SI16;
        }
        else
        {
            f_Phi_si16 = (-f_Phi_si16);
        }

        l_Flag_bl = 1;
    }

    // interpolate tangent
    l_Tangent_si16 = g_mtl_LinIntPow2Step_si16(mc_mtl_TanTab_psi16, md_MTL_TAN_TABLE_SIZE,
                                               ((SInt16)0), md_MTL_TAN_MAX_ANGLE_si16,
                                               md_MTL_TAN_ANGLE_STEPBITS, f_Phi_si16);

    // adapt sign
    if (l_Flag_bl == 1)
    {
        l_Tangent_si16 = -l_Tangent_si16;
    }

    // return tangent
    return l_Tangent_si16;
} // mtl_Tan_si16

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ArcTan_si16
|     Purpose: determine arctan(x) with CORDIC algorithm
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: slope
|              scaling bits
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: arctan(x) in rad, resolution 1/(2^12) rad
|
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
SInt16 g_mtl_ArcTan_si16(SInt16 f_Slope_si16, UInt8 f_ScaleBits_ui8)
{
    const SInt32 l_X_si32 = (1 << 15);
    SInt32 l_Y_si32;
    UInt32 l_Phi_ui32;
    SInt16 l_Phi_si16;

    // slope = 2^ScaleBits * deltaY/deltaX, where deltaX=1
    // generate artificial coordinates
    l_Y_si32 = ((SInt32)(1 << (15 - f_ScaleBits_ui8))) * ((SInt32)f_Slope_si16);

    // determine phi, result in range [0, pi/2], [3/2pi, 2pi]
    l_Phi_ui32 = g_mtl_ArcTan2_ui32(l_Y_si32, l_X_si32);

    // transform to 16 bit resolution
    l_Phi_si16 = (SInt16)(l_Phi_ui32 >> (gd_MTL_ANGLE_SCALE_BITS_u32_ui8 -
                                         gd_MTL_ANGLE_SCALE_BITS_u16_ui8));

    // transform angle range [0, pi/2], [3/2pi, 2pi] -> [-pi/2, pi/2]
    if (l_Phi_si16 > ((SInt32)gd_MTL_PI_ui16))
    {
        l_Phi_si16 -= ((SInt16)gd_MTL_2_PI_ui16);
    }

    // return arctan
    return l_Phi_si16;
} // mtl_ArcTan_si16

// length of angle table
#define md_MTL_ATAN2_TABLE_SIZE 23
// angle table for arctan2: from pi/4 to the smallest representable angle >0
// 2^22 * arctan(2^(-n)), n=0..23
static const SInt32 mc_ArcTan2Tab_SInt32[md_MTL_ATAN2_TABLE_SIZE] = {
    3294199, 1944679, 1027515, 521583, 261803, 131029, 65531, 32767,
    16384,   8192,    4096,    2048,   1024,   512,    256,   128,
    64,      32,      16,      8,      4,      2,      1};

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_ArcTan2_ui32
|     Purpose: determine arctan2(x,y) with CORDIC algorithm
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters: x, y coordinate values
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: arctan2(x,y) in rad, resolution 1/(2^22) rad
|
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   dvs3bmh
*****************************************************************************/
UInt32 g_mtl_ArcTan2_ui32(SInt32 f_Y_si32, SInt32 f_X_si32)
{
    UInt8 l_TabIndex_ui8 = 0;
    SInt32 l_X_si32;
    SInt32 l_Y_si32;
    SInt32 l_Phi_si32 = 0L;
    SInt32 l_Xnew_si32;
    SInt32 l_Ynew_si32;
    Boolean l_Flag_bl = 0;
    UInt32 l_X_ui32;
    UInt32 l_Y_ui32;

    // rotate vector to quadrants 1,4
    if (f_X_si32 >= 0L)
    {
        if (f_Y_si32 == md_mtl_MIN_si32)
        {
            f_Y_si32 = f_Y_si32 + 1;
        }
        else
        {
            // do nothing
        }

        l_X_si32 = f_X_si32;
        l_Y_si32 = f_Y_si32;
    }
    else
    {
        if (f_X_si32 == md_mtl_MIN_si32)
        {
            f_X_si32 = f_X_si32 + 1;
        }
        else
        {
            // do nothing
        }
        if (f_Y_si32 == md_mtl_MIN_si32)
        {
            f_Y_si32 = f_Y_si32 + 1;
        }
        else
        {
            // do nothing
        }

        l_X_si32  = -f_X_si32;
        l_Y_si32  = -f_Y_si32;
        l_Flag_bl = 1;
    }

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 297 // Review Pampus/Benjamin 27.11.2013
#endif

    if ((l_X_si32 != 0) || (l_Y_si32 != 0))
    {
        // scale in order to increase accuracy
        while ((g_mtl_Abs_mac(l_Y_si32) < ((SInt32)(1 << 27))) &&
               (g_mtl_Abs_mac(l_X_si32) < ((SInt32)(1 << 27))))
        {
            l_Y_si32 *= 2L;
            l_X_si32 *= 2L;
        }

        // limit maximum value for overflow protection
        while ((g_mtl_Abs_mac(l_Y_si32) > ((SInt32)(1 << 28))) ||
               (g_mtl_Abs_mac(l_X_si32) > ((SInt32)(1 << 28))))
        {
            l_Y_si32 /= 2L;
            l_X_si32 /= 2L;
        }
    }

    // perform CORDIC
    while ((l_Y_si32 != 0L) && (l_TabIndex_ui8 < md_MTL_ATAN2_TABLE_SIZE))
    {
        // rotation clockwise
        if (l_Y_si32 > 0L)
        {
            // type cast to UInt32 for right shift
            // the represented value does not change since l_X_si32 is positive
            l_X_ui32 = ((UInt32)g_mtl_Abs_mac(l_X_si32));

            // type cast to UInt32 for right shift
            // the represented value does not change since l_Y_si32 is positive
            l_Y_ui32 = ((UInt32)l_Y_si32);

            l_Xnew_si32 = l_X_si32 + (SInt32)(l_Y_ui32 >> l_TabIndex_ui8);
            l_Ynew_si32 = l_Y_si32 - (SInt32)(l_X_ui32 >> l_TabIndex_ui8);
            l_Phi_si32 += mc_ArcTan2Tab_SInt32[l_TabIndex_ui8];
        }
        // rotation counter clockwise
        else
        {
            // type cast to UInt32 for right shift
            // the represented value does not change since l_X_si32 is positive
            // g_mtl_Abs_mac is used to make the positiveness visible for QA-C
            l_X_ui32 = ((UInt32)g_mtl_Abs_mac(l_X_si32));

            // make l_Y_si32 positive for right shift
            l_Y_ui32 = ((UInt32)(0 - l_Y_si32));

            // change the sign back --> addition instead of subtraction
            l_Xnew_si32 = l_X_si32 + (SInt32)(l_Y_ui32 >> l_TabIndex_ui8);

            l_Ynew_si32 = l_Y_si32 + (SInt32)(l_X_ui32 >> l_TabIndex_ui8);
            l_Phi_si32 -= mc_ArcTan2Tab_SInt32[l_TabIndex_ui8];
        }

        // update results of rotation
        l_X_si32 = l_Xnew_si32;
        l_Y_si32 = l_Ynew_si32;

        // next rotation step -> next table index
        l_TabIndex_ui8++;
    }

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 297 // Review Pampus/Benjamin 27.11.2013
#endif

    // angle correction with pi
    if (l_Flag_bl == 1)
    {
        l_Phi_si32 += ((SInt32)gd_MTL_PI_ui32);
    }

    // enforce positive angle
    if (l_Phi_si32 < 0L)
    {
        l_Phi_si32 += ((SInt32)gd_MTL_2_PI_ui32);
    }

    return ((UInt32)l_Phi_si32);
} // mtl_ArcTan2_ui32

#endif // #if( GS_MTL_TRIGONOMETRIC_FUNC != SW_OFF )

#if (GS_MTL_FILTER_FUNC != SW_OFF)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Filter_ui16
|     Purpose: recursive smoothing filter of first order
|              y(k) = sfc*y(k-1) + (1-sfc)*x(k)
|                   = y(k-1) + (1-sfc)*(x(k)-y(k-1))
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:
|                  f_X_ui16: unfiltered value x(k),      domain [0, 65535]
|                  f_Y_ui16: last filtered value y(k-1), domain [0, 65535]
|  f_ScaledFilterConst_ui16: scaled filter constant sfc, domain [0, 32768]
|                             = 2^15 * filter constant (with domain [0, 1])
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: y(k)
|
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
UInt16 g_mtl_Filter_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16,
                         UInt16 f_ScaledFilterConst_ui16)
{
    SInt32 l_Value_si32;

    /* check if scaled filter constant is out of range */
    if (f_ScaledFilterConst_ui16 >= 32768)
    {
        /* CASE: scaled filter constant is out of range */

        /* limit scaled filter constant to maximum 2^15, i.e. filter constant = 1;
           -> sample value has no influence */
        l_Value_si32 = f_Y_ui16;
    }
    else
    {
        /* CASE: scaled filter constant is valid */

        // x(k) - y(k-1)
        l_Value_si32 = (((SInt32)f_X_ui16) - ((SInt32)f_Y_ui16));
        // (1-sfc) * (x(k) - y(k-1))
        l_Value_si32 *= (((SInt32)(1 << 15)) - ((SInt32)f_ScaledFilterConst_ui16));
        // scale back
        l_Value_si32 /= ((SInt32)(1 << 15));
        // y(k-1) + (1-sfc)*(x(k) - y(k-1))
        l_Value_si32 += ((SInt32)f_Y_ui16);

        // if the filter is in steady state then ensure that it will react on small
        // changes of x
        if (l_Value_si32 == ((SInt32)f_Y_ui16))
        {
            if (l_Value_si32 < ((SInt32)f_X_ui16))
            {
                l_Value_si32++;
            }
            if (l_Value_si32 > ((SInt32)f_X_ui16))
            {
                l_Value_si32--;
            }
        }

        // limit to boundaries
        l_Value_si32 = g_mtl_Min_s32_Max_s32_si32(l_Value_si32, (SInt32)GD_MIN_UI16,
                                                  (SInt32)GD_MAX_UI16);
    }

    // return result
    return ((UInt16)l_Value_si32);
} // mtl_Filter_ui16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
|        Name: mtl_Filter_si16
|     Purpose: recursive smoothing filter of first order
|              y(k) = fac*y(k-1) + (1-fac)*x(k)
|                   = y(k-1) + (1-fac)*(x(k)-y(k-1))
|-----------------------------------------------------------------------------
| S I D E  E F F E C T S
|
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
| I N P U T
|-----------------------------------------------------------------------------
|  Parameters:
|              x(k)
|              y(k-1)
|              fac * 2^15
|-----------------------------------------------------------------------------
| O U T P U T
|-----------------------------------------------------------------------------
|     Results: y(k)
|
|-----------------------------------------------------------------------------
| A U T H O R  I D E N T I T Y (use network account)
|-----------------------------------------------------------------------------
|   wij2lr
*****************************************************************************/
#if (GS_MTL_USE_ASM != SW_ON)
SInt16 g_mtl_Filter_si16(SInt16 f_X_si16, SInt16 f_Y_si16,
                         UInt16 f_ScaledFilterConst_ui16)
{
    SInt32 l_Value_si32;

    /* check if scaled filter constant is out of range */
    if (f_ScaledFilterConst_ui16 >= 32768)
    {
        /* CASE: scaled filter constant is out of range */

        /* limit scaled filter constant to maximum 2^15, i.e. filter constant = 1;
           -> sample value has no influence */
        l_Value_si32 = f_Y_si16;
    }
    else
    {
        /* CASE: scaled filter constant is valid */

        // x(k) - y(k-1)
        l_Value_si32 = (((SInt32)f_X_si16) - ((SInt32)f_Y_si16));
        // (1-A) * (x(k) - y(k-1))
        l_Value_si32 *= (((SInt32)(1 << 15)) - ((SInt32)f_ScaledFilterConst_ui16));
        // scale back
        l_Value_si32 /= ((SInt32)(1 << 15));
        // y(k-1) + (1-A)*(x(k) - y(k-1))
        l_Value_si32 += ((SInt32)f_Y_si16);

        // reach limit
        if (l_Value_si32 == ((SInt32)f_Y_si16))
        {
            if (l_Value_si32 < ((SInt32)f_X_si16))
            {
                l_Value_si32++;
            }
            if (l_Value_si32 > ((SInt32)f_X_si16))
            {
                l_Value_si32--;
            }
        }

        // limit to boundaries
        l_Value_si32 = g_mtl_Min_s32_Max_s32_si32(l_Value_si32, (SInt32)GD_MIN_SI16,
                                                  (SInt32)GD_MAX_SI16);
    }

    // return result
    return ((SInt16)l_Value_si32);
} // mtl_Filter_si16
#endif // #if (GS_MTL_USE_ASM != SW_ON)

#endif // #if( GS_MTL_FILTER_FUNC != SW_OFF )

/*========================= EoF (mathlib.c) ========================*/
