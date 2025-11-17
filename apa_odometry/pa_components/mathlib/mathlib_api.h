/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: ap_psd.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef MATHLIB_API_H
#define MATHLIB_API_H

#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/_appl_mathlib.h>

/*--------------------------------------------------------------------------*/
/*- Check if Compiler Switches (utilised in this file) are defined         -*/
/*--------------------------------------------------------------------------*/

#ifndef SW_ON
#error ('global switch SW_ON is not defined')
#endif

#ifndef SW_OFF
#error ('global switch SW_OFF is not defined')
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

#ifdef COMPONENT_MTL
#ifndef LS_MTL_TESTHARNESS
#error ('switch LS_MTL_TESTHARNESS is not defined')
#endif
#endif

/*--------------------------------------------------------------------------*/
/*- exported symbolic constants                                            -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_MTL // Only defined within this component!
#define APPL_MATHLIB_H_REF 0x090921
#define APPL_MATHLIB_C_REF 0x090921
#endif // COMPONENT_MTL

// angle scale: 2^22/rad
#define gd_MTL_ANGLE_SCALE_BITS_u32_ui8 ((UInt8)22)
// 2^22 * pi
#define gd_MTL_PI_ui32 ((UInt32)13176795)
// 2^22 * pi/2
#define gd_MTL_PI_2_ui32 ((UInt32)6588397)
// 2^22 * 2pi
#define gd_MTL_2_PI_ui32 ((UInt32)26353589)

// angle scale: 2^12/rad
#define gd_MTL_ANGLE_SCALE_BITS_u16_ui8 ((UInt8)12)
// 2^12 * pi
#define gd_MTL_PI_ui16 ((UInt16)12868)
// 2^12 * pi/2
#define gd_MTL_PI_2_ui16 ((UInt16)6434)
// 2^12 * 2pi
#define gd_MTL_2_PI_ui16 ((UInt16)25736)

// time of flight for 1 cm [us]
// v = 344m/s = speed of sound at 20�C
// time of flight for 1 cm = 1cm/v = 1cm / (34400cm/1000000us) = 10000/344 us
#define gd_MTL_TIME_OF_FLIGHT_PER_CM_ui8 ((UInt8)(10000 / 344))

/*--------------------------------------------------------------------------*/
/*- exported macros                                                        -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_MTL // Only defined within this component!
#endif               // COMPONENT_MTL

// min(a,b)
#define g_mtl_Min_mac(a, b) (((a) < (b)) ? (a) : (b))
// max(a,b)
#define g_mtl_Max_mac(a, b) (((a) > (b)) ? (a) : (b))
// |x|
#define g_mtl_Abs_mac(x) (((x) >= (0)) ? (x) : (-(x)))
// x^2
#define g_mtl_Square_mac(x) ((x) * (x))

// signum, determine sign of a value
// x>=0 -> signum(x)=1
// x<0  -> signum(x)=-1
#define g_mtl_Signum_mac(x) (((x) >= 0) ? (1) : (-1))
// even value
#define g_mtl_EvenValue_bl(x) (((x) & 0x1) == 0)
// odd value
#define g_mtl_OddValue_bl(x) (((x) & 0x1) == 1)

// unsigned increments/decrements with overflow/undeflow protection
#define g_mtl_Inc_u8_ui8(x)   (((x) >= GD_MAX_UI8) ? (x) : ((x) + 1))
#define g_mtl_Dec_u8_ui8(x)   (((x) <= GD_MIN_UI8) ? (x) : ((x) - 1))
#define g_mtl_Inc_u16_ui16(x) (((x) >= GD_MAX_UI16) ? (x) : ((x) + 1))
#define g_mtl_Dec_u16_ui16(x) (((x) <= GD_MIN_UI16) ? (x) : ((x) - 1))
#define g_mtl_Inc_u32_ui32(x) (((x) >= GD_MAX_UI32) ? (x) : ((x) + 1))
#define g_mtl_Dec_u32_ui32(x) (((x) <= GD_MIN_UI32) ? (x) : ((x) - 1))

// signed increments/decrements with overflow/undeflow protection
#define g_mtl_Inc_s8_si8(x)   (((x) >= GD_MAX_SI8) ? (x) : ((x) + 1))
#define g_mtl_Dec_s8_si8(x)   (((x) <= GD_MIN_SI8) ? (x) : ((x) - 1))
#define g_mtl_Inc_s16_si16(x) (((x) >= GD_MAX_SI16) ? (x) : ((x) + 1))
#define g_mtl_Dec_s16_si16(x) (((x) <= GD_MIN_SI16) ? (x) : ((x) - 1))
#define g_mtl_Inc_s32_si32(x) (((x) >= GD_MAX_SI32) ? (x) : ((x) + 1))
#define g_mtl_Dec_s32_si32(x) (((x) <= GD_MIN_SI32) ? (x) : ((x) - 1))

// access low / high byte
#define g_mtl_LowByte_ui8(x)  (*((UInt8 *)((void *)&(x))))
#define g_mtl_HighByte_ui8(x) (*((UInt8 *)((void *)&(x)) + 1))

#define g_mtl_LowWord_ui16(x)  (*((UInt16 *)((void *)&(x))))
#define g_mtl_HighWord_ui16(x) (*((UInt16 *)((void *)&(x)) + 1))

// test bit, set bit, clear bit
#define g_mtl_TstBit_u8_bl(x, bitnum)  (((x) & (UInt8)(((UInt8)1) << (bitnum))) != 0)
#define g_mtl_SetBit_u8_mac(x, bitnum) ((x) |= (UInt8)(((UInt8)1) << (bitnum)))
#define g_mtl_ClrBit_u8_mac(x, bitnum) ((x) &= (UInt8)(~(UInt8)(((UInt8)1) << (bitnum))))

#define g_mtl_TstBit_u16_bl(x, bitnum)  (((x) & (UInt16)(((UInt16)1) << (bitnum))) != 0)
#define g_mtl_SetBit_u16_mac(x, bitnum) ((x) |= (UInt16)(((UInt16)1) << (bitnum)))
#define g_mtl_ClrBit_u16_mac(x, bitnum) \
    ((x) &= (UInt16)(~(UInt16)(((UInt16)1) << (bitnum))))

#define g_mtl_TstBit_u32_bl(x, bitnum)  (((x) & (UInt32)(((UInt32)1) << (bitnum))) != 0)
#define g_mtl_SetBit_u32_mac(x, bitnum) ((x) |= (UInt32)(((UInt32)1) << (bitnum)))
#define g_mtl_ClrBit_u32_mac(x, bitnum) \
    ((x) &= (UInt32)(~(UInt32)(((UInt32)1) << (bitnum))))

// Mask testing
#define g_mtl_TstMask_u8_bl(x, mask)  (((x) & (mask)) != 0)
#define g_mtl_SetMask_u8_mac(x, mask) ((x) |= (mask))
#define g_mtl_ClrMask_u8_mac(x, mask) ((x) &= (UInt8)(~(mask)))

#define g_mtl_TstMask_u16_bl(x, mask)  (((x) & (mask)) != 0)
#define g_mtl_SetMask_u16_mac(x, mask) ((x) |= (mask))
#define g_mtl_ClrMask_u16_mac(x, mask) ((x) &= (UInt16)(~(mask)))

#define g_mtl_TstMask_u32_bl(x, mask)  (((x) & (mask)) != 0)
#define g_mtl_SetMask_u32_mac(x, mask) ((x) |= (mask))
#define g_mtl_ClrMask_u32_mac(x, mask) ((x) &= (UInt32)(~(mask)))

/*--------------------------------------------------------------------------*/
/*- exported data types                                                    -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_MTL // Only defined within this component!
#endif               // COMPONENT_MTL

/*--------------------------------------------------------------------------*/
/*- external object declarations                                           -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_MTL // Only defined within this component!
#endif               // COMPONENT_MTL

// mask with one bit set for every bit position
extern const UInt8 gc_mtl_Mask8bit_pui8[8];

// mask with one bit cleared for every bit position
extern const UInt8 gc_mtl_Maskinv8bit_pui8[8];

// mask from above left shifted by one bit position
extern const UInt8 gc_mtl_Mask8bitShl_pui8[8];

// mask from above right shifted by one bit position
extern const UInt8 gc_mtl_Mask8bitShr_pui8[8];

// mask with two bit set for every bit position
extern const UInt8 gc_mtl_Mask8bitDouble_pui8[7];

// mask with one bit set for each 2 bit positions
extern const UInt8 gc_mtl_Mask8bitspecial_pui8[6];

// mask with one bit set for every bit position, reverse bit order
extern const UInt8 gc_mtl_Mask8bitRev_pui8[8];

// mask with one bit cleared for every bit position, reverse bit order
extern const UInt8 gc_mtl_Maskinv8bitRev_pui8[8];

// mask with one bit set for every bit position
extern const UInt16 gc_mtl_Mask16bit_pui16[16];

// mask with two bit set for every bit position
extern const UInt16 gc_mtl_Mask16bitDouble_pui16[15];

/*--------------------------------------------------------------------------*/
/*- prototypes of exported functions                                       -*/
/*--------------------------------------------------------------------------*/
#ifdef COMPONENT_MTL // Only defined within this component!
#endif               // COMPONENT_MTL

#if (GS_MTL_USE_ERROR_HOOK == SW_ON)
extern void g_mtl_ErrorHook_vd(UInt8 *f_Msg_pui8);
#endif

// get error information
extern Boolean g_mtl_GetError_bl(void);
extern void g_mtl_HexToStr_vd(UInt8 *pbDest, UInt8 *pbSrc, int nLen);

//----------------------------------------------------------------------------
// basic operations including over- and underflow treatment
//----------------------------------------------------------------------------
// x + y
extern SInt8 g_mtl_s8_Add_s8_si8(SInt8 f_X_si8, SInt8 f_Y_si8);
extern SInt8 g_mtl_s8_Add_u8_si8(SInt8 f_X_si8, UInt8 f_Y_ui8);
extern UInt8 g_mtl_u8_Add_u8_ui8(UInt8 f_X_ui8, UInt8 f_Y_ui8);
extern UInt8 g_mtl_u8_Add_s8_ui8(UInt8 f_X_ui8, SInt8 f_Y_si8);

extern SInt16 g_mtl_s16_Add_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16);
extern SInt16 g_mtl_s16_Add_u16_si16(SInt16 f_X_si16, UInt16 f_Y_ui16);
extern UInt16 g_mtl_u16_Add_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16);

extern SInt32 g_mtl_s32_Add_s32_si32(SInt32 f_X_si32, SInt32 f_Y_si32);
extern SInt32 g_mtl_s32_Add_u32_si32(SInt32 f_X_si32, UInt32 f_Y_ui32);
extern UInt32 g_mtl_u32_Add_u32_ui32(UInt32 f_X_ui32, UInt32 f_Y_ui32);

// x - y
extern SInt8 g_mtl_s8_Sub_s8_si8(SInt8 f_X_si8, SInt8 f_Y_si8);
extern SInt8 g_mtl_s8_Sub_u8_si8(SInt8 f_X_si8, UInt8 f_Y_ui8);
extern SInt8 g_mtl_u8_Sub_u8_si8(UInt8 f_X_ui8, UInt8 f_Y_ui8);
extern SInt8 g_mtl_u8_Sub_s8_si8(UInt8 f_X_ui8, SInt8 f_Y_si8);
extern UInt8 g_mtl_u8_Sub_u8_ui8(UInt8 f_X_ui8, UInt8 f_Y_ui8);

extern SInt16 g_mtl_s16_Sub_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16);
extern SInt16 g_mtl_s16_Sub_u16_si16(SInt16 f_X_si16, UInt16 f_Y_ui16);
extern SInt16 g_mtl_u16_Sub_u16_si16(UInt16 f_X_ui16, UInt16 f_Y_ui16);
extern SInt16 g_mtl_u16_Sub_s16_si16(UInt16 f_X_ui16, SInt16 f_Y_si16);
extern UInt16 g_mtl_u16_Sub_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16);

extern SInt32 g_mtl_s32_Sub_s32_si32(SInt32 f_X_si32, SInt32 f_Y_si32);
extern SInt32 g_mtl_s32_Sub_u32_si32(SInt32 f_X_si32, UInt32 f_Y_ui32);
extern SInt32 g_mtl_u32_Sub_u32_si32(UInt32 f_X_ui32, UInt32 f_Y_ui32);
extern SInt32 g_mtl_u32_Sub_s32_si32(UInt32 f_X_ui32, SInt32 f_Y_si32);

// x * y
extern SInt16 g_mtl_s16_Mul_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16);
extern SInt16 g_mtl_s16_Mul_u16_si16(SInt16 f_X_si16, UInt16 f_Y_ui16);
extern UInt16 g_mtl_u16_Mul_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16);

// x / y
extern SInt8 g_mtl_s16_Div_s16_si8(SInt16 f_X_si16, SInt16 f_Y_si16);
extern SInt8 g_mtl_s16_Div_u16_si8(SInt16 f_X_si16, UInt16 f_Y_ui16);
extern UInt8 g_mtl_u16_Div_u16_ui8(UInt16 f_X_ui16, UInt16 f_Y_ui16);
extern SInt8 g_mtl_u16_Div_s16_si8(UInt16 f_X_ui16, SInt16 f_Y_si16);
extern SInt32 g_mtl_s32_Div_s32_si32(SInt32 f_X_si32, SInt32 f_Y_si32);
extern UInt32 g_mtl_u32_Div_u32_ui32(UInt32 f_X_ui32, UInt32 f_Y_ui32);
extern SInt16 g_mtl_s16_Div_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16);

// (x * y) / z
extern UInt16 g_mtl_u16_Mul_u16_Div_u16_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16,
                                             UInt16 f_Z_ui16);
extern SInt16 g_mtl_s16_Mul_s16_Div_s16_si16(SInt16 f_X_si16, SInt16 f_Y_si16,
                                             SInt16 f_Z_si16);

// limitations: x is kept inside minimum and maximum boundaries
extern UInt8 g_mtl_Min_u8_Max_u8_ui8(UInt8 f_X_ui8, UInt8 f_Min_ui8, UInt8 f_Max_ui8);
extern SInt8 g_mtl_Min_s8_Max_s8_si8(SInt8 f_X_si8, SInt8 f_Min_si8, SInt8 f_Max_si8);
extern UInt16 g_mtl_Min_u16_Max_u16_ui16(UInt16 f_X_ui16, UInt16 f_Min_ui16,
                                         UInt16 f_Max_ui16);
extern SInt16 g_mtl_Min_s16_Max_s16_si16(SInt16 f_X_si16, SInt16 f_Min_si16,
                                         SInt16 f_Max_si16);
extern SInt32 g_mtl_Min_s32_Max_s32_si32(SInt32 f_X_si32, SInt32 f_Min_si32,
                                         SInt32 f_Max_si32);

//----------------------------------------------------------------------------
// miscellaneous functions
//----------------------------------------------------------------------------

// square root
extern UInt8 g_mtl_Sqrt_u16_ui8(UInt16 f_X_ui16);
extern UInt16 g_mtl_Sqrt_u32_ui16(UInt32 f_X_ui32);
extern UInt16 g_mtl_Sqrt_VeryFastLessAccurate_u32_ui16(UInt32 f_X_ui32);

// precision reduction by right shifting with rounding
extern SInt16 g_mtl_ReducePrecision_si16(SInt16 f_X_si16, UInt8 f_BitNum_ui8);
extern SInt32 g_mtl_ReducePrecision_si32(SInt32 f_X_si32, UInt8 f_BitNum_ui8);
extern UInt16 g_mtl_ReducePrecision_ui16(UInt16 f_X_ui16, UInt8 f_BitNum_ui8);
extern UInt32 g_mtl_ReducePrecision_ui32(UInt32 f_X_ui32, UInt8 f_BitNum_ui8);

// endian conversion
// read a value from a reference pointer and output its endian converted value
extern UInt16 g_mtl_ReadChangeEndian_ui16(const void *f_RefIn_pvd);
extern UInt32 g_mtl_ReadChangeEndian_ui32(const void *f_RefIn_pvd);
// input a value and write the endian converted value in a reference pointer
extern void g_mtl_WriteChangeEndian_u16_vd(void *f_RefOut_pvd, UInt16 f_Value_ui16);
extern void g_mtl_WriteChangeEndian_u32_vd(void *f_RefOut_pvd, UInt32 f_Value_ui32);

// x^2 limited to 255
// x^2 > 255 -> result 255
extern UInt8 g_mtl_Square_u8_ui8(UInt8 f_X_ui8);

// general linear interpolation
// limitation: the step size between to x values must be a power of 2 -> table
// position can be found in one step f_Y_psi16:                array of values
// used for iterpolation f_Xmin_si16, f_Xmax_si16: corresponding x axis limits
// f_StepBitsX_ui8:          step size between 2 x axis values
// f_X_si16:                 argument for which y has to be interpolated
extern SInt16 g_mtl_LinIntPow2Step_si16(const SInt16 *f_Y_psi16, UInt8 f_LenY_ui8,
                                        SInt16 f_Xmin_si16, SInt16 f_Xmax_si16,
                                        UInt8 f_StepBitsX_ui8, SInt16 f_X_si16);

#if (GS_MTL_FILTER_FUNC == SW_ON)
// recursive smoothing filter of first order
// y(k) = fac*y(k-1) + (1-fac)*x(k)
//      = y(k-1) + (1-fac)*(x(k)-y(k-1))
// factor scale 2^15
extern UInt16 g_mtl_Filter_ui16(UInt16 f_X_ui16, UInt16 f_Y_ui16,
                                UInt16 f_ScaledFilterConst_ui16);
extern SInt16 g_mtl_Filter_si16(SInt16 f_X_si16, SInt16 f_Y_si16,
                                UInt16 f_ScaledFilterConst_ui16);

#endif // GS_MTL_FILTER_FUNC

//----------------------------------------------------------------------------
// trigonometric functions
//----------------------------------------------------------------------------

#if (GS_MTL_TRIGONOMETRIC_FUNC == SW_ON)
// sin(phi)
// input
//      UInt: rad
//      scale: 2^12
//      range: [-2.546*pi,2.546*pi] -> [-2^15, 2^15-1]
// output
//      scale: 2^14
//      range: [-1,1] -> [-2^14, 2^14]
extern SInt16 g_mtl_Sin_si16(SInt16 f_Phi_si16);

// cos(phi)
// input
//      unit: rad
//      scale: 2^12
//      range: [-2.546*pi,2.546*pi] -> [-2^15, 2^15-1]
// output
//      scale: 2^14
//      range: [-1,1] -> [-2^14, 2^14]
extern SInt16 g_mtl_Cos_si16(SInt16 f_Phi_si16);

// tan(phi)
// input
//      UInt: rad
//      scale: 2^12
//      range: [0, 41/32] -> [0, 41/32*2^12]
// output
//      scale: 2^13
//      range: [0, 3.36] -> [0, 3.36 * 2^13]
extern SInt16 g_mtl_Tan_si16(SInt16 f_Phi_si16);

// arctan(slope)
// input
//      range: slope: [-2^15, 2^15-1], ScaleBits for slope: 0...15 -> 1...2^15
// output
//      UInt: rad
//      scale: 2^12
//      range: [-pi/2, pi/2] -> [-2^12 * pi/2, 2^12 * pi/2]
extern SInt16 g_mtl_ArcTan_si16(SInt16 f_Slope_si16, UInt8 f_ScaleBits_ui8);

// arctan2(x,y)
// input
//      range: [2^31,2^31-1] for x and y
// output
//      unit: rad
//      scale: 2^22
//      range: [0,2pi] -> [0, 2^22 * 2pi]
extern UInt32 g_mtl_ArcTan2_ui32(SInt32 f_Y_si32, SInt32 f_X_si32);

#endif // #if( GS_MTL_TRIGONOMETRIC_FUNC == SW_ON )

//----------------------------------------------------------------------------
// interface functions
//----------------------------------------------------------------------------

// Called once during start of AUTOSAR SW-Component PA
extern void g_mathlibInit_vd(void);

//----------------------------------------------------------------------------
// for compatibility with outdated notations
//----------------------------------------------------------------------------

#define gc_mask8bit_pui8        gc_mtl_Mask8bit_pui8
#define gc_maskinv8bit_pui8     gc_mtl_Maskinv8bit_pui8
#define gc_mask8bitShl_pui8     gc_mtl_Mask8bitShl_pui8
#define gc_mask8bitShr_pui8     gc_mtl_Mask8bitShr_pui8
#define gc_mask8bitDouble_pui8  gc_mtl_Mask8bitDouble_pui8
#define gc_mask8bitspecial_pui8 gc_mtl_Mask8bitspecial_pui8
#define gc_mask8bitRev_pui8     gc_mtl_Mask8bitRev_pui8

#define gd_TIME_OF_FLIGHT_PER_CM_ui8 gd_MTL_TIME_OF_FLIGHT_PER_CM_ui8
#define g_MATHLIB_PreInit_vd         g_mtl_PreInit_vd
#define g_MATHLIB_Initial_vd         g_mtl_Initial_vd
#define g_Isqrt_ui8                  g_mtl_Sqrt_u16_ui8

#define g_Min_mac g_mtl_Min_mac
#define g_Max_mac g_mtl_Max_mac

#define g_Abs_mac g_mtl_Abs_mac

#define g_Isqr_mac g_mtl_Square_mac

#endif /* MATHLIB_API_H */
/*========================= EoF (mathlib_api.h) ====================*/
