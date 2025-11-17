#ifndef _BASE_TYPEDEF_H_
#define _BASE_TYPEDEF_H_

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- macro define                                                           -*/
/*--------------------------------------------------------------------------*/
#define EXTERN    extern
#define DG_STATIC static
#ifndef VISIBILITY
#define VISIBILITY static
#endif
#ifndef SW_OFF
#define SW_OFF (0) // 2
#endif

#ifndef SW_ON
#define SW_ON (1) // 1
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef NULL
#define NULL (0)
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

#define _1_ (1)
#define _0_ (0)

// Maximum and Minimum values for all supported data types
#ifndef GD_MAX_UI8
//! Maximum for unsigned 8 Bit value (UInt8)
#define GD_MAX_UI8 ((UInt8)0xFF)
#endif
#ifndef GD_MIN_UI8
//! Minimum for unsigned 8 Bit value (UInt8)
#define GD_MIN_UI8 ((UInt8)0x00)
#endif
#ifndef GD_MAX_UI16
//! Maximum for unsigned 16 Bit value (UInt16)
#define GD_MAX_UI16 ((UInt16)0xFFFF)
#endif
#ifndef GD_MIN_UI16
//! Minimum for unsigned 16 Bit value (UInt16)
#define GD_MIN_UI16 ((UInt16)0x0000)
#endif
#ifndef GD_MAX_UI32
//! Maximum for unsigned 32 Bit value (UInt32)
#define GD_MAX_UI32 ((UInt32)0xFFFFFFFF)
#endif
#ifndef GD_MIN_UI32
//! Minimum for unsigned 32 Bit value (UInt32)
#define GD_MIN_UI32 ((UInt32)0x00000000)
#endif

#ifndef GD_MAX_SI8
//! Maximum for signed 8 Bit value (SInt8)
#define GD_MAX_SI8 ((SInt8)127)
#endif
#ifndef GD_MIN_SI8
//! Minimum for signed 8 Bit value (SInt8)
#define GD_MIN_SI8 ((SInt8)-128)
#endif
#ifndef GD_MAX_SI16
//! Maximum for signed 16 Bit value (SInt16)
#define GD_MAX_SI16 ((SInt16)32767)
#endif
#ifndef GD_MIN_SI16
//! Minimum for signed 16 Bit value (SInt16)
#define GD_MIN_SI16 ((SInt16)-32768)
#endif
#ifndef GD_MAX_SI32
//! Maximum for signed 32 Bit value (SInt32)
#define GD_MAX_SI32 ((SInt32)2147483647)
#endif
#ifndef GD_MIN_SI32
//! Minimum for signed 32 Bit value (SInt32)
#define GD_MIN_SI32 (-GD_MAX_SI32 - 1)
#endif

#define INL_OPT
/*--------------------------------------------------------------------------*/
/*- Type  define                                                           -*/
/*--------------------------------------------------------------------------*/
#ifndef _USS_IF_STANDARD_TYPE_
#define _USS_IF_STANDARD_TYPE_
typedef unsigned char Boolean;
typedef signed char SInt8;      //        -128 .. +127         ARREF:PLATFORM016
typedef unsigned char UInt8;    //           0 .. 255          ARREF:PLATFORM013
typedef signed short SInt16;    //      -32768 .. +32767       ARREF:PLATFORM017
typedef unsigned short UInt16;  //           0 .. 65535        ARREF:PLATFORM014
typedef signed int SInt32;      // -2147483648 .. +2147483647  ARREF:PLATFORM018
typedef unsigned int UInt32;    //           0 .. 4294967295   ARREF:PLATFORM015
typedef signed long long SInt64;
#endif
typedef unsigned long long UInt64;
//typedef unsigned char Boolean;
typedef unsigned int Bitfield;
typedef unsigned char
    Bitfield8;  // supported by GHS compiler (build_v800.pdf page 499)
typedef unsigned short
    Bitfield16;  // supported by GHS compiler (build_v800.pdf page 499)
typedef unsigned int
    Bitfield32;  // supported by GHS compiler (build_v800.pdf page 499)

//! Bit0 is at first position in Bitfield:            e.g. Bit0 ... Bit7
#define BITFLD_ORDER_NORMAL 1
//! Last bit is placed at first position in Bitfield: e.g. Bit7 ... Bit0
#define BITFLD_ORDER_REVERS 2
//! Bit0 is at first position in Bitfield:            e.g. Bit0 ... Bit7
#define GS_BITFLD_ORDER_NORMAL 1
//! Last bit is placed at first position in Bitfield: e.g. Bit7 ... Bit0
#define GS_BITFLD_ORDER_REVERS 2
/*--------------------------------------------------------------------------*/
/*                                                                          */
/* VARIABLE DECLARE                                                         */
/*                                                                          */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* FUNCTION DECLARE                                                         */
/*                                                                          */
/*--------------------------------------------------------------------------*/

#endif /* _BASE_TYPEDEF_H_ */
/* END OF FILE */
