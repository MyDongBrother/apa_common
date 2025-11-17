/*************************************************************
 * @file dataOutput_format.h
 * @author rain (rain.zhang@bm-intelligent.com)
 * @brief
 * @version 0.1
 * @date 2022-11-16 10:28:31
 *
 * @copyright Copyright (c) 2022 by ForVision
 *
 ************************************************************/
#ifndef _DATA_OUTPUT_FORMAT_H_
#define _DATA_OUTPUT_FORMAT_H_

/*if use the standard type please define _STANDARD_TYPE_*/
#ifndef _USS_IF_STANDARD_TYPE_
#define _USS_IF_STANDARD_TYPE_

typedef unsigned char Boolean;
typedef signed char SInt8;       //        -128 .. +127
typedef unsigned char UInt8;     //           0 .. 255
typedef signed short SInt16;     //      -32768 .. +32767
typedef unsigned short UInt16;   //           0 .. 65535
typedef signed int SInt32;       // -2147483648 .. +2147483647
typedef unsigned int UInt32;     //           0 .. 4294967295
typedef signed long long SInt64; //-9223372036854775808..+9223372036854775807
#endif
#ifdef _USS_IF_STANDARD_TYPE_
#endif

typedef struct
{
    SInt32 VehPosOdoSMm_si32;      // 后轴中心点走过的距离  单位 MM
    SInt32 VehPosOdoXMm_si32;      // 后轴中心点的全局坐标x 单位 MM
    SInt32 VehPosOdoYMm_si32;      // 后轴中心点的全局坐标y 单位 MM
    UInt32 VehPosOdoYawAngle_ui32; // 车辆的纵向与全局坐标系x的夹角单位(rad *
                                   // 2^22)，需要对此值转为浮点值 /
                                   // 2^22获得rad单位的值
    SInt64 ODOTimeStamp_si64;      // 时间戳
    UInt16 StructLen_ui16;         // sizeof 结构体大小
} __attribute__((packed)) gType_VehOdoFrame_st;

#endif /* _DATA_OUTPUT_FORMAT_H_ */

/* END OF FILE */
