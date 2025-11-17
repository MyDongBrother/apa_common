/**
 * @file dataInput_format.h
 * @brief  //TODO
 * @author xin.shi (xin.shi@bm-intelligent.com)
 * @version 1.0
 * @date 2023-03-13
 *
 * @copyright Copyright (c) 2023  bm-intelligent
 *
 * @par 修改日志:
 *
 * Date        Version  Author   Description
 * 2023-03-13 1.0     shixin      First version
 *
 */

#ifndef __DATA_INPUT_FORMAT_H__
#define __DATA_INPUT_FORMAT_H__
/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- Type  define                                                           -*/
/*--------------------------------------------------------------------------*/
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
/*--------------------------FV CAN INPUT STD FRAME---------------------------*/
typedef struct
{
    // wic
    UInt16 ESC_FLWheelSpeedRC;
    UInt16 ESC_FRWheelSpeedRC;
    UInt16 ESC_RLWheelSpeedRC;
    UInt16 ESC_RRWheelSpeedRC;
    // wic tick
    UInt16 FL_WicTick_ui16;
    UInt16 FR_WicTick_ui16;
    UInt16 RL_WicTick_ui16;
    UInt16 RR_WicTick_ui16;

    // wic valid   0:valid  1: invalid
    UInt8 ESC_FLWheelSpeedRC_valid;
    UInt8 ESC_FRWheelSpeedRC_valid;
    UInt8 ESC_RLWheelSpeedRC_valid;
    UInt8 ESC_RRWheelSpeedRC_valid;
    // wheel direction
    //  0: standstill  1: forward 2: backward  3:invalid
    UInt8 ESC_FLWheelDirection;
    UInt8 ESC_FRWheelDirection;
    UInt8 ESC_RLWheelDirection;
    UInt8 ESC_RRWheelDirection;
    // gear
    //  0:P  1:R  2:N 3:D  4:invalid
    UInt8 Gear_ui8;
    // steer angle
    // 0.01 degree
    SInt32 SteerWheelAngle_si32;
    // steer angle valid
    // 0: valid   1: invalid
    UInt8 SteerWheelAngle_valid;
    // wheel speed: 0.01km/h
    UInt16 ESC_FLWheelSpeedKPH_ui16;
    UInt16 ESC_FRWheelSpeedKPH_ui16;
    UInt16 ESC_RLWheelSpeedKPH_ui16;
    UInt16 ESC_RRWheelSpeedKPH_ui16;
    // wheel speed valid
    //  0: valid  1: invalid
    UInt8 ESC_FLWheelSpeed_valid;
    UInt8 ESC_FRWheelSpeed_valid;
    UInt8 ESC_RLWheelSpeed_valid;
    UInt8 ESC_RRWheelSpeed_valid;
    // vehicle speed : 0.1km/h
    UInt16 VehicleSpeed_ui16;
    // 0: valid  1: invalid
    UInt8 VehicleSpeed_valid;
    // 0.01 m/s^2
    SInt16 LateralAcce_si16;
    // 0: valid  1: invalid
    UInt8 LateralAcce_valid;
    // 0.01 m/s^2
    SInt16 LongitAcce_si16;
    // 0: valid  1: invalid
    UInt8 LongitAcce_valid;
    // 0.01 degree/s
    SInt16 YawRate_si16;
    // 0: valid  1: invalid
    UInt8 YawRate_valid;
    // 1 ℃
    SInt8 Temperature_si8;
    // 0: valid  1: invalid
    UInt8 TemperatureInvalid;
    // 0: middle  1: left 2: right 3: WarningLights
    UInt8 TurnIndicator;
    // power mode
    // 0: off  1: acc on  2: IGN on
    UInt8 PowerMode;
    // 0: valid  1: invalid
    UInt8 PowerMode_Valid;
    // engine status
    // 0: stop  1: running  2:invalid
    UInt8 EngineStatus;
    // 0:no rain  1: light rain 2: middle rain  3: heavy rain  4:invalid
    UInt8 RainfallLevel;
    SInt64 VehSigTimeStamp_si64; // 时间戳
    UInt16 StructLen_ui16;       // sizeof 结构体大小
} __attribute__((packed)) gType_FVInputCanFrame_st;

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
} __attribute__((packed)) gType_FVInputVehOdoFrame_st;

typedef struct
{
    UInt8 Parameter_ID_ui8;   // 参数 ID
    UInt16 ParameterLen_ui16; // 参数的字节长度
    UInt8 DataBuff[200];      // 参数
    UInt8 Vaild;
} __attribute__((packed)) gType_ParameterFrame_st;

typedef struct
{
    UInt16 VaildParameterNum_ui16;        // 参数的个数
    gType_ParameterFrame_st dataframe[5]; //
    SInt64 ParameterTimestamp_si64;       // 时间戳
} __attribute__((packed)) gType_ParameterInfo_st;
#endif