/**
 * @file ApaRecvInfoPro.c
 * @brief APA自动泊车辅助系统CAN消息接收与处理模块
 *
 * - 负责接收、解析各类CAN报文，更新车辆传感器数据状态，管理CAN环形数据同步和线程安全。
 * - 支持实车和仿真模式，包含CAN数据缓存、回放等功能。
 *
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#include "ApaRecvInfoPro.h"
#include "ApaCtrlCarBody.h"
#include "MathFunc.h"
#include "Record_Log.h"
#include "Rte_BSW.h"
#include "Rte_Func.h"
#include "Ram.h"
#include "Platform_Types.h"
#include "PK_StateManage.h"
#include "PK_Calibration_Debug.h"
#include "Rte_ComIF.h"
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

static uint8_t canRecvReady = 0; ///< CAN接收是否准备就绪标志
static uint32_t printCanId  = 0; ///< 需打印的CAN ID（调试用）
static int issavecanlog     = 0; ///< 是否保存CAN日志标志（1表示保存）

/**@enum CANID_RX_INDEXxxx
 * @brief CAN 报文 ID 列表
 */
enum
{
    CANID_RX_INDEX0D5  = 0x0d5,
    CANID_RX_INDEX11F  = 0x11F, ///< 方向盘转角与转速数据
    CANID_RX_INDEX121  = 0x121, ///< 车辆速度信息
    CANID_RX_INDEX123  = 0x123, ///< 制动踏板状态信息
    CANID_RX_INDEX134  = 0x134,
    CANID_RX_INDEX135  = 0x135, ///< EPS（电子助力转向）相关数据
    CANID_RX_INDEX13f  = 0x13f,
    CANID_RX_INDEX173  = 0x173, ///< IPB2Park1状态信息
    CANID_RX_INDEX1F0  = 0x1F0, ///< 车轮速度及有效性状态数据
    CANID_RX_INDEX218  = 0x218, ///< 电子驻车状态
    CANID_RX_INDEX220  = 0x220, ///< 车辆保持状态
    CANID_RX_INDEX222  = 0x222, ///< 偏航率及相关状态信息
    CANID_RX_INDEX223  = 0x223, ///< 加速度及偏移量数据
    CANID_RX_INDEX241  = 0x241, ///< 电机扭矩信息
    CANID_RX_INDEX242  = 0x242, ///< 电机转速及传感器状态数据
    CANID_RX_INDEX251  = 0x251, ///< 电机实时扭矩数据
    CANID_RX_INDEX294  = 0x294, ///< 四门两盖安全带
    CANID_RX_INDEX2B6  = 0x2B6, ///< 时间戳数据
    CANID_RX_INDEX318  = 0x318,
    CANID_RX_INDEX341  = 0x341,
    CANID_RX_INDEX342  = 0x342, ///< 加速度踏板深度及档位信息
    CANID_RX_INDEX35C  = 0x35C, ///< 档位状态、APA状态及准备状态
    CANID_RX_INDEX38A  = 0x38A, ///< 前照灯状态
    CANID_RX_INDEX39E  = 0x39E,
    CANID_RX_INDEX407  = 0x407,  ///< 后视镜折叠
    CANID_RX_INDEX422  = 0x422,  ///< 车轮传感器计数器及有效性状态
    CANID_RX_INDEX500  = 0x500,  ///< IMU 加速度信息
    CANID_RX_INDEX501  = 0x501,  ///< IMU 角速度信息
    CANID_RX_INDEX502  = 0x502,  ///< IMU 姿态角信息
    CANID_RX_INDEX503  = 0x503,  ///< GPS 海拔与时间戳信息
    CANID_RX_INDEX504  = 0x504,  ///< GPS 纬度和经度信息
    CANID_RX_INDEX505  = 0x505,  ///< 北向/东向/地面速度信息
    CANID_RX_INDEX506  = 0x506,  ///< IMU 状态和车辆状态信息
    CANID_RX_INDEX507  = 0x507,  ///< IMU 位置与航向标准差信息
    CANID_RX_INDEX6001 = 0x6001, ///< AVM车位角点数据，仅在模拟环境下使用
    CANID_RX_INDEX6002 = 0x6002, ///< AVM车位角点数据，仅在模拟环境下使用
    CANID_RX_INDEX6100 = 0x6100, ///< 时间戳数据，计算超声波发送的时间基准偏移
    CANID_RX_INDEX6101 = 0x6101, ///< 超声波雷达数据，更新超声波传感器信息
    CANID_RX_INDEX6200 = 0x6200, ///< 用户命令数据，设置用户命令
    CANID_RX_INDEX6300 = 0x6300,
    CANID_RX_INDEX6301 = 0x6301,
    CANID_RX_INDEX6400 = 0x6400,
    CANID_RX_INDEX6410 = 0x6410,
    CANID_RX_INDEX6411 = 0x6411, ///< LED控制命令，仿真模式下转发命令
    CANID_RX_INDEX6412 = 0x6412, ///< LED多帧控制命令，仿真模式下拼接后转发
    CANID_RX_INDEX6500 = 0x6500,

    CANID_RX_INDEXMAX,
};

//! CAN 报文处理函数结构体生成宏
#define CAN_RXDATA_PROC_NAME(ID) \
    {ID,                         \
     CanRxDataProc##ID,          \
     {                           \
         0,                      \
     },                          \
     0}

//! 定义某个 ID 的数据处理函数
#define DEFINE_CAN_RXDATA_PROC(ID) \
    static void CanRxDataProc##ID(const DataOneByte canData[8])

/**
 * @brief 校验CAN数据帧的校验和是否合法
 *
 * 校验规则为：前7字节求和后异或0xFF，与第8字节比较。
 *
 * @param CanId     当前报文的 CAN ID（用于日志输出）
 * @param canData   接收到的 8 字节 CAN 数据帧
 * @retval 0        校验通过
 * @retval 1        校验失败，数据非法
 */
static int CheckCanData(const int CanId, const DataOneByte canData[8])
{
    uint8_t retSum = 0;
    for (int i = 0; i < 7; i++)
    {
        retSum += canData[i].byte;
    }
    retSum ^= 0xFF;

    if (retSum != canData[7].byte)
    {
        log_warn("canid 0x%x data=%02x %02x %02x %02x %02x %02x %02x %02x\n", CanId,
                 canData[0].byte, canData[1].byte, canData[2].byte, canData[3].byte,
                 canData[4].byte, canData[5].byte, canData[6].byte, canData[7].byte);
        return 1;
    }
    else
    {
        return 0;
    }
}

/** @brief 处理 CAN ID 0x500 的 IMU 加速度信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX500)
{
    /* Motorola */
    uint16_t accX, accY, accZ;
    accX = (uint16_t)canData[0].byte;
    accX <<= 8;
    accX += (uint16_t)canData[1].byte;

    accY = (uint16_t)canData[2].byte;
    accY <<= 8;
    accY += (uint16_t)canData[3].byte;

    accZ = (uint16_t)canData[4].byte;
    accZ <<= 8;
    accZ += (uint16_t)canData[5].byte;

    float faccX = accX * 0.0001220703125 - 4.0;
    float faccY = accY * 0.0001220703125 - 4.0;
    float faccZ = accZ * 0.0001220703125 - 4.0;

    RTE_BSW_Set_Accel_X(9.80665 * faccX);
    RTE_BSW_Set_Accel_Y(-9.80665 * faccY);
    RTE_BSW_Set_Accel_Z(-9.80665 * faccZ);
}

/** @brief 处理 CAN ID 0x501 的 IMU 角速度信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX501)
{
    /* Motorola */
    uint16_t gyroX, gyroY, gyroZ;
    gyroX = (uint16_t)canData[0].byte;
    gyroX <<= 8;
    gyroX += (uint16_t)canData[1].byte;

    gyroY = (uint16_t)canData[2].byte;
    gyroY <<= 8;
    gyroY += (uint16_t)canData[3].byte;

    gyroZ = (uint16_t)canData[4].byte;
    gyroZ <<= 8;
    gyroZ += (uint16_t)canData[5].byte;

    // angle speed 0.0001220703125unit deg/s
    float fgyroX = gyroX * 0.0076293 - 250;
    float fgyroY = gyroY * 0.0076293 - 250;
    float fgyroZ = gyroZ * 0.0076293 - 250;

    RTE_BSW_Set_AngRate_X(fgyroX);
    RTE_BSW_Set_AngRate_Y(-1.0 * fgyroY);
    RTE_BSW_Set_AngRate_Z(-1.0 * fgyroZ);
}

/** @brief 处理 CAN ID 0x502 的 IMU 姿态角信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX502)
{
    /* Motorola */
    uint16_t insPitch = canData[0].byte;
    insPitch <<= 8;
    insPitch |= canData[1].byte;

    uint16_t insRoll = canData[2].byte;
    insRoll <<= 8;
    insRoll |= canData[3].byte;

    uint16_t insHead = canData[4].byte;
    insHead <<= 8;
    insHead |= canData[5].byte;

    // 502 [-360,360] 0.010986 deg
    float angle[3];
    angle[0] = ((int)insPitch) * 0.010986 - 360;
    angle[1] = ((int)insRoll) * 0.010986 - 360;
    angle[2] = ((int)insHead) * 0.010986 - 360;

    RTE_BSW_Set_IMU_Pitch(angle[0]);
    RTE_BSW_Set_IMU_Roll(angle[1]);
    RTE_BSW_Set_IMU_Head(angle[2]);
    RTE_BSW_Set_ImuAngle(angle);

    RTE_BSW_Set_TPMS_PressureFL(1.8);
    RTE_BSW_Set_TPMS_PressureFR(1.8);
    RTE_BSW_Set_TPMS_PressureRL(1.8);
    RTE_BSW_Set_TPMS_PressureRR(1.8);
    RTE_BSW_Set_GPS_Bearing(0.1);
}

/** @brief 处理 CAN ID 0x503 的 GPS 海拔与时间戳信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX503)
{
    /* Motorola */
    uint32_t insLocalH, insTime;
    insLocalH = (uint32_t)canData[0].byte;
    insLocalH <<= 8;
    insLocalH += (uint32_t)canData[1].byte;
    insLocalH <<= 8;
    insLocalH += (uint32_t)canData[2].byte;
    insLocalH <<= 8;
    insLocalH += (uint32_t)canData[3].byte;

    insTime = (uint32_t)canData[4].byte;
    insTime <<= 8;
    insTime += (uint32_t)canData[5].byte;
    insTime <<= 8;
    insTime += (uint32_t)canData[6].byte;
    insTime <<= 8;
    insTime += (uint32_t)canData[7].byte;

    float localH = (float)insLocalH * 0.001 - 10000;
    RTE_BSW_Set_GPS_Altitude(localH);

    float curPos[3];
    RTE_BSW_Get_ImuCurPos(curPos);
    curPos[2] = localH;
    RTE_BSW_Set_ImuCurPos(curPos);
}

/** @brief 处理 CAN ID 0x504 的 GPS 纬度和经度信息（单位：度） */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX504)
{
    /* Motorola */
    uint32_t insLatitude, insLongitude;
    insLatitude = (uint32_t)canData[0].byte;
    insLatitude <<= 8;
    insLatitude += (uint32_t)canData[1].byte;
    insLatitude <<= 8;
    insLatitude += (uint32_t)canData[2].byte;
    insLatitude <<= 8;
    insLatitude += (uint32_t)canData[3].byte;

    insLongitude = (uint32_t)canData[4].byte;
    insLongitude <<= 8;
    insLongitude += (uint32_t)canData[5].byte;
    insLongitude <<= 8;
    insLongitude += (uint32_t)canData[6].byte;
    insLongitude <<= 8;
    insLongitude += (uint32_t)canData[7].byte;

    float latitude  = (float)insLatitude * 1e-7 - 180;
    float longitude = (float)insLongitude * 1e-7 - 180;
    RTE_BSW_Set_GPS_Latitude(latitude);
    RTE_BSW_Set_GPS_Longitude(longitude);

    float curPos[3];
    RTE_BSW_Get_ImuCurPos(curPos);
    curPos[0] = latitude;
    curPos[1] = longitude;
    RTE_BSW_Set_ImuCurPos(curPos);
}

/** @brief 处理 CAN ID 0x505 的北向/东向/地面速度信息（单位：m/s） */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX505)
{
    /* Motorola */
    uint16_t northSpd, eastSpd, groundSpd;
    northSpd = (uint16_t)canData[0].byte;
    northSpd <<= 8;
    northSpd |= (uint16_t)canData[1].byte;

    eastSpd = (uint16_t)canData[2].byte;
    eastSpd <<= 8;
    eastSpd |= (uint16_t)canData[3].byte;

    groundSpd = (uint16_t)canData[4].byte;
    groundSpd <<= 8;
    groundSpd |= (uint16_t)canData[5].byte;

    float northSpdf  = (float)northSpd * 0.0030517 - 100;
    float eastSpdf   = (float)eastSpd * 0.0030517 - 100;
    float groundSpdf = (float)groundSpd * 0.0030517 - 100;

    float gpsSpdf =
        sqrt(northSpdf * northSpdf + eastSpdf * eastSpdf + groundSpdf * groundSpdf);
    RTE_BSW_Set_IMU_VelSpd(gpsSpdf);

    float params[3] = {northSpdf, eastSpdf, groundSpdf};
    RTE_BSW_Set_Imuspd(params);
    // RTE_BSW_Set_CDU_SCU_GPSSpeed(gpsSpd);
}

/** @brief 处理 CAN ID 0x506 的 IMU 状态和车辆状态信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX506)
{
    /* Motorola */
    // uint8_t insGpsPosFlag = canData[0].byte;
    // uint8_t insSvNum = canData[1].byte;
    // uint8_t insGpsHeadingFlag = canData[2].byte;
    // uint8_t insGpsAge = canData[3].byte;
    uint8_t insCarStatus = canData[4].byte;
    uint8_t insStatus    = canData[7].byte;
    // RTE_BSW_Set_IMU_Self_check(insStatus);
    RTE_BSW_Set_IMU_SensorSt((insCarStatus << 8) | insStatus);
}

/** @brief 处理 CAN ID 0x507 的 IMU 位置与航向标准差信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX507)
{
    uint16 stdLat = (uint16)canData[0].byte;
    stdLat <<= 8;
    stdLat += (uint16)canData[1].byte;

    uint16 stdLon = (uint16)canData[2].byte;
    stdLon <<= 8;
    stdLon += (uint16)canData[3].byte;

    uint16 stdLocatHeight = (uint16)canData[4].byte;
    stdLocatHeight <<= 8;
    stdLocatHeight += (uint16)canData[5].byte;

    uint16 stdHeading = (uint16)canData[6].byte;
    stdHeading <<= 8;
    stdHeading += (uint16)canData[7].byte;

    float imuValStd[4];
    imuValStd[0] = stdLat * 0.001;
    imuValStd[1] = stdLon * 0.001;
    imuValStd[2] = stdLocatHeight * 0.01;
    imuValStd[3] = stdHeading * 0.01;
    RTE_BSW_Set_ImuValStd(imuValStd);
}

DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX0D5) {}

/** @brief 处理 CAN ID 0x11F 的方向盘转角与转速数据 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX11F)
{
    // The signal indicates a checksum. Byte 1 up to Byte 4 and low nibble of Byte 5 of
    // message LWS_Standard is used to calculate the check sum. temp_result = Byte 1 XOR
    // Byte 2 XOR Byte 3 XOR Byte 4 CHK_SUM = higher nibble(temp_result) XOR lower
    // nibble(temp_result) XOR MSG_CNT
    DataOneByte sum;
    sum.byte = canData[0].byte ^ canData[1].byte;
    sum.byte = sum.byte ^ canData[2].byte;
    sum.byte = sum.byte ^ canData[3].byte;
    sum.byte = sum.DataL4Bit03.b03 ^ sum.DataL4Bit03.b47;
    sum.byte = sum.byte ^ canData[4].DataL4Bit03.b03;
    if (sum.DataL4Bit03.b03 != canData[4].DataL4Bit03.b47)
    {
        log_warn("canid 0x%x data=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                 CANID_RX_INDEX11F, canData[0].byte, canData[1].byte, canData[2].byte,
                 canData[3].byte, canData[4].byte, canData[5].byte, canData[6].byte,
                 canData[7].byte);
        return;
    }

    /* Intel */
    int16_t lwsAngle = (int16_t)canData[1].byte;
    lwsAngle <<= 8;
    lwsAngle |= canData[0].byte;

    uint8_t lwsSpeed = canData[2].byte;
    // INT*4°/s; [0x00,0xFE];[0,1016]°/s
    RTE_BSW_Set_EPS_AngleSpd(lwsSpeed * 4);

    uint8_t failureStatus = canData[3].DataL1Bit1.b0;
    // 0x0:Sensor information invalid   0x1:Sensor information Valid
    RTE_BSW_Set_EPS_AngleSt(failureStatus);

    // [ -780.0°,+779.9°]     Resolution: 0.1°/bit
    int16_t angle = (int16_t)lwsAngle;
    RTE_BSW_Set_EPS_Angle(angle * 0.1);
}

/** @brief 处理 CAN ID 0x121 的车辆速度信息。*/
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX121)
{
    if (CheckCanData(CANID_RX_INDEX121, canData) != 0)
    {
        return;
    }

    /* Intel */
    uint16_t vehSpd = canData[1].DataL4Bit03.b03;
    vehSpd          = (vehSpd & 0x0F) << 8;
    vehSpd |= (uint16_t)canData[0].byte;

    uint8_t vehSpdStatus = canData[1].DataL1Bit1.b7;
    // uint8_t aebdecActive = canData[3].DataL1Bit1.b6;

    // 0x0:Valid0x1:InvalidInit:0x0/Default:0x1
    RTE_BSW_Set_ESC_VehSpdVD(vehSpdStatus);

    // Vehicle_speed  0.06875km/h [0; 281.4625] km/h Default:0xFFF"
    RTE_BSW_Set_ESC_VehSpd(vehSpd * 0.06875);
    RTE_BSW_Set_ESC_MinVehSpd(0.06875);
}

/** @brief 处理 CAN ID 0x123 的制动踏板状态信息。*/
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX123)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX123, canData) != 0)
    {
        return;
    }

    uint8_t ESP_BRAKE_PEDAL_STATUS = canData[5].DataL2Bit23.b34;
    RTE_BSW_Set_BrakeSt(ESP_BRAKE_PEDAL_STATUS == 0x01);
}

/**
 * @brief 解析12位有符号CAN数据
 *
 * @param data 原始16位数据，实际有效位12位
 * @return int16_t 解析后的有符号值
 *
 * @note:
 * - 只保留低12位数据
 * - 最高有效位为符号位，1表示负数，2's补码形式
 */
static int16_t parseCanData(uint16_t data)
{
    // 仅保留12位有效数据
    data &= 0x0FFF;

    // 判断最高位是否为符号位
    if (data & 0x0800)
    {                          // 最高位为1，表示负数
        data = ~data & 0x0FFF; // 取反并保持为12位
        return -(data + 1);    // 取反加1并返回负数
    }
    else
    {
        return data; // 正数直接返回
    }
}

/** @brief 解析CAN ID 0x135的EPS（电子助力转向）相关数据*/
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX135)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX135, canData) != 0)
    {
        return;
    }

    uint8_t EPS_StrWhlTorqVD = canData[2].DataL1Bit1.b7;
    RTE_BSW_Set_EpsStrWhlTorqVD(EPS_StrWhlTorqVD);

    //"PH=INT*0.01Nm INT∈[0xB1E，0x4E2] PH∈[-12.50，12.50]Nm Resolution：0.01Nm
    // Left：Positive Right：Negative"
    uint16_t EPS_StrWhlTorqVal_Tmp = (((uint16)canData[4].byte) & 0x000F);
    EPS_StrWhlTorqVal_Tmp <<= 8;
    EPS_StrWhlTorqVal_Tmp += (uint16)canData[3].byte;

    int16_t EPS_StrWhlTorqVal = parseCanData(EPS_StrWhlTorqVal_Tmp);
    RTE_BSW_Set_EpsStrWhlTorqVal(EPS_StrWhlTorqVal * 0.01f);

    uint8_t EPS_APA_CtrlStas = canData[5].DataL1Bit1.b0;
    EPS_APA_CtrlStas <<= 2;
    EPS_APA_CtrlStas += canData[4].DataL2Bit01.b67;

    RTE_BSW_Set_EpsCtrlStat((RTE_BSW_EpsCtrlStatType)EPS_APA_CtrlStas);
}

/** @brief 解析CAN ID 0x173的IPB2Park1状态信息，仅用于记录 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX173)
{
    // only for record!!!
    if (CheckCanData(CANID_RX_INDEX173, canData) != 0)
    {
        return;
    }

    uint8_t IPB2Park1_LSMPosLongACapbl_S  = canData[0].DataL2Bit01.b01;
    uint8_t IPB2Park1_LSMSubMTLevelEcho_S = canData[0].DataL2Bit01.b23;
    uint8_t IPB2Park1_LSMSubMTLongEcho_S  = canData[0].DataL2Bit01.b45;
    uint8_t IPB2Park1_LSMSubMTReqEcho_S   = canData[0].DataL2Bit01.b67;
    uint8_t IPB2Park1_Lsm12ComfortAvl_S   = canData[1].DataL2Bit01.b01;
    uint8_t IPB2Park1_Lsm12EmergencyAvl_S = canData[1].DataL2Bit01.b23;
    uint8_t IPB2Park1_Lsm23Avl_S          = canData[1].DataL2Bit01.b45;
    uint8_t IPB_BrakeOverride             = canData[2].DataL1Bit1.b0;

    RTE_BSW_Set_LSMPosLongACapbl(IPB2Park1_LSMPosLongACapbl_S);
    RTE_BSW_Set_LSMSubMTLevelEcho(IPB2Park1_LSMSubMTLevelEcho_S);
    RTE_BSW_Set_LSMSubMTLongEcho(IPB2Park1_LSMSubMTLongEcho_S);
    RTE_BSW_Set_LSMSubMTReqEcho(IPB2Park1_LSMSubMTReqEcho_S);
    RTE_BSW_Set_Lsm12ComfortAvl(IPB2Park1_Lsm12ComfortAvl_S);
    RTE_BSW_Set_Lsm12EmergencyAvl(IPB2Park1_Lsm12EmergencyAvl_S);
    RTE_BSW_Set_Lsm23Avl(IPB2Park1_Lsm23Avl_S);
    RTE_BSW_Set_BrakeOverride(IPB_BrakeOverride);
}

/** @brief 解析CAN ID 0x1F0的车轮速度及有效性状态数据*/
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX1F0)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX1F0, canData) != 0)
    {
        return;
    }

    // 2.4  Wheel_Speed_FR_Stats_1F0_S 0x0：Valid  0x1：Invalid
    uint8_t WheelSpeed_Valid = canData[1].DataL1Bit1.b4;
    uint8_t valid            = WheelSpeed_Valid == 0 ? 1 : 0;
    RTE_BSW_Set_ESC_WheelSpdFRVD(valid);

    // 2.5  Wheel_Speed_FL_Stats_1F0_S 0x0：Valid  0x1：Invalid
    WheelSpeed_Valid = canData[1].DataL1Bit1.b5;
    valid            = WheelSpeed_Valid == 0 ? 1 : 0;
    RTE_BSW_Set_ESC_WheelSpdFLVD(valid);

    // 2.6  Wheel_Speed_RR_Stats_1F0_S 0x0：Valid  0x1：Invalid
    WheelSpeed_Valid = canData[1].DataL1Bit1.b6;
    valid            = WheelSpeed_Valid == 0 ? 1 : 0;
    RTE_BSW_Set_ESC_WheelSpdRRVD(valid);

    // 2.7  Wheel_Speed_RL_Stats_1F0_S 0x0：Valid  0x1：Invalid
    WheelSpeed_Valid = canData[1].DataL1Bit1.b7;
    valid            = WheelSpeed_Valid == 0 ? 1 : 0;
    RTE_BSW_Set_ESC_WheelSpdRLVD(valid);

    // SG_ WheelSpeed_FL : 0|12@1+ (0.06875,0) [0|281.4625] "km/h"  MRR
    uint16_t WheelSpeed = canData[1].DataL4Bit03.b03;
    WheelSpeed <<= 8;
    WheelSpeed |= (uint8_t)canData[0].byte;

    float valuef = ((int)WheelSpeed) * 0.06875 / 3.6;
    RTE_BSW_Set_ESC_WheelSpdFL(valuef);

    // SG_ WheelSpeed_FR : 16|12@1+ (0.06875,0) [0|281.4625] "km/h"  MRR
    WheelSpeed = canData[3].DataL4Bit03.b03;
    WheelSpeed <<= 8;
    WheelSpeed |= (uint8_t)canData[2].byte;
    valuef = ((int)WheelSpeed) * 0.06875 / 3.6;
    RTE_BSW_Set_ESC_WheelSpdFR(valuef);

    // SG_ WheelSpeed_RL : 28|12@1+ (0.06875,0) [0|281.4625] "km/h"  MRR
    WheelSpeed = (uint16_t)canData[4].byte;
    WheelSpeed <<= 4;
    WheelSpeed |= (uint8_t)canData[3].DataL4Bit03.b47;
    valuef = ((int)WheelSpeed) * 0.06875 / 3.6;
    RTE_BSW_Set_ESC_WheelSpdRL(valuef);

    // SG_ WheelSpeed_RR : 40|12@1+ (0.06875,0) [0|281.4625] "km/h"  MRR
    WheelSpeed = canData[6].DataL4Bit03.b03;
    WheelSpeed <<= 8;
    WheelSpeed |= (uint8_t)canData[5].byte;
    valuef = ((int)WheelSpeed) * 0.06875 / 3.6;
    RTE_BSW_Set_ESC_WheelSpdRR(valuef);
}

/** @brief 解析CAN ID 0x218的电子驻车状态 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX218)
{
    // only for record!!!
    if (CheckCanData(CANID_RX_INDEX218, canData) != 0)
    {
        return;
    }

    uint8_t ElecParkBrakeSta = (((uint16)canData[1].byte) & 0x0E);
    ElecParkBrakeSta >>= 1;
    RTE_BSW_Set_EpbStat((RTE_BSW_EpbModeType)ElecParkBrakeSta);

    uint8_t ElecParkLeftCaliperState  = canData[5].DataL3Bit24.b24;
    uint8_t ElecParkRightCaliperState = canData[5].DataL3Bit24.b57;

    RTE_BSW_Set_ElecParkLeftCaliperState(ElecParkLeftCaliperState);
    RTE_BSW_Set_ElecParkRightCaliperState(ElecParkRightCaliperState);
}

/** @brief 解析CAN ID 0x220的车辆保持状态 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX220)
{
    // only for record!!!
    if (CheckCanData(CANID_RX_INDEX220, canData) != 0)
    {
        return;
    }

    /* 左前轮行驶方向 + 状态 */
    uint8_t flDir  = ((uint16)canData[2].byte) & 0x03;
    uint8_t flStat = ((uint16)canData[2].byte >> 2) & 0x01;
    RTE_BSW_Set_WheelDrive_FL((RTE_BSW_MoveDirType)flDir);
    RTE_BSW_Set_WheelDrive_FLVD(flStat);

    /* 右前轮行驶方向 + 状态 */
    uint8_t frDir  = ((uint16)canData[3].byte) & 0x03;
    uint8_t frStat = ((uint16)canData[3].byte >> 2) & 0x01;
    RTE_BSW_Set_WheelDrive_FR((RTE_BSW_MoveDirType)frDir);
    RTE_BSW_Set_WheelDrive_FRVD(frStat);

    /* 左后轮行驶方向 + 状态 */
    uint8_t rlDir  = ((uint16)canData[4].byte) & 0x03;
    uint8_t rlStat = ((uint16)canData[4].byte >> 2) & 0x01;
    RTE_BSW_Set_WheelDrive_RL((RTE_BSW_MoveDirType)rlDir);
    RTE_BSW_Set_WheelDrive_RLVD(rlStat);

    /* 右后轮行驶方向 + 状态 */
    uint8_t rrDir  = ((uint16)canData[5].byte) & 0x03;
    uint8_t rrStat = ((uint16)canData[5].byte >> 2) & 0x01;
    RTE_BSW_Set_WheelDrive_RR((RTE_BSW_MoveDirType)rrDir);
    RTE_BSW_Set_WheelDrive_RRVD(rrStat);
}

/** @brief 解析CAN ID 0x222的偏航率及相关状态信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX222)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX222, canData) != 0)
    {
        return;
    }

    // 1.0-2.3  12  YawRate "PH:[-2.0943rad/s,+2.0943rad/s]
    // PH=INT*0.002132603-2.0943[rad/s] Resolution: 0.002132603[rad/s]Initialization
    // value:0[rad/s]Invalid value:FFFh"
    uint16_t yawRate = canData[1].DataL4Bit03.b03;
    yawRate <<= 8;
    yawRate |= (uint8_t)canData[0].byte;

    float yawRatef = ((int)yawRate) * 0.002132603 - 2.0943;
    RTE_BSW_Set_ESC_YAW_RATE(yawRatef);

    // 2.4-3.7 12   YawRateOffset   "PH:[-0.13rad/s,+0.13rad/s]
    // PH=INT*0.002132603-0.13[rad/s] Resolution:0.002132603[rad/s]Initialization
    // value:0[rad/s]Invalid value:FFFh"
    uint16_t yawRateOffset = (uint16_t)canData[2].byte;
    yawRateOffset <<= 4;
    yawRateOffset |= (uint8_t)canData[1].DataL4Bit03.b47;
    float yawRateOffsetf = ((int)yawRateOffset) * 0.002132603 - 0.13;
    RTE_BSW_Set_ESC_YAW_RATE_Offset(yawRateOffsetf);

    uint8_t escApaStatus = canData[4].DataL3Bit02.b02;
    RTE_BSW_Set_EscApaStat((RTE_BSW_EscCtrlStatType)escApaStatus);

    // 4.7 1   Yaw_Rate_Status_S   Yaw rate status "0x0：Not Valid 0x1：Valid"
    // "0x0：NotValid0x1：Valid"
    uint8_t yawStatus = canData[3].DataL3Bit13.b7;
    RTE_BSW_Set_ESC_YAW_RATE_VD(yawStatus);
}

/** @brief 解析CAN ID 0x223的加速度及偏移量数据 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX223)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX223, canData) != 0)
    {
        return;
    }

    // 1.0-2.3 12  AX  "PH:[-21.593m/s2,+21.593m/s2] PH=INT* 0.027126736-21.593m/s2]
    // Resolution:0.027126736[m/s2] Initialisation value:0[m/s2] Invalid value:7FFFh"
    uint16_t value = (uint16_t)canData[1].DataL2Bit01.b01;
    value <<= 8;
    value |= (uint8_t)canData[0].byte;
    float fvalue = ((int)value) * 0.027126736 - 21.593;
    RTE_BSW_Set_ESC_AX(fvalue);

    // 2.4-3.7 12  AX offset "PH:[-21.593m/s2,+21.593m/s2]  PH=INT*
    // 0.027126736-21.593m/s2] Resolution:0.027126736[m/s2]  Initialisation value:0[m/s2]
    // Invalid value:7FFFh"
    value = (uint16_t)canData[2].byte;
    value <<= 4;
    value |= (uint8_t)canData[1].DataL4Bit03.b47;
    fvalue = ((int)value) * 0.027126736 - 21.593;
    RTE_BSW_Set_ESC_AXOffSet(fvalue);

    // 4.0-5.3 12  AY "PH:[-21.593m/s2,+21.593m/s2]  PH=INT* 0.027126736-21.593m/s2]
    // Resolution:0.027126736[m/s2]  Initialisation value:0[m/s2] Invalid value:7FFFh"
    value = (uint16_t)canData[4].DataL2Bit01.b01;
    value <<= 8;
    value |= (uint8_t)canData[3].byte;
    fvalue = ((int)value) * 0.027126736 - 21.593;
    RTE_BSW_Set_ESC_AY(fvalue);

    // 5.4-6.7 12  AY Offset  "PH:[-21.593m/s2,+21.593m/s2] PH=INT*
    // 0.027126736-21.593m/s2]     Resolution:0.027126736[m/s2] Initialisation
    // value:0[m/s2] Invalid value:7FFFh"
    value = (uint16_t)canData[5].byte;
    value <<= 4;
    value |= (uint8_t)canData[4].DataL4Bit03.b47;
    fvalue = ((int)value) * 0.027126736 - 21.593;
    RTE_BSW_Set_ESC_AYOffSet(fvalue);

    // 7.0-7.3 4   Message counter 0-15    APA ECU unsigned
    // 7.6 1   AY status   "0x0:Not valid  0x1:Valid"     APA ECU unsigned
    // 7.7 1   AX status   "0x0:Not valid  0x1:Valid"       APA ECU unsigned
}

/** @brief 解析CAN ID 0x241的电机扭矩信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX241)
{
    /* Intel */
    // if (CheckCanData(CANID_RX_INDEX241, canData) != 0) {  return;  }

    uint16_t motorTorq = canData[1].byte;
    motorTorq <<= 8;
    motorTorq |= (uint16_t)canData[0].byte;

    //  "Vehicle Torque front torque PH = INT-12000 [-12000;12000] NM[0x0000;0x5DC0]"
    float fmotorTorq = (float)motorTorq;
    fmotorTorq       = fmotorTorq - 12000;
    RTE_BSW_Set_IPU_MotorTorq(fmotorTorq);
}

/** @brief 解析CAN ID 0x242的电机转速及传感器状态数据 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX242)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX242, canData) != 0)
    {
        return;
    }
    uint16_t motorSpd = canData[1].byte;
    motorSpd <<= 8;
    motorSpd |= (uint16_t)canData[0].byte;

    float fmotorSpd = (float)motorSpd;
    fmotorSpd       = fmotorSpd - 30000;

    //  "Motor speed Torque front motor roat speed" "PH = INT-30000 [-30000;30000] N"
    RTE_BSW_Set_Motorspd(fmotorSpd);

    // SG_ ERROR_Front_Motor_SPEED : 32|1@1+ (1,0) [0|1] "bit" Vector__XXX
    uint8_t spdSt = canData[4].DataL1Bit1.b0;
    // RTE_BSW_Set_Motor_SensorSt(spdSt);
}

/** @brief 解析CAN ID 0x251的电机实时扭矩数据 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX251)
{
    if (CheckCanData(CANID_RX_INDEX251, canData) != 0)
    {
        return;
    }

    uint16_t motorTorq = canData[3].byte;
    motorTorq <<= 8;
    motorTorq |= (uint16_t)canData[2].byte;
    float fmotorTorq = (float)motorTorq;
    fmotorTorq       = fmotorTorq - 12000;
    RTE_BSW_Set_IPU_MotorRTorq(fmotorTorq);
}

/** @brief 解析CAN ID 0x294的车门、安全带、后背门及紧急告警状态数据 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX294)
{
    if (CheckCanData(CANID_RX_INDEX294, canData) != 0)
    {
        return;
    }
    RTE_BSW_BCM_StateType bcmSt;
    RTE_BSW_Get_BCM_VehicleState(&bcmSt);

    // 1.0-1.1 2   Left_Front_Door_Status_294_S    左前门状态
    // 0x0:Invalid0x1:Close0x2:Open0x3:Reserved
    bcmSt.BCM_DriverDoorAjarSt = (canData[0].DataL2Bit01.b01 == 2) ? 1 : 0;

    // 1.2-1.3 2   Right_Front_Door_Status_294_S   右前门状态
    // 0x0:Invalid0x1:Close0x2:Open0x3:Reserved
    bcmSt.BCM_PsngrDoorAjarSt = (canData[0].DataL2Bit01.b23 == 2) ? 1 : 0;

    // 1.4-1.5 2   Left_Back_Door_Status_294_S 左后门状态
    // 0x0:Invalid0x1:Close0x2:Open0x3:Reserved
    bcmSt.BCM_RLDoorAjarSt = (canData[0].DataL2Bit01.b45 == 2) ? 1 : 0;

    // 1.6-1.7 2   Right_Back_Door_Status_294_S    右后门状态
    // 0x0:Invalid0x1:Close0x2:Open0x3:Reserved
    bcmSt.BCM_RRDoorAjarSt = (canData[0].DataL2Bit01.b67 == 2) ? 1 : 0;

    // 2.0 1   LeftF_Door_Stats_Valid_Bit_S    左前门状态有效位 0x0:Valid0x1:Invalid 2.1
    // 1   RightF_Door_Sts_Valid_Bit_294_S 右前门状态有效位        0x0:Valid0x1:Invalid
    // 2.2     1   LeftR_Door_Stats_Valid_Bit_S    左后门状态有效位 0x0:Valid0x1:Invalid
    // 2.3     1   RightR_Door_Sts_Valid_Bit_294_S 右后门状态有效位 0x0:Valid0x1:Invalid
    // 2.4     1   Driver_Belt_Stat_Effective_Bit_S    驾驶员安全带状态有效位
    // 0x0:Valid0x1:Invalid 2.5     1   Back_Door_Sts_Effectiv_Bit_294_S 后背门状态有效位
    // 0x0:Valid0x1:Invalid 2.6     1   Frnt_Hatch_Stat_Effective_Bit_S 前舱盖状态有效位
    // 0x0:Valid0x1:Invalid

    // 3.0-3.1 2   Driver_Belt_Status_S    驾驶员安全带状态      0x0:Invalid 0x1:Unlock
    // 0x2:Buckle 0x3:Reserved
    uint8_t value = canData[2].DataL2Bit01.b01;
    if (value == 2)
    {
        bcmSt.SRS_DriverSeatBeltWarning = 0; // 无告警
    }
    else
    {
        bcmSt.SRS_DriverSeatBeltWarning = 1; // 有告警
    }

    // 3.2-3.3 2   Back_Door_Status_294_S  后背门状态      0x0:Invalid 0x1:Close 0x2:Open
    // 0x3:Reserved
    bcmSt.RDM_RearDoorStatus = (canData[2].DataL2Bit01.b23 == 2) ? 1 : 0;

    // 3.4-3.5 2   Former_Hatch_Status_294_S   前舱盖状态      0x0:Invalid 0x1:Close
    // 0x2:Open 0x3:Reserved
    bcmSt.BCM_BonnetAjarSt = (canData[2].DataL2Bit01.b45 == 2) ? 1 : 0;

    bcmSt.BCM_DriverSeatOccupied = 0;

    RTE_BSW_Set_BCM_VehicleState(&bcmSt);
}

/** @brief 解析CAN ID 0x2B6的时间戳数据，更新系统时间结构体 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX2B6)
{
    // static int first = 0;
    // if (first == 1) {  return;  }
    // first = 1;
    struct tm stdtm;
    stdtm.tm_sec   = canData[5].byte;
    stdtm.tm_min   = canData[4].byte;
    stdtm.tm_hour  = canData[3].byte;
    stdtm.tm_mday  = canData[2].byte;
    stdtm.tm_mon   = canData[1].byte;
    stdtm.tm_year  = canData[0].byte;
    stdtm.tm_wday  = 0;
    stdtm.tm_yday  = 0;
    stdtm.tm_isdst = 0;

    // printf("%s:%d ======= %d\n",__FILE__, __LINE__, stdtm.tm_year);
    RTE_BSW_Set_AbsStamp(&stdtm);
}

/** @brief 解析CAN ID 0x342的数据，更新加速度踏板深度及档位信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX342)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX342, canData) != 0)
    {
        return;
    }

    //"PH=INT*1% [0，100] [0x00，0x64] 初始值/默认值：0x0"
    uint8_t VCU_Accelerograph_Depth = canData[0].byte;
    RTE_BSW_Set_VcuAccelerographDepth(VCU_Accelerograph_Depth * 0.01f);

    uint8_t VCU_Acelrogra_Dpth_Virtual_Val = canData[2].DataL1Bit1.b0;
    RTE_BSW_Set_VcuAccelerographDepthVD(VCU_Acelrogra_Dpth_Virtual_Val);

    // uint8_t throttleDepth = canData[0].byte;
    // uint8_t throttleSignificantDepth = canData[2].DataL1Bit1.b0;
    // uint8_t driveMode = canData[2].DataL2Bit01.b45;
    uint8_t targetGear = canData[3].DataL4Bit03.b03;

    // 0x0; Reserved 0x1:P gear 0x2:R 0x3:N 0x4：D 0x5-0xC:Reserved
    // 0:GEAR_REAL_NONE, 1:GEAR_REAL_D,  2:GEAR_REAL_N,  3: GEAR_REAL_R, 4: GEAR_REAL_P,
    // GEAR_REAL_MAX
    uint8_t gear             = 0;
    RTE_BSW_GearType curGear = GEAR_REAL_NONE;
    if (targetGear == 4)
    {
        gear    = 1;
        curGear = GEAR_REAL_D;
    }
    else if (targetGear == 2)
    {
        gear    = 3;
        curGear = GEAR_REAL_R;
    }
    else if (targetGear == 1)
    {
        gear    = 4;
        curGear = GEAR_REAL_P;
    }
    else if (targetGear == 3)
    {
        gear    = 2;
        curGear = GEAR_REAL_N;
    }
    else if (targetGear > 4)
    {
        gear    = targetGear;
        curGear = GEAR_REAL_MAX;
    }

    RTE_BSW_Set_CurrentGear(curGear);
}

/** @brief 解析CAN ID 0x35C的数据，更新档位状态、APA状态及准备状态 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX35C)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX35C, canData) != 0)
    {
        return;
    }

    //"0x0：Normal 0x1：Abnormal"
    uint8_t Gear_System_Status = canData[0].DataL1Bit1.b0;
    RTE_BSW_Set_GearSystemStatus(Gear_System_Status);

    // 1.6-1.7 2 0x0：No operation 未控制0x1：NormalOperation正常控制0x2：Driver Gear
    // Interference驾驶员干扰档位控制0x3：预留
    uint8_t SCU_Feedback_APA_Status_Info = canData[1].DataL3Bit02.b67;
    RTE_BSW_Set_TcuApaStat((RTE_TCU_ApaStatusType)SCU_Feedback_APA_Status_Info);

    uint8_t apaCtrlState = canData[2].DataL3Bit02.b35;
    RTE_BSW_Set_EvReadySt(apaCtrlState);
}

/** @brief 解析CAN ID 0x38A的前照灯状态数据，更新近光灯和远光灯状态 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX38A)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX38A, canData) != 0)
    {
        return;
    }

    // 1.2 - 1.3 Dipped_Headlight_38A_S 0x0：无效 0x1：打开 0x2：关闭 0x3：预留
    uint8_t Dipped_Headlight_St = canData[0].DataL2Bit01.b23;
    RTE_BSW_Set_Dipped_Headlight_St((RTE_BSW_LightStateType)Dipped_Headlight_St);

    // 1.4 - 1.5 High_Beam_Light_38A_S 0x0：无效 0x1：打开 0x2：关闭 0x3：预留
    uint8_t High_Beam_Light_St = canData[0].DataL2Bit01.b45;
    RTE_BSW_Set_High_Beam_Light_St((RTE_BSW_LightStateType)High_Beam_Light_St);
}

/** @brief 解析CAN ID 0x407的外后视镜折叠/展开状态 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX407)
{
    /* Intel */
    // if (CheckCanData(CANID_RX_INDEX407, canData) != 0) {  return;  }

    //"0x0：NoAction 0x1：Folded 0x2：Expanded 0x3：Reserved"
    uint8_t Fldig_Extrr_Mrors_Expand = canData[2].DataL2Bit01.b67;

    if (Fldig_Extrr_Mrors_Expand == 1 || Fldig_Extrr_Mrors_Expand == 2)
    {
        RTE_BSW_Set_FldigExtrrMrorsExpand(
            (RTE_BSW_FldigExtrrMrorsType)Fldig_Extrr_Mrors_Expand);
    }
}

/** @brief 解析CAN ID 0x422的车轮传感器计数器及有效性状态 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX422)
{
    /* Intel */
    if (CheckCanData(CANID_RX_INDEX422, canData) != 0)
    {
        return;
    }
    uint16_t flCounter, frCounter, rlCounter, rrCounter;
    uint8_t fl_validity, fr_validity, rl_validity, rr_validity;
    flCounter = (uint16_t)canData[1].DataL2Bit01.b01;
    flCounter <<= 8;
    flCounter += (uint16_t)canData[0].byte;

    fl_validity = canData[1].DataL1Bit1.b4;
    fr_validity |= canData[1].DataL1Bit1.b5;
    rl_validity |= canData[1].DataL1Bit1.b6;
    rr_validity |= canData[1].DataL1Bit1.b7;

    frCounter = (uint16_t)canData[3].DataL2Bit01.b01;
    frCounter <<= 8;
    frCounter += (uint16_t)canData[2].byte;
    rlCounter = (uint16_t)canData[4].DataL6Bit05.b05;
    rlCounter <<= 4;
    rlCounter += (uint16_t)canData[3].DataL4Bit03.b47;
    rrCounter = (uint16_t)canData[6].DataL2Bit01.b01;
    rrCounter <<= 8;
    rrCounter += (uint16_t)canData[5].byte;

    RTE_BSW_Set_WheelCounter_FLVD(fl_validity);
    RTE_BSW_Set_WheelCounter_FRVD(fr_validity);
    RTE_BSW_Set_WheelCounter_RLVD(rl_validity);
    RTE_BSW_Set_WheelCounter_RRVD(rr_validity);

    RTE_BSW_Set_WheelCounter_FL(flCounter);
    RTE_BSW_Set_WheelCounter_FR(frCounter);
    RTE_BSW_Set_WheelCounter_RL(rlCounter);
    RTE_BSW_Set_WheelCounter_RR(rrCounter);
}

/** @brief 解析CAN ID 0x6001的AVM车位角点数据，仅在模拟环境下使用 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6001)
{
#ifdef SIMULATE
    static int index = 0;
    static AVM_SlotPointsType slotPoint;

    index = index % 3;
    if (canData[0].byte != index)
    {
        memset(&slotPoint, 0, sizeof(slotPoint));
        index = 0;
        return;
    }

    int16_t value = 0;
    if (index == 0)
    {
        value                 = (int16_t)((canData[1].byte << 8) | canData[2].byte);
        slotPoint.NearRear[0] = (float)value;
        slotPoint.NearRear[0] *= 0.01;

        value                 = (int16_t)((canData[3].byte << 8) | canData[4].byte);
        slotPoint.NearRear[1] = (float)value;
        slotPoint.NearRear[1] *= 0.01;

        value                = (int16_t)((canData[5].byte << 8) | canData[6].byte);
        slotPoint.FarRear[0] = (float)value;
        slotPoint.FarRear[0] *= 0.01;
    }

    if (index == 1)
    {
        value                = (int16_t)((canData[1].byte << 8) | canData[2].byte);
        slotPoint.FarRear[1] = (float)value;
        slotPoint.FarRear[1] *= 0.01;

        value                 = (int16_t)((canData[3].byte << 8) | canData[4].byte);
        slotPoint.FarFront[0] = (float)value;
        slotPoint.FarFront[0] *= 0.01;

        value                 = (int16_t)((canData[5].byte << 8) | canData[6].byte);
        slotPoint.FarFront[1] = (float)value;
        slotPoint.FarFront[1] *= 0.01;
    }

    if (index == 2)
    {
        value                  = (int16_t)((canData[1].byte << 8) | canData[2].byte);
        slotPoint.NearFront[0] = (float)value;
        slotPoint.NearFront[0] *= 0.01;

        value                  = (int16_t)((canData[3].byte << 8) | canData[4].byte);
        slotPoint.NearFront[1] = (float)value;
        slotPoint.NearFront[1] *= 0.01;

        slotPoint.shape   = (PK_SlotShapeType)canData[5].byte;
        slotPoint.hasStop = canData[6].byte;
        slotPoint.hasLock = canData[7].byte;

        RTE_PD_Set_AVM_SlotPoints(&slotPoint);
        // printf("restore slot:type:%d lock:%d stop:%d
        // Point:%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f\n",
        //      slotPoint.shape, slotPoint.hasLock, slotPoint.hasStop,
        //      slotPoint.NearRear[0], slotPoint.NearRear[1], slotPoint.FarRear[0],
        //      slotPoint.FarRear[1], slotPoint.FarFront[0], slotPoint.FarFront[1],
        //      slotPoint.NearFront[0], slotPoint.NearFront[1]);
        memset(&slotPoint, 0, sizeof(slotPoint));
    }

    index++;
#endif
}

/** @brief 解析CAN ID 0x6002的AVM车位角点数据，仅在模拟环境下使用 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6002)
{
#ifdef SIMULATE
    static int index        = 0;
    static uint8_t data[21] = {0};

    index = index % 3;
    if (canData[0].byte != index)
    {
        memset(data, 0, sizeof(data));
        index = 0;
        return;
    }

    if (index == 0)
    {
        memcpy(&data[0], (uint8_t *)&canData[1], 7);
    }
    if (index == 1)
    {
        memcpy(&data[7], (uint8_t *)&canData[1], 7);
    }
    if (index == 2)
    {
        memcpy(&data[14], (uint8_t *)&canData[1], 7);
    }

    if (index == 2)
    {
        AVM_ObjPointsType avmObj;
        uint16_t value[8];
        memset(value, 0, sizeof(value));

        int offset   = 0;
        value[0]     = (data[offset++] << 8);
        value[0]     = value[0] | data[offset++];
        value[1]     = (data[offset++] << 8) & 0xff00;
        value[1]     = value[1] | data[offset++];
        value[2]     = (data[offset++] << 8) & 0xff00;
        value[2]     = value[2] | data[offset++];
        avmObj.index = data[offset++];

        value[3]            = (data[offset++] << 8) & 0xff00;
        value[3]            = value[3] | data[offset++];
        value[4]            = (data[offset++] << 8) & 0xff00;
        value[4]            = value[4] | data[offset++];
        value[5]            = (data[offset++] << 8) & 0xff00;
        value[5]            = value[5] | data[offset++];
        avmObj.obstacleType = data[offset++];

        value[6] = (data[offset++] << 8) & 0xff00;
        value[6] = value[6] | data[offset++];
        value[7] = (data[offset++] << 8) & 0xff00;
        value[7] = value[7] | data[offset++];

        avmObj.mid[0]        = (float)(value[0] / 100.0);
        avmObj.mid[1]        = (float)(value[1] / 100.0);
        avmObj.obstacleTheta = (float)(value[2] / 100.0);
        avmObj.beLen         = (float)(value[3] / 100.0);
        avmObj.pt1[0]        = (float)(value[4] / 100.0);
        avmObj.pt1[1]        = (float)(value[5] / 100.0);
        avmObj.pt2[0]        = (float)(value[6] / 100.0);
        avmObj.pt2[1]        = (float)(value[7] / 100.0);

        extern void PK_Set_Avm_Obstacle(AVM_ObjPointsType & avmObj);
        PK_Set_Avm_Obstacle(avmObj);
    }

    index++;
#endif
}

/** @brief 解析CAN ID 0x6100的时间戳数据，计算超声波发送的时间基准偏移 */
static uint32_t s_recTimeBase = 0;
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6100)
{
    uint16_t sendMs = 0;
    uint8_t buff[8];
    memcpy(buff, canData, candata_bytes);
    BufToUint16(&buff[0], &sendMs);
    uint32_t recvMs = RTE_BSW_Get_CurTime();
    s_recTimeBase   = recvMs - sendMs;
}

/** @brief 解析CAN ID 0x6101的超声波雷达数据，更新超声波传感器信息 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6101)
{
    // printf("BSW : data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",
    //         canData[0].byte, canData[1].byte, canData[2].byte, canData[3].byte,
    //         canData[4].byte, canData[5].byte, canData[6].byte, canData[7].byte);

    Eth_Radar_Data utranic;
    uint8_t ultronicId = canData[0].DataL4Bit03.b47; // 1 ~12
    uint8_t index      = canData[0].DataL4Bit03.b03; // 0,1

    uint32_t ultranicIdx = (index * 12 + ultronicId) - 1;
    if (ultranicIdx > 23)
    {
        return;
    }

    uint16_t simpleTime = canData[1].byte;
    simpleTime <<= 8;
    simpleTime += canData[2].byte;
    utranic.simpleTime = simpleTime + s_recTimeBase;

    uint16_t distance = canData[3].byte;
    distance <<= 8;
    distance += canData[4].byte;
    utranic.fDirect = distance;
    utranic.fDirect = (distance == 0) ? MAX_RANGE_ULTRASONIC : distance * 10;
    // utranic.fDirect = MAX_RANGE_ULTRASONIC;

    // if (ultronicId >= 9) {  utranic.fDirect = 5100;  }
    utranic.peak   = canData[5].byte;
    utranic.width  = canData[6].byte;
    utranic.status = canData[7].byte;

    // RTE_BSW_Set_U_Radar(ultranicIdx, &utranic);

    // 更新组合结构
    U_RadarType l_U_Rada;
    RTE_PD_Get_U_Radar(&l_U_Rada);
    switch (ultranicIdx)
    {
        case 0:
            l_U_Rada.FOL = utranic.fDirect;
            break;
        case 1:
            l_U_Rada.FCL = utranic.fDirect;
            break;
        case 2:
            l_U_Rada.FCR = utranic.fDirect;
            break;
        case 3:
            l_U_Rada.FOR = utranic.fDirect;
            break;
        case 4:
            l_U_Rada.ROL = utranic.fDirect;
            break;
        case 5:
            l_U_Rada.RCL = utranic.fDirect;
            break;
        case 6:
            l_U_Rada.RCR = utranic.fDirect;
            break;
        case 7:
            l_U_Rada.ROR = utranic.fDirect;
            break;
        case 8:
            l_U_Rada.FSL = utranic.fDirect;
            break;
        case 9:
            l_U_Rada.FSR = utranic.fDirect;
            break;
        case 10:
            l_U_Rada.RSL = utranic.fDirect;
            break;
        case 11:
            l_U_Rada.RSR = utranic.fDirect;
            break;
        case 12:
            l_U_Rada.FCL_FOL = utranic.fDirect;
            break;
        case 13:
            l_U_Rada.FCR_FCL = utranic.fDirect;
            break;
        case 14:
            l_U_Rada.FCL_FCR = utranic.fDirect;
            break;
        case 15:
            l_U_Rada.FCR_FOR = utranic.fDirect;
            break;
        case 16:
            l_U_Rada.RCL_ROL = utranic.fDirect;
            break;
        case 17:
            l_U_Rada.RCR_RCL = utranic.fDirect;
            break;
        case 18:
            l_U_Rada.RCL_RCR = utranic.fDirect;
            break;
        case 19:
            l_U_Rada.RCR_ROR = utranic.fDirect;
            break;
        case 20:
            l_U_Rada.FSL_2 = utranic.fDirect;
            break;
        case 21:
            l_U_Rada.FSR_2 = utranic.fDirect;
            break;
        case 22:
            l_U_Rada.RSL_2 = utranic.fDirect;
            break;
        case 23:
            l_U_Rada.RSR_2 = utranic.fDirect;
            break;
    }
    RTE_PD_Set_U_Radar(&l_U_Rada);

    U_RadarDataType l_U_RadarExt[24];
    RTE_PD_Get_U_RadarExt(l_U_RadarExt);
    l_U_RadarExt[ultranicIdx].Timestamp = utranic.simpleTime;
    l_U_RadarExt[ultranicIdx].Distance  = utranic.fDirect;
    l_U_RadarExt[ultranicIdx].PeakLevel = utranic.peak;
    l_U_RadarExt[ultranicIdx].WaveWidth = utranic.width;
    RTE_PD_Set_U_RadarExt(l_U_RadarExt);
}

/**@brief 解析CAN ID 0x6200的用户命令数据，设置用户命令 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6200)
{
    // RTE_BSW_Set_Debug_Command((uint8_t *)canData);
}

DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6300)
{
    // only for record 0x134
}

/** @brief 解析CAN ID 0x6411的LED控制命令，仿真模式下转发命令 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6411)
{
#ifdef SIMULATE
    uint8_t buff[9];
    buff[0] = 0;
    memcpy(buff + 1, canData, candata_bytes);

    extern void Led_ProcessCmd(const uint8_t command[9], int len);
    Led_ProcessCmd(buff, 9);
#endif
}

/** @brief 解析CAN ID 0x6412的LED多帧控制命令，仿真模式下拼接后转发 */
DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6412)
{
#ifdef SIMULATE
    static int index = 1;
    static uint8_t buff[25];

    if (canData[0].byte != index)
    {
        memset(buff, 0, sizeof(buff));
        buff[0] = 1;
        index   = 1;
        return;
    }

    memcpy(&buff[8 * (index - 1) + 1], canData, 8);
    if (index == 3)
    {
        extern void Led_ProcessCmd(const uint8_t *command, int len);
        buff[0] = 1;
        Led_ProcessCmd(buff, 25);
        index = 1;
    }
    else
    {
        index++;
    }
#endif
}

DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX6500)
{
    // only for record 0x13f
}

DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX134)
{
    // only for record 0x13f
}

DEFINE_CAN_RXDATA_PROC(CANID_RX_INDEX13f)
{
    // only for record 0x13f
}

/**
 * @brief CAN消息处理结构体
 *
 * 用于描述一条CAN消息及其处理函数和数据缓存。
 */
typedef struct
{
    uint32_t can_id; /**< CAN ID，用于唯一标识一类CAN消息 */
    void (*can_rxdata_proc)(
        const DataOneByte
            can_rxdata[8]);    /**< 处理该CAN ID数据的函数指针，接收8字节数据 */
    DataOneByte can_rxdata[8]; /**< 存储该CAN消息的8字节数据 */
    uint32_t update;           /**< 更新标志或计数，表示数据是否被刷新过 */
} CanRxDataProc;

/**
 * @brief CAN 接收处理表
 *
 * 存储所有需要处理的 CAN ID 及其对应的处理函数、数据缓存等。
 * 每个条目通过宏 CAN_RXDATA_PROC_NAME(CANID_RX_INDEXxxx) 展开生成。
 */
static CanRxDataProc s_can_rxdata_table[] = {
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX0D5),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX11F),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX121),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX123),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX134),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX135),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX13f),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX173),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX1F0),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX218),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX220),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX222),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX223),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX241),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX242),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX251),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX294),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX2B6),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX342),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX35C),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX38A),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX407),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX422),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX500),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX501),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX502),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX505),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX506),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX507),  CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6001),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6002), CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6100),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6101), CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6200),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6300), CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6411),
    CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6412), CAN_RXDATA_PROC_NAME(CANID_RX_INDEX6500)};

/// 周期性保存间隔（单位：ms），5秒保存一次
const uint32_t can_restore_period = 10 * 1000; // 10ms
/// 接收表中CAN消息项的数量
const uint32_t table_count = sizeof(s_can_rxdata_table) / sizeof(s_can_rxdata_table[0]);
/// 单条CAN消息的缓存长度（= 数据字节数 + CAN ID长度 + 时间戳）
const uint32_t can_item_len = candata_bytes + sizeof(uint16) + sizeof(uint32_t);

/// 持久化缓存（用于保存CAN数据到Flash或文件）
const uint32_t save_cache_len                               = 4096;
static uint8_t saveCacheBuff[save_cache_len * can_item_len] = {0};
static RAM_INF saveRamInf = {saveCacheBuff, can_item_len, sizeof(saveCacheBuff), 0, 0};

/// RAM缓存（用于运行时CAN数据的临时存储）
const uint32_t can_cache_len                              = 2048;
static uint8_t canCacheBuff[can_cache_len * can_item_len] = {0};
static RAM_INF canRamInf = {canCacheBuff, can_item_len, sizeof(canCacheBuff), 0, 0};

/// CAN数据互斥锁（递归锁），用于进入/退出临界区
static pthread_mutex_t candata_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief 初始化CAN数据互斥锁（递归锁）
 *
 */
static void CanInitMutex()
{
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
    int result = pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_RECURSIVE_NP);
    ASSERT_INFO(result == 0, "pthread_mutexattr_settype");
    result = pthread_mutex_init(&candata_mutex, &mutexattr);
    ASSERT_INFO(result == 0, "pthread_mutex_init");
    printf("Init rte mutex success\r\n");
}

/**
 * @brief 进入CAN数据临界区（加锁）
 */
void CanEnterCriticalSection() { pthread_mutex_lock(&candata_mutex); }

/**
 * @brief 离开CAN数据临界区（解锁）
 */
void CanLeaveCriticalSection() { pthread_mutex_unlock(&candata_mutex); }

/**
 * @brief 置位车辆数据错误标志（bit0 = 1）
 */
static void SetVehDataError()
{
    uint32_t oldError = RTE_BSW_Get_ModuleCheck();
    if ((oldError | 0x01) != 0x01)
    {
        RTE_BSW_Set_ModuleCheck((oldError | 0x01));
    }
}

/**
 * @brief 清除车辆数据错误标志（bit0 = 0）
 */
static void ClearVehDataError()
{
    uint32_t oldError = RTE_BSW_Get_ModuleCheck();
    if ((oldError | 0x01) == 0x01)
    {
        RTE_BSW_Set_ModuleCheck((oldError & 0xFFFFFFFE));
    }
}

/**
 * @brief 通过CAN ID查找对应的CanRxDataProc结构体
 *
 * @param canId 接收到的CAN帧ID
 * @return CanRxDataProc* 指向对应处理结构体的指针，找不到返回NULL
 */
static CanRxDataProc *CanIdToIndex(const uint32_t canId)
{
    const uint32_t start_index = s_can_rxdata_table[0].can_id;
    const uint32_t end_index   = s_can_rxdata_table[table_count - 1].can_id;

#ifndef SIMULATE
    if (canId >= 0x6300 && canId < CANID_RX_INDEXMAX)
    {
        return &s_can_rxdata_table[table_count - 1];
    }
#endif

    if (canId < start_index || canId > end_index)
    {
        return NULL;
    }

    uint32_t start = 0;
    uint32_t end   = table_count;
    while (start <= end)
    {
        uint32_t mid = (start + end) / 2;
        if (s_can_rxdata_table[mid].can_id == canId)
        {
            return &s_can_rxdata_table[mid];
        }
        else if (s_can_rxdata_table[mid].can_id < canId)
        {
            start = mid + 1;
        }
        else
        {
            end = mid - 1;
        }
    }

    return NULL;
}

/**
 * @brief CAN 数据保存线程主循环函数
 *
 * 该函数从环形缓冲区读取 CAN 数据，并按条件写入文件保存。
 * - 根据当前状态决定是否创建新文件。
 * - 当缓冲区数据达到一定量时，写入文件。
 * - 定期同步文件数据到存储设备。
 * - 当写入数据量达到阈值或状态变化时关闭文件。
 *
 * @param params 线程参数（未使用）
 * @return NULL
 */
static void *CanSaveLoop(void *params)
{
    const uint32_t can_save_len                             = save_cache_len / 4;
    static uint8_t canSaveBuff[can_save_len * can_item_len] = {0};

    char fileName[128]  = {0};
    int dataFid         = -1;
    uint32_t writeCount = 0;
    uint32_t totalCount = 0;
    while (1)
    {
        int readLen = 0;
        CanEnterCriticalSection();
        if (dataFid > 0)
        {
            if (Ram_Len(&saveRamInf) > save_cache_len / 8)
            {
                readLen = Ram_Get(&saveRamInf, 0, canSaveBuff, can_save_len);
                Ram_Remove(&saveRamInf, readLen);
            }
        }
        else
        {
            if (Ram_Len(&saveRamInf) > save_cache_len * 0.8)
            {
                readLen = Ram_Get(&saveRamInf, 0, canSaveBuff, can_save_len * 0.5);
                Ram_Remove(&saveRamInf, readLen);
            }
        }
        CanLeaveCriticalSection();

        if (dataFid < 0 && (CarSta_Passive != RTE_SM_Get_ApaWorkState() &&
                            CarSta_PowerOn != RTE_SM_Get_ApaWorkState()))
        {
            struct tm absStamp;
            RTE_BSW_Get_AbsStamp(&absStamp);
            if ((absStamp.tm_sec == 0 && absStamp.tm_min == 0) || absStamp.tm_year < 2000)
            {
                usleep(can_restore_period * 20);
                continue;
            }

            snprintf(fileName, sizeof(fileName) - 1,
                     "./log/%d_%02d_%02d_%02d_%02d_%02d_candata.txt", absStamp.tm_year,
                     absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour, absStamp.tm_min,
                     absStamp.tm_sec);

            dataFid = open(fileName, O_CREAT | O_TRUNC | O_RDWR, S_IRWXU | S_IRWXO);
            if (dataFid < 0)
            {
                usleep(10000);
                continue;
            }

            printf("have created %d_%02d_%02d_%02d_%02d_%02d_candata.txt\n",
                   absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
                   absStamp.tm_min, absStamp.tm_sec);
        }
        else if (dataFid < 0)
        {
            usleep(10000);
            continue;
        }

        if (readLen > 0)
        {
            writeCount += readLen;
            totalCount += readLen;
            uint32_t writeLen = 0;
            while (readLen * can_item_len > writeLen)
            {
                int result = write(dataFid, &canSaveBuff[writeLen],
                                   readLen * can_item_len - writeLen);
                if (result > 0)
                {
                    writeLen = writeLen + result;
                }
            }
        }
        else
        {
            if (writeCount > save_cache_len)
            {
                fsync(dataFid);
                writeCount = 0;
            }
            else
            {
                usleep(can_restore_period * 20);
            }
        }

        if (totalCount > save_cache_len * 1000)
        {
            totalCount = 0;
            close(dataFid);
            dataFid = -1;
        }

        if (dataFid >= 0 && (CarSta_Passive == RTE_SM_Get_ApaWorkState() ||
                             CarSta_PowerOn == RTE_SM_Get_ApaWorkState()))
        {
            totalCount = 0;
            close(dataFid);
            dataFid = -1;
        }
    }

    if (dataFid >= 0)
    {
        close(dataFid);
    }
    return NULL;
}

/**
 * @brief 设置需要打印的CAN ID
 * @param canId 目标CAN ID
 */
void Apa_SetPrintCanId(int canId) { printCanId = canId; }

/**
 * @brief CAN 接收线程主循环函数
 *
 * 从环形缓冲区读取 CAN 数据，处理并根据条件保存日志。
 * - 优先读取缓存中的 CAN 数据并逐条处理。
 * - 对指定 CAN ID 数据打印日志。
 * - 根据 CAN ID 找到对应处理函数执行。
 * - 支持数据保存到环形缓冲区。
 * - 检测连续未接收到数据次数，触发错误或清除错误。
 *
 * @param argv 线程参数（未使用）
 * @return NULL
 */
static void *CanRecvLoop(void *argv)
{
    const uint32_t canlen                         = can_cache_len / 4;
    uint8_t canBuff[can_cache_len * can_item_len] = {0};
    uint8_t buff[can_item_len]                    = {0};

    struct timeval tv;
    canRecvReady = 1;
    CanEnterCriticalSection();
    Ram_Clear(&canRamInf);
    CanLeaveCriticalSection();

    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    int counter = 0;
    while (1)
    {
        uint32_t curMs = (uint32_t)RTE_BSW_Get_CurTime();
        counter++;
        while (1)
        {
            CanEnterCriticalSection();
            // 从canRamInf中读取最多canlen条CAN数据，返回实际读取条数readLen
            uint32_t readLen = Ram_Get(&canRamInf, 0, canBuff, canlen);
            Ram_Remove(&canRamInf, readLen);
            CanLeaveCriticalSection();
            if (readLen == 0)
            {
                break;
            }

            // 遍历读取的每条CAN数据
            for (uint32_t index = 0; index < readLen; index++)
            {
                uint16_t canId =
                    *(uint16_t *)(&canBuff[index * can_item_len + sizeof(uint32_t)]);
                uint8_t *data =
                    &canBuff[index * can_item_len + sizeof(uint32_t) + sizeof(uint16_t)];

                if (printCanId == canId)
                { // || frame[i].can_id > 0x6000) {
                    gettimeofday(&tv, 0);
                    printf(
                        "%ld: %ld can_id: 0x%x data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x "
                        "0x%x\r\n",
                        tv.tv_sec, tv.tv_usec, canId, data[0], data[1], data[2], data[3],
                        data[4], data[5], data[6], data[7]);
                }

                counter = 0;
                // 根据CAN ID获取对应的处理函数指针
                CanRxDataProc *caninfo = CanIdToIndex(canId);
                if (caninfo == NULL)
                {
                    continue;
                }
                // 调用对应的CAN数据处理函数
                caninfo->can_rxdata_proc((DataOneByte *)data);

                if (issavecanlog == 0)
                {
                    continue;
                }

                uint8_t offset = 0;
                offset += Uint32ToBuf(curMs, &buff[offset]);
                offset += Uint16ToBuf(canId, &buff[offset]);
                memcpy(&buff[offset], data, candata_bytes);

                uint8_t isOverFlow = 1;
                CanEnterCriticalSection();
                if (Ram_Len(&saveRamInf) < save_cache_len)
                {
                    // 写入缓存,用于录制数据
                    Ram_Put(&saveRamInf, -1, buff, 1);
                    isOverFlow = 0;
                }
                CanLeaveCriticalSection();

                if (isOverFlow > 0)
                {
                    log_info("save can cache ram overflow!\r\n");
                }
            }
        }

        usleep(can_restore_period * 2); // 10ms
        if (counter >= 2)
        {
            log_warn("%d cann't receive can data!\n", counter);
        }

        if (counter >= 20)
        {
            log_err("%d cann't receive can data!\n", counter);
            SetVehDataError();
        }
        else if (counter == 0)
        {
            ClearVehDataError();
        }
    }

    return NULL;
}

/**
 * @brief CAN 数据恢复线程函数
 *
 * 从指定的 CAN 日志文件中读取数据，并按照时间戳间隔恢复发送至系统。
 * 用于仿真或日志回放，恢复过程中还会调用状态恢复接口。
 *
 * @param argv 文件路径（字符串指针）
 * @return NULL
 */
static void *CanRestoreLoop(void *argv)
{
    int dataFid = -1;
    if ((dataFid = open((char *)argv, O_RDONLY)) < 0)
    {
        return 0;
    }

    uint32_t fileLength = lseek(dataFid, 0, SEEK_END);
    lseek(dataFid, 0, SEEK_SET);
    uint8_t *mapAddr =
        (uint8_t *)mmap(NULL, fileLength, PROT_READ, MAP_PRIVATE, dataFid, 0);
    if (mapAddr == (uint8_t *)-1)
    {
        printf("map failedd! err info is %s\r\n", strerror(errno));
        if (dataFid > 0)
        {
            close(dataFid);
            dataFid = -1;
            return 0;
        }
    }

    RAM_INF ramInfo = {mapAddr, can_item_len, fileLength, 0, fileLength / can_item_len};

    uint32_t lastStamp = 0;
    uint32_t stampTemp = 0;
    uint8_t canBuff[can_item_len];
    uint16 canId = 0;

    printf("start to restore, total %d, item len %d\r\n", fileLength,
           fileLength / can_item_len);
    struct timeval stamp;
    for (uint32_t item = 0; item < fileLength / can_item_len; item++)
    {
        if (Ram_Get(&ramInfo, 0, canBuff, 1) == 0)
        {
            break;
        }

        uint32_t count = (fabs(stampTemp - lastStamp) * 1000) / can_restore_period;
        if (count > 0 && count < 10)
        {
            while (count > 0)
            {
                usleep(can_restore_period);
                count--;
            }

            extern void PK_RestoreState();
            PK_RestoreState();
        }
        else if (count > 10)
        {
            printf("record's stamp  gap invalid %d %d %d!", lastStamp, stampTemp, item);
        }

        gettimeofday(&stamp, 0);
        // printf("curstamp %ld:%ld record stamp %ld\n", stamp.tv_sec, stamp.tv_usec,
        // stampTemp);
        lastStamp = stampTemp;
        BufToUint32(&canBuff[0], &stampTemp);
        BufToUint16(&canBuff[4], &canId);
        Can_PushData(canId, &canBuff[6]);
        Ram_Remove(&ramInfo, 1);
    }

    munmap(mapAddr, fileLength);
    close(dataFid);

    exit(0);
    return NULL;
}

/**
 * @brief 初始化CAN接收相关信息
 *
 * 仿真模式下启动回放线程，实车模式下设置时间戳，统一初始化互斥锁并启动接收线程。
 *
 * @param logName 仿真时的CAN日志文件名，为空则使用默认文件名
 */
void Can_InitRecvInfo(const char *logName)
{
#ifdef SIMULATE
    static char canlog[256];
    if (logName == NULL)
    {
        strcpy(canlog, "candata.txt");
    }
    else
    {
        strcpy(canlog, logName);
    }

    ASyncRun(CanRestoreLoop, (void *)canlog, "dbg_recv_can");
    issavecanlog = 0;
#else
    struct tm stdtm;
    memset(&stdtm, 0, sizeof(stdtm));
    RTE_BSW_Set_AbsStamp(&stdtm);
#endif

    CanInitMutex();

    if (issavecanlog > 0)
    {
        ASyncRun(CanSaveLoop, NULL, "dbg_save_can");
    }
    usleep(can_restore_period);
    ASyncRun(CanRecvLoop, (void *)NULL, "apa_can_recv");
}

/**
 * @brief 推送CAN数据到环形缓存
 *
 * 将CAN ID和对应8字节数据封装后，写入环形缓存中，支持多线程保护。
 * 如果缓存满了，会记录溢出日志。
 *
 * 打包格式：
 * │ 时间戳(4B) │ canId(2B)  │ data(8B)           │
 *
 * @param canId  CAN 帧 ID（2字节）
 * @param data   8 字节的 CAN 数据
 */
void Can_PushData(const uint16_t canId, const uint8_t data[8])
{
    if (canRecvReady == 0)
    {
        return;
    }
    uint8_t canbuf[can_item_len];
    uint8_t start = 0;
    start += Uint32ToBuf(0, &canbuf[start]);
    start += Uint16ToBuf(canId, &canbuf[start]);
    memcpy(&canbuf[start], data, candata_bytes);

    int overflow = 0;
    CanEnterCriticalSection();
    if (Ram_Len(&canRamInf) >= can_cache_len)
    {
        overflow = 1;
    }
    Ram_Put(&canRamInf, -1, canbuf, 1);
    CanLeaveCriticalSection();

    if (overflow != 0)
    {
        log_info("Can Ram overflow\n");
    }
}
