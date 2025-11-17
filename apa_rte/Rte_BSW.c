/**
 * @file Rte_BSW.c
 * @brief 基础软件(Basic Software, BSW)接口实现
 *
 * 本文件实现了与平台和底层相关的基础软件接口，包括：
 * - 车身 CAN 总线数据访问接口；
 * - 系统服务接口（如系统时间、进程、管道等）；
 * - 通信中间件接口（如 DDS、以太网 等）。
 *
 * 这些接口为上层 SWC 提供统一的数据访问方式，屏蔽底层硬件与平台差异。
 *
 * 同时，所有接口均引入了锁机制以保证线程安全，确保多线程环境下数据一致性。
 *
 * @author jiandong.liu
 * @email liujiandong@bm-intelligent.com
 * @version 0.1
 * @date 2025-08-26
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author          <th>Description
 * <tr><td>2025-08-26 <td>1.0      <td>jiandong.liu    <td>First version
 * </table>
 */

#include "Rte_BSW.h"
#include "Rte_Func.h"
#include "PK_Config.h"
#include "Ram.h"

// ==================== 可适配的必要底盘CAN信号(功能强相关) ======================
// 电源状态
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_EvReadySt, RTE_BSW_Get_EvReadySt)

// 方向盘相关输入
DEFINE_RTE_INTERFACE(uint8, 0, RTE_BSW_Set_EPS_AngleSt, RTE_BSW_Get_EPS_AngleSt)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_EPS_AngleSpd, RTE_BSW_Get_EPS_AngleSpd)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_EPS_Angle, RTE_BSW_Get_EPS_Angle)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_EpsStrWhlTorqVD, RTE_BSW_Get_EpsStrWhlTorqVD)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_EpsStrWhlTorqVal, RTE_BSW_Get_EpsStrWhlTorqVal)

// 轮速计数器 + 有效性标志
DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_WheelCounter_FL, RTE_BSW_Get_WheelCounter_FL)
DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_WheelCounter_RL, RTE_BSW_Get_WheelCounter_RL)
DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_WheelCounter_FR, RTE_BSW_Get_WheelCounter_FR)
DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_WheelCounter_RR, RTE_BSW_Get_WheelCounter_RR)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelCounter_FLVD,
                     RTE_BSW_Get_WheelCounter_FLVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelCounter_RLVD,
                     RTE_BSW_Get_WheelCounter_RLVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelCounter_FRVD,
                     RTE_BSW_Get_WheelCounter_FRVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelCounter_RRVD,
                     RTE_BSW_Get_WheelCounter_RRVD)

// 车轮方向 + 有效性标志
DEFINE_RTE_INTERFACE(RTE_BSW_MoveDirType, 0, RTE_BSW_Set_WheelDrive_FL,
                     RTE_BSW_Get_WheelDrive_FL)
DEFINE_RTE_INTERFACE(RTE_BSW_MoveDirType, 0, RTE_BSW_Set_WheelDrive_RL,
                     RTE_BSW_Get_WheelDrive_RL)
DEFINE_RTE_INTERFACE(RTE_BSW_MoveDirType, 0, RTE_BSW_Set_WheelDrive_FR,
                     RTE_BSW_Get_WheelDrive_FR)
DEFINE_RTE_INTERFACE(RTE_BSW_MoveDirType, 0, RTE_BSW_Set_WheelDrive_RR,
                     RTE_BSW_Get_WheelDrive_RR)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelDrive_FLVD, RTE_BSW_Get_WheelDrive_FLVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelDrive_RLVD, RTE_BSW_Get_WheelDrive_RLVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelDrive_FRVD, RTE_BSW_Get_WheelDrive_FRVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_WheelDrive_RRVD, RTE_BSW_Get_WheelDrive_RRVD)

// 车速相关 + 有效性标志
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_VehSpd, RTE_BSW_Get_ESC_VehSpd)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ESC_VehSpdVD, RTE_BSW_Get_ESC_VehSpdVD)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_WheelSpdFL, RTE_BSW_Get_ESC_WheelSpdFL)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_WheelSpdFR, RTE_BSW_Get_ESC_WheelSpdFR)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_WheelSpdRR, RTE_BSW_Get_ESC_WheelSpdRR)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_WheelSpdRL, RTE_BSW_Get_ESC_WheelSpdRL)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ESC_WheelSpdFLVD,
                     RTE_BSW_Get_ESC_WheelSpdFLVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ESC_WheelSpdFRVD,
                     RTE_BSW_Get_ESC_WheelSpdFRVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ESC_WheelSpdRRVD,
                     RTE_BSW_Get_ESC_WheelSpdRRVD)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ESC_WheelSpdRLVD,
                     RTE_BSW_Get_ESC_WheelSpdRLVD)

// 档位状态
DEFINE_RTE_INTERFACE(RTE_BSW_GearType, GEAR_REAL_NONE, RTE_BSW_Set_CurrentGear,
                     RTE_BSW_Get_CurrentGear)

// 制动状态（true/false）
DEFINE_RTE_INTERFACE(boolean, false, RTE_BSW_Set_BrakeSt, RTE_BSW_Get_BrakeSt)

// 油门踏板深度
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_VcuAccelerographDepthVD,
                     RTE_BSW_Get_VcuAccelerographDepthVD)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_VcuAccelerographDepth,
                     RTE_BSW_Get_VcuAccelerographDepth)

// 四大底盘关联件状态 EPS ESC EPB
DEFINE_RTE_INTERFACE(RTE_BSW_EpsCtrlStatType, 0, RTE_BSW_Set_EpsCtrlStat,
                     RTE_BSW_Get_EpsCtrlStat)
DEFINE_RTE_INTERFACE(RTE_BSW_EscCtrlStatType, 0, RTE_BSW_Set_EscApaStat,
                     RTE_BSW_Get_EscApaStat)
DEFINE_RTE_INTERFACE(RTE_TCU_ApaStatusType, 0, RTE_BSW_Set_TcuApaStat,
                     RTE_BSW_Get_TcuApaStat)
DEFINE_RTE_INTERFACE(RTE_BSW_EpbModeType, 0, RTE_BSW_Set_EpbStat, RTE_BSW_Get_EpbStat)

// 四门两盖安全带
DEFINE_RTE_STRUCT_INTERFACE(RTE_BSW_BCM_StateType, RTE_BSW_Set_BCM_VehicleState,
                            RTE_BSW_Get_BCM_VehicleState)

// 后视镜
DEFINE_RTE_INTERFACE(RTE_BSW_FldigExtrrMrorsType, 0, RTE_BSW_Set_FldigExtrrMrorsExpand,
                     RTE_BSW_Get_FldigExtrrMrorsExpand)

// 挡位干预
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_GearSystemStatus,
                     RTE_BSW_Get_GearSystemStatus)

// ======================= 可适配的非必要底盘CAN信号(功能弱相关) =====================
// 灯光
DEFINE_RTE_INTERFACE(RTE_BSW_LightStateType, 0, RTE_BSW_Set_Dipped_Headlight_St,
                     RTE_BSW_Get_Dipped_Headlight_St)
DEFINE_RTE_INTERFACE(RTE_BSW_LightStateType, 0, RTE_BSW_Set_High_Beam_Light_St,
                     RTE_BSW_Get_High_Beam_Light_St)
DEFINE_RTE_INTERFACE(RTE_BSW_LightStateType, 0, RTE_BSW_Set_LeftTurnSt,
                     RTE_BSW_Get_LeftTurnSt)
DEFINE_RTE_INTERFACE(RTE_BSW_LightStateType, 0, RTE_BSW_Set_RightTurnSt,
                     RTE_BSW_Get_RightTurnSt)

//  胎压
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_TPMS_PressureFL, RTE_BSW_Get_TPMS_PressureFL)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_TPMS_PressureFR, RTE_BSW_Get_TPMS_PressureFR)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_TPMS_PressureRL, RTE_BSW_Get_TPMS_PressureRL)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_TPMS_PressureRR, RTE_BSW_Get_TPMS_PressureRR)

// 横摆角速度
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_YAW_RATE, RTE_BSW_Get_ESC_YAW_RATE)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ESC_YAW_RATE_VD, RTE_BSW_Get_ESC_YAW_RATE_VD)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_YAW_RATE_Offset,
                     RTE_BSW_Get_ESC_YAW_RATE_Offset)

DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_AX, RTE_BSW_Get_ESC_AX)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_AY, RTE_BSW_Get_ESC_AY)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_AXOffSet, RTE_BSW_Get_ESC_AXOffSet)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_AYOffSet, RTE_BSW_Get_ESC_AYOffSet)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_IPU_MotorTorq, RTE_BSW_Get_IPU_MotorTorq)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_IPU_MotorRTorq, RTE_BSW_Get_IPU_MotorRTorq)

// ======================= 根据不同车型的无法适配CAN信号 ========================

DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_Motorspd, RTE_BSW_Get_Motorspd)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_ESC_MinVehSpd, RTE_BSW_Get_ESC_MinVehSpd)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ElecParkLeftCaliperState,
                     RTE_BSW_Get_ElecParkLeftCaliperState)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_ElecParkRightCaliperState,
                     RTE_BSW_Get_ElecParkRightCaliperState)

// ============================  组合导航的必要信号  =================================

DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_IMU_SensorSt, RTE_BSW_Get_IMU_SensorSt)

// IMU 速度（float[3]，通常代表 XYZ 方向速度）
DEFINE_RTE_ARRAY_INTERFACE(float, 3, RTE_BSW_Set_Imuspd, RTE_BSW_Get_Imuspd)

// IMU 当前坐标（float[3]）
DEFINE_RTE_ARRAY_INTERFACE(float, 3, RTE_BSW_Set_ImuCurPos, RTE_BSW_Get_ImuCurPos)

// IMU 姿态角（float[3]，Pitch/Roll/Yaw）
DEFINE_RTE_ARRAY_INTERFACE(float, 3, RTE_BSW_Set_ImuAngle, RTE_BSW_Get_ImuAngle)

// IMU 数据标准差（float[4]）
DEFINE_RTE_ARRAY_INTERFACE(float, 4, RTE_BSW_Set_ImuValStd, RTE_BSW_Get_ImuValStd)

// IMU 三轴加速度（X/Y/Z）
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_Accel_X, RTE_BSW_Get_Accel_X)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_Accel_Y, RTE_BSW_Get_Accel_Y)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_Accel_Z, RTE_BSW_Get_Accel_Z)

// IMU 三轴角速度（X/Y/Z）
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_AngRate_X, RTE_BSW_Get_AngRate_X)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_AngRate_Y, RTE_BSW_Get_AngRate_Y)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_AngRate_Z, RTE_BSW_Get_AngRate_Z)

// IMU 单独姿态角
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_IMU_Pitch, RTE_BSW_Get_IMU_Pitch)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_IMU_Roll, RTE_BSW_Get_IMU_Roll)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_IMU_Head, RTE_BSW_Get_IMU_Head)

// IMU 速度（单值）
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_IMU_VelSpd, RTE_BSW_Get_IMU_VelSpd)

// GPS
DEFINE_RTE_INTERFACE(int32_t, 0, RTE_BSW_Set_GPS_Longitude, RTE_BSW_Get_GPS_Longitude)
DEFINE_RTE_INTERFACE(int32_t, 0, RTE_BSW_Set_GPS_Latitude, RTE_BSW_Get_GPS_Latitude)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_GPS_Altitude, RTE_BSW_Get_GPS_Altitude)
DEFINE_RTE_INTERFACE(float, 0, RTE_BSW_Set_GPS_Bearing, RTE_BSW_Get_GPS_Bearing)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_GPS_Accuracy, RTE_BSW_Get_GPS_Accuracy)

// ======================= 元   控制信号 =======================
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_OutStopCmdVal, RTE_BSW_Get_OutStopCmdVal)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_TargetGearVal, RTE_BSW_Get_TargetGearVal)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_APA_MEB_States_S,
                     RTE_BSW_Get_APA_MEB_States_S)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_TargetSpeedLimitVal,
                     RTE_BSW_Get_TargetSpeedLimitVal)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_FailureBrakeModeVal,
                     RTE_BSW_Get_FailureBrakeModeVal)
DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_TargeAngleVal, RTE_BSW_Get_TargeAngleVal)
DEFINE_RTE_INTERFACE(uint16, 0, RTE_BSW_Set_TargetDistanceVal,
                     RTE_BSW_Get_TargetDistanceVal)

// ======================= 元  控制反馈信号 =======================
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_LSMPosLongACapbl,
                     RTE_BSW_Get_LSMPosLongACapbl)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_LSMSubMTLevelEcho,
                     RTE_BSW_Get_LSMSubMTLevelEcho)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_LSMSubMTLongEcho,
                     RTE_BSW_Get_LSMSubMTLongEcho)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_LSMSubMTReqEcho, RTE_BSW_Get_LSMSubMTReqEcho)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_Lsm12ComfortAvl, RTE_BSW_Get_Lsm12ComfortAvl)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_Lsm12EmergencyAvl,
                     RTE_BSW_Get_Lsm12EmergencyAvl)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_Lsm23Avl, RTE_BSW_Get_Lsm23Avl)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_BrakeOverride, RTE_BSW_Get_BrakeOverride)
DEFINE_RTE_INTERFACE(uint8_t, 0, RTE_BSW_Set_Brake_backup, RTE_BSW_Get_Brake_backup)

// End of file
