/**
 * @file Rte_BSW.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-08-28
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-08-28 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef RTE_BSW_H
#define RTE_BSW_H

/*******************************************\
Heard file include
\*******************************************/
#include "Std_Types.h"
#include "Rte_Types.h"
#include "PK_Utility.h"

/**
 * @brief 设置整车 Ready 状态
 *
 * @details
 * - 由底层 BSW 模块周期性调用
 * - 内部实现为线程安全的写操作
 *
 * @param value 新的整车电源状态
 * @arg 0 Off（关闭）
 * @arg 1 ACC On（附件电源）
 * @arg 2 IGN On（点火）
 */
void RTE_BSW_Set_EvReadySt(const uint8 value);

/**
 * @brief 获取整车 Ready 状态
 *
 * @details
 * - 表示整车电源状态
 * - 由底层 BSW 模块更新
 * - 内部实现为线程安全的读操作
 *
 * @return uint8 当前整车电源状态
 * @retval 0 Off（关闭）
 * @retval 1 ACC On（附件电源）
 * @retval 2 IGN On（点火）
 */
uint8 RTE_BSW_Get_EvReadySt(void);

/**
 * @brief 获取 EPS 转角信号状态
 *
 * @details
 * - 0x0：传感器信息无效
 * - 0x1：传感器信息有效
 * - 内部为线程安全读操作
 *
 * @return uint8_t EPS 转角信号状态
 */
uint8_t RTE_BSW_Get_EPS_AngleSt(void);

/**
 * @brief 设置 EPS 转角信号状态
 *
 * @details
 * - 由底层驱动或诊断模块写入
 * - 内部为线程安全写操作
 *
 * @param value EPS 转角信号状态（0x0=无效，0x1=有效）
 */
void RTE_BSW_Set_EPS_AngleSt(const uint8_t value);

/**
 * @brief 获取 EPS 转角值
 *
 * @details
 * - 单位：°
 * - 有效范围：[-780.0°, +779.9°]
 * - 约定：左转为正，右转为负
 * - 内部为线程安全读操作
 *
 * @return float EPS 当前转角
 */
float RTE_BSW_Get_EPS_Angle(void);

/**
 * @brief 设置 EPS 转角值
 *
 * @details
 * - 单位：°
 * - 有效范围：[-780.0°, +779.9°]
 * - 左转为正，右转为负
 * - 内部为线程安全写操作
 *
 * @param value EPS 当前转角
 */
void RTE_BSW_Set_EPS_Angle(float value);

/**
 * @brief 获取 EPS 转角速度
 *
 * @details
 * - 单位：°/s
 * - 有效范围：[0.0°/s, 1016.0°/s]
 * - 内部为线程安全读操作
 *
 * @return float EPS 转角速度
 */
float RTE_BSW_Get_EPS_AngleSpd(void);

/**
 * @brief 设置 EPS 转角速度
 *
 * @details
 * - 单位：°/s
 * - 有效范围：[0.0°/s, 1016.0°/s]
 * - 内部为线程安全写操作
 *
 * @param value EPS 转角速度
 */
void RTE_BSW_Set_EPS_AngleSpd(const float value);

/**
 * @brief 设置 EPS 方向盘扭矩有效性标志
 *
 * @param value 有效性标志
 *              - 0: 无效
 *              - 1: 有效
 */
void RTE_BSW_Set_EpsStrWhlTorqVD(const uint8_t value);

/**
 * @brief 获取 EPS 方向盘扭矩有效性标志
 *
 * @return 有效性标志
 *         - 0: 无效
 *         - 1: 有效
 */
uint8_t RTE_BSW_Get_EpsStrWhlTorqVD(void);

/**
 * @brief 设置 EPS 方向盘扭矩值
 *
 * @param value 方向盘扭矩，单位：Nm
 *              - 范围：[-12.50, +12.50] Nm
 *              - 分辨率：0.01 Nm
 *              - 左转为正，右转为负
 */
void RTE_BSW_Set_EpsStrWhlTorqVal(const float value);

/**
 * @brief 获取 EPS 方向盘扭矩值
 *
 * @return 方向盘扭矩，单位：Nm
 *         - 范围：[-12.50, +12.50] Nm
 *         - 分辨率：0.01 Nm
 *         - 左转为正，右转为负
 */
float RTE_BSW_Get_EpsStrWhlTorqVal(void);

/**
 * @brief 获取左前轮计数器值
 *
 * @details
 * - 单位：计数值（raw）
 * - 内部为线程安全读操作
 *
 * @return uint16_t 左前轮计数器
 */
uint16_t RTE_BSW_Get_WheelCounter_FL(void);

/**
 * @brief 设置左前轮计数器值
 *
 * @details
 *
 * @param value 左前轮计数器
 */
void RTE_BSW_Set_WheelCounter_FL(const uint16_t value);

/**
 * @brief 获取右前轮计数器值
 *
 * @return uint16_t 右前轮计数器
 */
uint16_t RTE_BSW_Get_WheelCounter_FR(void);

/**
 * @brief 设置右前轮计数器值
 *
 * @param value 右前轮计数器
 */
void RTE_BSW_Set_WheelCounter_FR(const uint16_t value);

/**
 * @brief 获取左后轮计数器值
 *
 * @return uint16_t 左后轮计数器
 */
uint16_t RTE_BSW_Get_WheelCounter_RL(void);

/**
 * @brief 设置左后轮计数器值
 *
 * @param value 左后轮计数器
 */
void RTE_BSW_Set_WheelCounter_RL(const uint16_t value);

/**
 * @brief 获取右后轮计数器值
 *
 * @return uint16_t 右后轮计数器
 */
uint16_t RTE_BSW_Get_WheelCounter_RR(void);

/**
 * @brief 设置右后轮计数器值
 *
 * @param value 右后轮计数器
 */
void RTE_BSW_Set_WheelCounter_RR(const uint16_t value);

/**
 * @brief 获取左前轮计数器有效性
 *
 * @details
 * - 0x0：无效
 * - 0x1：有效
 *
 * @return uint8_t 左前轮计数器有效性
 */
uint8_t RTE_BSW_Get_WheelCounter_FLVD(void);

/**
 * @brief 设置左前轮计数器有效性
 *
 * @param value 左前轮计数器有效性（0=无效，1=有效）
 */
void RTE_BSW_Set_WheelCounter_FLVD(const uint8_t value);

/**
 * @brief 获取右前轮计数器有效性
 *
 * @return uint8_t 右前轮计数器有效性
 */
uint8_t RTE_BSW_Get_WheelCounter_FRVD(void);

/**
 * @brief 设置右前轮计数器有效性
 *
 * @param value 右前轮计数器有效性（0=无效，1=有效）
 */
void RTE_BSW_Set_WheelCounter_FRVD(const uint8_t value);

/**
 * @brief 获取左后轮计数器有效性
 *
 * @return uint8_t 左后轮计数器有效性
 */
uint8_t RTE_BSW_Get_WheelCounter_RLVD(void);

/**
 * @brief 设置左后轮计数器有效性
 *
 * @param value 左后轮计数器有效性（0=无效，1=有效）
 */
void RTE_BSW_Set_WheelCounter_RLVD(const uint8_t value);

/**
 * @brief 获取右后轮计数器有效性
 *
 * @return uint8_t 右后轮计数器有效性
 */
uint8_t RTE_BSW_Get_WheelCounter_RRVD(void);

/**
 * @brief 设置右后轮计数器有效性
 *
 * @param value 右后轮计数器有效性（0=无效，1=有效）
 */
void RTE_BSW_Set_WheelCounter_RRVD(const uint8_t value);

/**
 * @brief 设置前左轮移动方向
 */
void RTE_BSW_Set_WheelDrive_FL(const RTE_BSW_MoveDirType value);
/**
 * @brief 获取前左轮移动方向
 */
RTE_BSW_MoveDirType RTE_BSW_Get_WheelDrive_FL(void);

/**
 * @brief 设置后左轮移动方向
 */
void RTE_BSW_Set_WheelDrive_RL(const RTE_BSW_MoveDirType value);
/**
 * @brief 获取后左轮移动方向
 */
RTE_BSW_MoveDirType RTE_BSW_Get_WheelDrive_RL(void);

/**
 * @brief 设置前右轮移动方向
 */
void RTE_BSW_Set_WheelDrive_FR(const RTE_BSW_MoveDirType value);
/**
 * @brief 获取前右轮移动方向
 */
RTE_BSW_MoveDirType RTE_BSW_Get_WheelDrive_FR(void);

/**
 * @brief 设置后右轮移动方向
 */
void RTE_BSW_Set_WheelDrive_RR(const RTE_BSW_MoveDirType value);
/**
 * @brief 获取后右轮移动方向
 */
RTE_BSW_MoveDirType RTE_BSW_Get_WheelDrive_RR(void);

/**
 * @brief 设置前左轮有效性标志
 */
void RTE_BSW_Set_WheelDrive_FLVD(uint8_t value);
/**
 * @brief 获取前左轮有效性标志
 */
uint8_t RTE_BSW_Get_WheelDrive_FLVD(void);

/**
 * @brief 设置后左轮有效性标志
 */
void RTE_BSW_Set_WheelDrive_RLVD(uint8_t value);
/**
 * @brief 获取后左轮有效性标志
 */
uint8_t RTE_BSW_Get_WheelDrive_RLVD(void);

/**
 * @brief 设置前右轮有效性标志
 */
void RTE_BSW_Set_WheelDrive_FRVD(uint8_t value);
/**
 * @brief 获取前右轮有效性标志
 */
uint8_t RTE_BSW_Get_WheelDrive_FRVD(void);

/**
 * @brief 设置后右轮有效性标志
 */
void RTE_BSW_Set_WheelDrive_RRVD(uint8_t value);
/**
 * @brief 获取后右轮有效性标志
 */
uint8_t RTE_BSW_Get_WheelDrive_RRVD(void);

/**
 * @brief 获取前左轮速度
 * @return 前左轮速度 (km/h), 范围 [0.0, 281.4625]
 */
float RTE_BSW_Get_ESC_WheelSpdFL(void);

/**
 * @brief 设置前左轮速度
 * @param v 前左轮速度 (km/h), 范围 [0.0, 281.4625]
 */
void RTE_BSW_Set_ESC_WheelSpdFL(const float v);

/**
 * @brief 获取前右轮速度
 * @return 前右轮速度 (km/h), 范围 [0.0, 281.4625]
 */
float RTE_BSW_Get_ESC_WheelSpdFR(void);

/**
 * @brief 设置前右轮速度
 * @param v 前右轮速度 (km/h), 范围 [0.0, 281.4625]
 */
void RTE_BSW_Set_ESC_WheelSpdFR(const float v);

/**
 * @brief 获取后左轮速度
 * @return 后左轮速度 (km/h), 范围 [0.0, 281.4625]
 */
float RTE_BSW_Get_ESC_WheelSpdRL(void);

/**
 * @brief 设置后左轮速度
 * @param v 后左轮速度 (km/h), 范围 [0.0, 281.4625]
 */
void RTE_BSW_Set_ESC_WheelSpdRL(const float v);

/**
 * @brief 获取后右轮速度
 * @return 后右轮速度 (km/h), 范围 [0.0, 281.4625]
 */
float RTE_BSW_Get_ESC_WheelSpdRR(void);

/**
 * @brief 设置后右轮速度
 * @param v 后右轮速度 (km/h), 范围 [0.0, 281.4625]
 */
void RTE_BSW_Set_ESC_WheelSpdRR(const float v);

/**
 * @brief 获取前左轮速度有效性
 * @return 0: 无效, 1: 有效
 */
uint8_t RTE_BSW_Get_ESC_WheelSpdFLVD(void);

/**
 * @brief 设置前左轮速度有效性
 * @param v 0: 无效, 1: 有效
 */
void RTE_BSW_Set_ESC_WheelSpdFLVD(const uint8 v);

/**
 * @brief 获取前右轮速度有效性
 * @return 0: 无效, 1: 有效
 */
uint8_t RTE_BSW_Get_ESC_WheelSpdFRVD(void);

/**
 * @brief 设置前右轮速度有效性
 * @param v 0: 无效, 1: 有效
 */
void RTE_BSW_Set_ESC_WheelSpdFRVD(const uint8 v);

/**
 * @brief 获取后左轮速度有效性
 * @return 0: 无效, 1: 有效
 */
uint8_t RTE_BSW_Get_ESC_WheelSpdRLVD(void);

/**
 * @brief 设置后左轮速度有效性
 * @param v 0: 无效, 1: 有效
 */
void RTE_BSW_Set_ESC_WheelSpdRLVD(const uint8 v);

/**
 * @brief 获取后右轮速度有效性
 * @return 0: 无效, 1: 有效
 */
uint8_t RTE_BSW_Get_ESC_WheelSpdRRVD(void);

/**
 * @brief 设置后右轮速度有效性
 * @param v 0: 无效, 1: 有效
 */
void RTE_BSW_Set_ESC_WheelSpdRRVD(const uint8 v);

/**
 * @brief 获取当前档位（实际档位）
 * @return 当前实际档位，类型为 RTE_BSW_GearType
 */
RTE_BSW_GearType RTE_BSW_Get_CurrentGear(void);

/**
 * @brief 设置当前档位（实际档位）
 * @param v 当前实际档位，类型为 RTE_BSW_GearType
 */
void RTE_BSW_Set_CurrentGear(const RTE_BSW_GearType v);
/**
 * @brief 获取车速有效性标志
 * @return 0x0: Valid, 0x1: InvalidInit
 */
uint8 RTE_BSW_Get_ESC_VehSpdVD(void);

/**
 * @brief 设置车速有效性标志
 * @param v 0x0: Valid, 0x1: InvalidInit
 */
void RTE_BSW_Set_ESC_VehSpdVD(const uint8 v);

/**
 * @brief 获取车辆速度
 * @return 车辆速度，单位 km/h，量程 0~281.4625
 */
float RTE_BSW_Get_ESC_VehSpd(void);

/**
 * @brief 设置车辆速度
 * @param v 车辆速度，单位 km/h，量程 0~281.4625
 */
void RTE_BSW_Set_ESC_VehSpd(const float v);

/**
 * @brief 获取刹车状态
 * @return true: 刹车踩下, false: 刹车未踩
 */
boolean RTE_BSW_Get_BrakeSt(void);

/**
 * @brief 设置刹车状态
 * @param v true: 刹车踩下, false: 刹车未踩
 */
void RTE_BSW_Set_BrakeSt(const boolean v);

/**
 * @brief 设置油门踏板开度有效性标志
 *
 * @param value 有效性标志
 *              - 0: 无效
 *              - 1: 有效
 */
void RTE_BSW_Set_VcuAccelerographDepthVD(const uint8_t value);

/**
 * @brief 获取油门踏板开度有效性标志
 *
 * @return 有效性标志
 *         - 0: 无效
 *         - 1: 有效
 */
uint8_t RTE_BSW_Get_VcuAccelerographDepthVD(void);

/**
 * @brief 设置油门踏板开度
 *
 * @param value 油门踏板开度，单位：百分比 [%]
 *              - 范围：[0, 100] %
 *              - 分辨率：1 %
 *              - 初始值/默认值：0 %
 */
void RTE_BSW_Set_VcuAccelerographDepth(const float value);

/**
 * @brief 获取油门踏板开度
 *
 * @return 油门踏板开度，单位：百分比 [%]
 *         - 范围：[0, 100] %
 *         - 分辨率：1 %
 *         - 初始值/默认值：0 %
 */
float RTE_BSW_Get_VcuAccelerographDepth(void);

/**
 * @brief 获取EPS控制状态
 * @return EPS控制状态枚举
 */
RTE_BSW_EpsCtrlStatType RTE_BSW_Get_EpsCtrlStat(void);

/**
 * @brief 设置EPS控制状态
 * @param value EPS控制状态枚举值
 */
void RTE_BSW_Set_EpsCtrlStat(RTE_BSW_EpsCtrlStatType value);

/**
 * @brief 设置 ESC APA 状态
 * @param value ESC APA 控制状态枚举值
 */
void RTE_BSW_Set_EscApaStat(const RTE_BSW_EscCtrlStatType value);

/**
 * @brief 获取 ESC APA 状态
 * @return ESC APA 控制状态枚举值
 */
RTE_BSW_EscCtrlStatType RTE_BSW_Get_EscApaStat();

/**
 * @brief 设置 TCU APA 挡位控制反馈状态
 *
 * @param value TCU APA 状态，参考 RTE_TCU_ApaStatusType 枚举
 */
void RTE_BSW_Set_TcuApaStat(const RTE_TCU_ApaStatusType value);

/**
 * @brief 获取 TCU APA 挡位控制反馈状态
 *
 * @return 当前 TCU APA 状态，参考 RTE_TCU_ApaStatusType 枚举
 */
RTE_TCU_ApaStatusType RTE_BSW_Get_TcuApaStat();

/**
 * @brief 设置 EPB（电子驻车）状态
 * @param value EPB 状态枚举值
 */
void RTE_BSW_Set_EpbStat(const RTE_BSW_EpbModeType value);

/**
 * @brief 获取 EPB（电子驻车）状态
 * @return EPB 状态枚举值
 */
RTE_BSW_EpbModeType RTE_BSW_Get_EpbStat();

/**
 * @brief 设置 BCM 状态数据
 * @param st BCM 状态结构体指针
 */
void RTE_BSW_Set_BCM_VehicleState(const RTE_BSW_BCM_StateType *st);

/**
 * @brief 获取 BCM 状态数据
 * @param st BCM 状态结构体指针，用于存放读取的数据
 */
void RTE_BSW_Get_BCM_VehicleState(RTE_BSW_BCM_StateType *st);

/**
 * @brief 设置外后视镜折叠/展开状态
 * @param value 枚举值，参考 RTE_BSW_FldigExtrrMrorsType
 */
void RTE_BSW_Set_FldigExtrrMrorsExpand(const RTE_BSW_FldigExtrrMrorsType value);

/**
 * @brief 获取外后视镜折叠/展开状态
 * @return 枚举值，参考 RTE_BSW_FldigExtrrMrorsType
 */
RTE_BSW_FldigExtrrMrorsType RTE_BSW_Get_FldigExtrrMrorsExpand();

/**
 * @brief 设置档位系统状态
 *
 * @param value 档位系统状态
 *              - 0x0: Normal（正常）
 *              - 0x1: Abnormal（异常）
 */
void RTE_BSW_Set_GearSystemStatus(const uint8_t value);

/**
 * @brief 获取档位系统状态
 *
 * @return 档位系统状态
 *         - 0x0: Normal（正常）
 *         - 0x1: Abnormal（异常）
 */
uint8_t RTE_BSW_Get_GearSystemStatus(void);

/**
 * @brief 设置近光灯状态
 *
 * @param value 灯状态枚举值
 */
void RTE_BSW_Set_Dipped_Headlight_St(const RTE_BSW_LightStateType value);

/**
 * @brief 获取近光灯状态
 *
 * @return 灯状态枚举值
 */
RTE_BSW_LightStateType RTE_BSW_Get_Dipped_Headlight_St(void);

/**
 * @brief 设置远光灯状态
 *
 * @param value 灯状态枚举值
 */
void RTE_BSW_Set_High_Beam_Light_St(const RTE_BSW_LightStateType value);

/**
 * @brief 获取远光灯状态
 *
 * @return 灯状态枚举值
 */
RTE_BSW_LightStateType RTE_BSW_Get_High_Beam_Light_St(void);

/**
 * @brief 设置左转向灯状态
 *
 * @param value 灯状态枚举值
 */
void RTE_BSW_Set_LeftTurnSt(const RTE_BSW_LightStateType value);

/**
 * @brief 获取左转向灯状态
 *
 * @return 灯状态枚举值
 */
RTE_BSW_LightStateType RTE_BSW_Get_LeftTurnSt(void);

/**
 * @brief 设置右转向灯状态
 *
 * @param value 灯状态枚举值
 */
void RTE_BSW_Set_RightTurnSt(const RTE_BSW_LightStateType value);

/**
 * @brief 获取右转向灯状态
 *
 * @return 灯状态枚举值
 */
RTE_BSW_LightStateType RTE_BSW_Get_RightTurnSt(void);

/**
 * @brief 获取左前轮胎压力
 *
 * @return 压力值（单位：bar）
 */
float RTE_BSW_Get_TPMS_PressureFL(void);

/**
 * @brief 设置左前轮胎压力
 *
 * @param v 压力值（单位：bar）
 */
void RTE_BSW_Set_TPMS_PressureFL(const float v);

/**
 * @brief 获取右前轮胎压力
 *
 * @return 压力值（单位：bar）
 */
float RTE_BSW_Get_TPMS_PressureFR(void);

/**
 * @brief 设置右前轮胎压力
 *
 * @param v 压力值（单位：bar）
 */
void RTE_BSW_Set_TPMS_PressureFR(const float v);

/**
 * @brief 获取左后轮胎压力
 *
 * @return 压力值（单位：bar）
 */
float RTE_BSW_Get_TPMS_PressureRL(void);

/**
 * @brief 设置左后轮胎压力
 *
 * @param v 压力值（单位：bar）
 */
void RTE_BSW_Set_TPMS_PressureRL(const float v);

/**
 * @brief 获取右后轮胎压力
 *
 * @return 压力值（单位：bar）
 */
float RTE_BSW_Get_TPMS_PressureRR(void);

/**
 * @brief 设置右后轮胎压力
 *
 * @param v 压力值（单位：bar）
 */
void RTE_BSW_Set_TPMS_PressureRR(const float v);

// functions for variable ESC_YAW_RATE
float RTE_BSW_Get_ESC_YAW_RATE(void);
void RTE_BSW_Set_ESC_YAW_RATE(const float v);

// functions for variable ESC_YAW_RATEVD
uint8 RTE_BSW_Get_ESC_YAW_RATE_VD(void);
void RTE_BSW_Set_ESC_YAW_RATE_VD(const uint8 v);

float RTE_BSW_Get_ESC_YAW_RATE_Offset(void);
void RTE_BSW_Set_ESC_YAW_RATE_Offset(const float v);

// functions for variable ESC_AX
float RTE_BSW_Get_ESC_AX(void);
void RTE_BSW_Set_ESC_AX(const float v);

// functions for variable ESC_AY
float RTE_BSW_Get_ESC_AY(void);
void RTE_BSW_Set_ESC_AY(const float v);

// functions for variable ESC_Accel x
float RTE_BSW_Get_ESC_AXOffSet(void);
void RTE_BSW_Set_ESC_AXOffSet(const float v);

// functions for variable ESC_Accel y
float RTE_BSW_Get_ESC_AYOffSet(void);
void RTE_BSW_Set_ESC_AYOffSet(const float v);

// functions for variable IPU_MotorTorq
float RTE_BSW_Get_IPU_MotorTorq(void);
void RTE_BSW_Set_IPU_MotorTorq(const float v);

float RTE_BSW_Get_IPU_MotorRTorq(void);
void RTE_BSW_Set_IPU_MotorRTorq(const float v);

// functions for variable Motorspd
float RTE_BSW_Get_Motorspd(void);
void RTE_BSW_Set_Motorspd(const float value);

// functions for variable ESC_VehSpd
float RTE_BSW_Get_ESC_MinVehSpd(void);
void RTE_BSW_Set_ESC_MinVehSpd(const float v);

// ×óÓÒµ¥±ß¿¨Ç¯×´Ì¬
void RTE_BSW_Set_ElecParkLeftCaliperState(const uint8_t value);
uint8_t RTE_BSW_Get_ElecParkLeftCaliperState();

void RTE_BSW_Set_ElecParkRightCaliperState(const uint8_t value);
uint8_t RTE_BSW_Get_ElecParkRightCaliperState();

// functions for variable IMU_SensorSt
uint16 RTE_BSW_Get_IMU_SensorSt(void);
void RTE_BSW_Set_IMU_SensorSt(const uint16 value);

void RTE_BSW_Get_Imuspd(float paras[3]);
void RTE_BSW_Set_Imuspd(const float paras[3]);

void RTE_BSW_Get_ImuCurPos(float params[3]);
void RTE_BSW_Set_ImuCurPos(const float params[3]);

void RTE_BSW_Get_ImuAngle(float params[3]);
void RTE_BSW_Set_ImuAngle(const float params[3]);

void RTE_BSW_Get_ImuValStd(float params[4]);
void RTE_BSW_Set_ImuValStd(const float params[4]);

// functions for variable Accel_X
float RTE_BSW_Get_Accel_X(void);
void RTE_BSW_Set_Accel_X(const float value);

// functions for variable Accel_Y
float RTE_BSW_Get_Accel_Y(void);
void RTE_BSW_Set_Accel_Y(const float value);

// functions for variable Accel_Z
float RTE_BSW_Get_Accel_Z(void);
void RTE_BSW_Set_Accel_Z(const float value);

// functions for variable AngRate_X
float RTE_BSW_Get_AngRate_X(void);
void RTE_BSW_Set_AngRate_X(const float value);

// functions for variable AngRate_Y
float RTE_BSW_Get_AngRate_Y(void);
void RTE_BSW_Set_AngRate_Y(const float value);

// functions for variable AngRate_Z
float RTE_BSW_Get_AngRate_Z(void);
void RTE_BSW_Set_AngRate_Z(const float value);

// functions for variable pitchangle
float RTE_BSW_Get_IMU_Pitch();
void RTE_BSW_Set_IMU_Pitch(const float val);

// functions for variable rollangle
float RTE_BSW_Get_IMU_Roll();
void RTE_BSW_Set_IMU_Roll(const float val);

// functions for variable headangle
float RTE_BSW_Get_IMU_Head();
void RTE_BSW_Set_IMU_Head(const float val);

// functions for variable speed
float RTE_BSW_Get_IMU_VelSpd();
void RTE_BSW_Set_IMU_VelSpd(const float val);

// functions for variable GPS_Latitude
sint32 RTE_BSW_Get_GPS_Latitude(void);
void RTE_BSW_Set_GPS_Latitude(const sint32 v);

// functions for variable GPS_Longitude
sint32 RTE_BSW_Get_GPS_Longitude(void);
void RTE_BSW_Set_GPS_Longitude(const sint32 v);

// functions for variable GPS_Altitude
float RTE_BSW_Get_GPS_Altitude(void);
void RTE_BSW_Set_GPS_Altitude(const float v);

// functions for variable GPS_Bearing
float RTE_BSW_Get_GPS_Bearing(void);
void RTE_BSW_Set_GPS_Bearing(const float v);

// functions for variable GPS_Accuracy
uint8 RTE_BSW_Get_GPS_Accuracy(void);
void RTE_BSW_Set_GPS_Accuracy(const uint8 v);

// Apa_GetOutStopCmdVal()
uint8_t RTE_BSW_Get_OutStopCmdVal(void);
void RTE_BSW_Set_OutStopCmdVal(const uint8_t value);

// Apa_GetTargetGearVal()
uint8_t RTE_BSW_Get_TargetGearVal(void);
void RTE_BSW_Set_TargetGearVal(const uint8_t value);

void RTE_BSW_Set_APA_MEB_States_S(const uint8_t value);
uint8_t RTE_BSW_Get_APA_MEB_States_S();

// Apa_GetTargetSpeedLimitVal()
uint8_t RTE_BSW_Get_TargetSpeedLimitVal(void);
void RTE_BSW_Set_TargetSpeedLimitVal(const uint8_t value);

// Apa_GetFailureBrakeModeVal()
uint8_t RTE_BSW_Get_FailureBrakeModeVal(void);
void RTE_BSW_Set_FailureBrakeModeVal(const uint8_t value);

// Apa_GetTargeAngleVal()
uint16 RTE_BSW_Get_TargeAngleVal(void);
void RTE_BSW_Set_TargeAngleVal(const uint16 value);

// Apa_GetTargetDistanceVal()
uint16 RTE_BSW_Get_TargetDistanceVal(void);
void RTE_BSW_Set_TargetDistanceVal(const uint16 value);

// functions for variable EvReadySt
uint8 RTE_BSW_Get_EvReadySt(void);
void RTE_BSW_Set_EvRdadySt(const uint8 v);

void RTE_BSW_Set_LSMPosLongACapbl(const uint8_t value);
uint8_t RTE_BSW_Get_LSMPosLongACapbl();

void RTE_BSW_Set_LSMSubMTLevelEcho(const uint8_t value);
uint8_t RTE_BSW_Get_LSMSubMTLevelEcho();

void RTE_BSW_Set_LSMSubMTLongEcho(const uint8_t value);
uint8_t RTE_BSW_Get_LSMSubMTLongEcho();

void RTE_BSW_Set_LSMSubMTReqEcho(const uint8_t value);
uint8_t RTE_BSW_Get_LSMSubMTReqEcho();

void RTE_BSW_Set_Lsm12ComfortAvl(const uint8_t value);
uint8_t RTE_BSW_Get_Lsm12ComfortAvl();

void RTE_BSW_Set_Lsm12EmergencyAvl(const uint8_t value);
uint8_t RTE_BSW_Get_Lsm12EmergencyAvl();

void RTE_BSW_Set_Lsm23Avl(const uint8_t value);
uint8_t RTE_BSW_Get_Lsm23Avl();

void RTE_BSW_Set_BrakeOverride(const uint8_t value);
uint8_t RTE_BSW_Get_BrakeOverride();

void RTE_BSW_Set_Brake_backup(const uint8_t value);
uint8_t RTE_BSW_Get_Brake_backup();

#endif

// End of file
