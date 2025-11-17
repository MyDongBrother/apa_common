/**
 * @file Rte_SWC.c.h
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
#ifndef RTE_SWC_H
#define RTE_SWC_H

/*******************************************\
Heard file include
\*******************************************/
#include "Std_Types.h"
#include "Rte_Types.h"
#include "PK_Utility.h"

// ==================== 状态机模块 相关接口 ====================

/**
 * @brief 设置 APA 状态
 *
 * 由状态机写入，提供给各个模块使用
 *
 * @param pa APA 状态枚举值
 */
void RTE_SM_Set_ApaWorkState(const Apa_WorkStateType e);

/**
 * @brief 获取 APA 状态
 *
 * 各个模块可调用
 *
 * @return Apa_WorkStateType 当前的 APA 状态
 */
Apa_WorkStateType RTE_SM_Get_ApaWorkState(void);

/**
 * @brief 设置 HMI 状态
 *
 * 由状态机写入，供 HMI 显示。
 *
 * @param pa HMI 状态枚举值
 */
void RTE_SM_Set_HMI_State(const HMI_StateType pa);

/**
 * @brief 获取 HMI 状态
 *
 * 由HMI通信接口获取，提供给HMI使用。
 *
 * @return HMI_StateType 当前的 HMI 状态枚举值
 */
HMI_StateType RTE_SM_Get_HMI_State(void);

/**
 * @brief 设置 HMI 工作状态
 *
 * 由状态机写入，供 HMI 显示。
 *
 * @param pa HMI 工作状态枚举值
 */
void RTE_SM_Set_HMI_WorkState(const HMI_WorkStateType pa);

/**
 * @brief 获取 HMI 工作状态
 *
 * 由HMI通信接口获取，提供给HMI使用。
 *
 * @return HMI_StateType 当前的 HMI 状态枚举值
 */
HMI_WorkStateType RTE_SM_Get_HMI_WorkState(void);

/**
 * @brief 设置 HMI 显示信息
 *
 * 由状态机写入，供 HMI 显示。
 *
 * @param pa HMI 显示信息枚举值
 */
void RTE_SM_Set_HMI_WorkTextInfo(const HMI_WorkTextInfo pa);

/**
 * @brief 获取 HMI 显示信息
 *
 * 由HMI通信接口获取，提供给HMI使用。
 *
 * @return HMI_WorkTextInfo HMI 显示信息枚举值
 */
HMI_WorkTextInfo RTE_SM_Get_HMI_WorkTextInfo(void);

void RTE_PK_DataConvt_Get_Target_PlanInfo(PlanInfoType *pa);
void RTE_PK_DataConvt_Set_Target_PlanInfo(const PlanInfoType *pa);

void RTE_PK_DataConvt_Set_TargAVM_SlotInfo(const Avm_Pot_T *pa);
void RTE_PK_DataConvt_Get_TargAVM_SlotInfo(Avm_Pot_T *pa);

// 车位类型标志位
void RTE_PK_DataConvt_Set_IsAVM_Slot(const int pa);
int RTE_PK_DataConvt_Get_IsAVM_Slot(void);

void RTE_PK_SF_Set_AVM_Invalid_Buff_Left(const AVM_Buff_Info *ps);
void RTE_PK_SF_Get_AVM_Invalid_Buff_Left(AVM_Buff_Info *ps);

void RTE_PK_SF_Set_AVM_Invalid_Buff_Right(const AVM_Buff_Info *ps);
void RTE_PK_SF_Get_AVM_Invalid_Buff_Right(AVM_Buff_Info *ps);

void RTE_PK_SF_Set_AVM_Buff_Left(const AVM_Buff_Info *ps);
void RTE_PK_SF_Get_AVM_Buff_Left(AVM_Buff_Info *ps);

void RTE_PK_SF_Set_AVM_Buff_Right(const AVM_Buff_Info *ps);
void RTE_PK_SF_Get_AVM_Buff_Right(AVM_Buff_Info *ps);

/**
 * @brief  HMI模式
 *
 * @param HMI_ApaModeType
 */
void RTE_PK_SM_SetApaMode(const HMI_ApaModeType pa);

/**
 * @brief HMI模式
 *
 * @return HMI_ApaModeType
 */
HMI_ApaModeType RTE_PK_SM_GetApaMode(void);

// 获取模块通信状态：车辆定位
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_Location(void);
/**
 * @brief 设置模块通信状态：车辆定位
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_Location(const PK_ModuleComType e);

// 获取模块通信状态：传感器融合
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_SensorFusion(void);
/**
 * @brief 设置模块通信状态：传感器融合
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_SensorFusion(const PK_ModuleComType e);

// 获取模块通信状态：车位检测
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_SlotDetect(void);
/**
 * @brief 设置模块通信状态：车位检测
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_SlotDetect(const PK_ModuleComType e);

// 获取模块通信状态：路径规划
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_PathPlan(void);
/**
 * @brief 设置模块通信状态：路径规划
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_PathPlan(const PK_ModuleComType e);

// 获取模块通信状态：路径执行
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_PathExecute(void);
/**
 * @brief 设置模块通信状态：路径执行
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_PathExecute(const PK_ModuleComType e);

// 获取模块通信状态：障碍物避让
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_ObjAvoid(void);
/**
 * @brief 设置模块通信状态：障碍物避让
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_ObjAvoid(const PK_ModuleComType e);

// 获取模块通信状态：距离控制
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_DistCtrl(void);
/**
 * @brief 设置模块通信状态：距离控制
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_DistCtrl(const PK_ModuleComType e);

// 获取模块通信状态：速度控制
PK_ModuleComType RTE_PK_StateManage_Get_ModuleCom_SpeedCtrl(void);
/**
 * @brief 设置模块通信状态：速度控制
 * @param e 新的模块通信状态
 */
void RTE_PK_StateManage_Set_ModuleCom_SpeedCtrl(const PK_ModuleComType e);

// 获取模块状态：SensorT2S 模块
PK_ModuleStateType RTE_PK_SensorT2S_Get_ModuleState_SensorT2S(void);
/**
 * @brief 设置模块状态：SensorT2S 模块
 * @param e 新的模块状态
 */
void RTE_PK_SensorT2S_Set_ModuleState_SensorT2S(const PK_ModuleStateType e);

// ==================== 定位模块 相关接口 ====================
// 获取模块状态：车辆定位
PK_ModuleStateType RTE_PK_Location_Get_ModuleState_Location(void);
/**
 * @brief 设置模块状态：车辆定位
 * @param e 新的模块状态
 */
void RTE_PK_Location_Set_ModuleState_Location(PK_ModuleStateType e);

// 获取带时间戳的当前位置
void RTE_PK_Location_Get_CurPos_With_Timestamp(RD_PosStamp *pe);
/**
 * @brief 设置带时间戳的当前位置
 * @param pe 当前位置及时间戳结构体指针
 */
void RTE_PK_Location_Set_CurPos_With_Timestamp(const RD_PosStamp *pe);

// 获取当前位置（四元数/坐标数组）
void RTE_PK_Location_Get_CurPos(float pa[4]);
/**
 * @brief 设置当前位置（四元数/坐标数组）
 * @param pa 坐标数组指针
 */
void RTE_PK_Location_Set_CurPos(const float pa[4]);

// 获取车辆速度
float RTE_PK_Location_Get_Vel_Spd(void);
/**
 * @brief 设置车辆速度
 * @param v 速度值（单位：m/s）
 */
void RTE_PK_Location_Set_Vel_Spd(const float v);

// 获取滤波后的车辆速度
float RTE_PK_Location_Get_Vel_SpdFiltered(void);
/**
 * @brief 设置滤波后的车辆速度
 * @param v 速度值（单位：m/s）
 */
void RTE_PK_Location_Set_Vel_SpdFiltered(const float v);

// 获取曲线行驶速度
float RTE_PK_Location_Get_Vel_TrajCurve(void);
/**
 * @brief 设置曲线行驶速度
 * @param v 速度值（单位：m/s）
 */
void RTE_PK_Location_Set_Vel_TrajCurve(const float v);

// 获取偏航角速度
float RTE_PK_Location_Get_Vel_Yawrate(void);
/**
 * @brief 设置偏航角速度
 * @param v 偏航角速度值（单位：rad/s）
 */
void RTE_PK_Location_Set_Vel_Yawrate(const float v);

// 获取车辆运动状态
Vel_MoveMentStType RTE_PK_Location_Get_Vel_MoveMentSt(void);
/**
 * @brief 设置车辆运动状态
 * @param e 车辆运动状态枚举值
 */
void RTE_PK_Location_Set_Vel_MoveMentSt(const Vel_MoveMentStType e);

// 获取俯仰角速度
float RTE_PK_Location_Get_Vel_Pitch(void);
/**
 * @brief 设置俯仰角速度
 * @param v 俯仰角速度值（单位：rad/s）
 */
void RTE_PK_Location_Set_Vel_Pitch(const float v);

// ==================== 感知融合模块 相关接口 ====================

// ==================== 感知融合模块 相关接口 ====================

/**
 * @brief 获取感知融合模块状态
 * @return PK_ModuleStateType 模块当前状态
 */
PK_ModuleStateType RTE_PK_SensorFusion_Get_ModuleState_SensorFusion(void);

/**
 * @brief 设置感知融合模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_SensorFusion_Set_ModuleState_SensorFusion(const PK_ModuleStateType e);

/**
 * @brief 获取左侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针，用于存储获取到的数据
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(FusionObj_T *pa);

/**
 * @brief 设置左侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针，用于写入数据
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left(const FusionObj_T *pa);

/**
 * @brief 获取右侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(FusionObj_T *pa);

/**
 * @brief 设置右侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right(const FusionObj_T *pa);

/**
 * @brief 获取左后侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left(FusionObj_T *pa);

/**
 * @brief 设置左后侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left(const FusionObj_T *pa);

/**
 * @brief 获取右后侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(FusionObj_T *pa);

/**
 * @brief 设置右后侧融合障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right(const FusionObj_T *pa);

/**
 * @brief 获取左侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(FusionObj_T *pa);

/**
 * @brief 设置左侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left_Raw(const FusionObj_T *pa);

/**
 * @brief 获取右侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(FusionObj_T *pa);

/**
 * @brief 设置右侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right_Raw(const FusionObj_T *pa);

/**
 * @brief 获取左后侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw(FusionObj_T *pa);

/**
 * @brief 设置左后侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left_Raw(const FusionObj_T *pa);

/**
 * @brief 获取右后侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw(FusionObj_T *pa);

/**
 * @brief 设置右后侧原始感知障碍物信息
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right_Raw(const FusionObj_T *pa);

/**
 * @brief 获取 AVM 障碍物信息
 * @param pa 指向 AvmObstacle_T 类型结构体的指针
 */
void RTE_PK_SensorFusion_Get_Avm_Obstacle(AvmObstacle_T *pa);

/**
 * @brief 设置 AVM 障碍物信息
 * @param pa 指向 AvmObstacle_T 类型结构体的指针
 */
void RTE_PK_SensorFusion_Set_Avm_Obstacle(const AvmObstacle_T *pa);

/**
 * @brief 获取左侧融合障碍物信息 B 组
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left(FusionObj_T *pa);

/**
 * @brief 设置左侧融合障碍物信息 B 组
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Left(const FusionObj_T *pa);

/**
 * @brief 获取右侧融合障碍物信息 B 组
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right(FusionObj_T *pa);

/**
 * @brief 设置右侧融合障碍物信息 B 组
 * @param pa 指向 FusionObj_T 类型数组的指针
 */
void RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Right(const FusionObj_T *pa);

// ==================== 车位管理模块 相关接口 ====================

// 获取车位检测模块状态
PK_ModuleStateType RTE_PK_SlotDetect_Get_ModuleState_SlotDetect(void);
/**
 * @brief 设置车位检测模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_SlotDetect_Set_ModuleState_SlotDetect(const PK_ModuleStateType e);

// 获取车位形状类型
PK_SlotShapeType RTE_PK_SlotDetect_Get_SlotShape(void);
/**
 * @brief 设置车位形状类型
 * @param e 车位形状枚举值
 */
void RTE_PK_SlotDetect_Set_SlotShape(const PK_SlotShapeType e);

// 获取目标停车位位置
void RTE_PK_SlotDetect_Get_TargPos(float pa[3]);
/**
 * @brief 设置目标停车位位置
 * @param pa 三维坐标数组 [x, y, yaw]
 */
void RTE_PK_SlotDetect_Set_TargPos(const float pa[3]);

// 获取单个车位信息对象
void RTE_PK_SlotDetect_Get_SlotObj(SlotObj_T *pa);
/**
 * @brief 设置单个车位信息对象
 * @param pa 指向车位对象结构体的指针
 */
void RTE_PK_SlotDetect_Set_SlotObj(const SlotObj_T *pa);

// 获取多车位信息数组
void RTE_PK_SlotDetect_Get_Multi_SlotInfo(Multi_Slot_Array_T *pa);
/**
 * @brief 设置多车位信息数组
 * @param pa 指向多车位数组结构体的指针
 */
void RTE_PK_SlotDetect_Set_Multi_SlotInfo(const Multi_Slot_Array_T *pa);

// ==================== 路径规划模块 相关接口 ====================

/**
 * @brief 获取路径规划模块状态
 * @return PK_ModuleStateType 模块当前状态
 */
PK_ModuleStateType RTE_PK_PathPlan_Get_ModuleState_PathPlan(void);

/**
 * @brief 设置路径规划模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_PathPlan_Set_ModuleState_PathPlan(const PK_ModuleStateType e);

/**
 * @brief 获取多路径规划信息
 * @param pa 指向 MultiPlanInfo 类型数组的指针，用于存储获取到的数据
 */
void RTE_PK_PathPlan_Get_Multi_PlanInfo(MultiPlanInfo *pa);

/**
 * @brief 设置多路径规划信息
 * @param pa 指向 MultiPlanInfo 类型数组的指针，用于写入数据
 */
void RTE_PK_PathPlan_Set_Multi_PlanInfo(const MultiPlanInfo *pa);

/**
 * @brief 设置多路径规划结果
 * @param pa 存储规划结果的整型数组，长度为 MAX_PARKED_SLOTS
 */
void RTE_PK_PathPlan_Set_Multi_PlanRst(const int pa[MAX_PARKED_SLOTS]);

/**
 * @brief 获取多路径规划结果
 * @param pa 存储规划结果的整型数组，长度为 MAX_PARKED_SLOTS
 */
void RTE_PK_PathPlan_Get_Multi_PlanRst(int pa[MAX_PARKED_SLOTS]);

/**
 * @brief 获取停车路径规划信息
 * @param pa 指向 PlanInfoType 类型结构体的指针
 */
void RTE_PK_PathPlan_Get_Park_PlanInfo(PlanInfoType *pa);

/**
 * @brief 设置停车路径规划信息
 * @param pa 指向 PlanInfoType 类型结构体的指针
 */
void RTE_PK_PathPlan_Set_Park_PlanInfo(const PlanInfoType *pa);

// ==================== 路径追踪模块 相关接口 ====================

/**
 * @brief 获取路径追踪模块状态
 * @return PK_ModuleStateType 模块当前状态
 */
PK_ModuleStateType RTE_PK_PathExecute_Get_ModuleState_PathExecute(void);

/**
 * @brief 设置路径追踪模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_PathExecute_Set_ModuleState_PathExecute(const PK_ModuleStateType e);

/**
 * @brief 获取执行失败状态
 * @return uint8_t 执行失败标志，非零表示失败
 */
uint8_t RTE_PK_PathExecute_Get_ModuleState_ExecuteFail(void);

/**
 * @brief 设置执行失败状态
 * @param e 执行失败标志，非零表示失败
 */
void RTE_PK_PathExecute_Set_ModuleState_ExecuteFail(const uint8_t e);

/**
 * @brief 获取制动状态
 * @return uint8 制动状态
 */
uint8 RTE_PK_PathExecute_Get_BrkSt(void);

/**
 * @brief 设置制动状态
 * @param v 制动状态值
 */
void RTE_PK_PathExecute_Set_BrkSt(const uint8 v);

/**
 * @brief 获取制动力
 * @return float 当前制动力
 */
float RTE_PK_PathExecute_Get_rx_brake(void);

/**
 * @brief 设置制动力
 * @param v 制动力值
 */
void RTE_PK_PathExecute_Set_rx_brake(const float v);

/**
 * @brief 获取目标速度
 * @return float 目标速度值
 */
float RTE_PK_PathExecute_Get_TargVelspd(void);

/**
 * @brief 设置目标速度
 * @param v 目标速度值
 */
void RTE_PK_PathExecute_Set_TargVelspd(const float v);

/**
 * @brief 获取目标EPS转向角
 * @return float 目标EPS转向角
 */
float RTE_PK_PathExecute_Get_TargEPS_Angle(void);

/**
 * @brief 设置目标EPS转向角
 * @param v 目标EPS转向角值
 */
void RTE_PK_PathExecute_Set_TargEPS_Angle(const float v);

/**
 * @brief 获取目标EPS角速度限制
 * @return float 目标EPS角速度限制
 */
float RTE_PK_PathExecute_Get_TargEPS_AngleSpdlimit(void);

/**
 * @brief 设置目标EPS角速度限制
 * @param v 目标EPS角速度限制值
 */
void RTE_PK_PathExecute_Set_TargEPS_AngleSpdlimit(const float v);

/**
 * @brief 获取当前路径轨迹信息
 * @param pa 指向 PlanInfoType 结构体的指针，用于存储获取的数据
 */
void RTE_PK_PathExecute_Get_CurTraj(PlanInfoType *pa);

/**
 * @brief 设置当前路径轨迹信息
 * @param pa 指向 PlanInfoType 结构体的指针，用于写入数据
 */
void RTE_PK_PathExecute_Set_CurTraj(const PlanInfoType *pa);

// ==================== 障碍物规避模块 相关接口 ====================

/**
 * @brief 获取障碍物规避模块状态
 * @return PK_ModuleStateType 模块当前状态
 */
PK_ModuleStateType RTE_PK_ObjAvoid_Get_ModuleState_ObjAvoid(void);

/**
 * @brief 设置障碍物规避模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_ObjAvoid_Set_ModuleState_ObjAvoid(const PK_ModuleStateType e);

/**
 * @brief 获取障碍物规避目标EPS转向角
 * @return float EPS转向角
 */
float RTE_PK_ObjAvoid_Get_ObjAvoid_EPS_Angle(void);

/**
 * @brief 设置障碍物规避目标EPS转向角
 * @param v EPS转向角值
 */
void RTE_PK_ObjAvoid_Set_ObjAvoid_EPS_Angle(const float v);

/**
 * @brief 获取障碍物规避目标速度
 * @return float 目标速度值
 */
float RTE_PK_ObjAvoid_Get_ObjAvoid_Velspd(void);

/**
 * @brief 设置障碍物规避目标速度
 * @param v 目标速度值
 */
void RTE_PK_ObjAvoid_Set_ObjAvoid_Velspd(const float v);

/**
 * @brief 设置制动距离
 * @param v 制动距离值
 */
void RTE_PK_ObjAvoid_Set_BrkDist(const float v);

/**
 * @brief 获取制动距离
 * @return float 制动距离值
 */
float RTE_PK_ObjAvoid_Get_BrkDist(void);

/**
 * @brief 获取制动状态
 * @return uint8 制动状态值
 */
uint8 RTE_PK_ObjAvoid_Get_BrkSt(void);

/**
 * @brief 设置制动状态
 * @param v 制动状态值
 */
void RTE_PK_ObjAvoid_Set_BrkSt(const uint8 v);

/**
 * @brief 设置危险状态
 * @param v 危险状态值
 */
void RTE_PK_ObjAvoid_Set_DangerSt(const uint16_t v);

/**
 * @brief 获取危险状态
 * @return uint16_t 危险状态值
 */
uint16_t RTE_PK_ObjAvoid_Get_DangerSt(void);

// ==================== 控制模块 相关接口 ====================

/**
 * @brief 获取距离控制模块激活状态
 * @return uint32_t 激活状态值
 */
uint32_t RTE_PK_Get_ModuleActive_DistCtrl(void);

/**
 * @brief 设置距离控制模块激活状态
 * @param e 激活状态值
 */
void RTE_PK_Set_ModuleActive_DistCtrl(const uint32_t e);

/**
 * @brief 获取速度控制模块状态
 * @return PK_ModuleStateType 模块当前状态
 */
PK_ModuleStateType RTE_PK_SpeedCtrl_Get_ModuleState_SpeedCtrl(void);

/**
 * @brief 设置速度控制模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_SpeedCtrl_Set_ModuleState_SpeedCtrl(const PK_ModuleStateType e);

/**
 * @brief 获取距离控制模块状态
 * @return PK_ModuleStateType 模块当前状态
 */
PK_ModuleStateType RTE_PK_DistCtrl_Get_ModuleState_DistCtrl(void);

/**
 * @brief 设置距离控制模块状态
 * @param e 模块状态枚举值
 */
void RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(const PK_ModuleStateType e);

/**
 * @brief 获取速度控制扭矩
 * @return float 扭矩值
 */
float RTE_PK_SpeedCtrl_Get_SpeedCtrl_Torque(void);

/**
 * @brief 设置速度控制扭矩
 * @param v 扭矩值
 */
void RTE_PK_SpeedCtrl_Set_SpeedCtrl_Torque(const float v);

/**
 * @brief 获取速度控制压力
 * @return float 压力值
 */
float RTE_PK_SpeedCtrl_Get_SpeedCtrl_Pressure(void);

/**
 * @brief 设置速度控制压力
 * @param v 压力值
 */
void RTE_PK_SpeedCtrl_Set_SpeedCtrl_Pressure(const float v);

/**
 * @brief 获取速度控制电子驻车状态
 * @return uint8 EPB控制状态
 */
uint8 RTE_PK_SpeedCtrl_Get_SpeedCtrl_EpbCtl(void);

/**
 * @brief 设置速度控制电子驻车状态
 * @param v EPB控制状态值
 */
void RTE_PK_SpeedCtrl_Set_SpeedCtrl_EpbCtl(const uint8 v);

#endif

// End of file
