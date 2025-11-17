/**
 * @file Rte_SWC.c
 * @brief 模块间接口实现
 *
 * 本文件实现了 SWC 输出数据的对接接口，主要包括：
 * - 提供给 BSW 层使用的数据接口；
 * - 提供给线程间通信的数据接口；
 * - 提供给其他 SWC 调用的 RTE 接口。
 *
 * 这些接口为不同层级或模块之间提供了统一的数据交互方式，确保 SWC
 * 的输出数据能够被安全、有效地访问。
 *
 * 同时，所有接口均引入了锁机制以保证线程安全，确保在多线程环境下的数据一致性。
 *
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-08-27
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-08-27 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "Rte_Types.h"
#include "Api_Func.h"
#include "PK_Utility.h"

// ==================== 状态机模块 相关接口 ====================

// 车辆工作状态（自动泊车流程）设置/获取
DEFINE_API_INTERFACE(Apa_WorkStateType, CarSta_Passive, RTE_SM_Set_ApaWorkState,
                     RTE_SM_Get_ApaWorkState)

// HMI反馈信息 - ApaMode
DEFINE_API_INTERFACE(HMI_StateType, HmiState_Invalid, RTE_SM_Set_HMI_State,
                     RTE_SM_Get_HMI_State)

// HMI反馈信息 - WorkingSta
DEFINE_API_INTERFACE(HMI_WorkStateType, HmiWorkState_Passive, RTE_SM_Set_HMI_WorkState,
                     RTE_SM_Get_HMI_WorkState)

// HMI反馈信息 - WorkingText
DEFINE_API_INTERFACE(HMI_WorkTextInfo, ApaWorkingText_NoInfo, RTE_SM_Set_HMI_WorkTextInfo,
                     RTE_SM_Get_HMI_WorkTextInfo)
// APA模式
DEFINE_API_INTERFACE(HMI_ApaModeType, 0, RTE_PK_SM_SetApaMode, RTE_PK_SM_GetApaMode)

// 目标路径规划信息设置/获取
DEFINE_API_STRUCT_INTERFACE(PlanInfoType, RTE_PK_DataConvt_Set_Target_PlanInfo,
                            RTE_PK_DataConvt_Get_Target_PlanInfo)

// 视觉车位目标信息，限位杆
DEFINE_API_STRUCT_INTERFACE(Avm_Pot_T, RTE_PK_DataConvt_Set_TargAVM_SlotInfo,
                            RTE_PK_DataConvt_Get_TargAVM_SlotInfo)

// 车位类型信息
DEFINE_API_INTERFACE(int, 0, RTE_PK_DataConvt_Set_IsAVM_Slot,
                     RTE_PK_DataConvt_Get_IsAVM_Slot)

// Location模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_Location,
                     RTE_PK_StateManage_Get_ModuleCom_Location)

// SensorFusion模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_SensorFusion,
                     RTE_PK_StateManage_Get_ModuleCom_SensorFusion)

// SlotDetect模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_SlotDetect,
                     RTE_PK_StateManage_Get_ModuleCom_SlotDetect)

// PathPlan模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_PathPlan,
                     RTE_PK_StateManage_Get_ModuleCom_PathPlan)

// PathExecute模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_PathExecute,
                     RTE_PK_StateManage_Get_ModuleCom_PathExecute)

// ObjAvoid模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_ObjAvoid,
                     RTE_PK_StateManage_Get_ModuleCom_ObjAvoid)

// DistCtrl模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_DistCtrl,
                     RTE_PK_StateManage_Get_ModuleCom_DistCtrl)

// SpeedCtrl模块通信状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleComType, MCOM_INIT,
                     RTE_PK_StateManage_Set_ModuleCom_SpeedCtrl,
                     RTE_PK_StateManage_Get_ModuleCom_SpeedCtrl)

// SensorT2S模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_OFF,
                     RTE_PK_SensorT2S_Set_ModuleState_SensorT2S,
                     RTE_PK_SensorT2S_Get_ModuleState_SensorT2S)
// ==================== 定位模块 相关接口 ====================
// Location模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_NORM,
                     RTE_PK_Location_Set_ModuleState_Location,
                     RTE_PK_Location_Get_ModuleState_Location)

// 当前坐标设置/获取
DEFINE_API_ARRAY_INTERFACE(float, 4, RTE_PK_Location_Set_CurPos,
                           RTE_PK_Location_Get_CurPos)

// 当前位置信息带时间戳设置/获取
DEFINE_API_STRUCT_INTERFACE(RD_PosStamp, RTE_PK_Location_Set_CurPos_With_Timestamp,
                            RTE_PK_Location_Get_CurPos_With_Timestamp)

// 车辆当前速度设置/获取
DEFINE_API_INTERFACE(float, 0, RTE_PK_Location_Set_Vel_Spd, RTE_PK_Location_Get_Vel_Spd)

// Location滤波后速度设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_Location_Set_Vel_SpdFiltered,
                     RTE_PK_Location_Get_Vel_SpdFiltered)

// 当前轨迹曲线速度设置/获取
DEFINE_API_INTERFACE(float, 0, RTE_PK_Location_Set_Vel_TrajCurve,
                     RTE_PK_Location_Get_Vel_TrajCurve)

// 当前偏航角速度设置/获取
DEFINE_API_INTERFACE(float, 0, RTE_PK_Location_Set_Vel_Yawrate,
                     RTE_PK_Location_Get_Vel_Yawrate)

// 移动状态设置/获取
DEFINE_API_INTERFACE(Vel_MoveMentStType, 0, RTE_PK_Location_Set_Vel_MoveMentSt,
                     RTE_PK_Location_Get_Vel_MoveMentSt)

// Location俯仰角速度设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_Location_Set_Vel_Pitch,
                     RTE_PK_Location_Get_Vel_Pitch)

// ==================== 感知融合 相关接口 ====================

// SensorFusion模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_NORM,
                     RTE_PK_SensorFusion_Set_ModuleState_SensorFusion,
                     RTE_PK_SensorFusion_Get_ModuleState_SensorFusion)

// 感知左前方融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T, RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left)

// 感知右前方融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T, RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right)

// 感知左后方融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T,
                            RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left)

// 感知右后方融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T,
                            RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right)

// 左前方原始融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T,
                            RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left_Raw,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw)

// 右前方原始融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T,
                            RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right_Raw,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw)

// 左后方原始融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T,
                            RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left_Raw,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw)

// 右后方原始融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T,
                            RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right_Raw,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw)

// AVM感知障碍物信息设置/获取
DEFINE_API_STRUCT_INTERFACE(AvmObstacle_T, RTE_PK_SensorFusion_Set_Avm_Obstacle,
                            RTE_PK_SensorFusion_Get_Avm_Obstacle)

// 感知左后方B类融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T, RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Left,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Left)

// 感知右后方B类融合目标对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(FusionObj_T, RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Right,
                            RTE_PK_SensorFusion_Get_Fusion_ObsInfo_B_Right)

// AVM左侧无效缓存信息设置/获取
DEFINE_API_STRUCT_INTERFACE(AVM_Buff_Info, RTE_PK_SF_Set_AVM_Invalid_Buff_Left,
                            RTE_PK_SF_Get_AVM_Invalid_Buff_Left)

// AVM右侧无效缓存信息设置/获取
DEFINE_API_STRUCT_INTERFACE(AVM_Buff_Info, RTE_PK_SF_Set_AVM_Invalid_Buff_Right,
                            RTE_PK_SF_Get_AVM_Invalid_Buff_Right)

// AVM左侧有效缓存信息设置/获取
DEFINE_API_STRUCT_INTERFACE(AVM_Buff_Info, RTE_PK_SF_Set_AVM_Buff_Left,
                            RTE_PK_SF_Get_AVM_Buff_Left)

// AVM右侧有效缓存信息设置/获取
DEFINE_API_STRUCT_INTERFACE(AVM_Buff_Info, RTE_PK_SF_Set_AVM_Buff_Right,
                            RTE_PK_SF_Get_AVM_Buff_Right)

// ================ 车位管理 相关接口 SlotDetect ====================

// SlotDetect模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_OFF,
                     RTE_PK_SlotDetect_Set_ModuleState_SlotDetect,
                     RTE_PK_SlotDetect_Get_ModuleState_SlotDetect)

// 车位槽检测形状设置/获取
DEFINE_API_INTERFACE(PK_SlotShapeType, PK_SLOT_NO, RTE_PK_SlotDetect_Set_SlotShape,
                     RTE_PK_SlotDetect_Get_SlotShape)

// 目标车位位置设置/获取
DEFINE_API_ARRAY_INTERFACE(float, 3, RTE_PK_SlotDetect_Set_TargPos,
                           RTE_PK_SlotDetect_Get_TargPos)

// 车位对象信息设置/获取
DEFINE_API_STRUCT_INTERFACE(SlotObj_T, RTE_PK_SlotDetect_Set_SlotObj,
                            RTE_PK_SlotDetect_Get_SlotObj)

// 多车位数组信息设置/获取
DEFINE_API_STRUCT_INTERFACE(Multi_Slot_Array_T, RTE_PK_SlotDetect_Set_Multi_SlotInfo,
                            RTE_PK_SlotDetect_Get_Multi_SlotInfo)

// ==================== 路径规划 相关接口 ====================

// PathPlan模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_OFF,
                     RTE_PK_PathPlan_Set_ModuleState_PathPlan,
                     RTE_PK_PathPlan_Get_ModuleState_PathPlan)
// 多路径规划信息设置/获取
DEFINE_API_STRUCT_INTERFACE(MultiPlanInfo, RTE_PK_PathPlan_Set_Multi_PlanInfo,
                            RTE_PK_PathPlan_Get_Multi_PlanInfo)
// 多路径规划结果数组设置/获取
DEFINE_API_ARRAY_INTERFACE(int, MAX_PARKED_SLOTS, RTE_PK_PathPlan_Set_Multi_PlanRst,
                           RTE_PK_PathPlan_Get_Multi_PlanRst)

// 停车规划信息设置/获取
DEFINE_API_STRUCT_INTERFACE(PlanInfoType, RTE_PK_PathPlan_Set_Park_PlanInfo,
                            RTE_PK_PathPlan_Get_Park_PlanInfo)

// ==================== 路径追踪 相关接口 ====================

// PathExecute模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_NORM,
                     RTE_PK_PathExecute_Set_ModuleState_PathExecute,
                     RTE_PK_PathExecute_Get_ModuleState_PathExecute)
// PathExecute执行失败状态设置/获取
DEFINE_API_INTERFACE(uint8_t, 0, RTE_PK_PathExecute_Set_ModuleState_ExecuteFail,
                     RTE_PK_PathExecute_Get_ModuleState_ExecuteFail)

// PathExecute刹车状态设置/获取
DEFINE_API_INTERFACE(uint8_t, 0, RTE_PK_PathExecute_Set_BrkSt,
                     RTE_PK_PathExecute_Get_BrkSt)

// PathExecute右后刹车力设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_PathExecute_Set_rx_brake,
                     RTE_PK_PathExecute_Get_rx_brake)

// PathExecute目标速度设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_PathExecute_Set_TargVelspd,
                     RTE_PK_PathExecute_Get_TargVelspd)

// PathExecute目标EPS角度设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_PathExecute_Set_TargEPS_Angle,
                     RTE_PK_PathExecute_Get_TargEPS_Angle)

// PathExecute目标EPS角速度限制设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_PathExecute_Set_TargEPS_AngleSpdlimit,
                     RTE_PK_PathExecute_Get_TargEPS_AngleSpdlimit)

// 当前轨迹设置/获取
DEFINE_API_STRUCT_INTERFACE(PlanInfoType, RTE_PK_PathExecute_Set_CurTraj,
                            RTE_PK_PathExecute_Get_CurTraj)

// ==================== 避障 相关接口 ====================

// ObjAvoid模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_OFF,
                     RTE_PK_ObjAvoid_Set_ModuleState_ObjAvoid,
                     RTE_PK_ObjAvoid_Get_ModuleState_ObjAvoid)

// ObjAvoid EPS角度设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_ObjAvoid_Set_ObjAvoid_EPS_Angle,
                     RTE_PK_ObjAvoid_Get_ObjAvoid_EPS_Angle)

// ObjAvoid目标速度设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_ObjAvoid_Set_ObjAvoid_Velspd,
                     RTE_PK_ObjAvoid_Get_ObjAvoid_Velspd)

// ObjAvoid刹车距离设置/获取
DEFINE_API_INTERFACE(float, 0.0f, RTE_PK_ObjAvoid_Set_BrkDist,
                     RTE_PK_ObjAvoid_Get_BrkDist)

// ObjAvoid刹车状态设置/获取
DEFINE_API_INTERFACE(uint8_t, 0, RTE_PK_ObjAvoid_Set_BrkSt, RTE_PK_ObjAvoid_Get_BrkSt)

// ObjAvoid危险状态设置/获取
DEFINE_API_INTERFACE(uint16_t, 0, RTE_PK_ObjAvoid_Set_DangerSt,
                     RTE_PK_ObjAvoid_Get_DangerSt)

// ============ 横纵向控制 相关接口 DistCtrl SpeedCtrl ===============

// DistCtrl模块激活状态设置/获取
DEFINE_API_INTERFACE(uint32_t, 0, RTE_PK_Set_ModuleActive_DistCtrl,
                     RTE_PK_Get_ModuleActive_DistCtrl)

// SpeedCtrl模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_NORM,
                     RTE_PK_SpeedCtrl_Set_ModuleState_SpeedCtrl,
                     RTE_PK_SpeedCtrl_Get_ModuleState_SpeedCtrl)

// DistCtrl模块状态设置/获取
DEFINE_API_INTERFACE(PK_ModuleStateType, MSTAT_OFF,
                     RTE_PK_DistCtrl_Set_ModuleState_DistCtrl,
                     RTE_PK_DistCtrl_Get_ModuleState_DistCtrl)

// 速度控制扭矩设置/获取
DEFINE_API_INTERFACE(float, 0, RTE_PK_SpeedCtrl_Set_SpeedCtrl_Torque,
                     RTE_PK_SpeedCtrl_Get_SpeedCtrl_Torque)

// 速度控制压力设置/获取
DEFINE_API_INTERFACE(float, 0, RTE_PK_SpeedCtrl_Set_SpeedCtrl_Pressure,
                     RTE_PK_SpeedCtrl_Get_SpeedCtrl_Pressure)

// 电子驻车控制设置/获取
DEFINE_API_INTERFACE(uint8, 0, RTE_PK_SpeedCtrl_Set_SpeedCtrl_EpbCtl,
                     RTE_PK_SpeedCtrl_Get_SpeedCtrl_EpbCtl)

// ==================================== end  =======================================
