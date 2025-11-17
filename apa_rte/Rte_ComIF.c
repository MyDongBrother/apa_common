/**
 * @file Rte_ComIF.c
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

#include "Rte_ComIF.h"
#include "PK_Config.h"
#include "Rte_Func.h"
#include "Ram.h"
#include "PK_Utility.h"

// ==================== HMI相关 ====================
// HMI信息
DEFINE_RTE_STRUCT_INTERFACE(HMI_ButtonInfo, RTE_PK_SM_SetHmiBtInfo,
                            RTE_PK_SM_GetHmiBtInfo)

// 自选车位信息设置/获取
DEFINE_RTE_STRUCT_INTERFACE(SlotInfo_T, RTE_PK_DataConv_Set_SlotInfo,
                            RTE_PK_DataConv_Get_SlotInfo)

// ==================== 超声波传感器相关 ====================
// Radar 单帧数据接口
DEFINE_RTE_STRUCT_INTERFACE(U_RadarType, RTE_PD_Set_U_Radar, RTE_PD_Get_U_Radar)

// Radar 扩展数据接口（数组，24 个元素）
DEFINE_RTE_ARRAY_INTERFACE(U_RadarDataType, 24, RTE_PD_Set_U_RadarExt,
                           RTE_PD_Get_U_RadarExt)

// ==================== 视觉传感器相关 ====================
// 视觉状态
DEFINE_RTE_INTERFACE(VisionState, VisionState_Alive, RTE_PK_SM_SetVisiInfo,
                     RTE_PK_SM_GetVisiInfo)

// AVM Slot 点数据接口（RAM buffer，24 个元素，带清除接口）
DEFINE_RTE_RAM_INF_INTERFACE(AVM_SlotPointsType, 24, RTE_PD_Set_AVM_SlotPoints,
                             RTE_PD_Get_AVM_SlotPoints, RTE_PD_Clr_AVM_SlotPoints)

// AVM 观测物体结构体接口
DEFINE_RTE_STRUCT_INTERFACE(Avm_Obj_T, RTE_PD_Set_AVM_SlotObs_B, RTE_PD_Get_AVM_SlotObs_B)

// AVM Slot 信息结构体接口
DEFINE_RTE_STRUCT_INTERFACE(Avm_Pot_T, RTE_PD_Set_AVM_SlotInfo_B,
                            RTE_PD_Get_AVM_SlotInfo_B)
