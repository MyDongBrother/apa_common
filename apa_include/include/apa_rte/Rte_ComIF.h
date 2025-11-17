/**
 * @file Rte_ComIF.h
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

#ifndef RTE_COMIF_H
#define RTE_COMIF_H

/*******************************************\
Heard file include
\*******************************************/
#include "Rte_Types.h"
#include "PK_Utility.h"

// ==================== HMI相关 ====================
/**
 * @brief 设置来自HMI的用户按钮状态
 *
 * @param pe
 */
void RTE_PK_SM_GetHmiBtInfo(HMI_ButtonInfo *pe);

/**
 * @brief 获取来自HMI的用户按钮状态
 * @param pe
 */
void RTE_PK_SM_SetHmiBtInfo(const HMI_ButtonInfo *pe);

/**
 * @brief 自选车位
 *
 * @param pa
 */
void RTE_PK_DataConv_Set_SlotInfo(const SlotInfo_T *pa);

/**
 * @brief 自选车位
 *
 * @param pa
 */
void RTE_PK_DataConv_Get_SlotInfo(SlotInfo_T *pa);

// ==================== 超声波传感器相关 ====================
void RTE_PD_Get_U_Radar(U_RadarType *pa);
void RTE_PD_Set_U_Radar(const U_RadarType *pa);

void RTE_PD_Get_U_RadarExt(U_RadarDataType radar[24]);
void RTE_PD_Set_U_RadarExt(const U_RadarDataType radar[24]);

// ==================== 视觉传感器相关 ====================

// 摄像头状态有效性
void RTE_PK_SM_SetVisiInfo(const VisionState pa);
VisionState RTE_PK_SM_GetVisiInfo(void);

// 视觉车位坐标
void RTE_PD_Get_AVM_SlotPoints(AVM_SlotPointsType *ps);
void RTE_PD_Set_AVM_SlotPoints(const AVM_SlotPointsType *ps);
void RTE_PD_Clr_AVM_SlotPoints();

// 视觉车位
void RTE_PD_Get_AVM_SlotInfo_B(Avm_Pot_T *pa);
void RTE_PD_Set_AVM_SlotInfo_B(const Avm_Pot_T *pa);

// 视觉障碍物位置
void RTE_PD_Get_AVM_SlotObs_B(Avm_Obj_T *pa);
void RTE_PD_Set_AVM_SlotObs_B(const Avm_Obj_T *pa);

#endif
// End of file
