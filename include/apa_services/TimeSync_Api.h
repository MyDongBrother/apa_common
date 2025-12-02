/**
 * @file TimeSync_Api.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-29
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-29 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef __TIME_SYNC_H__
#define __TIME_SYNC_H__

#include "Global_Types.h"

/**
 * @brief 设置绝对时间戳
 * @param stdtm 传入时间结构体（年/月/日等）
 */
void RTE_BSW_Set_AbsStamp(struct tm *stdtm);

/**
 * @brief 获取绝对时间戳
 * @param stdtm 输出时间结构体（已调整）
 */
void RTE_BSW_Get_AbsStamp(struct tm *stdtm);

/**
 * @brief 获取绝对时间戳（毫秒级）
 * @return 当前绝对时间戳（毫秒）
 */
uint64_t RTE_BSW_Get_AbsStampMs(void);

#endif
