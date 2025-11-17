/**
 * @file DataBus_Api.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-11-05 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef __DATA_BUS_API_H__
#define __DATA_BUS_API_H__

#include "Std_Types.h"

/**
 * @brief DataBus 消息 ID
 * 每种消息一个独立 ID，避免冲突
 */
typedef uint32_t DataBusID_T;

/**
 * @brief DataBus 消息回调函数类型
 *
 * @param data 指向消息内容的 byte buffer
 * @param len  消息长度（固定长度协议）
 */
typedef void (*RxDataProcFunc)(const uint8_t *data, uint16_t len);

/**
 * @brief 注册一个 DataBus 订阅回调
 *
 * @param id   消息 ID
 * @param len  消息固定长度
 * @param proc 回调函数
 *
 * @return true  注册成功
 * @return false 参数错误或重复注册
 */
bool DataBus_Register(DataBusID_T id, uint16_t len, RxDataProcFunc proc);

/**
 * @brief 发布消息到 DataBus
 *
 * @param id   消息 ID
 * @param data 指向消息内容
 * @param len  消息长度（必须与注册值一致）
 *
 * @return true  推送成功
 * @return false 缓冲满/未注册/长度错误
 */
bool DataBus_Push(DataBusID_T id, const uint8_t *data, uint16_t len);

void DataBusInit(void);

#endif /* __DATA_BUS_API_H__ */
