/**
 * @file McuMsgData.h
 * @brief 定义用于网络通信的CAN数据结构及命令ID
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
 * <tr><td>2025-07-02 <td>1.0      <td>jiandong.liu <td>first version
 * </table>
 */

#ifndef _MCUMSG_DATA_H
#define _MCUMSG_DATA_H

#define MAX_CAN_DATA_SIZE 1024 ///< 网络通信中单个CAN数据包的最大字节数

/// 网络发送数据包中各头部长度定义
const uint16_t CmdDataHeadLen   = 4; ///< netSenddata结构体头部长度（字节）
const uint16_t ReadydataHeadLen = 2; ///< netReadydata结构体头部长度（字节）

/// 命令ID枚举，表示不同数据来源或类型
enum
{
    Cmd_A000 = 0, ///< 车身模块命令ID
    Cmd_A001,     ///< 毫米波模块命令ID
    Cmd_A002,     ///< 超声波模块命令ID
    Cmd_A003,     ///< 备用命令ID
    Cmd_Max       ///< 命令ID最大值（数量）
};

/// 网络接收准备数据结构
typedef struct netReadydata
{
    uint16_t canLen; ///< CAN数据长度，不包含canLen字段本身
    uint16_t canId;  ///< CAN消息ID，用于标识CAN帧
    uint8_t data[8]; ///< CAN数据字段，标准8字节
} netReadydata;

/// 网络发送数据结构
typedef struct netSenddata
{
    uint16_t cmdid;                      ///< 命令ID，标识本次发送的数据类型
    uint16_t datalen;                    ///< 本次发送数据的总长度（字节）
    uint8_t data[MAX_CAN_DATA_SIZE - 4]; ///< 实际发送的数据内容，减去cmdid和datalen的长度
} netSenddata;

#endif
