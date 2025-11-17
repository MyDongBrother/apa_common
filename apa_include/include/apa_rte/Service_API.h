/**
 * @file Service_API.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-22
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-22 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef SERVICE_API_H_
#define SERVICE_API_H_

/*******************************************\
Heard file include
\*******************************************/
#include "Std_Types.h"
#include "Rte_Types.h"
#include "PK_Utility.h"

/**
 * @brief 获取当前算法时间戳
 *
 * @details
 * - 作为统一的时间基准，供各算法模块调用
 * - 时间戳由主线程周期性更新
 * - 内部实现为线程安全的读操作
 *
 * @return uint64_t 当前时间戳（单位：ms），颗粒度非常大一般为20ms
 */
uint64_t RTE_BSW_Get_CurTime(void);

/**
 * @brief 设置当前算法时间戳
 *
 * @details
 * - 由主线程周期性调用，通过系统时间接口写入
 * - 内部实现为线程安全的写操作
 *
 * @param t 新的时间戳（单位：ms），颗粒度非常大一般为20ms
 *
 * @code
 * struct timeval stamp;
 * gettimeofday(&stamp, 0);
 * uint64_t system_time = stamp.tv_sec * 1000 + stamp.tv_usec / 1000;
 * RTE_BSW_Set_CurTime(system_time);
 * @endcode
 */
void RTE_BSW_Set_CurTime(const uint64_t t);

/**
 * @brief 获取用户调试命令
 *
 * @details
 * - 通过管道读取命令,算法中使用该接口做调试使用
 * - 使用后应主动清除
 *
 * @see DebugCmdType
 */
void RTE_BSW_Get_Debug_Command(DebugCmdType *cmd);

/**
 * @brief 设置用户调试命令
 *
 * @see DebugCmdType
 */
void RTE_BSW_Set_Debug_Command(const DebugCmdType *cmd);

/**
 * @brief 获取日志存放路径
 * @param dir[out] 字符数组，长度至少64，用于存放日志路径
 */
void Rte_BSW_Get_LogPath(char dir[64]);

/**
 * @brief 设置日志存放路径
 * @param dir[in] 字符数组，长度不超过64，存放要设置的日志路径
 */
void Rte_BSW_Set_LogPath(const char dir[64]);

/**
 * @brief 获取本地IP地址
 * @param[out] addr 存储IP地址的缓冲区, 至少16字节
 */
void RTE_BSW_Get_LocalIPAddr(char addr[16]);

/**
 * @brief 设置本地IP地址
 * @param[in] addr 要设置的IP地址, 字符串形式, 最多15字符+结束符
 */
void RTE_BSW_Set_LocalIPAddr(const char addr[16]);

/**
 * @brief 设置模块故障状态字
 *
 * @details
 * - 采用 bitmask 表示各模块的故障状态
 * - 每一位/标志位代表一个模块是否存在故障
 * - 内部实现为线程安全写操作
 *
 * 故障位定义如下（按位或可组合）：
 * - bit0 (0x01)：EPS 故障（转向助力）
 * - bit1 (0x02)：ESC/IPB 故障（电子稳定/集成制动系统）
 * - bit2 (0x04)：SCU 故障（档位控制器）
 * - bit3 (0x08)：SAS 故障（转角传感器）
 * - bit4 (0x10)：APS 内部故障（自动泊车 ECU 硬件）
 * - bit5 (0x20)：车速异常（无效或过高）
 * - bit6 (0x40)：制动踏板信号故障
 * - bit7 (0x80)：转角传感器未标定
 *
 * @param value 模块故障状态字（按位或）
 */
void RTE_BSW_Set_ModuleCheck(const uint32_t value);

/**
 * @brief 获取模块故障状态字
 *
 * @details
 * - 采用 bitmask 方式返回当前故障状态
 * - 内部实现为线程安全读操作
 *
 * 故障位定义与 @ref RTE_BSW_Set_ModuleCheck 相同
 *
 * @return uint32_t 当前模块故障状态字
 */
uint32_t RTE_BSW_Get_ModuleCheck(void);

#endif

// End of file
