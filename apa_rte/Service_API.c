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

#include "Rte_Types.h"
#include "Api_Func.h"
#include "Rte_Func.h"

// ==================== HMI相关 ====================
// 算法时间戳
DEFINE_RTE_INTERFACE(uint64_t, 0, RTE_BSW_Set_CurTime, RTE_BSW_Get_CurTime)

// 用户调试指令
DEFINE_API_STRUCT_INTERFACE(DebugCmdType, RTE_BSW_Set_Debug_Command,
                            RTE_BSW_Get_Debug_Command)

// 故障模式
DEFINE_API_INTERFACE(uint32_t, 0, RTE_BSW_Set_ModuleCheck, RTE_BSW_Get_ModuleCheck)

// 当前log路径字符串设置/获取
DEFINE_API_ARRAY_INTERFACE(char, 64, Rte_BSW_Set_LogPath, Rte_BSW_Get_LogPath)

// 本地IP地址设置/获取
DEFINE_API_ARRAY_INTERFACE(char, 16, RTE_BSW_Set_LocalIPAddr, RTE_BSW_Get_LocalIPAddr)
