/**
 * @file SignalConversion.h
 * @brief 定义信号转换相关的枚举和接口。
 *
 * 本文件用于集中管理从底层信号到上层业务语义的转换，
 * 包括车辆门盖状态、刹车状态、档位状态等。
 *
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-15
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-15 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef SIGNAL_CONVERSION_H_
#define SIGNAL_CONVERSION_H_

#include <cstdint>

#define STATE_MACHINE_PERIOD_MS    20 // 状态机调用周期，由线程调度确定
#define APA_SPEE_LIMIT_MAX_WARNING 25
#define APA_SPEE_LIMIT_MAX_STOP    30

/**
 * @brief 四门两盖状态
 */
typedef enum
{
    AllClosed  = 0, ///< 四门两盖全部关闭
    DoorOpen   = 1, ///< 任意门开启
    BonnetOpen = 2, ///< 前舱盖开启
    TrunkOpen  = 3  ///< 后备箱开启
} CarDoorStatus;

/**
 * @brief 安全带状态
 */
typedef enum
{
    CarSafeBeltFastened         = 0, /**< 全部已系 */
    CarSafeBeltDriverUnfastened = 1, /**< 驾驶员未系 */
    CarSafeBeltAllUnfastened    = 2  /**< 全部未系 */
} CarSafeBeltStatus;

/**
 * @brief 启动泊车失败原因
 */
typedef enum
{
    StartPark_OK = 0,         ///< 成功
    StartFail_PlanIsEmpty,    ///< 路径规划结果为空
    StartFail_NotManagedSlot, ///< 目标车位不是被管理的车位
    StartFail_InvalidShape,   ///< 车位形状无效
    StartFail_NotInPlan,      ///< 目标车位未出现在路径规划结果
    StartFail_NoValidPath     ///< 目标车位的路径无效
} StartFailResultType;

#endif // SIGNAL_CONVERSION_H_