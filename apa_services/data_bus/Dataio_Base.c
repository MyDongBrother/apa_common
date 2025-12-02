/**
 * @file dataio_base.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-28
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-28 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#include "Dataio_Base.h"

/// 定义全局自旋锁实现
RTE_SPIN_LOCK_IMPL RTE_SPIN_LOCK_STUP::lockImpl;

/// 内存池空间
char MEMORY_POOL_IMPL::rte_pool_[DATAIO_MEMORY_FRAME_MAX_SIZE];
/// 当前内存池偏移量
size_t MEMORY_POOL_IMPL::rte_offset_ = 0;
