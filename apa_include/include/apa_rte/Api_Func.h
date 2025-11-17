/**
 * @file Rte_Func.h
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
#ifndef API_FUNC_H
#define API_FUNC_H

#include "Rte_Types.h"
#include "Ram.h"
#include "Rte_Memory.h"

/**
 * @brief 定义线程安全的基本类型接口
 */
#define DEFINE_API_INTERFACE(TYPE, DEFAULT, SET, GET)                           \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::API_ALLOC(sizeof(TYPE)); \
    static void init_##SET() { *val_##SET = (TYPE)DEFAULT; }                    \
    void SET(const TYPE value)                                                  \
    {                                                                           \
        RTE_SPIN_LOCK_STUP lock;                                                \
        *val_##SET = value;                                                     \
    }                                                                           \
    TYPE GET()                                                                  \
    {                                                                           \
        RTE_SPIN_LOCK_STUP lock;                                                \
        return *val_##SET;                                                      \
    }

/**
 * @brief 定义线程安全的结构体接口
 */
#define DEFINE_API_STRUCT_INTERFACE(TYPE, SET, GET)                             \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::API_ALLOC(sizeof(TYPE)); \
    void SET(const TYPE *value)                                                 \
    {                                                                           \
        RTE_SPIN_LOCK_STUP lock;                                                \
        memcpy(val_##SET, value, sizeof(TYPE));                                 \
    }                                                                           \
    void GET(TYPE *value)                                                       \
    {                                                                           \
        RTE_SPIN_LOCK_STUP lock;                                                \
        memcpy(value, val_##SET, sizeof(TYPE));                                 \
    }
/**
 * @brief 定义线程安全的数组访问接口
 */
#define DEFINE_API_ARRAY_INTERFACE(TYPE, LEN, SET, GET)                               \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::API_ALLOC(sizeof(TYPE) * LEN); \
    static void init_##SET() { memset(val_##SET, 0, sizeof(TYPE) * LEN); }            \
    void SET(const TYPE value[LEN])                                                   \
    {                                                                                 \
        RTE_SPIN_LOCK_STUP lock;                                                      \
        memcpy(val_##SET, value, sizeof(TYPE) * LEN);                                 \
    }                                                                                 \
    void GET(TYPE value[LEN])                                                         \
    {                                                                                 \
        RTE_SPIN_LOCK_STUP lock;                                                      \
        memcpy(value, val_##SET, sizeof(TYPE) * LEN);                                 \
    }

#endif
