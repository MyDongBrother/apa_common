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
#ifndef RTE_FUNC_H
#define RTE_FUNC_H

#include "Rte_Types.h"
#include "Ram.h"
#include <assert.h>
#include "Rte_Memory.h"

/**
 * @brief 定义线程安全的基本类型接口
 */
#define DEFINE_RTE_INTERFACE(TYPE, DEFAULT, SET, GET)                           \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::RTE_ALLOC(sizeof(TYPE)); \
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
#define DEFINE_RTE_STRUCT_INTERFACE(TYPE, SET, GET)                             \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::RTE_ALLOC(sizeof(TYPE)); \
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
#define DEFINE_RTE_ARRAY_INTERFACE(TYPE, LEN, SET, GET)                               \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::RTE_ALLOC(sizeof(TYPE) * LEN); \
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

/**
 * @brief 定义线程安全的环形数组访问接口
 */
#define DEFINE_RTE_RAM_INF_INTERFACE(TYPE, LEN, SET, GET, CLR)                           \
    typedef struct                                                                       \
    {                                                                                    \
        TYPE data[LEN];                                                                  \
        uint32_t itemLen;                                                                \
        uint32_t buffLen;                                                                \
        uint32_t readBegin;                                                              \
        uint32_t itemNum;                                                                \
    } RAM_INF_##SET;                                                                     \
                                                                                         \
    static RAM_INF_##SET *inf_##SET = NULL;                                              \
                                                                                         \
    static bool init_##SET()                                                             \
    {                                                                                    \
        inf_##SET = (RAM_INF_##SET *)MEMORY_POOL_IMPL::RTE_ALLOC(sizeof(RAM_INF_##SET)); \
        memset(inf_##SET, 0, sizeof(RAM_INF_##SET));                                     \
        inf_##SET->itemLen   = sizeof(TYPE);                                             \
        inf_##SET->buffLen   = LEN;                                                      \
        inf_##SET->readBegin = 0;                                                        \
        inf_##SET->itemNum   = 0;                                                        \
        return true;                                                                     \
    }                                                                                    \
    static bool init_flag_##SET = init_##SET();                                          \
                                                                                         \
    void SET(const TYPE *pa)                                                             \
    {                                                                                    \
        RTE_SPIN_LOCK_STUP lock;                                                         \
        if (inf_##SET->itemNum < inf_##SET->buffLen)                                     \
        {                                                                                \
            uint32_t pos =                                                               \
                (inf_##SET->readBegin + inf_##SET->itemNum) % inf_##SET->buffLen;        \
            inf_##SET->data[pos] = *pa;                                                  \
            inf_##SET->itemNum++;                                                        \
        }                                                                                \
        else                                                                             \
        {                                                                                \
            inf_##SET->data[inf_##SET->readBegin] = *pa;                                 \
            inf_##SET->readBegin = (inf_##SET->readBegin + 1) % inf_##SET->buffLen;      \
        }                                                                                \
    }                                                                                    \
                                                                                         \
    void GET(TYPE *pa)                                                                   \
    {                                                                                    \
        RTE_SPIN_LOCK_STUP lock;                                                         \
        if (inf_##SET->itemNum > 0)                                                      \
        {                                                                                \
            *pa                  = inf_##SET->data[inf_##SET->readBegin];                \
            inf_##SET->readBegin = (inf_##SET->readBegin + 1) % inf_##SET->buffLen;      \
            inf_##SET->itemNum--;                                                        \
        }                                                                                \
        else                                                                             \
        {                                                                                \
            memset(pa, 0, sizeof(TYPE));                                                 \
        }                                                                                \
    }                                                                                    \
                                                                                         \
    void CLR()                                                                           \
    {                                                                                    \
        RTE_SPIN_LOCK_STUP lock;                                                         \
        inf_##SET->readBegin = 0;                                                        \
        inf_##SET->itemNum   = 0;                                                        \
        memset(inf_##SET->data, 0, sizeof(TYPE) * inf_##SET->buffLen);                   \
    }

void ASyncRun(void *(*routine)(void *), void *arg, const char *thread_name);
pthread_t SyncRun(void *(*routine)(void *), void *arg, const char *thread_name);
void WaitSyncRun(const pthread_t threadId);
int ASyncPool(void *(*routine)(void *), void *arg);
void Rte_EnterCriticalSection();
void Rte_LeaveCriticalSection();
void RTE_BSW_Set_AbsStamp(struct tm *stdtm);
void RTE_BSW_Get_AbsStamp(struct tm *stdtm);

#endif
