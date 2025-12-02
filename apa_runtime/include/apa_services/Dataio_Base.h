/**
 * @file dataio_base.h
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
#ifndef DATA_IO_BASE_H_
#define DATA_IO_BASE_H_

#include <pthread.h>
#include <assert.h>
#include <string.h>

/// 宏定义：DATAIO接口内存限制 50k
#define DATAIO_MEMORY_FRAME_MAX_SIZE (50 * 1024)

/**
 * @brief 自旋锁实现类，封装 pthread 自旋锁
 * @note 该类仅供 RTE_SPIN_LOCK_STUP 使用，不可直接拷贝或使用
 */
class RTE_SPIN_LOCK_STUP;
class RTE_SPIN_LOCK_IMPL
{
    pthread_spinlock_t lockid; ///< pthread 自旋锁 ID

  public:
    /**
     * @brief 构造函数，初始化自旋锁
     */
    RTE_SPIN_LOCK_IMPL() { pthread_spin_init(&lockid, PTHREAD_PROCESS_PRIVATE); }

    /**
     * @brief 析构函数，销毁自旋锁
     */
    ~RTE_SPIN_LOCK_IMPL() { pthread_spin_destroy(&lockid); }

    friend class RTE_SPIN_LOCK_STUP;
};

/**
 * @brief RAII 风格的自旋锁自动加锁解锁类
 * @details 在作用域内自动加锁，作用域结束自动解锁
 */
class RTE_SPIN_LOCK_STUP
{
    static RTE_SPIN_LOCK_IMPL lockImpl; ///< 全局唯一的自旋锁实例

  public:
    /**
     * @brief 构造函数，加锁
     */
    RTE_SPIN_LOCK_STUP() { pthread_spin_lock(&lockImpl.lockid); }

    /**
     * @brief 析构函数，解锁
     */
    ~RTE_SPIN_LOCK_STUP() { pthread_spin_unlock(&lockImpl.lockid); }
};

/**
 * @brief RTE 内存池实现类
 * @details 提供内存分配、保存、恢复、落盘等功能，线程安全
 */
class MEMORY_POOL_IMPL
{

  public:
    /**
     * @brief 获取当前已用内存大小
     * @return 已用字节数
     * @note 线程安全
     */
    static size_t GET_MEMORY_SIZE()
    {
        size_t val = 0;
        RTE_SPIN_LOCK_STUP lock;
        val = rte_offset_;
        return val;
    }

    /**
     * @brief 从内存池分配内存（无锁）
     * @param size 申请的字节数
     * @return 分配的内存指针
     * @note 仅初始化阶段调用，线程不安全
     */
    static void *ALLOC(size_t size)
    {
        assert(rte_offset_ + size <= sizeof(rte_pool_) && "MEMORY_POOL_IMPL overflow!");
        void *ptr = &rte_pool_[rte_offset_];
        rte_offset_ += size;
        return ptr;
    }

  private:
    static char rte_pool_[DATAIO_MEMORY_FRAME_MAX_SIZE]; ///< RTE内存池
    static size_t rte_offset_;                           ///< RTE当前已用大小
};

/**
 * @brief 定义线程安全的基本类型接口
 */
#define DEFINE_RTE_INTERFACE(TYPE, DEFAULT, SET, GET)                       \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::ALLOC(sizeof(TYPE)); \
    static void init_##SET() { *val_##SET = (TYPE)DEFAULT; }                \
    void SET(const TYPE value)                                              \
    {                                                                       \
        RTE_SPIN_LOCK_STUP lock;                                            \
        *val_##SET = value;                                                 \
    }                                                                       \
    TYPE GET()                                                              \
    {                                                                       \
        RTE_SPIN_LOCK_STUP lock;                                            \
        return *val_##SET;                                                  \
    }

/**
 * @brief 定义线程安全的结构体接口
 */
#define DEFINE_RTE_STRUCT_INTERFACE(TYPE, SET, GET)                         \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::ALLOC(sizeof(TYPE)); \
    void SET(const TYPE *value)                                             \
    {                                                                       \
        RTE_SPIN_LOCK_STUP lock;                                            \
        memcpy(val_##SET, value, sizeof(TYPE));                             \
    }                                                                       \
    void GET(TYPE *value)                                                   \
    {                                                                       \
        RTE_SPIN_LOCK_STUP lock;                                            \
        memcpy(value, val_##SET, sizeof(TYPE));                             \
    }

/**
 * @brief 定义线程安全的数组访问接口
 */
#define DEFINE_RTE_ARRAY_INTERFACE(TYPE, LEN, SET, GET)                           \
    static TYPE *val_##SET = (TYPE *)MEMORY_POOL_IMPL::ALLOC(sizeof(TYPE) * LEN); \
    static void init_##SET() { memset(val_##SET, 0, sizeof(TYPE) * LEN); }        \
    void SET(const TYPE value[LEN])                                               \
    {                                                                             \
        RTE_SPIN_LOCK_STUP lock;                                                  \
        memcpy(val_##SET, value, sizeof(TYPE) * LEN);                             \
    }                                                                             \
    void GET(TYPE value[LEN])                                                     \
    {                                                                             \
        RTE_SPIN_LOCK_STUP lock;                                                  \
        memcpy(value, val_##SET, sizeof(TYPE) * LEN);                             \
    }

/**
 * @brief 定义线程安全的环形数组访问接口
 */
#define DEFINE_RTE_RAM_INF_INTERFACE(TYPE, LEN, SET, GET, CLR)                       \
    typedef struct                                                                   \
    {                                                                                \
        TYPE data[LEN];                                                              \
        uint32_t itemLen;                                                            \
        uint32_t buffLen;                                                            \
        uint32_t readBegin;                                                          \
        uint32_t itemNum;                                                            \
    } RAM_INF_##SET;                                                                 \
                                                                                     \
    static RAM_INF_##SET *inf_##SET = NULL;                                          \
                                                                                     \
    static bool init_##SET()                                                         \
    {                                                                                \
        inf_##SET = (RAM_INF_##SET *)MEMORY_POOL_IMPL::ALLOC(sizeof(RAM_INF_##SET)); \
        memset(inf_##SET, 0, sizeof(RAM_INF_##SET));                                 \
        inf_##SET->itemLen   = sizeof(TYPE);                                         \
        inf_##SET->buffLen   = LEN;                                                  \
        inf_##SET->readBegin = 0;                                                    \
        inf_##SET->itemNum   = 0;                                                    \
        return true;                                                                 \
    }                                                                                \
    static bool init_flag_##SET = init_##SET();                                      \
                                                                                     \
    void SET(const TYPE *pa)                                                         \
    {                                                                                \
        RTE_SPIN_LOCK_STUP lock;                                                     \
        if (inf_##SET->itemNum < inf_##SET->buffLen)                                 \
        {                                                                            \
            uint32_t pos =                                                           \
                (inf_##SET->readBegin + inf_##SET->itemNum) % inf_##SET->buffLen;    \
            inf_##SET->data[pos] = *pa;                                              \
            inf_##SET->itemNum++;                                                    \
        }                                                                            \
        else                                                                         \
        {                                                                            \
            inf_##SET->data[inf_##SET->readBegin] = *pa;                             \
            inf_##SET->readBegin = (inf_##SET->readBegin + 1) % inf_##SET->buffLen;  \
        }                                                                            \
    }                                                                                \
                                                                                     \
    bool GET(TYPE *pa)                                                               \
    {                                                                                \
        RTE_SPIN_LOCK_STUP lock;                                                     \
        bool ret = true;                                                             \
        if (inf_##SET->itemNum > 0)                                                  \
        {                                                                            \
            *pa                  = inf_##SET->data[inf_##SET->readBegin];            \
            inf_##SET->readBegin = (inf_##SET->readBegin + 1) % inf_##SET->buffLen;  \
            inf_##SET->itemNum--;                                                    \
        }                                                                            \
        else                                                                         \
        {                                                                            \
            ret = false;                                                             \
        }                                                                            \
        return ret;                                                                  \
    }                                                                                \
                                                                                     \
    void CLR()                                                                       \
    {                                                                                \
        RTE_SPIN_LOCK_STUP lock;                                                     \
        inf_##SET->readBegin = 0;                                                    \
        inf_##SET->itemNum   = 0;                                                    \
        memset(inf_##SET->data, 0, sizeof(TYPE) * inf_##SET->buffLen);               \
    }

#endif
