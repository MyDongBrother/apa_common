/**
 * @file Rte_Memory.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-08
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-08 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef RTE_MEMORYPOOL_H
#define RTE_MEMORYPOOL_H

#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <pthread.h>

/// 宏定义：RTE接口内存限制 10k
#define RTE_MEMORY_FRAME_MAX_SIZE (10 * 1024)
/// 宏定义：API接口内存限制 50k
#define API_MEMORY_FRAME_MAX_SIZE (50 * 1024)

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
        val = rte_offset_ + api_offset_;
        return val;
    }

    /**
     * @brief 从内存池分配内存（无锁）
     * @param size 申请的字节数
     * @return 分配的内存指针
     * @note 仅初始化阶段调用，线程不安全
     */
    static void *RTE_ALLOC(size_t size);

    /**
     * @brief 从内存池分配内存（无锁）
     * @param size 申请的字节数
     * @return 分配的内存指针
     * @note 仅初始化阶段调用，线程不安全
     */
    static void *API_ALLOC(size_t size);

  private:
    static char rte_pool_[RTE_MEMORY_FRAME_MAX_SIZE]; ///< RTE内存池
    static size_t rte_offset_;                        ///< RTE当前已用大小
    static char api_pool_[API_MEMORY_FRAME_MAX_SIZE]; ///< API内存池
    static size_t api_offset_;                        ///< API当前已用大小

#ifdef DEBUG_VERSION
    static uint8_t *heap_cache_;   ///< 堆缓存指针
    static uint8_t *flush_buffer_; ///< 落盘缓冲区指针
    static size_t heap_cache_len_; ///< 堆缓存已使用字节数
    static FILE *fp;               ///< 文件句柄
    static const char *kFilePath;  ///< 文件保存路径

    /**
     * @brief 计算校验和
     * @param data 数据指针
     * @param len 数据长度
     * @return 校验和
     */
    static uint32_t CalcChecksum(const void *data, size_t len);

    /**
     * @brief 保存当前内存池内容到堆缓存
     * @return 是否保存成功
     * @details 格式：[magic(4)][length(4)][payload(length)][checksum(4)]
     */
    static bool SAVE_FRAME();

    /**
     * @brief
     * @return
     * @details
     */
    static bool RESET_FRAME();

    /**
     * @brief 从外部 buffer 恢复内存
     * @return 是否恢复成功
     * @note buffer 格式同 SAVE_FRAME，线程安全
     */
    static bool LOAD_FRAME();

  public:
    /**
     * @brief 初始化内存池（创建堆缓存、启动 Flush 线程）
     */
    static void Init();

    /**
     * @brief 重新初始化内存池（清空缓存，重置状态）
     */
    static void ReInit();

    /**
     * @brief 将堆缓存内容写入文件（同步方式）
     * @return 是否写入成功
     * @note 该函数内部会加锁，适合 Flush 线程调用
     */
    static bool FLUSH_FILE();

    /**
     * @brief
     * @return
     * @details
     */
    static bool REPLAY();
#endif
};

#endif
