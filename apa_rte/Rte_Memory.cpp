/**
 * @file Rte_Memory.cpp
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

#include "Rte_Memory.h"
#include <unistd.h>
#include "Rte_Func.h"
#include "Rte.h"

/// 定义全局自旋锁实现
RTE_SPIN_LOCK_IMPL RTE_SPIN_LOCK_STUP::lockImpl;

//
// ================== MEMORY_POOL_IMPL 静态成员定义 ==================
//

/// 内存池空间
char MEMORY_POOL_IMPL::rte_pool_[RTE_MEMORY_FRAME_MAX_SIZE];
/// 当前内存池偏移量
size_t MEMORY_POOL_IMPL::rte_offset_ = 0;

/// 内存池空间
char MEMORY_POOL_IMPL::api_pool_[API_MEMORY_FRAME_MAX_SIZE];
/// 当前内存池偏移量
size_t MEMORY_POOL_IMPL::api_offset_ = 0;

//
// ================== 调试功能:rte接口数据录制 ==================
//
#ifdef DEBUG_VERSION
/// 文件句柄
FILE *MEMORY_POOL_IMPL::fp = nullptr;
/// 落盘文件路径
const char *MEMORY_POOL_IMPL::kFilePath = "./rte_memory_pool.dat";
/// 堆缓存区
uint8_t *MEMORY_POOL_IMPL::heap_cache_ = nullptr;
/// 落盘缓冲区
uint8_t *MEMORY_POOL_IMPL::flush_buffer_ = nullptr;
/// 堆缓存已使用长度
size_t MEMORY_POOL_IMPL::heap_cache_len_ = 0;

/// 后台刷盘线程 ID
static pthread_t flush_tid;
/// 刷盘线程互斥锁
static pthread_mutex_t flush_mutex = PTHREAD_MUTEX_INITIALIZER;
/// 刷盘线程条件变量
static pthread_cond_t flush_cond = PTHREAD_COND_INITIALIZER;
/// 是否需要刷盘标志
static bool flush_needed = false;
/// 线程退出标志
static bool exit_flag = false;

/**
 * @brief 后台刷盘线程入口函数
 *
 * 等待条件变量触发，当 `flush_needed` 为真时调用 `FLUSH_FILE()` 落盘。
 * 当 `exit_flag` 为真时，安全退出线程。
 */
void *FlushThread(void *)
{
    while (true)
    {
        pthread_mutex_lock(&flush_mutex);
        while (!flush_needed && !exit_flag)
        {
            pthread_cond_wait(&flush_cond, &flush_mutex);
        }
        if (exit_flag)
        {
            pthread_mutex_unlock(&flush_mutex);
            break;
        }
        flush_needed = false;
        pthread_mutex_unlock(&flush_mutex);

        MEMORY_POOL_IMPL::FLUSH_FILE();
    }
    return nullptr;
}

/**
 * @brief 停止后台刷盘线程
 *
 * 设置退出标志并唤醒线程，等待其安全退出。
 */
void StopFlushThread()
{
    pthread_mutex_lock(&flush_mutex);
    exit_flag = true;
    pthread_cond_signal(&flush_cond);
    pthread_mutex_unlock(&flush_mutex);
    pthread_join(flush_tid, nullptr);
}

/**
 * @brief 初始化内存池模块
 *
 * - 分配堆缓存和落盘缓冲
 * - 启动后台刷盘线程
 */
void MEMORY_POOL_IMPL::Init()
{
    // 构造阶段就申请堆缓存
    heap_cache_   = new uint8_t[RTE_MEMORY_FRAME_MAX_SIZE * 10]; ///< 内存池保存缓存
    flush_buffer_ = new uint8_t[RTE_MEMORY_FRAME_MAX_SIZE * 10]; ///< 落盘缓冲
    ASyncRun(FlushThread, (void *)NULL, "dbg_flush");
    heap_cache_len_ = 0; ///< 当前缓存长度
}

/**
 * @brief 反初始化内存池模块
 *
 * - 停止后台线程
 * - 释放堆缓存与缓冲区
 * - 关闭文件
 */
void MEMORY_POOL_IMPL::ReInit()
{
    StopFlushThread();
    delete[] heap_cache_;
    heap_cache_ = nullptr;
    delete[] flush_buffer_;
    flush_buffer_ = nullptr;
    if (fp)
    {
        fclose(fp);
        fp = nullptr;
    }
}

/**
 * @brief 计算简单校验和
 * @param data 数据指针
 * @param len 数据长度
 * @return 校验和
 */
uint32_t MEMORY_POOL_IMPL::CalcChecksum(const void *data, size_t len)
{
    uint32_t sum     = 0;
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < len; ++i) sum += p[i];
    return sum;
}

/**
 * @brief 从内存池分配内存
 * @param size 申请大小
 * @return 分配指针
 */
void *MEMORY_POOL_IMPL::RTE_ALLOC(size_t size)
{
    assert(rte_offset_ + size <= sizeof(rte_pool_) && "MEMORY_POOL_IMPL overflow!");
    void *ptr = &rte_pool_[rte_offset_];
    rte_offset_ += size;
    return ptr;
}

/**
 * @brief 从内存池分配内存
 * @param size 申请大小
 * @return 分配指针
 */
void *MEMORY_POOL_IMPL::API_ALLOC(size_t size)
{
    assert(api_offset_ + size <= sizeof(api_pool_) && "MEMORY_POOL_IMPL overflow!");
    void *ptr = &api_pool_[api_offset_];
    api_offset_ += size;
    return ptr;
}

/**
 * @brief 保存当前内存池内容到堆缓存
 * @return 是否成功
 */
bool MEMORY_POOL_IMPL::SAVE_FRAME()
{
    uint8_t *temp   = new uint8_t[RTE_MEMORY_FRAME_MAX_SIZE];
    uint32_t length = 0;

    {
        RTE_SPIN_LOCK_STUP lock; ///< 上锁，保证多线程安全
        length = (uint32_t)rte_offset_;
        memcpy(temp, rte_pool_, length); ///< 拷贝当前内存池
    }

    const uint32_t kMagic = 0x52544550;                 ///< 魔数标识
    uint32_t checksum     = CalcChecksum(temp, length); ///< 计算校验和
    size_t total          = sizeof(kMagic) + sizeof(length) + length + sizeof(checksum);

    assert(heap_cache_len_ + total <= 10 * RTE_MEMORY_FRAME_MAX_SIZE);

    uint8_t *p = heap_cache_ + heap_cache_len_;
    memcpy(p, &kMagic, sizeof(kMagic));
    p += sizeof(kMagic);
    memcpy(p, &length, sizeof(length));
    p += sizeof(length);
    memcpy(p, temp, length);
    p += length;
    memcpy(p, &checksum, sizeof(checksum));

    heap_cache_len_ += total;

    if (heap_cache_len_ > 7 * RTE_MEMORY_FRAME_MAX_SIZE)
    {
        pthread_mutex_lock(&flush_mutex);
        flush_needed = true;
        pthread_cond_signal(&flush_cond);
        pthread_mutex_unlock(&flush_mutex);
    }

    delete[] temp;
    temp = nullptr;
    return true;
}

/**
 * @brief 从文件加载内存池内容
 * @return 是否成功
 */
bool MEMORY_POOL_IMPL::LOAD_FRAME()
{
    uint8_t *temp = new uint8_t[RTE_MEMORY_FRAME_MAX_SIZE];

    if (!fp)
        fp = fopen(kFilePath, "rb");
    if (!fp)
    {
        delete[] temp;
        return false;
    }

    uint32_t magic = 0;
    if (fread(&magic, sizeof(magic), 1, fp) != 1)
    {
        delete[] temp;
        return false;
    }
    if (magic != 0x52544550)
    { // "PETR"
        delete[] temp;
        return false;
    }

    uint32_t length = 0;
    if (fread(&length, sizeof(length), 1, fp) != 1)
    {
        delete[] temp;
        return false;
    }

    if (length != rte_offset_)
    {
        delete[] temp;
        return false;
    }

    size_t read = fread(temp, 1, length, fp);
    if (read != length)
    {
        delete[] temp;
        return false;
    }

    uint32_t checksum_file = 0;
    if (fread(&checksum_file, sizeof(checksum_file), 1, fp) != 1)
    {
        delete[] temp;
        return false;
    }

    uint32_t checksum = CalcChecksum(temp, read);
    if (checksum != checksum_file)
    {
        delete[] temp;
        return false;
    }

    {
        RTE_SPIN_LOCK_STUP lock;
        memcpy(rte_pool_, temp, length); ///< 恢复内存池
    }

    delete[] temp;
    return true;
}

/**
 * @brief 从文件加载内存池内容
 * @return 是否成功
 */
bool MEMORY_POOL_IMPL::RESET_FRAME()
{
    if (fp)
    {
        fclose(fp);
        fp = nullptr;
    }
    return true;
}
/**
 * @brief 将堆缓存落盘
 * @return 是否成功
 */
bool MEMORY_POOL_IMPL::FLUSH_FILE()
{
    if (!fp)
        fp = fopen(kFilePath, "wb");
    if (!fp)
        return false;

    uint32_t length = 0;
    {
        RTE_SPIN_LOCK_STUP lock;                             ///< 上锁
        memcpy(flush_buffer_, heap_cache_, heap_cache_len_); ///< 拷贝堆缓存
        length          = heap_cache_len_;
        heap_cache_len_ = 0; ///< 清空堆缓存
    }
    fwrite(flush_buffer_, length, 1, fp);
    fflush(fp);        ///< 刷新用户态缓存
    fsync(fileno(fp)); ///< 确保落盘

    return true;
}

/**
 * @brief 录制回灌功能
 *
 * @return true
 * @return false
 */
bool MEMORY_POOL_IMPL::REPLAY()
{
    // 检查录制回灌调试功能
    DebugCmdType cmd_temp = {};
    RTE_BSW_Get_Debug_Command(&cmd_temp);
    if (cmd_temp.record_cmd.switch_flag == 1)
    {
        MEMORY_POOL_IMPL::SAVE_FRAME();
    }
    else if (cmd_temp.record_cmd.switch_flag == 2)
    {
        MEMORY_POOL_IMPL::LOAD_FRAME();
        // 回灌时会把DebugCmdType覆盖掉
        RTE_BSW_Set_Debug_Command(&cmd_temp);
    }
    else // cmd.record_cmd.switch_flag == 0
    {
        MEMORY_POOL_IMPL::RESET_FRAME();
    }
    return true;
}

#endif /// end DEBUG_VERSION