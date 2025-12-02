/**
 * @file DataBus.cpp
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-11-05 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include <unordered_map>
#include <pthread.h>
#include <atomic>
#include "LogManager_Api.h"
#include "DataBus_Api.h"
#include "ThreadManager_Api.h"
#include <cstring>

/**
 * @brief 简易自旋锁
 * 多生产者写入 DataBus 时使用，避免线程切换带来的阻塞
 */
struct SpinLock
{
    std::atomic_flag f = ATOMIC_FLAG_INIT;

    void Lock()
    {
        // 自旋直到拿到锁，适用于短临界区
        while (f.test_and_set(std::memory_order_acquire))
        {
        }
    }

    void Unlock() { f.clear(std::memory_order_release); }
};

/*------------------ 全局静态缓冲与元数据 ------------------*/

/** @brief 缓冲区总大小 */
const uint32_t c_size = 1024 * 1024;
/** @brief 对g_pos和g_buff加锁 */
static SpinLock g_lock;
/** @brief 写缓冲区（生产者写入） */
static uint8_t g_buff[c_size];
/** @brief 当前写指针位置 */
static uint32_t g_pos{0};

/** @brief 读缓冲区（消费者读取，避免锁内解析） */
static uint8_t g_read_buff[c_size];

/**
 * @brief 注册字典：id → (len, callback)
 * g_read_map: 消费者解析用，无锁，当前不支持热更新
 */
static std::unordered_map<DataBusID_T, std::pair<uint16_t, RxDataProcFunc>> g_read_map;

/**
 * @brief 自旋锁守卫，作用域自动加解锁
 */
struct SpinGuard
{
    SpinGuard(SpinLock &l) : lock_(l) { lock_.Lock(); }
    ~SpinGuard() { lock_.Unlock(); }
    SpinLock &lock_;
};
/*------------------ 内部 Buffer 操作 ------------------*/

/**
 * @brief 写入环形缓冲
 * 数据格式: [id | data]
 * @note 简化模型，没有 wrap-around
 */
static bool BuffWrite(DataBusID_T id, const uint8_t *data, uint16_t len)
{
    SpinGuard _g(g_lock);

    uint32_t need = sizeof(DataBusID_T) + len;

    if (g_pos + need > c_size)
    {
        log_warn("DataBus buffer full, drop packet id=%d len=%u", id, len);
        return false; // 缓冲已满，丢弃
    }

    memcpy(&g_buff[g_pos], &id, sizeof(id));
    g_pos += sizeof(id);

    memcpy(&g_buff[g_pos], data, len);
    g_pos += len;

    return true;
}

/**
 * @brief Flush 写缓冲到读缓冲
 * @return 拷贝的数据长度
 */
static uint32_t BuffRead(uint8_t *out)
{
    SpinGuard _g(g_lock);

    if (g_pos == 0)
        return 0;

    uint32_t n = g_pos;
    g_pos      = 0;

    memcpy(out, g_buff, n);
    return n;
}

/*------------------ 对外 API ------------------*/

/**
 * @brief 注册 DataBus 数据类型
 *
 * @param id   数据类型 ID
 * @param len  数据长度（固定长度协议）
 * @param proc 回调处理函数
 * @return true 注册成功
 * @return false 参数无效
 */
bool DataBus_Register(DataBusID_T id, uint16_t len, RxDataProcFunc proc)
{
    if (!id || !len || !proc)
    {
        log_err("DataBus_Register param error");
        return false;
    }
    // 注册阶段调用，运行时不变，无需锁
    g_read_map[id] = {len, proc};

    return true;
}

/**
 * @brief 生产者写入数据
 * @param id   数据 ID
 * @param data 二进制数据指针
 * @param len  长度
 */
bool DataBus_Push(DataBusID_T id, const uint8_t *data, uint16_t len)
{
    if (!id || !data || !len)
        return false;

    auto it = g_read_map.find(id);
    if (it == g_read_map.end() || it->second.first != len)
        return false;

    return BuffWrite(id, data, len);
}

static FILE *g_record_fp = nullptr;
#ifdef APA_PLATFORM_X86
static bool g_replay_mode = true;
#else
static bool g_replay_mode = false;
#endif

static FILE *g_replay_fp = nullptr;

#define MAGIC_HEAD 0xABCD1234u
#define MAGIC_TAIL 0xDCBA4321u

/**
 * @brief 消费线程调用，解析所有缓冲数据
 * @note 单线程消费，无回环，无丢包保护
 */
static void *ProcessDataBus(void *paras)
{
    while (1)
    {
        // Replay mode
        if (g_replay_mode)
        {
            if (!g_replay_fp)
            {
                g_replay_fp = fopen("./vis_log.bin", "rb");
                if (!g_replay_fp)
                {
                    perror("open replay file");
                    sleep(1);
                    continue; // keep thread alive
                }
            }

            uint32_t n = 0;
            if (fread(&n, 4, 1, g_replay_fp) != 1)
            {
                // 文件读完了，重新回到开头（循环播放）
                // fseek(g_replay_fp, 0, SEEK_SET);
                continue;
            }

            if (fread(g_read_buff, 1, n, g_replay_fp) != n)
                continue;

            uint32_t pos = 0;
            while (pos < n)
            {
                DataBusID_T id;
                memcpy(&id, &g_read_buff[pos], sizeof(id));
                pos += sizeof(id);

                auto it = g_read_map.find(id);
                if (it == g_read_map.end())
                    break;

                uint16_t len = it->second.first;
                auto cb      = it->second.second;

                cb(&g_read_buff[pos], len);
                pos += len;
            }

            usleep(19000); // mimic real bus pacing
            continue;
        }

        // Normal record mode
        if (!g_record_fp)
        {
            g_record_fp = fopen("./vis_log.bin", "wb");
            if (!g_record_fp)
            {
                perror("open record file");
                printf("cwd = %s\n", getcwd(NULL, 0));
                sleep(1);
                continue;
            }
        }

        uint32_t n = BuffRead(g_read_buff);
        if (n == 0)
        {
            usleep(1000); // no data, avoid spinning
            continue;
        }

        fwrite(&n, 4, 1, g_record_fp);
        fwrite(g_read_buff, 1, n, g_record_fp);
        fflush(g_record_fp);

        uint32_t pos = 0;
        while (pos < n)
        {
            DataBusID_T id;
            memcpy(&id, &g_read_buff[pos], sizeof(id));
            pos += sizeof(id);

            auto it = g_read_map.find(id);
            if (it == g_read_map.end())
                break;

            uint16_t len = it->second.first;
            auto cb      = it->second.second;

            cb(&g_read_buff[pos], len);
            pos += len;
        }
    }

    return NULL;
}

void DataBusInit() { ASyncRun(ProcessDataBus, NULL, "run_databus"); }
