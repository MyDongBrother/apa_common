/**
 * @file Record_Buff.c
 * @brief 循环缓冲区日志保存模块实现。支持异步落盘、缓冲控制、加锁保护。
 *
 * @details
 * 1. Buff_Init / Buff_UnInit：初始化与销毁循环缓冲结构。
 * 2. Buff_Put / Buff_Get / Buff_Len：实现环形缓冲的写入、读出与长度查询。
 * 3. TrySaveLog：满足触发条件时自动异步落盘。
 * 4. SaveBuffToLog：实际执行写文件的异步任务。
 *
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-11-24
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-11-24 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#include "TimeSync_Api.h"
#include "ThreadManager_Api.h"
#include "Log_Buff.h"

static void TrySaveLog(void *bufFd);
static void *SaveBuffToLog(void *args);

/**
 * @brief 初始化循环缓冲区。
 *
 * @param buff     外部提供的内存块（含结构体 + 数据区）。
 * @param buffLen  内存块总大小。
 * @param logFd    已打开的日志文件指针。
 * @return void*   成功返回 Buff_Inf 指针；失败返回 NULL。
 *
 * @note
 * buff 内存布局：| Buff_Inf | data... |
 * 其中 data 区从 buff + sizeof(Buff_Inf) 开始。
 */
void *Buff_Init(uint8_t *buff, const int buffLen, FILE *logFd)
{
    if (buff == nullptr)
    {
        return NULL;
    }

    if (logFd == NULL)
    {
        return NULL;
    }
    if ((uint32_t)buffLen <= sizeof(Buff_Inf) + 256)
    {
        return NULL;
    }

    memset(buff, 0, sizeof(Buff_Inf));
    Buff_Inf *buffInf = (Buff_Inf *)buff;

    if (pthread_mutex_init(&buffInf->mutex, NULL) < 0)
    {
        printf("pthread_mutex_init %d %s\n", errno, strerror(errno));
        return NULL;
    }

    buffInf->data     = buff + sizeof(Buff_Inf);
    buffInf->buffLen  = buffLen - sizeof(Buff_Inf);
    buffInf->logFd    = logFd;
    buffInf->head     = 0;
    buffInf->validLen = 0;
    buffInf->counter  = 0;

    return (void *)buffInf;
}

/**
 * @brief 销毁缓冲区。若缓冲区中还有未写完的数据，会异步写入文件。
 *
 * @param bufFd Buff_Inf 指针。
 *
 * @note
 * 1. 若 validLen 为 0，则直接关闭文件。
 * 2. 若 validLen > 0，复制当前缓冲并异步写入。
 * 3. 释放旧的 Buff_Inf 结构，但异步线程会负责释放新创建的拷贝。
 */
void Buff_UnInit(void *bufFd)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;

    if (buffInf == nullptr)
    {
        return;
    }
    if (buffInf->logFd == NULL)
    {
        pthread_mutex_destroy(&buffInf->mutex);
        return;
    }

    if (buffInf->validLen <= 0)
    {
        fclose(buffInf->logFd);
        buffInf->logFd = NULL;
        pthread_mutex_destroy(&buffInf->mutex);
        return;
    }

    // cache old buff
    int save_buff_len = Buff_Len(bufFd);
    int offset        = sizeof(Buff_Inf);
    uint8_t *data     = (uint8_t *)malloc(save_buff_len + offset);
    int readLen       = Buff_Get(bufFd, data + offset, save_buff_len);
    Buff_Inf *tempInf = (Buff_Inf *)data;
    tempInf->data     = data + offset;
    tempInf->logFd    = buffInf->logFd;
    tempInf->validLen = -1;
    tempInf->buffLen  = readLen;

    // async save.
    int result = ASyncPool(SaveBuffToLog, tempInf);
    if (result != 0)
    {
        free((void *)data);
        fclose(buffInf->logFd);
    }

    // free old cache
    buffInf->logFd    = NULL;
    buffInf->validLen = 0;
    pthread_mutex_destroy(&buffInf->mutex);
}

/**
 * @brief 向缓冲区写入数据。
 *
 * @param bufFd   Buff_Inf 指针。
 * @param data    写入数据。
 * @param dataLen 写入长度。
 * @return int    实际写入长度；失败返回 0。
 *
 * @note
 * 该函数为环形缓冲写入逻辑，采用 modulo 实现尾部回绕。
 */
int Buff_Put(void *bufFd, const char *data, const int dataLen)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;
    if (buffInf == nullptr || dataLen <= 0)
    {
        return 0;
    }

    pthread_mutex_lock(&buffInf->mutex);
    if (dataLen > buffInf->buffLen - buffInf->validLen)
    {
        pthread_mutex_unlock(&buffInf->mutex);
        return 0;
    }

    auto start = (buffInf->head + buffInf->validLen) % buffInf->buffLen;
    if (start + dataLen <= buffInf->buffLen)
    {
        memcpy(buffInf->data + start, data, dataLen);
    }
    else
    {
        auto left = buffInf->buffLen - start;
        memcpy(buffInf->data + start, data, left);
        memcpy(buffInf->data, &data[left], dataLen - left);
    }

    buffInf->validLen = buffInf->validLen + dataLen;
    buffInf->counter++;
    pthread_mutex_unlock(&buffInf->mutex);

    TrySaveLog(bufFd);

    return dataLen;
}
/**
 * @brief 从缓冲区读取数据（从 head 开始）。
 *
 * @param bufFd   Buff_Inf 指针。
 * @param data    输出缓冲区。
 * @param dataLen 最大读取长度。
 * @return int    实际读取长度。
 */
int Buff_Get(void *bufFd, uint8_t *data, const int dataLen)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;

    if (buffInf == nullptr || buffInf->validLen <= 0)
    {
        return 0;
    }

    pthread_mutex_lock(&buffInf->mutex);
    int readLen = (dataLen < buffInf->validLen) ? (dataLen) : (buffInf->validLen);
    if (buffInf->head + readLen <= buffInf->buffLen)
    {
        memcpy(data, buffInf->data + buffInf->head, readLen);
        buffInf->head     = buffInf->head + readLen;
        buffInf->validLen = buffInf->validLen - readLen;
    }
    else
    {
        auto left = buffInf->buffLen - buffInf->head;
        memcpy(data, buffInf->data + buffInf->head, left);
        memcpy(&data[left], buffInf->data, readLen - left);
        buffInf->head     = readLen - left;
        buffInf->validLen = buffInf->validLen - readLen;
    }
    pthread_mutex_unlock(&buffInf->mutex);

    return readLen;
}

/**
 * @brief 查询当前缓冲区有效数据长度。
 *
 * @param bufFd Buff_Inf 指针。
 * @return int  validLen。
 */
int Buff_Len(void *bufFd)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;
    if (buffInf == nullptr)
    {
        return 0;
    }
    pthread_mutex_lock(&buffInf->mutex);
    int result = buffInf->validLen;
    pthread_mutex_unlock(&buffInf->mutex);
    return result;
}

/**
 * @brief 异步线程：将一段缓冲写入日志文件。
 *
 * @param args Buff_Inf 指针（为临时拷贝，包含要写的数据）。
 * @return void*
 *
 * @note 使用 fwrite 按顺序写入全部数据。写完后释放结构体。
 */
static void *SaveBuffToLog(void *args)
{
    Buff_Inf *buffInf = (Buff_Inf *)args;
    if (buffInf == nullptr)
    {
        return NULL;
    }

    if (buffInf->logFd == NULL)
    {
        free((void *)buffInf);
        return 0;
    }

    int buffLen   = buffInf->buffLen;
    uint8_t *data = buffInf->data;
    int writelen  = 0;
    while (buffLen > 0)
    {
        int result = fwrite(data + writelen, 1, buffLen, buffInf->logFd);
        if (result > 0)
        {
            writelen = writelen + result;
            buffLen  = buffLen - result;
        }
    }

    if (buffInf->validLen < 0)
    {
        fclose(buffInf->logFd);
    }

    free((void *)buffInf);
    return 0;
}

/**
 * @brief 检查是否需要触发日志保存，并执行异步落盘。
 *
 * @param bufFd Buff_Inf 指针。
 *
 * @note 满足以下任意条件会落盘：
 *       1. 有效数据超过 75%（4/3 buffLen）。
 *       2. counter 超过 300 次写入。
 */
static void TrySaveLog(void *bufFd)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;
    if (buffInf == nullptr || buffInf->logFd == NULL)
    {
        return;
    }

    pthread_mutex_lock(&buffInf->mutex);
    bool savelog = (4 * buffInf->validLen > 3 * buffInf->buffLen) ||
                   (buffInf->validLen > 0 && buffInf->counter > 300);
    if (savelog)
    {
        buffInf->counter = 0;
    }
    auto logfd = buffInf->logFd;
    pthread_mutex_unlock(&buffInf->mutex);

    if (savelog)
    {
        // cache old buff
        int save_buff_len = Buff_Len(bufFd);
        int offset        = sizeof(Buff_Inf);
        uint8_t *data     = (uint8_t *)malloc(save_buff_len + offset);
        int readLen       = Buff_Get(bufFd, data + offset, save_buff_len);

        if (readLen <= 0)
        {
            free(data);
            return;
        }

        Buff_Inf *tempInf = (Buff_Inf *)data;
        tempInf->data     = data + offset;
        tempInf->logFd    = logfd;
        tempInf->buffLen  = readLen;
        tempInf->validLen = readLen;

        // async save
        int result = ASyncPool(SaveBuffToLog, tempInf);
        if (result != 0)
        {
            free((void *)data);
        }
    }
}
