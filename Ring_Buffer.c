/**
 * @file ring_buffer.c
 * @brief 平台中重要的环形数组数据结构
 *
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

#include "Ring_Buffer.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>
#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief 获取缓冲区的最大容量（数据项个数）
     *
     * @param ram 指向 RAM_INF 的指针
     * @return 缓冲区最多能存放的数据项数量
     */
    static inline uint32_t RamItemTotal(RAM_INF *ram)
    {
        return ram->buffLen / ram->itemLen;
    }

    /**
     * @brief 从缓冲区中读取指定索引的一条数据项（不改变读指针）
     *
     * @param ram      指向 RAM_INF 的指针
     * @param index    相对读指针的索引（0 表示最旧数据）
     * @param outBuff  输出缓冲区，长度必须为 itemLen 字节
     */
    static inline void ReadOne(RAM_INF *ram, const uint32_t index, uint8_t *outBuff)
    {
        uint32_t readIndex = (index + ram->readBegin) % RamItemTotal(ram);
        memcpy(outBuff, ram->data + readIndex * ram->itemLen, ram->itemLen);
    }

    /**
     * @brief 向环形缓冲区中写入一条固定长度数据项。
     *
     * 当 startIndex == -1 且 inCount == 1 时，将数据写入当前写指针（自动环形），
     * 并自动管理覆盖最早的数据（先进先出）；
     *
     * 数据项结构（定长）：
     * ┌────────────┬────────────┬────────────┬────────────┐
     * │ item0      │ item1      │ ...        │ itemN-1    │
     * └────────────┴────────────┴────────────┴────────────┘
     * 每个 item 长度为 ram->itemLen 字节
     *
     * @param ram       指向 RAM_INF 缓冲区信息结构体的指针
     * @param startIndex 写入索引，-1 表示自动追加（FIFO 模式）
     * @param inBuff    输入数据指针，长度必须为 itemLen 字节
     * @param inCount   数据项个数，目前仅支持 1
     */
    void Ram_Put(RAM_INF *ram, const int startIndex, const uint8_t *inBuff,
                 const uint32_t inCount)
    {
        if (ram == NULL || inBuff == NULL || inCount == 0)
        {
            return;
        }

        if (startIndex == -1 && inCount == 1)
        {
            uint32_t writeIndex = (ram->itemNum + ram->readBegin) % RamItemTotal(ram);
            memcpy(ram->data + writeIndex * ram->itemLen, inBuff, ram->itemLen);
            if (ram->itemNum >= RamItemTotal(ram))
            {
                ram->readBegin = (ram->readBegin + 1) % RamItemTotal(ram);
            }
            else
            {
                ram->itemNum++;
            }
        }
        else
        {
            assert(0);
        }
    }
    /**
     * @brief 获取当前缓冲区中已有的数据项数量
     *
     * @param ram 指向 RAM_INF 的指针
     * @return 当前已存储的数据项数量
     */
    uint32_t Ram_Len(RAM_INF *ram) { return (ram == NULL) ? 0 : ram->itemNum; }

    /**
     * @brief 从缓冲区中读取数据项（不清除）
     *
     * @param ram       指向 RAM_INF 的指针
     * @param startIndex 起始读取索引（相对于读指针）
     * @param outBuff   输出缓冲区，大小为 outCount * itemLen
     * @param outCount  要读取的数据项个数
     * @return 实际读取到的数据项数量
     */
    uint32_t Ram_Get(RAM_INF *ram, const int startIndex, uint8_t *outBuff,
                     const uint32_t outCount)
    {
        if (ram == NULL || outBuff == NULL || outCount == 0)
        {
            return 0;
        }

        if (ram->itemNum == 0 || ram->itemNum < (uint32_t)startIndex + 1)
        {
            return 0;
        }

        uint32_t readIndex = startIndex;
        uint32_t readCount = 0;
        while (readIndex < ram->itemNum && readCount < outCount)
        {
            ReadOne(ram, readIndex, outBuff + readCount * ram->itemLen);
            readIndex++;
            readCount++;
        }

        return readCount;
    }

    /**
     * @brief 清空缓冲区数据（不会释放内存）
     *
     * @param ram 指向 RAM_INF 的指针
     */
    void Ram_Clear(RAM_INF *ram)
    {
        if (ram == NULL)
        {
            return;
        }
        ram->itemNum   = 0;
        ram->readBegin = 0;
    }

    /**
     * @brief 移除最早的 endIndex 个数据项（更新读指针）
     *
     * @param ram      指向 RAM_INF 的指针
     * @param endIndex 要移除的数据项个数
     */
    void Ram_Remove(RAM_INF *ram, const uint32_t endIndex)
    {
        if (ram == NULL)
        {
            return;
        }

        if (ram->itemNum <= endIndex)
        {
            Ram_Clear(ram);
        }
        else
        {
            ram->itemNum   = ram->itemNum - endIndex;
            ram->readBegin = (ram->readBegin + endIndex) % RamItemTotal(ram);
        }
    }

#ifdef __cplusplus
}
#endif