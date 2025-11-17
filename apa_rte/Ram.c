/**
 * @file Ram.c
 * @brief 环形缓冲区（Ring Buffer）实现文件
 *        提供数据写入、读取、清除等基本操作，支持 FIFO 自动覆盖最早数据
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "Ram.h"
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

#ifdef TEST_RAM
typedef struct
{
    uint8_t v1;
    uint8_t v2;
    char v3[2];
} TEST_DAT_RAM;

int main(int argc, char *argv[])
{
    uint8_t buff[32];
    RAM_INF ramInf    = {buff, sizeof(TEST_DAT_RAM), sizeof(buff), 0, 0};
    TEST_DAT_RAM dat1 = {1, 1, "1"};
    TEST_DAT_RAM dat2 = {2, 2, "2"};
    TEST_DAT_RAM dat3 = {3, 3, "3"};
    TEST_DAT_RAM out[40];
    uint32_t num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num > 0)
    {
        printf("error! ead data is %d %d %d %d %c\r\n", __LINE__, num, out[0].v1,
               out[0].v2, out[0].v3[0]);
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat1, 1);
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);

    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat1, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat1, 1);
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 1, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 2, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 3, (uint8_t *)out, 1);
    if (num == 0)
    {
        printf("success!");
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat2, 1);
    num = Ram_Get(&ramInf, 3, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 2; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat2, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat2, 1);
    printf("put data len is %d, success is 6\r\n", Ram_Len(&ramInf));
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; read data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    printf("put data len is %d, success is 8\r\n", Ram_Len(&ramInf));
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    printf("put data len is %d, success is 8\r\n", Ram_Len(&ramInf));
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 4, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 2; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 7, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 3; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    printf("put data len is %d, success is 8\r\n", Ram_Len(&ramInf));

    num = Ram_Get(&ramInf, 1, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 2; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 3, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 3; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    num = Ram_Get(&ramInf, 7, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 3; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Clear(&ramInf);
    printf("Clear data len is %d, success is 0\r\n", Ram_Len(&ramInf));

    Ram_Put(&ramInf, -1, (uint8_t *)&dat1, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat2, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    printf("put data len is %d, success is 3\r\n", Ram_Len(&ramInf));

    num = Ram_Get(&ramInf, 2, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 3; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Remove(&ramInf, 0);
    printf("Remove 0, len is %d, success is 3\r\n", Ram_Len(&ramInf));
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 1; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Remove(&ramInf, 1);
    printf("Remove 0, len is %d, success is 2\r\n", Ram_Len(&ramInf));
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 2; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }

    Ram_Put(&ramInf, -1, (uint8_t *)&dat1, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat2, 1);
    Ram_Put(&ramInf, -1, (uint8_t *)&dat3, 1);
    printf("len is %d, success is 5\r\n", Ram_Len(&ramInf));
    num = Ram_Get(&ramInf, 0, (uint8_t *)out, 1);
    if (num == 1)
    {
        printf("success! num = 1, value = 2; ead data is %d %d %d %d %c\r\n", __LINE__,
               num, out[0].v1, out[0].v2, out[0].v3[0]);
    }
    else
    {
        printf("error! %d\r\n", __LINE__);
        return 1;
    }
    return 0;
}
#endif
