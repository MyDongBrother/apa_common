/**
 * @file Ram.h
 * @brief 环形缓冲区（Ring Buffer）实现接口定义
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
#ifndef RAM_HEADER_
#define RAM_HEADER_
#include "Std_Types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 固定长度的环形缓冲区结构体，用于存储定长数据项
     */
    typedef struct
    {
        uint8_t *data;      // 缓冲区起始指针，连续内存，长度为 buffLen 字节
        uint32_t itemLen;   // 每个数据项的长度（字节），即 data 中每块的单位大小
        uint32_t buffLen;   // 缓冲区总长度（字节），应为 itemLen 的整数倍
        uint32_t readBegin; // 当前读指针起始项的索引（单位：项）
        uint32_t itemNum;   // 当前已存储的数据项数量
    } RAM_INF;

    void Ram_Put(RAM_INF *ram, const int startIndex, const uint8_t *inBuff,
                 const uint32_t inCount);

    uint32 Ram_Len(RAM_INF *ram);

    uint32 Ram_Get(RAM_INF *ram, const int startIndex, uint8_t *outBuff,
                   const uint32_t outCount);

    void Ram_Clear(RAM_INF *ram);

    void Ram_Remove(RAM_INF *ram, const uint32_t endIndex);

#ifdef __cplusplus
}
#endif

#endif
