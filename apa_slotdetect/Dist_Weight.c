/**
 * @file Dist_Weight.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include <stdio.h>
#include <assert.h>
#include "Ethenet_AvmSlot.h"

// #define ENABLE_DIST_WEIGHT     // 取消注释以启用查找表功
#define DIST_TABLE_SIZE 3 // 定义查找表的大小

// 查找表的数据结构
typedef struct
{
    double minRange; // 范围的下限
    double maxRange; // 范围的上限
    double weight;   // 对应的权重
} DistWeightTableEntry;

// 查找表 ， 最好以 B E 两点的中心为查找基准最简单直接
DistWeightTableEntry weightTable[DIST_TABLE_SIZE] = {
    {0.0, 0.5, 15.0}, // 0到1米，权重为15
    {0.5, 1.0, 10.0}, // 1到2米，权重为10
    {1.0, 1.5, 5.0}   // 2到3米，权重为5
};
#define DEFAULT_DIST_WEIGHT 1.0 // 默认权重

// 根据查找表获取权重
float getDistWeight(float data)
{
#ifdef ENABLE_DIST_WEIGHT
    float value = fabs(data);
    for (int i = 0; i < DIST_TABLE_SIZE; i++)
    {
        if (value >= weightTable[i].minRange && value < weightTable[i].maxRange)
        {
            return weightTable[i].weight;
        }
    }
#endif
    return DEFAULT_DIST_WEIGHT; // 默认权重为1
}

// 测试 getWeight 函数
void testGetDistWeight()
{
    // 测试在范围内的值
    assert(getDistWeight(0.5) == 3.0); // 0到1米，权重为3
    assert(getDistWeight(1.5) == 2.0); // 1到2米，权重为2
    assert(getDistWeight(2.5) == 1.0); // 2到3米，权重为1

    // 测试超出范围的值
    assert(getDistWeight(-1.0) == DEFAULT_DIST_WEIGHT); // 负值超出范围，使用默认权重
    assert(getDistWeight(3.5) == DEFAULT_DIST_WEIGHT); // 超出定义的最大范围，使用默认权重
    printf("All tests passed!\n");
}
