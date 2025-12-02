/**
 * @file TimeSync.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-29
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-29 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "TimeSync_Api.h"
#include "LogManager_Api.h"
#include <ctime>
#include <cstring>
#include <sys/time.h>

/*--------------------------------------------------------------------------*/
/*- 静态变量                                          -*/
/*--------------------------------------------------------------------------*/
static time_t rte_bsw_absstamp = 0; // 记录时间戳差值（基准时间戳）
static bool set_time_flag      = true;

/**
 * @brief 设置绝对时间戳
 * @param value 传入时间结构体（年/月/日等）
 * @note 计算传入时间与当前系统时间的秒差，保存为基准差值
 */
void RTE_BSW_Set_AbsStamp(struct tm *value)
{
    struct timeval tv;
    gettimeofday(&tv, (struct timezone *)0);

    struct tm stdtm;
    memcpy(&stdtm, value, sizeof(stdtm));
    stdtm.tm_year =
        stdtm.tm_year + 100;         // tm_year 从1900开始计数，这里+100表示从2000年开始？
    stdtm.tm_mon = stdtm.tm_mon - 1; // tm_mon 0-11，需要-1调整
    time_t nowt  = mktime(&stdtm);   // 转换成时间戳

    // 计算传入时间和当前时间的差值，保存到全局变量
    rte_bsw_absstamp = nowt - tv.tv_sec;

    if (set_time_flag && value->tm_sec != 0)
    {
        set_time_flag = false;

        struct timeval tv_set;
        tv_set.tv_sec  = nowt; // 使用 nowt 作为秒数
        tv_set.tv_usec = 0;    // 微秒设为 0

        if (settimeofday(&tv_set, NULL) < 0)
        {
            printf("settimeofday failed");
            return;
        }
        // 打印传入的时间戳（秒数）
        printf("Received time (timestamp): %ld\n", nowt);
        struct timeval tv_show;
        gettimeofday(&tv_show, NULL);
        struct tm *tm_info = localtime(&tv_show.tv_sec);
        char buffer[26];
        strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
        printf("Current time (formatted): %s\n", buffer);
    }
}

/**
 * @brief 获取绝对时间戳
 * @param value 输出参数，写入当前时间（已调整）
 * @note 通过基准差值计算当前绝对时间，并转换成struct tm格式返回
 */
void RTE_BSW_Get_AbsStamp(struct tm *value)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    time_t nowt      = rte_bsw_absstamp + tv.tv_sec; // 当前系统时间+差值
    struct tm *stdtm = localtime(&nowt);             // 转换为时间结构体

    memcpy(value, stdtm, sizeof(struct tm));
    value->tm_mon  = value->tm_mon + 1;     // 月份修正为1-12
    value->tm_year = value->tm_year + 1900; // 年份修正为完整年份
}

/**
 * @brief 获取绝对时间戳（毫秒级）
 * @return 当前绝对时间戳（从1970-01-01起的毫秒数）
 */
uint64_t RTE_BSW_Get_AbsStampMs()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);

    time_t sec       = rte_bsw_absstamp + tv.tv_sec;
    suseconds_t usec = tv.tv_usec;

    uint64_t absTimeMs = ((uint64_t)sec * 1000) + (usec / 1000);
    return absTimeMs;
}
