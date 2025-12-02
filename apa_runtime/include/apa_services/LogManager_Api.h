/**
 * @file LogManager_Api.h
 * @brief 日志管理模块接口定义。
 *
 * 提供统一日志输出接口，支持：
 * 1. 日志级别控制
 * 2. 文件/终端输出（通过内部实现）
 * 3. 断言及浮点数检查
 *
 * 日志使用宏封装，方便模块直接调用。
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

#ifndef LOG_MANAGER_H_
#define LOG_MANAGER_H_

#include <signal.h>
#include <math.h>
#include <cstdint>
#include <stdio.h>

/**
 * @enum LOG_LEVEL
 * @brief 日志级别枚举。
 *
 * 开发者可以根据日志严重性选择级别，用于过滤日志输出。
 *
 * 日志级别从低到高：
 * - LOG_INFO：普通信息，可用于调试和状态输出。
 * - LOG_WARNING：警告信息，表示潜在问题但不影响运行。
 * - LOG_ERROR：错误信息，表示运行异常，需要关注。
 * - LOG_FATAL：致命错误，可能导致程序中止。
 */
enum LOG_LEVEL
{
    LOG_INFO = 0, /**< 普通信息日志 */
    LOG_WARNING,  /**< 警告日志 */
    LOG_ERROR,    /**< 错误日志 */
    LOG_FATAL,    /**< 致命错误日志 */
    LOG_OFF       /**< 日志关闭 */
};

#ifdef RELEASE_VERSION
#define LOG_PRINTF_LEVEL LOG_OFF   // 发布版本不在终端打印日志
#define LOG_SAVE_LEVEL   LOG_FATAL // 发布版本只保存致命日志到文件
#define LOG_PATH_DEFAULT "./log/"  // 发布版本日志文件目录
#else
#define LOG_PRINTF_LEVEL LOG_INFO // 调试版本终端打印所有级别日志
#define LOG_SAVE_LEVEL   LOG_INFO // 调试版本保存所有级别日志
#define LOG_PATH_DEFAULT "./log/" // 调试版本日志文件目录
#endif

/**
 * @enum LOG_CHANNEL_E
 * @brief 日志通道枚举。
 *
 * 用于区分不同模块或用途的日志通道，保存不同的日志文件。
 * 下游应用可以选择不同通道输出不同模块日志。
 * 负值表示注册或输出错误。
 *
 * @note 通过
 */
enum LOG_CHANNEL_E
{
    LOGCHA_ERROR5  = -5, /**< 内存分配失败 */
    LOGCHA_ERROR4  = -4, /**< 无法创建文件路径 */
    LOGCHA_ERROR3  = -3, /**< 错误的通道 */
    LOGCHA_ERROR2  = -2, /**< 错误的时间戳 */
    LOGCHA_ERROR1  = -1, /**< 不存在的文件 */
    LOGCHA_DEFAULT = 0,  /**< 默认日志通道 */
    LOGCHA_RUNTIME = 1,  /**< runtime日志通道 */
    LOGCHA_2       = 2,  /**< 其他日志通道，自由分配 */
    LOGCHA_3       = 3,
    LOGCHA_4       = 4,
    LOGCHA_5       = 5,
    LOGCHA_6       = 6,
    LOGCHA_7       = 7,
    LOGCHA_8       = 8,
    LOGCHA_9       = 9,
    LOGCHA_10      = 10,
    LOGCHA_11      = 11,
    LOGCHA_12      = 12,
    LOGCHA_13      = 13,
    LOGCHA_14      = 14,
    LOGCHA_15      = 15,
    LOGCHA_MAX, /**< 日志通道最大值不可使用 */
};

/**
 * @brief 注册日志通道
 *
 * 下游应用在输出日志前需要先注册通道。
 *
 * @param cha 日志通道号
 * @param p_lv 日志打印级别，优先级低于 LOG_PRINTF_LEVEL
 * @param s_lv 日志保存级别，优先级低于 LOG_SAVE_LEVEL
 * @return 注册成功返回 cha，失败返回负值错误码
 *
 * 示例：
 * @code
 * LOG_REG(LOGCHA_2, LOG_INFO, LOG_FATAL); // 注册通道2，INFO级别，保存日志
 * @endcode
 */
LOG_CHANNEL_E LOG_Register(LOG_CHANNEL_E cha, LOG_LEVEL p_lv, LOG_LEVEL s_lv);
#define LOG_REG(cha, p_lv, s_lv) LOG_Register(cha, p_lv, s_lv)

/**
 * @brief 日志输出接口
 *
 * 核心日志输出函数，下游应用一般通过宏调用，无需直接调用。
 *
 * @param cha 日志通道
 * @param level 日志级别
 * @param file 调用日志的源文件名（宏会自动传入 __FILE__）
 * @param line 调用行号（宏会自动传入 __LINE__）
 * @param fmt printf风格格式化字符串
 * @param ... 可变参数，对应 fmt 的占位符
 *
 * @note：
 * 1. 根据通道和日志级别决定是否输出。
 * 2. 自动记录文件和行号，方便调试。
 * 3. 输出到终端，同时可保存到文件。
 * 4. 为兼容旧代码，之前代码统一使用 LOGCHA_DEFAULT通道。
 *
 * 示例：
 * @code
 * log_info(LOGCHA_2, "value=%d", value);
 * log_warn(LOGCHA_2, "threshold exceeded: %d", threshold);
 * log_err(LOGCHA_2, "processing failed for id=%d", id);
 * log_fatal(LOGCHA_2, "critical error occurred");
 * @endcode
 */
void LOG_Output(LOG_CHANNEL_E type, LOG_LEVEL level, const char *file, int line,
                const char *fmt, ...);

#define log_info(_fmt, _args...)                                                 \
    {                                                                            \
        LOG_Output(LOGCHA_DEFAULT, LOG_INFO, __FILE__, __LINE__, _fmt, ##_args); \
    }
#define LOG_INFO(cha, _fmt, _args...)                                 \
    {                                                                 \
        LOG_Output(cha, LOG_INFO, __FILE__, __LINE__, _fmt, ##_args); \
    }

#define log_warn(_fmt, _args...)                                                    \
    {                                                                               \
        LOG_Output(LOGCHA_DEFAULT, LOG_WARNING, __FILE__, __LINE__, _fmt, ##_args); \
    }
#define LOG_WARN(cha, _fmt, _args...)                                    \
    {                                                                    \
        LOG_Output(cha, LOG_WARNING, __FILE__, __LINE__, _fmt, ##_args); \
    }

#define log_err(_fmt, _args...)                                                   \
    {                                                                             \
        LOG_Output(LOGCHA_DEFAULT, LOG_ERROR, __FILE__, __LINE__, _fmt, ##_args); \
    }
#define LOG_ERR(cha, _fmt, _args...)                                   \
    {                                                                  \
        LOG_Output(cha, LOG_ERROR, __FILE__, __LINE__, _fmt, ##_args); \
    }

#define log_fatal(_fmt, _args...)                                                 \
    {                                                                             \
        LOG_Output(LOGCHA_DEFAULT, LOG_FATAL, __FILE__, __LINE__, _fmt, ##_args); \
    }
#define LOG_FATAL(cha, _fmt, _args...)                                 \
    {                                                                  \
        LOG_Output(cha, LOG_FATAL, __FILE__, __LINE__, _fmt, ##_args); \
    }

/**
 * @brief 条件断言，失败时输出日志并阻塞（可自定义实现）
 *
 * @param check 条件表达式
 * @param file  调用源文件
 * @param line  调用行号
 * @param info  附加信息，可为 NULL
 */
void LOG_ASSERT(bool check, const char *file, int line, const char *info = NULL);

/**
 * @brief 简单断言宏
 */
#define ASSERT(check)                            \
    {                                            \
        LOG_ASSERT((check), __FILE__, __LINE__); \
    }

/**
 * @brief 带附加信息的断言宏
 */
#define ASSERT_INFO(check, info)                       \
    {                                                  \
        LOG_ASSERT((check), __FILE__, __LINE__, info); \
    }

/**
 * @brief 浮点数检查宏
 *
 * 检查浮点数是否为 NaN 或 Infinite，如果是则触发断言。
 *
 * @param value 浮点数变量
 *
 * 示例：
 * @code
 * double x = sqrt(-1);
 * VERIFY_FLOAT(x); // 会触发断言，因为 x 是 NaN
 * @endcode
 */
#define VERIFY_FLOAT(value)                                                         \
    {                                                                               \
        if (fpclassify((value)) == FP_NAN || fpclassify((value)) == FP_INFINITE)    \
            LOG_ASSERT(false, __FILE__, __LINE__,                                   \
                       (fpclassify((value)) == FP_NAN) ? "FP_NAN" : "FP_INFINITE"); \
    }

#endif
