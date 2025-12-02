/**
 * @file LogManager.c
 * @brief 日志管理模块实现。
 *
 * 本模块为通用日志组件的底层实现部分，提供：
 * - 按通道(Channel)管理独立的日志输出配置；
 * - 按日志级别决定是否输出到控制台、是否写入文件；
 * - 支持环形缓冲区，由独立线程异步写入日志；
 * - 支持自动按时间生成日志文件；
 * - 支持 backtrace 调用栈打印（若编译环境支持）；
 * - 支持模块注册，按需初始化缓存与文件。
 *
 * 下游应用无需直接调用本文件中的函数，
 * 应通过 LogManager_Api.h 提供的宏接口进行日志输出。
 */

#include "LogManager_Api.h"
#include "TimeSync_Api.h"
#include "Log_Buff.h"

#include <sys/time.h>
#include <time.h>
#include <cstring>
#include <stdarg.h>
#include <execinfo.h>
#include <unistd.h>
#include <cstdio>
#include <pthread.h>
#include <sys/stat.h>
#include <memory>

#define LOG_BUFF_SIZE (10 * 1024) /**< 单个日志缓冲区大小（10KB） */

/**
 * @brief 单个日志通道所维护的信息。
 *
 * 每个日志通道具有：
 * - p_lv：printf 输出的最低日志等级阈值；
 * - s_lv：保存文件输出的最低日志等级阈值；
 * - buff：日志的环形缓冲区，文件写入依赖此结构。
 */
typedef struct
{
    LOG_LEVEL p_lv;                   /**< printf 输出级别阈值 */
    LOG_LEVEL s_lv;                   /**< 文件保存级别阈值 */
    std::unique_ptr<Buff_Inf[]> buff; /**< 日志环形缓冲区 */
} LogInfo;

/**
 * @brief 日志管理器全局对象。
 *
 * - infos：按通道编号存储每个通道的 LogInfo；
 * - mutex：保护注册与输出的互斥锁，避免竞态。
 */
typedef struct
{
    LogInfo infos[LOGCHA_MAX]; /**< 所有日志通道的信息 */
    pthread_mutex_t mutex;     /**< 互斥锁 */
} LogManager;

static LogManager g_logManager = {}; /**< 全局日志管理器实例 */

/**
 * @brief 递归创建目录。
 *
 * @param path 目录路径，例如 "/data/log/app/"
 * @param mode 权限，如 0755
 * @return 0 表示成功，-1 表示失败
 *
 * 功能说明：
 * 逐层创建多级目录，例如：
 * "/data/log/app/" → 创建 /data → 创建 /data/log → 创建 /data/log/app
 */
static int MkdirRecursive(const char *path, mode_t mode)
{
    char tmp[256];
    strncpy(tmp, path, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    size_t len = strlen(tmp);
    if (len == 0)
        return -1;

    if (tmp[len - 1] == '/')
        tmp[len - 1] = '\0';

    for (char *p = tmp + 1; *p; ++p)
    {
        if (*p == '/')
        {
            *p = '\0';
            if (mkdir(tmp, mode) != 0 && errno != EEXIST)
                return -1;
            *p = '/';
        }
    }

    if (mkdir(tmp, mode) != 0 && errno != EEXIST)
        return -1;

    return 0;
}

/**
 * @brief 注册日志通道。
 *
 * @param cha   通道编号（由 LOG_CHANNEL_E 指定）
 * @param p_lv  printf 输出的最低日志级别
 * @param s_lv  文件保存的最低日志级别
 *
 * @return cha 表示成功；
 *         LOGCHA_ERROR3 等负值表示失败。
 *
 * 注册功能说明：
 * 1. 每个日志通道仅初始化一次；
 * 2. 若需要保存文件，创建对应的缓冲区及日志文件；
 * 3. 文件名包含时间戳与 PID，便于多进程区分；
 * 4. 若系统时间异常（如 RTC 未同步），拒绝创建文件；
 * 5. 注册失败时返回错误码，不会崩溃。
 *
 */
LOG_CHANNEL_E LOG_Register(LOG_CHANNEL_E cha, LOG_LEVEL p_lv, LOG_LEVEL s_lv)
{
    pthread_mutex_lock(&g_logManager.mutex);

    if (cha >= LOGCHA_MAX || cha < LOGCHA_DEFAULT)
    {
        pthread_mutex_unlock(&g_logManager.mutex);
        return LOGCHA_ERROR3;
    }

    /* 模块信息 */
    LogInfo *info = &g_logManager.infos[cha];

    if (info->buff.get() != nullptr)
    {
        pthread_mutex_unlock(&g_logManager.mutex);
        return cha;
    }

    /* 只有允许保存时初始化文件和缓冲区 */
    info->p_lv = p_lv;
    if (s_lv != LOG_OFF && LOG_SAVE_LEVEL != LOG_OFF)
    {
        if (MkdirRecursive(LOG_PATH_DEFAULT, 0755) != 0)
        {
            info->s_lv = s_lv;
            pthread_mutex_unlock(&g_logManager.mutex);
            return LOGCHA_ERROR4;
        }

        struct tm absStamp;
        RTE_BSW_Get_AbsStamp(&absStamp);
        if (absStamp.tm_year >= 2000)
        {
            info->buff            = std::make_unique<Buff_Inf[]>(LOG_BUFF_SIZE);
            char logFileName[128] = {0};
            snprintf(logFileName, sizeof(logFileName) - 1,
                     "%s/%02d%02d%02d%02d_%02d_%02d_CH%d_PID%d.txt", LOG_PATH_DEFAULT,
                     absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
                     absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec, cha, getpid());

            FILE *logFd = fopen(logFileName, "w");
            if (logFd != NULL)
            {
                if (Buff_Init((uint8_t *)info->buff.get(), LOG_BUFF_SIZE, logFd) ==
                    info->buff.get())
                {
                    info->s_lv = s_lv;
                    /* 写入初始化成功提示 */
                    struct timeval stamp;
                    gettimeofday(&stamp, NULL);
                    char notes[128];
                    int noteslen = snprintf(notes, sizeof(notes),
                                            "%06ld:%06ld log init success! start...\n",
                                            stamp.tv_sec, stamp.tv_usec);
                    Buff_Put((uint8_t *)info->buff.get(), notes, noteslen);
                }
                else
                {
                    pthread_mutex_unlock(&g_logManager.mutex);
                    return LOGCHA_ERROR5;
                }
            }
            else
            {
                pthread_mutex_unlock(&g_logManager.mutex);
                return LOGCHA_ERROR1;
            }
        }
        else
        {
            pthread_mutex_unlock(&g_logManager.mutex);
            return LOGCHA_ERROR2;
        }
    }

    pthread_mutex_unlock(&g_logManager.mutex);
    return cha;
}

/**
 * @brief 输出日志核心实现。
 *
 * @param cat   日志通道
 * @param level 日志级别（INFO/WARN/ERROR/FATAL）
 * @param file  调用日志的源文件
 * @param line  调用日志的行号
 * @param fmt   printf 格式化字符串
 * @param ...   变参，用于格式化输出
 *
 * 日志输出流程：
 * 1. 根据通道配置及全局宏判断是否允许打印或保存；
 * 2. 获取本地时间（localtime_r，不受时区线程安全问题影响）；
 * 3. 生成格式化后的整行日志文本：
 *    [YYYYMMDD HH:MM:SS] [filename:line] 内容...
 * 4. 若允许 printf，则直接输出到终端；
 * 5. 若允许保存，则写入缓冲区，等待异步写入文件。
 *
 * 注意：
 * 本函数应由 log_info/log_err 等宏调用，
 * 下游开发不应直接使用 LOG_Output，以保证统一格式与文件名。
 */
void LOG_Output(LOG_CHANNEL_E cat, LOG_LEVEL level, const char *file, int line,
                const char *fmt, ...)
{

    LogInfo *info  = &g_logManager.infos[cat];
    bool is_save   = (level < LOG_SAVE_LEVEL || level < info->s_lv) ? false : true;
    bool is_printf = (level < LOG_PRINTF_LEVEL || level < info->p_lv) ? false : true;
    if (false == is_save || false == is_printf)
    {
        return;
    }

    struct timeval stamp;
    gettimeofday(&stamp, NULL);

    char printbuf[512];
    int start = 0;

    /* 转换成本地时间格式 */
    struct tm tm_time;
    localtime_r(&stamp.tv_sec, &tm_time);

    /* 取文件名（去掉路径） */
    const char *filename = file ? strrchr(file, '/') : nullptr;
    filename             = filename ? filename + 1 : (file ? file : "null");

    /* 日志头部：时间 + 文件名 + 行号 */
    start = snprintf(&printbuf[start], sizeof(printbuf) - start,
                     "[%04d%02d%02d %02d:%02d:%02d] [%s:%d] ", tm_time.tm_year + 1900,
                     tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour, tm_time.tm_min,
                     tm_time.tm_sec, filename, line);

    /* 可变参数格式化正文 */
    va_list args;
    va_start(args, fmt);
    start += vsnprintf(&printbuf[start], sizeof(printbuf) - start - 2, fmt, args);
    va_end(args);

    /* 自动补换行符 */
    if (printbuf[start - 1] != '\n' && (size_t)start < sizeof(printbuf) - 2)
    {
        printbuf[start++] = '\n';
    }

    printbuf[start] = '\0';

    /* 输出到终端 */
    if (true == is_printf)
    {
        printf("%s", printbuf);
    }

    /* 保存到文件 */
    if (true == is_save)
    {
        Buff_Put(info->buff.get(), printbuf, start);
    }
}

/**
 * @brief 条件断言输出日志，并阻塞系统。
 *
 * @param check 条件表达式，为 false 则触发断言
 * @param file  源文件名
 * @param line  行号
 * @param info  附加信息，可选
 *
 * 功能说明：
 * 当 check 为 false 时：
 * - 每两秒输出一次致命日志；
 * - 日志通道固定为 LOGCHA_RUNTIME；
 * - 输出级别为 LOG_FATAL；
 *
 * 用于捕获不应出现的严重逻辑错误，例如：
 *   ASSERT(ptr != NULL)
 *
 * 若需要自定义行为，可由上层封装替换此函数。
 */
void LOG_ASSERT(bool check, const char *file, int line, const char *info)
{
    while (!check)
    {
        sleep(2);
        LOG_Output(LOGCHA_RUNTIME, LOG_FATAL, file, line, "info: %s stop!",
                   (info == NULL) ? "NULL" : info);
    }
}

/**
 * @brief 全局日志管理器初始化（静态构造函数）。
 *
 * 该结构体对象在程序启动时构造，执行以下操作：
 * 1. 初始化所有通道的输出级别为 LOG_OFF（关闭所有输出）；
 * 2. 注册 runtime 通道（用于输出运行时严重日志）；
 * 3. 注册默认通道（用于普通日志输出）；
 *
 * 设计说明：
 * - 使用 C++ 全局对象的构造行为，使日志系统开箱即用；
 * - 统一在程序加载阶段确保日志系统处于可用状态；
 * - 避免下游应用忘记初始化导致无法输出日志。
 */
struct InitLogManager
{
    InitLogManager()
    {
        {
            pthread_mutex_lock(&g_logManager.mutex);
            for (int i = 0; i < LOGCHA_MAX; i++)
            {
                g_logManager.infos[i].p_lv = LOG_OFF;
                g_logManager.infos[i].s_lv = LOG_OFF;
            }
            pthread_mutex_unlock(&g_logManager.mutex);
        }
        LOG_Register(LOGCHA_RUNTIME, LOG_WARNING, LOG_WARNING);
        LOG_Register(LOGCHA_DEFAULT, LOG_PRINTF_LEVEL, LOG_SAVE_LEVEL);
    }
};
static InitLogManager _runtime_log_;