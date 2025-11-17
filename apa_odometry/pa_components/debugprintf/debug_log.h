/*this is the file "my_log.h"*/
#ifndef _MY_LOG_H_
#define _MY_LOG_H_

#include <string.h>
#include <sys/time.h>

#include "stdio.h"
#include "stdlib.h"
#include "time.h"

// Cut Other File
#define FILE_NAME(x) (strrchr(x, '/') ? strrchr(x, '/') + 1 : x)

/*[LOG_ABLE_EN]:enable log
**[LOG_ABLE_DIS]:disenable log
*/
#define LOG_ABLE_DIS 0
#define LOG_ABLE_EN  1
#define LOG_ABLE     LOG_ABLE_EN

/*[LOG_REDIRECT_CONSOLE]:log ourput to console
**[LOG_REDIRECT_FILE]:log output to "specific file".
**[LOG_FILE_PATH]:previous line mentions "specific file".
*/
#define LOG_REDIRECT_CONSOLE 0
#define LOG_REDIRECT_FILE    1
#define LOG_REDIRECT         LOG_REDIRECT_CONSOLE
#define LOG_FILE_PATH        "./log"

#define COLOR_END    "\033[0m"
#define COLOR_BLUE   "\033[34m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RED    "\033[31m"

#define COLOR_INF COLOR_BLUE
#define COLOR_DBG COLOR_BLUE
#define COLOR_WAR COLOR_YELLOW
#define COLOR_ERR COLOR_RED
#define COLOR_FER COLOR_RED

#ifndef MODULE_LOG_NAME
#define md_MODULENAME ("REVERSED")
#else
#define md_MODULENAME MODULE_LOG_NAME
#endif // ! md_SETMODULENAME

#if LOG_REDIRECT == LOG_REDIRECT_FILE
#define LOG_OUT_REDIRECT(log_type, f, ...)                                             \
    time_t time_re;                                                                    \
    time(&time_re);                                                                    \
    struct tm time_ex;                                                                 \
    localtime_r(&time_re, &time_ex);                                                   \
    char time_str[100] = {0};                                                          \
    sprintf(time_str, "%d-%d-%d-%d:%d:%d", time_ex.tm_year + 1900, time_ex.tm_mon + 1, \
            time_ex.tm_mday, time_ex.tm_hour, time_ex.tm_min, time_ex.tm_sec);         \
    FILE *file = fopen(LOG_FILE_PATH, "a");                                            \
    if (file == NULL)                                                                  \
        break;                                                                         \
    fprintf(file,                                                                      \
            "[" #log_type                                                              \
            "]-"                                                                       \
            "[%s]-"                                                                    \
            "[%s]:" f,                                                                 \
            time_str, __func__, ##__VA_ARGS__);                                        \
    fclose(file);

#else
#define LOG_OUT_REDIRECT(log_type, f, ...)                                               \
    struct timeval curr_time;                                                            \
    gettimeofday(&curr_time, NULL);                                                      \
    char time_str[100] = {0};                                                            \
    sprintf(time_str, "%lld", (curr_time.tv_sec * 1000LL + curr_time.tv_usec / 1000LL)); \
    printf(COLOR_##log_type "[" #log_type                                                \
                            "]-"                                                         \
                            "[%s]-"                                                      \
                            "[%s]-"                                                      \
                            "[%s,%d]:" f COLOR_END,                                      \
           md_MODULENAME, time_str, FILE_NAME(__FILE__), __LINE__, ##__VA_ARGS__);
#endif

#if LOG_ABLE == LOG_ABLE_EN
#define LOG(log_type, r, f, ...)                     \
    do                                               \
    {                                                \
        LOG_OUT_REDIRECT(log_type, f, ##__VA_ARGS__) \
        r;                                           \
    } while (0)
#else
#define LOG(log_type, r, f, ...)
#endif

#define INF(f, ...) LOG(INF, break, f, ##__VA_ARGS__)
#define DBG(f, ...) LOG(DBG, break, f, ##__VA_ARGS__)
#define WAR(f, ...) LOG(WAR, break, f, ##__VA_ARGS__)
#define ERR(f, ...) LOG(ERR, break, f, ##__VA_ARGS__)
#define FER(f, ...) LOG(FER, abort(), f, ##__VA_ARGS__)
//---------------------------------------------------------------
///// use for alg
/*
显示：0(默认)、1(粗体/高亮)、22(非粗体)、4(单条下划线)、24(无下划线)、5(闪烁)、25(无闪烁)、7(反显、翻转前景色和背景色)、27(无反显)
颜色值：0(黑)、1(红)、2(绿)、 3(黄)、4(蓝)、5(紫)、6(深绿)、7(白)
颜色分为背景色和字体色，30~39（30+颜色值）用来设置字体色（前景色），40~49（40+颜色值）设置背景（背景色）：如31表示前景色为红色，41表示背景色为红色。
*/
#define COLOR_NONE    "\033[0;m"
#define RED           "\033[0;31m"
#define LIGHT_RED     "\033[1;31m"   //红色高亮
#define LIGHT_RED_INV "\033[5;7;31m" //红色高亮，并反白显示，字体闪烁
#define GREEN         "\033[0;32m"
#define LIGHT_GREEN   "\033[1;32m"
#define BLUE          "\033[0;34m"
#define LIGHT_BLUE    "\033[1;34m" //蓝色高亮
#define DARY_GRAY     "\033[1;30m"
#define CYAN          "\033[0;36m"
#define LIGHT_CYAN    "\033[1;36m"
#define PURPLE        "\033[0;35m"
#define LIGHT_PURPLE  "\033[1;35m"
#define YELLOW        "\033[0;33m"
#define LIGHT_YELLOW  "\033[1;33m" //黄色高亮
#define WHITE         "\033[0;37m"
#define LIGHT_WHITE   "\033[1;37m" //白色高亮

#define LOGGER_FATAL_ALGLEVEL 1
#define LOGGER_ERROR_ALGLEVEL 2
#define LOGGER_WARN_ALGLEVEL  3
#define LOGGER_INFO_ALGLEVEL  4
#define LOGGER_DEBUG_ALGLEVEL 5

#define LOGGER_CFG_ALGLEVEL LOGGER_DEBUG_ALGLEVEL

#define logger_level_printf(level, fmt, arg...)                                        \
    do                                                                                 \
    {                                                                                  \
        if (level <= LOGGER_CFG_ALGLEVEL)                                              \
        {                                                                              \
            if (level == LOGGER_FATAL_ALGLEVEL)                                        \
            {                                                                          \
                printf("[%s %s %s:%d level=FATAL] " fmt, __DATE__, __TIME__, __func__, \
                       __LINE__, ##arg);                                               \
            }                                                                          \
            if (level == LOGGER_ERROR_ALGLEVEL)                                        \
            {                                                                          \
                printf("[%s %s %s:%d level=ERROR] " fmt, __DATE__, __TIME__, __func__, \
                       __LINE__, ##arg);                                               \
            }                                                                          \
            if (level == LOGGER_WARN_ALGLEVEL)                                         \
            {                                                                          \
                printf("[%s %s %s:%d level=WARN] " fmt, __DATE__, __TIME__, __func__,  \
                       __LINE__, ##arg);                                               \
            }                                                                          \
            if (level == LOGGER_INFO_ALGLEVEL)                                         \
            {                                                                          \
                printf("[%s %s %s:%d level=INFO] " fmt, __DATE__, __TIME__, __func__,  \
                       __LINE__, ##arg);                                               \
            }                                                                          \
            if (level == LOGGER_DEBUG_ALGLEVEL)                                        \
            {                                                                          \
                printf("[%s %s %s:%d level=DEBUG] " fmt, __DATE__, __TIME__, __func__, \
                       __LINE__, ##arg);                                               \
            }                                                                          \
        }                                                                              \
    } while (0)

#define FV_FATAL(fmt, arg...) \
    logger_level_printf(LOGGER_FATAL_ALGLEVEL, LIGHT_RED_INV fmt COLOR_NONE, ##arg)
#define FV_ERROR(fmt, arg...) \
    logger_level_printf(LOGGER_ERROR_ALGLEVEL, LIGHT_RED fmt COLOR_NONE, ##arg)
#define FV_WARN(fmt, arg...) \
    logger_level_printf(LOGGER_WARN_ALGLEVEL, LIGHT_YELLOW fmt COLOR_NONE, ##arg)
#define FV_INFO(fmt, arg...) \
    logger_level_printf(LOGGER_INFO_ALGLEVEL, LIGHT_BLUE fmt COLOR_NONE, ##arg)
#define FV_DEBUG(fmt, arg...) \
    logger_level_printf(LOGGER_DEBUG_ALGLEVEL, LIGHT_WHITE fmt COLOR_NONE, ##arg)

#endif