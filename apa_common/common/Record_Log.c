#include "Record_Log.h"
#include "Record_Buff.h"
#include "MathFunc.h"
#include "Rte_Func.h"
#include "Rte_BSW.h"
#include "Service_API.h"

const int log_buff_len     = 10 * 1024;
static void *log_inf       = nullptr;
static LOG_LEVEL log_level = LOG_WARNING;
static void Log_Init()
{
    static FILE *logFid = NULL;
    char fileName[128]  = {0};
    struct tm absStamp;

    RTE_BSW_Get_AbsStamp(&absStamp);
    if (absStamp.tm_year < 2000)
    {
        return;
    }
    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_error.txt",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    log_inf = nullptr;
    logFid  = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return;
    }

#ifndef ANDROID_PLATFORM
    auto exitFunc = [](int, void *) {
        static bool exitflag = false;
        if (exitflag)
        {
            return;
        }
        exitflag = true;
        if (logFid != NULL)
        {
            fclose(logFid);
        };
    };

    on_exit(exitFunc, NULL);
#endif

    static uint8_t log_buff[log_buff_len];
    log_inf = Buff_Init(log_buff, sizeof(log_buff), logFid);
    if (log_inf == NULL)
    {
        fclose(logFid);
        logFid = NULL;
    }
    else
    {
        struct timeval stamp;
        char notes[128];
        gettimeofday(&stamp, NULL);
        int noteslen = sprintf(notes, "%06ld:%06ld log int success! start...\n",
                               stamp.tv_sec, stamp.tv_usec);
        Buff_Put(log_inf, notes, noteslen);
    }
}

void LOG_Output(int cat, LOG_LEVEL level, const char *file, int line, const char *fmt,
                ...)
{
    struct timeval stamp;
    gettimeofday(&stamp, NULL);

    if (log_inf == nullptr)
    {
        Log_Init();
    }

    if (level < log_level)
    {
        return;
    }

    char printbuf[512];
    int start = 0;

    // 转换成本地时间
    struct tm tm_time;
    localtime_r(&stamp.tv_sec, &tm_time);

    // 文件名提取
    const char *filename = file ? strrchr(file, '/') : nullptr;
    filename             = filename ? filename + 1 : (file ? file : "null");

    // 格式化头部
    start =
        snprintf(&printbuf[start], sizeof(printbuf) - start,
                 "[%04d_%02d_%02d_%02d] [%s:%d] ", tm_time.tm_year + 1900,
                 tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour, filename, line);

    // 拼接日志正文
    va_list args;
    va_start(args, fmt);
    start += vsnprintf(&printbuf[start], sizeof(printbuf) - start - 2, fmt, args);
    va_end(args);

    // 补换行
    if (printbuf[start - 1] != '\n' && (size_t)start < sizeof(printbuf) - 2)
    {
        printbuf[start++] = '\n';
    }
    printbuf[start] = 0;

    // 输出
    printf("%s", printbuf);
    Buff_Put(log_inf, printbuf, start);
}

void LOG_ASSERT(bool check, const char *file, int line, const char *info)
{
    while (!check)
    {
        sleep(2);
        LOG_Output(1, LOG_INFO, file, line, "info: %s stop!",
                   (info == NULL) ? "NULL" : info);
    }
}

void LOG_SetLevel(int type, LOG_LEVEL level) { log_level = level; }

void ShowStack(void)
{
#ifndef ANDROID_PLATFORM
    const int BACKTRACE_SIZE = 16;
    void *buffer[BACKTRACE_SIZE];
    int stackNum = backtrace(buffer, BACKTRACE_SIZE);
    printf("[%s]:[%d] n = %d\n", __func__, __LINE__, stackNum);
    char **symbols = backtrace_symbols(buffer, stackNum);
    if (NULL == symbols)
    {
        perror("backtrace symbols");
        exit(0);
    }

    printf("[%s]:[%d]\n", __func__, __LINE__);
    for (int index = 0; index < stackNum; index++)
    {
        printf("%d: %s\n", index, symbols[index]);
    }

    free(symbols);
#endif
}
