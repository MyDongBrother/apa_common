/**
 * @file Rte_Func.c
 * @brief RTE（Runtime Environment）模块功能实现
 *
 * 本文件实现了RTE模块的关键功能，包括：
 * - 线程异步任务池管理，支持任务异步添加与执行
 * - 线程创建接口，支持同步和异步线程启动
 * - 互斥锁递归初始化，保证线程安全的临界区控制
 * - 时间戳基准设置与获取，支持绝对时间计算
 * - 自定义断言宏，用于调试与错误捕获
 * *
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-09
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-10 <td>1.0      <td>jiandong.liu <td>first version
 * </table>
 */
#include "Rte_Func.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>

#ifdef DEBUG
// 在调试模式下，自定义断言宏 ASSERT，若 check 条件不成立，则打印断言信息并进入死循环。
// info 参数可用于描述断言失败的具体信息。
#define ASSERT(check, info)                                                       \
    {                                                                             \
        while (!(check))                                                          \
        {                                                                         \
            printf("ASSERT %s %d %s %s\r\n", __FILE__, __LINE__, __func__, info); \
            sleep(1); /* 每秒打印一次错误，避免刷爆日志 */                        \
        }                                                                         \
    }

#else
#define ASSERT(check, info) \
    {                       \
        assert(check);      \
    }

#endif

/**
 * @brief 异步运行线程函数，线程以分离态创建，执行完自动释放资源
 *
 * @param routine 线程入口函数指针，签名为 void* (*)(void*)
 * @param arg 传递给线程函数的参数指针
 * @param thread_name 线程名
 */
void ASyncRun(void *(*routine)(void *), void *arg, const char *thread_name)
{
    char info[128];
    int result = 0;
    pthread_t threadId;
    pthread_attr_t attr;

    // 初始化线程属性
    result = pthread_attr_init(&attr);
    if (result != 0)
    {
        snprintf(info, sizeof(info) - 1, "init attr, result %d, errno = %d, errinfo = %s",
                 result, errno, strerror(errno));
        ASSERT(false, info);
    }

    // 设置线程为分离态（创建后线程资源自动回收，无需pthread_join）
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    // 创建线程，启动执行 routine 函数，传入参数 arg
    result = pthread_create(&threadId, &attr, routine, arg);
    if (result != 0)
    {
        snprintf(info, sizeof(info) - 1, "create, result %d, errno = %d, errinfo = %s",
                 result, errno, strerror(errno));
        ASSERT(false, info);
    }
    else if (thread_name != NULL)
    {
        pthread_setname_np(threadId, thread_name);
    }

    // 销毁线程属性对象，释放资源
    result = pthread_attr_destroy(&attr);
    if (result != 0)
    {
        snprintf(info, sizeof(info) - 1,
                 "destroy attr, result %d, errno = %d, errinfo = %s", result, errno,
                 strerror(errno));
        ASSERT(false, info);
    }
}

/**
 * 同步创建一个线程并返回线程ID
 *
 * @param routine 线程入口函数指针
 * @param arg     传递给线程入口函数的参数
 * @param thread_name 线程名称，用于调试，长度一般限制16字节内
 * @return pthread_t 创建成功返回线程ID，失败返回0
 */
pthread_t SyncRun(void *(*routine)(void *), void *arg, const char *thread_name)
{
    char info[128];
    int result = 0;
    pthread_t threadId;
    result = pthread_create(&threadId, NULL, routine, arg);
    if (result != 0)
    {
        snprintf(info, sizeof(info) - 1, "create, result %d, errno = %d, errinfo = %s",
                 result, errno, strerror(errno));
        ASSERT(false, info);
        return 0;
    }
    if (thread_name != NULL)
    {
        pthread_setname_np(threadId, thread_name);
    }
    return threadId;
}

/**
 * @brief 函数信息结构体，包含回调函数和参数
 */
typedef struct
{
    void *(*callback)(void *); ///< 指向回调函数的指针，函数接收void*参数，返回void*
    void *paras;               ///< 传递给回调函数的参数指针
} FuncInf;

/**
 * @brief 线程任务循环函数，实现任务队列的异步执行
 *
 * @param args 传入任务信息指针（FuncInf结构体）
 * @return void* 任务执行结果，初始化阶段返回NULL，任务添加成功返回任务指针
 */
static void *ASynLoop(void *args)
{
    const int cacheRun               = 64;       // 最大缓存任务数
    static FuncInf runs[cacheRun]    = {{0, 0}}; // 任务数组，保存待执行任务
    static pthread_mutex_t run_mutex = PTHREAD_MUTEX_INITIALIZER; // 保护任务数组的互斥锁
    static sem_t runsem;                                          // 信号量，用于任务调度
    static int init = 0;                                          // 初始化标志

    FuncInf *input = (FuncInf *)args;
    FuncInf add    = {NULL, NULL};
    if (input != NULL)
    {
        // 复制输入任务信息
        add.callback = input->callback;
        add.paras    = input->paras;
    }
    if (init == 0)
    {
        // 第一次调用，初始化资源
        memset(runs, 0, sizeof(runs)); // 清空任务缓存区
        pthread_mutexattr_t rtemutexattr;
        pthread_mutexattr_init(&rtemutexattr); // 初始化互斥锁属性
        int result = pthread_mutexattr_settype(
            &rtemutexattr, PTHREAD_MUTEX_RECURSIVE_NP); // 设置递归锁属性
        ASSERT(result == 0, "pthread_mutexattr_settype");
        result = pthread_mutex_init(&run_mutex, &rtemutexattr); // 初始化互斥锁
        ASSERT(result == 0, "pthread_mutex_init");
        printf("Init rte mutex success\r\n");
        init = 1;
        sem_init(&runsem, 0, 0); // 初始化信号量，初始值0
    }
    else
    {
        // 添加任务流程
        void *result = NULL;
        pthread_mutex_lock(&run_mutex); // 加锁，防止并发修改任务数组
        for (int i = 0; i < cacheRun; i++)
        {
            if (runs[i].callback == 0 && runs[i].paras == 0) // 找空闲槽
            {
                result = args;
                memcpy(&runs[i], &add, sizeof(FuncInf)); // 复制任务信息到任务数组
                sem_post(&runsem);                       // 任务数+1，唤醒等待线程
                break;
            }
        }
        pthread_mutex_unlock(&run_mutex);

        if (result == NULL)
        {
            // printf("add one task failed! %p\n", add.callback);
        }

        return result;
    }
    // 任务执行循环
    while (1)
    {
        sem_wait(&runsem);
        for (int i = 0; i < cacheRun; i++)
        {
            FuncInf runinf{0, 0};
            pthread_mutex_lock(&run_mutex);
            if (runs[i].callback != 0)
            {
                memcpy(&runinf, &runs[i], sizeof(runinf)); // 复制任务，防止持锁执行任务
            }
            pthread_mutex_unlock(&run_mutex);

            if (runinf.callback)
            {
                runinf.callback(runinf.paras);
                // printf("complete one task:  %d %p\n", i, runinf.callback);
            }

            pthread_mutex_lock(&run_mutex);
            memset(&runs[i], 0, sizeof(runinf)); // 清空已执行任务槽
            pthread_mutex_unlock(&run_mutex);
        }
    }

    sem_destroy(&runsem); // 程序永远不会执行到这里，资源销毁
    return 0;
}

/**
 * @brief 向异步任务队列添加任务并异步执行
 *
 * @param routine 任务函数指针
 * @param arg 任务参数
 * @return int 返回0表示任务添加成功，1表示失败（队列满或异常）
 */
int ASyncPool(void *(*routine)(void *), void *arg)
{
    FuncInf para;
    para.callback = routine;
    para.paras    = arg;
    return (ASynLoop((void *)&para) != NULL) ? 0 : 1;
}

/**
 * @brief 等待指定线程结束
 *
 * @param threadId 线程ID
 */
void WaitSyncRun(const pthread_t threadId)
{
    if (threadId == 0)
    {
        return;
    }
    char info[128];
    int result = pthread_join(threadId, NULL);
    if (result != 0)
    {
        snprintf(info, sizeof(info) - 1, "join, result %d, errno = %d, errinfo = %s",
                 result, errno, strerror(errno));
        ASSERT(false, info);
    }
}

// 全局互斥锁，初始为静态初始化状态
static pthread_mutex_t rte_mutex = PTHREAD_MUTEX_INITIALIZER;
// 用于保证初始化函数只执行一次
static pthread_once_t rte_once = PTHREAD_ONCE_INIT;
/**
 * @brief RTE模块互斥锁初始化函数
 *        - 设置为递归锁，允许同一线程重复加锁
 *        - 初始化全局互斥锁
 *        - 启动异步任务处理线程（ASynLoop）
 */
static void RteInitMutex()
{
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
    int result = pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_RECURSIVE_NP);
    ASSERT(result == 0, "pthread_mutexattr_settype");
    result = pthread_mutex_init(&rte_mutex, &mutexattr);
    ASSERT(result == 0, "pthread_mutex_init");
    printf("Init rte mutex success\r\n");
    ASyncRun(ASynLoop, 0, "dbg_save_log");
}
// 构造函数利用pthread_once确保Rte初始化只执行一次
struct InitRteStub
{
    InitRteStub() { pthread_once(&rte_once, RteInitMutex); }
};
// 静态全局对象，程序启动时自动调用构造函数完成初始化
static InitRteStub _rterunstub_;

/**
 * @brief 进入临界区，确保线程安全
 *        - 通过pthread_once确保Rte初始化完成
 *        - 加锁互斥量保护共享资源
 */
void Rte_EnterCriticalSection()
{
    pthread_once(&rte_once, RteInitMutex);
    pthread_mutex_lock(&rte_mutex);
}

/**
 * @brief 离开临界区，释放互斥锁
 */
void Rte_LeaveCriticalSection() { pthread_mutex_unlock(&rte_mutex); }

// 静态变量，记录时间戳差值（基准时间戳）
static time_t rte_bsw_absstamp = 0;

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

    static bool set_time_flag = true;
    if (set_time_flag && value->tm_sec != 0)
    {
        set_time_flag = false;

        struct timeval tv_set;
        tv_set.tv_sec  = nowt; // 使用 nowt 作为秒数
        tv_set.tv_usec = 0;    // 微秒设为 0

        if (settimeofday(&tv_set, NULL) < 0)
        {
            perror("settimeofday failed");
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
