/**
 * @file ThreadManager.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-27
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-27 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "ThreadManager_Api.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>

#define ASSERT(check, info)                                                       \
    {                                                                             \
        while (!(check))                                                          \
        {                                                                         \
            printf("ASSERT %s %d %s %s\r\n", __FILE__, __LINE__, __func__, info); \
            sleep(1); /* 每秒打印一次错误，避免刷爆日志 */                        \
        }                                                                         \
    }

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

// 构造函数利用pthread_once确保Rte初始化只执行一次
struct InitRteStub
{
    InitRteStub() { ASyncRun(ASynLoop, 0, "dbg_save_log"); }
};
// 静态全局对象，程序启动时自动调用构造函数完成初始化
static InitRteStub _rterunstub_;
