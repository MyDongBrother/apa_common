/**
 * @file ThreadManager_Api.h
 * @brief 线程与任务管理模块接口定义
 *
 * 提供以下功能接口：
 *  - 异步线程运行（自动分离）
 *  - 同步线程运行（需 join）
 *  - 异步任务池提交（任务队列）
 *  - 临界区管理（进入/退出）
 *
 * @version 0.1
 * @date 2025-10-27
 * @author
 *   jiandong.liu (liujiandong@bm-intelligent.com)
 */

#ifndef THREAD_MANAGER_API_H
#define THREAD_MANAGER_API_H

#include <pthread.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 异步执行函数（分离态线程）
     * @param routine 线程函数
     * @param arg 参数
     * @param thread_name 线程名（可选，最长16字节）
     */
    void ASyncRun(void *(*routine)(void *), void *arg, const char *thread_name);

    /**
     * @brief 同步执行函数（可 join）
     * @param routine 线程函数
     * @param arg 参数
     * @param thread_name 线程名（可选）
     * @return 创建成功返回线程ID，失败返回0
     */
    pthread_t SyncRun(void *(*routine)(void *), void *arg, const char *thread_name);

    /**
     * @brief 等待线程结束
     * @param threadId 线程ID
     */
    void WaitSyncRun(pthread_t threadId);

    /**
     * @brief 异步任务池接口
     * @param routine 任务函数
     * @param arg 参数
     * @return 0 成功, 1 失败
     */
    int ASyncPool(void *(*routine)(void *), void *arg);

#ifdef __cplusplus
}
#endif

#endif // THREAD_MANAGER_API_H
