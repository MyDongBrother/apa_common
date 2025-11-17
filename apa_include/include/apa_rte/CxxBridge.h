/**
 * @file CxxBridge.h
 * @brief
 *  * 使用方法：
 * 1. 在 C++ 源文件里写你的实现函数，例如：
 *      static void *MyWorker(void *arg) { ...; return NULL; }
 *
 * 2. 用 CXX_BRIDGE_WRAP 宏生成一个 C 接口函数：
 *      CXX_BRIDGE_WRAP(MyWorker);
 *
 * 3. 在 C 代码里，通过 GetMyWorkerFunc() 拿到函数指针，交给 pthread 或 ASyncRun。
 *
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

#ifndef CXX_BRIDGE_H
#define CXX_BRIDGE_H

#ifdef __cplusplus
extern "C"
{
#endif

    /// 标准回调类型：pthread_create / ASyncRun 通用
    typedef void *(*CxxThreadFunc)(void *arg);

    extern CxxThreadFunc GetProcessDdsPubFunc();
#ifdef __cplusplus
}
#endif

/// 宏：把一个 C++ 实现函数包装成 C 接口，并生成获取函数指针的入口
#define CXX_BRIDGE_WRAP(func)                                  \
    extern "C" void *func##_c(void *arg) { return func(arg); } \
    extern "C" CxxThreadFunc Get##func##Func() { return func##_c; }

#endif // CXX_BRIDGE_H
