/**
 * @file Build_Config.h
 * @brief 编译配置宏定义，平台与版本宏
 *
 * @details
 * 本头文件提供默认平台与版本宏定义。
 * 优先使用 CMake 或编译器命令行传入的宏定义。
 * 如果未传入，则提供合理默认值。
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
#ifndef BUILD_CONFIG_H_
#define BUILD_CONFIG_H_

/**
 * @brief 平台宏定义
 * @details
 * 如果 CMake 或编译器命令行未指定平台宏，则默认定义为 APA_PLATFORM_X86。
 */
#if !defined(APA_PLATFORM_ARM) && !defined(APA_PLATFORM_X86) && \
    !defined(APA_PLATFORM_NDK) && !defined(APA_PLATFORM_QNX)
#define APA_PLATFORM_X86
#endif

/**
 * @brief 构建版本宏定义
 * @details
 * 如果 CMake 或编译器命令行未指定版本宏，则默认定义为 RELEASE_VERSION。
 */
#if !defined(DEBUG_VERSION) && !defined(RELEASE_VERSION)
#define RELEASE_VERSION
#endif

#endif /* BUILD_CONFIG_H_ */
