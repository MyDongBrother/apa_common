/**
 * @file Global_Config.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-17
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-17 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#ifndef CLOBAL_CONFIG_H
#define CLOBAL_CONFIG_H

#ifndef SW_OFF
#define SW_OFF (0)
#endif

#ifndef SW_ON
#define SW_ON (1)
#endif

#if defined(APA_PLATFORM_ARM) || defined(APA_PLATFORM_X86) || \
    defined(APA_PLATFORM_NDK) || defined(APA_PLATFORM_QNX)
#define APA_USE_PTHREAD 1
#elif defined(APA_PLATFORM_MCU)
#define APA_USE_PTHREAD 0
#else
#error "Unknown platform"
#endif

#endif
