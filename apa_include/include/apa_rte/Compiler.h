/**
 * @file Compiler.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-08-28
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-08-28 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#ifndef COMPILER_HEADER_
#define COMPILER_HEADER_

#include <stdint.h>

#ifndef boolean
#ifdef __cplusplus
#define boolean bool
#else
#define boolean _Bool
#endif
#endif

#ifndef uint32
#define uint32 uint32_t
#endif

#ifndef sint32
#define sint32 int32_t
#endif

#ifndef uint8
#define uint8 uint8_t
#endif

#ifndef uint16
#define uint16 uint16_t
#endif

#define FALSE 0
#define TRUE  1

#ifndef NULL
#define NULL 0
#endif

#endif
