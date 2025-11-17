/**
 * @file UartIf.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-07-14
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-07-14 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */
#ifndef PK_UART_H_
#define PK_UART_H_
#include "Std_Types.h"

void Uart_PushCanData(const uint16_t *canId, const uint8_t data[][8],
                      const uint32_t number);

void Uart_InitRxThread();

#endif