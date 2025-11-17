/**
 * @file dev_dds.c
 * @brief
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
#include "Rte_Func.h"
#include "CxxBridge.h"

/**
 * @brief
 *
 */

void Dds_InitThread()
{
    CxxThreadFunc func = GetProcessDdsPubFunc();
    if (func != NULL)
    {
        ASyncRun(func, (void *)NULL, "apa_dds_main");
    }
}
