/**
 * @file DdsWorker.cpp
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
#include "Ram.h"
#include "NavPosFramePub.h"
#include "VisSoltFrameSub.h"
#include "CxxBridge.h"
#include <thread>

static void *ProcessDdsPub(void *paras)
{

    NavPosFramePub polygon_pub;
    VisSoltFrameSub solt_sub;
    if (polygon_pub.init())
    {
    }
    if (solt_sub.init())
    {
    }
    while (1)
    {
        polygon_pub.run();
        std::this_thread::sleep_for(std::chrono::milliseconds(19));
    }
}

/**
 * @brief 将cpp函数封装为C接口
 *
 */
CXX_BRIDGE_WRAP(ProcessDdsPub);
