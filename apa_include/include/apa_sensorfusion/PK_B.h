/**
 * @file PK_B.h
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

#ifndef PK_B_H
#define PK_B_H
#include "PK_Utility.h"
#include "SystemPara.h"
#include "Rte_Types.h"
#include "PK_Calibration.h"
#include "Rte_ComIF.h"

/** Component Version Information */
#define PK_B_SW_MAJOR_VERSION (0u)
#define PK_B_SW_MINOR_VERSION (0u)
#define PK_B_SW_PATCH_VERSION (35u)

#ifdef __cplusplus
extern "C"
{
#endif

    void PK_SF_B_Init(void);
    void PK_SF_B(void);
    void PK_SF_B_Clear(void);
    int PK_Get_Update_B_Flag();
    void PK_SF_B_End(void);

#ifdef __cplusplus
}
#endif
#endif // PK_B_H
