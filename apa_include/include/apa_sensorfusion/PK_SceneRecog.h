/**
 * @file PK_SceneRecog.h
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

#ifndef RD_SCENE_RECOG_H
#define RD_SCENE_RECOG_H

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "Std_Types.h"
#include "MathFunc.h"
#include "Rte.h"
#include "Module_API.h"
#include "Rte_Types.h"
#include "PK_Calibration.h"
#include "Rte_BSW.h"
#include "Module_API.h"

// #include "Rte_Com.h"
/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/

/** Version and module identification */
#define PK_SCENERECOG_VENDOR_ID (1u)
#define PK_SCENERECOG_MODULE_ID (34u)

/** Component Version Information */
#define PK_SCENERECOG_SW_MAJOR_VERSION (0u)
#define PK_SCENERECOG_SW_MINOR_VERSION (0u)
#define PK_SCENERECOG_SW_PATCH_VERSION (4u)

/*********************************************************************************************************
** Definition of exported function like macros
*********************************************************************************************************/
typedef enum
{
    SR_PK_SW_OFF_ACC  = 1,
    SR_PK_SW_ON_CMD   = 2,
    SR_PK_SW_ON_AVM   = 3,
    SR_PK_SW_ON_STR   = 4,
    SR_PK_SW_OFF_FAST = 5,
    SR_PK_SW_ON_NOFR  = 6,
    SR_PK_SW_OFF_FR   = 7
} SR_PK_SW_Status;

/*********************************************************************************************************
**End of check if informations are already included
*********************************************************************************************************/
#endif
/*********************************************************************************************************
**                                                                          End Of File:
*RD_ObjSelect.h
*********************************************************************************************************/