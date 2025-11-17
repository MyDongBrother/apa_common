/**
 * @file PK_SensorT2S.h
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

#ifndef PK_SENSORT2S_H
#define PK_SENSORT2S_H

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "Platform_Types.h"
#include "Rte_Types.h"
#include "Rte_ComIF.h"

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/

/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/

/** Version and module identification */
#define PK_SensorT2S_VENDOR_ID (1u)
#define PK_SensorT2S_MODULE_ID (18u)

/** Component Version Information */
#define PK_SensorT2S_SW_MAJOR_VERSION (0u)
#define PK_SensorT2S_SW_MINOR_VERSION (1u)
#define PK_SensorT2S_SW_PATCH_VERSION (4u)

/*********************************************************************************************************
** Definition of exported function like macros
*********************************************************************************************************/
void PK_SensorT2S_CurrUltrasData(U_RadarDataType ultrasData[USS_ECHO_INDEX_MAX]);
void PK_SensorT2S_CurrUltrasMotorPos(float ultrasMotorPos[USS_ECHO_INDEX_MAX][4]);

/*********************************************************************************************************
** customized structures
*********************************************************************************************************/

int PK_SensorT2S(void);
int PK_SensorT2S_CacheLocation(void);

#endif /* ifndef PK_SENSORT2S_H */
/*********************************************************************************************************
** End Of File: PK_SensorT2S.h
*********************************************************************************************************/
