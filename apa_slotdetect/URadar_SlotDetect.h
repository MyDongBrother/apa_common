/**
 * @file URadar_SlotDetect.h
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

/** Multiple inclusion protection */
#ifndef U_RADAR_SLOT_DETECT_
#define U_RADAR_SLOT_DETECT_

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_SD_Common.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>

/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/
typedef enum
{
    FIND_SLOT_FIRST_VEHICLE_INIT,
    FIND_SLOT_FIRST_VEHICLE,
    FIND_SLOT_INDEPENDENT_OBJ_INIT,
    FIND_SLOT_INDEPENDENT_OBJ,
    FIND_SLOT_SECOND_VEHICLE_INIT,
    FIND_SLOT_SECOND_VEHICLE,
    FIND_SLOT_CONFIRM,

    FIND_CURB_OR_VEHICLE_INIT,
    FIND_CURB,
    FIND_VEHICLE,
    FIND_VEHICLE_CURB_SLOT,
    FIND_CURB_VEHICLE_SLOT
} FIND_SLOT_ST;

typedef enum
{
    SLOT_DIR_LEFT,
    SLOT_DIR_RIGHT
} SLOT_DIR;

/*********************************************************************************************************
** Declearation of exported variables
*********************************************************************************************************/
#define MAX_U_SLOT_SIDE_NUM 6 //	the max num of ultra radar slots at one side

/*********************************************************************************************************
** Declearation of exported functions
*********************************************************************************************************/
void Init_URadar_SlotDetect(void);
RECAL_DIR URadar_SlotDetect(void);

/*********************************************************************************************************
** Declearation of exported variables
*********************************************************************************************************/
extern SlotInfo_T g_Left_U_Slot_Array[MAX_U_SLOT_SIDE_NUM];
extern int g_Left_U_Slot_Num;
extern SlotInfo_T g_Right_U_Slot_Array[MAX_U_SLOT_SIDE_NUM];
extern int g_Right_U_Slot_Num;

#endif /* ifndef U_Radar_SlotDetect */
/*********************************************************************************************************
**         End Of File: U_Radar_SlotDetect.h
*********************************************************************************************************/
