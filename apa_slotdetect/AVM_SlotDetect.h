/**
 * @file AVM_SlotDetect.h
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
#ifndef AVM_SLOT_DETECT_
#define AVM_SLOT_DETECT_

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Calibration.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>

/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/
#define AVM_SLOT_NUM_MAX 6 //  AVM Buffer size

/*********************************************************************************************************
** Declearation of exported functions
*********************************************************************************************************/
RECAL_DIR AVM_SlotDetect(void);
void Init_AVM_SlotDetect(void);

/*********************************************************************************************************
** Declearation of exported variables
*********************************************************************************************************/
extern SlotInfo_T g_Left_AVM_Slot_Array[AVM_SLOT_NUM_MAX];
extern int g_Left_AVM_Slot_Num;
extern SlotInfo_T g_Right_AVM_Slot_Array[AVM_SLOT_NUM_MAX];
extern int g_Right_AVM_Slot_Num;

#endif /* ifndef AVM_SlotDetect */
/*********************************************************************************************************
**         End Of File: AVM_SlotDetect.h
*********************************************************************************************************/