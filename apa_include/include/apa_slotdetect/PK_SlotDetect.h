/**
 * @file PK_SlotDetect.h
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
#ifndef PK_SLOTDETECT_H_
#define PK_SLOTDETECT_H_

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "Rte.h"
#include "Platform_Types.h"
#include "PK_Calibration.h"
#include "Platform_Types.h"
#include "PK_Utility.h"

/** Version and module identification */
#define PK_SlotDetect_VENDOR_ID (1u)  // SCU
#define PK_SlotDetect_MODULE_ID (13u) // PK_SlotDetect

/** Component Version Information */
#define PK_SlotDetect_SW_MAJOR_VERSION (3u)
#define PK_SlotDetect_SW_MINOR_VERSION (2u)
#define PK_SlotDetect_SW_PATCH_VERSION (57u)

/*********************************************************************************************************
**Include headers of other components
*********************************************************************************************************/
#include <vector>
#include <netinet/in.h>
/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/
#define MULT_SLOT_SIDE_NUM_MAX \
    (MAX_PARKED_SLOTS / 2) // the max slot num at one side to park

#ifdef __cplusplus
extern "C"
{
#endif

    /*********************************************************************************************************
    ** Declaration of exported functions
    *********************************************************************************************************/
    void PK_SlotDetect(void);

// for offline test part
#ifdef PK_SlotDetect_OffLineTest

    extern SlotObj_T g_SlotObj_A;
    extern VehPos_T g_TargPos_A;
    extern int g_IsASlotUpata;
    extern SlotInfo_T
        RTE_Left_Multi_Slot_Array[MULT_SLOT_SIDE_NUM_MAX]; // all the left slot info
                                                           // buffers to send to RTE
    extern int RTE_left_multi_slot_num;
    extern SlotInfo_T
        RTE_Right_Multi_Slot_Array[MULT_SLOT_SIDE_NUM_MAX]; // all the right slot info
                                                            // buffers to send to RTE
    extern int RTE_right_multi_slot_num;

#endif

#ifdef __cplusplus
};
#endif

#endif