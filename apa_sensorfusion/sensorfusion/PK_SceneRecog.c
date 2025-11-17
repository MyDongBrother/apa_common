/**
 * @file PK_SceneRecog.c
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

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "PK_SF_Subfun.h"
#include "Rte_Types.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "SystemPara.h"

/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "PK_SceneRecog.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/

/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */
#if ((PK_SCENERECOG_SW_MAJOR_VERSION != (0u)) || \
     (PK_SCENERECOG_SW_MINOR_VERSION != (0u)) || \
     (PK_SCENERECOG_SW_PATCH_VERSION != (4u)))
#error "Version numbers of PK_SCENERECOG.c and PK_SCENERECOG.h are inconsistent!"
#endif

#pragma section all "CPU1.Private"
/*********************************************************************************************************
** Definition of exported function like macros
*********************************************************************************************************/

/* Named Constants */
#define b_PK_SR_V_FILTER             (0.0F)
#define b_SPEED_THRESH               (8.3F)
#define b_SLOW_DRIVING               ((uint8)1U)
#define b_CUR_BUFFER_FR_IDX          ((uint8)1U)
#define b_BUFFER_FR_SIZE             ((uint8)100U)
#define b_BUFFER_FR_ONE_THRESH       (0.8F)
#define b_FR_LASTS                   ((uint8)0U)
#define b_PK_SR_STEER_ANGLE_FILTER   (0.0F)
#define b_STEER_THRESH               (150.0F)
#define b_JUST_STEER                 ((uint8)0U)
#define b_AFTER_STEER                ((uint16)0U)
#define b_PK_SR_SUPERPARKING_ACTIVE  ((uint8)0U)
#define b_AFTER_STEER_THRESH         ((uint16)500U)
#define b_PK_SR_Parking_Switch_State ((uint8)255U)
#define b_PK_SR_Parking_Switch       ((uint8)1U)
#define b_AVM_SLOT_THRESH            ((uint8)3U)

/* Variable Definitions */
float PK_SR_V_FILTER;
float SPEED_THRESH;
uint8 SLOW_DRIVING;
static RD_OSTargACCType TargACC_OS_SR;
uint8 CUR_BUFFER_FR_IDX;
uint8 BUFFER_FR_SIZE;
uint8 BUFFER_FR[100];
float BUFFER_FR_ONE_THRESH;
uint8 FR_LASTS;
float PK_SR_STEER_ANGLE_FILTER;
float STEER_THRESH;
uint8 JUST_STEER;
uint16 AFTER_STEER;
uint8 PK_SR_SUPERPARKING_ACTIVE;
uint16 AFTER_STEER_THRESH;
uint8 PK_SR_Parking_Switch_State;
uint8 PK_SR_Parking_Switch;

AVM_Buff_Info AVM_BUFF_SR;
uint8 AVM_SLOT_NUM_SR;

/*********************************************************************************************************
** function definiton
*********************************************************************************************************/
static void PK_SR_Jam_Judge(void);
void PK_SR_Jam_Judge_initialize(void);

static void Get_AVM_Slot_Num(void);

/*
@note judge if the car is in the jam
*/
static void PK_SR_Jam_Judge(void)
{
    uint8 BUFFER_FR_one_length;
    uint8 fr_exist;
    uint8 args_f1[99];
    int k;
    int j;

    PK_SR_Parking_Switch = (uint8)1U;

    BUFFER_FR_one_length = (uint8)0U;

    if (PK_SR_V_FILTER > SPEED_THRESH)
    {
        SLOW_DRIVING = (uint8)0U;
    }
    else
    {
        SLOW_DRIVING = (uint8)1U;
    }

    if ((TargACC_OS_SR.OS_TargID_ACC != 255) && (TargACC_OS_SR.OS_TargDistX_ACC < 30.0F))
    {
        fr_exist = (uint8)1U;
    }
    else
    {
        fr_exist = (uint8)0U;
    }

    if (CUR_BUFFER_FR_IDX < BUFFER_FR_SIZE)
    {
        BUFFER_FR[CUR_BUFFER_FR_IDX - 1] = fr_exist;

        CUR_BUFFER_FR_IDX++;
    }
    else
    {
        memcpy(&args_f1[0], &BUFFER_FR[1], 99U * sizeof(uint8));
        k = 0;
        for (j = 0; j < 99; j++)
        {
            k++;
            BUFFER_FR[k - 1] = args_f1[j];
        }

        BUFFER_FR[k] = fr_exist;
    }

    k = BUFFER_FR_SIZE;
    for (j = 0; j < k; j++)
    {
        if (BUFFER_FR[j] != 0)
        {
            BUFFER_FR_one_length++;
        }
    }

    if ((float)BUFFER_FR_one_length / (float)BUFFER_FR_SIZE > BUFFER_FR_ONE_THRESH)
    {
        FR_LASTS = (uint8)1U;
    }
    else
    {
        FR_LASTS = (uint8)0U;
    }

    if (fabsf(PK_SR_STEER_ANGLE_FILTER) > STEER_THRESH)
    {
        JUST_STEER = (uint8)1U;

        AFTER_STEER = (uint16)0U;
    }

    if (JUST_STEER != 0)
    {
        AFTER_STEER++;

        if (AFTER_STEER > AFTER_STEER_THRESH)
        {
            JUST_STEER = 0U;
        }
    }

    PK_SR_Parking_Switch_State = SR_PK_SW_OFF_ACC;
    PK_SR_Parking_Switch       = (uint8)2U;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void PK_SR_Jam_Judge_initialize(void)
{
    TargACC_OS_SR.OS_TargID_ACC           = 0U;
    TargACC_OS_SR.OS_TargDistX_ACC        = 0.0F;
    TargACC_OS_SR.OS_TargDistY_ACC        = 0.0F;
    TargACC_OS_SR.OS_TargRelVelX_ACC      = 0.0F;
    TargACC_OS_SR.OS_TargRelVelY_ACC      = 0.0F;
    TargACC_OS_SR.OS_TargRelAX_ACC        = 0.0F;
    TargACC_OS_SR.OS_TargRelAY_ACC        = 0.0F;
    TargACC_OS_SR.OS_TargObstacleProb_ACC = 0.0F;
    TargACC_OS_SR.OS_TargExistProb_ACC    = 0.0F;
    TargACC_OS_SR.OS_TargAge_ACC          = 0.0F;
    TargACC_OS_SR.OS_Type_ACC             = 0.0F;
    TargACC_OS_SR.V_id                    = 0.0F;
    PK_SR_Parking_Switch                  = b_PK_SR_Parking_Switch;
    PK_SR_Parking_Switch_State            = b_PK_SR_Parking_Switch_State;
    AFTER_STEER_THRESH                    = b_AFTER_STEER_THRESH;
    PK_SR_SUPERPARKING_ACTIVE             = b_PK_SR_SUPERPARKING_ACTIVE;
    AFTER_STEER                           = b_AFTER_STEER;
    JUST_STEER                            = b_JUST_STEER;
    STEER_THRESH                          = b_STEER_THRESH;
    PK_SR_STEER_ANGLE_FILTER              = b_PK_SR_STEER_ANGLE_FILTER;
    FR_LASTS                              = b_FR_LASTS;
    BUFFER_FR_ONE_THRESH                  = b_BUFFER_FR_ONE_THRESH;
    memset(&BUFFER_FR[0], 0, 100U * sizeof(uint8));
    BUFFER_FR_SIZE    = b_BUFFER_FR_SIZE;
    CUR_BUFFER_FR_IDX = b_CUR_BUFFER_FR_IDX;
    SLOW_DRIVING      = b_SLOW_DRIVING;
    SPEED_THRESH      = b_SPEED_THRESH;
    PK_SR_V_FILTER    = b_PK_SR_V_FILTER;
}

static void Get_AVM_Slot_Num(void)
{
    uint8 num_temp = 0;
    RTE_PK_SF_Get_AVM_Buff_Left(&AVM_BUFF_SR);
    num_temp += AVM_BUFF_SR.Num;
    RTE_PK_SF_Get_AVM_Buff_Right(&AVM_BUFF_SR);
    num_temp += AVM_BUFF_SR.Num;
    RTE_PK_SF_Get_AVM_Invalid_Buff_Left(&AVM_BUFF_SR);
    num_temp += AVM_BUFF_SR.Num;
    RTE_PK_SF_Get_AVM_Invalid_Buff_Right(&AVM_BUFF_SR);
    num_temp += AVM_BUFF_SR.Num;
    AVM_SLOT_NUM_SR = num_temp;
}
