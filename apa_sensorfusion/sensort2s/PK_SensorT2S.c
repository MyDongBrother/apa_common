/**
 * @file PK_SensorT2S.c
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
** Include common and project definition header
*********************************************************************************************************/
#include <stdlib.h>
#include <string.h>

/*********************************************************************************************************
** Include headers of the component
*********************************************************************************************************/
#include "PK_SensorT2S.h"
#include "T2S_List.h"
#include "Rte_Types.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "PK_Utility.h"
#include "PK_Calibration.h"
#include "Rte_Func.h"
#include "hobotlog/hobotlog.hpp"
#include "PK_StateManage.h"
#include "Rte_ComIF.h"

/*********************************************************************************************************
** Include other headers
*********************************************************************************************************/

/*********************************************************************************************************
** Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */
#if ((PK_SensorT2S_SW_MAJOR_VERSION != (0u)) || \
     (PK_SensorT2S_SW_MINOR_VERSION != (1u)) || (PK_SensorT2S_SW_PATCH_VERSION != (4U)))
#error "Versions of PK_SensorT2S.c and PK_SensorT2S.h are inconsistent!"
#endif

/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/

#pragma section all "CPU1.Private"

// 包含12个超声数据，8个交叉波数据和4个二次回波数据
static U_RadarDataType CurrUltrasData[USS_ECHO_INDEX_MAX];
static float CurrUltrasMotorPos[USS_ECHO_INDEX_MAX][4];

/*********************************************************************************************************
**Definition of static variables
*********************************************************************************************************/
static uint32 PrevUltrasTime[USS_ECHO_INDEX_MAX];
// static float  PrevUltrasMotorPos[20][4];

static void *listHandle = NULL;
static void interpolateUltrasMotorPos(const uint32 t, const uint32 tl, const uint32 th,
                                      const float SrcL[4], const float SrcH[4],
                                      float Dst[4]);
static void initUltrasMotorPos(void);
static void T2S_updateCCP(void);

#define T2S_DEBUG 0

void PK_SensorT2S_CurrUltrasData(U_RadarDataType ultrasData[USS_ECHO_INDEX_MAX])
{
    Rte_EnterCriticalSection();
    memcpy(ultrasData, CurrUltrasData, sizeof(CurrUltrasData));
    Rte_LeaveCriticalSection();
}

void PK_SensorT2S_SetCurrUltrasMotorPos(float ultrasMotorPos[USS_ECHO_INDEX_MAX][4])
{
    Rte_EnterCriticalSection();
    memcpy(CurrUltrasMotorPos, ultrasMotorPos, sizeof(CurrUltrasMotorPos));
    Rte_LeaveCriticalSection();
}

void PK_SensorT2S_CurrUltrasMotorPos(float ultrasMotorPos[USS_ECHO_INDEX_MAX][4])
{
    Rte_EnterCriticalSection();
    memcpy(ultrasMotorPos, CurrUltrasMotorPos, sizeof(CurrUltrasMotorPos));
    Rte_LeaveCriticalSection();
}

/*********************************************************************
 * 初始化长度为20的双向链表
 * *******************************************************************/
int PK_SensorT2S_CacheLocation(void)
{
    RD_PosStamp CurPos;
    PK_ModuleStateType ModuleState;
    static uint8 ResetCnt = 0;

    if (listHandle == NULL)
    {
        listHandle = T2S_CList_Init();
        RTE_PK_SensorT2S_Set_ModuleState_SensorT2S(MSTAT_DATCLC);
        if (NULL != listHandle)
        {
            initUltrasMotorPos();
        }
        else
        {
            return -1; // 链表初始化失败
        }
    }

    /* PK_SensorT2S_CacheLocation must follow PK_Location!!! */
    /* Only Cache the right location data.
     * If output wrong location, the following modules may get wrong!
     * But what if location module sends out wrong state?
     * Therefore, the following modules should not believe on all location data. */
    ModuleState                  = RTE_PK_Location_Get_ModuleState_Location();
    Apa_WorkStateType APA_Status = RTE_SM_Get_ApaWorkState();

    if (MSTAT_NORM == ModuleState &&
        (APA_Status == CarSta_Serching || APA_Status == CarSta_GuidanceActive ||
         APA_Status == CarSta_GuidanceAvpActive ||
         APA_Status == CarSta_ParkAssist_Standby))
    {
        /* Get the location info. */
        RTE_PK_Location_Get_CurPos_With_Timestamp(&CurPos);
        LOGD << "CurPos timestamp:" << CurPos.t << " CurPos postion: " << CurPos.pos[0]
             << ", " << CurPos.pos[1] << ", " << CurPos.pos[2] << ", " << CurPos.pos[3]
             << ", ";

        /* Add to the list */
        Rte_EnterCriticalSection();
        T2S_CList_NewIn(listHandle, &CurPos);
        Rte_LeaveCriticalSection();
        RTE_PK_SensorT2S_Set_ModuleState_SensorT2S(MSTAT_NORM);
    }
    else
    {
        /* Clear the cached data */
        Rte_EnterCriticalSection();
        T2S_Clist_Clear(listHandle);
        initUltrasMotorPos();
        Rte_LeaveCriticalSection();

        ResetCnt               = (0x07 == ResetCnt) ? 1 : (ResetCnt + 1);
        State_SensorT2S_CL_CCP = (ResetCnt << 4) | (ModuleState & 0x0F);
        RTE_PK_SensorT2S_Set_ModuleState_SensorT2S(MSTAT_DATCLC);
    }

    return 0;
}

int PK_SensorT2S(void)
{
    int LocCacheLen;
    RD_PosStamp *PosH; // high?
    RD_PosStamp *PosL; // low?
    uint32 t;
    int i;
    int ret = 0;

    if (listHandle == NULL)
    {
        RTE_PK_SensorT2S_Set_ModuleState_SensorT2S(MSTAT_OFF);
        ret = -1;
        goto NoLocMem;
    }

    /* Get the data from ultrasonic sensor */
    U_RadarDataType ultrasData[USS_ECHO_INDEX_MAX];
    RTE_PD_Get_U_RadarExt(ultrasData);
    LocCacheLen = T2S_Clist_GetLength(listHandle);
    if (0 == LocCacheLen)
    {
        RTE_PK_SensorT2S_Set_ModuleState_SensorT2S(MSTAT_OFF);
        ret = -1;
        goto NoLocMem;
    }

    /* Interpolate the location data with both timestamps */
    // ModuleState = MSTAT_NORM;

    /*该部分作用是将当前接收到的超声数据(CurrUltrasData)时间戳，根据链表数据,
    采用插值法计算出超声的运动位置(CurrUltrasMotorPos)
    */
    Rte_EnterCriticalSection();
    memcpy(CurrUltrasData, ultrasData, sizeof(CurrUltrasData));
    for (i = 0; i < USS_ECHO_INDEX_MAX; i++)
    {
        t = CurrUltrasData[i].Timestamp;
        if (t != PrevUltrasTime[i] || CurrUltrasMotorPos[i][3] < -9999.0f)
        { //<-9999.0说明尚未写入数据
            ret = T2S_CList_Search(listHandle, t, &PosL, &PosH);
            if (-1 == ret)
                break;

            /* 2^32/1000/60/60/24 = 49.71d, will the motor run so long? */
            interpolateUltrasMotorPos(t, PosL->t, PosH->t, PosL->pos, PosH->pos,
                                      CurrUltrasMotorPos[i]);
            PrevUltrasTime[i] = t;
        }
    }
    Rte_LeaveCriticalSection();

NoLocMem:
    /* Update the global variables */
    T2S_updateCCP();

    return ret;
}

static void interpolateUltrasMotorPos(const uint32 t, const uint32 tl, const uint32 th,
                                      const float SrcL[4], const float SrcH[4],
                                      float Dst[4])
{
    float a = 1.0f;
    if (tl == th)
    {
        memcpy(Dst, SrcH, 4 * sizeof(float));
    }
    else
    {
        a = (float)(t - tl) / (float)(th - tl);
        if (a < 0.0f)
            a = 0.0f;
        if (a > 1.0f)
            a = 1.0f;

        Dst[0] = SrcL[0] + a * (SrcH[0] - SrcL[0]);
        Dst[1] = SrcL[1] + a * (SrcH[1] - SrcL[1]);
        Dst[3] = SrcL[3] + a * (SrcH[3] - SrcL[3]);
        Dst[2] = Get_AverAngle_W(SrcH[2], SrcL[2], a, 1.0f - a);
    }

    return;
}

static void initUltrasMotorPos(void)
{
    memset(CurrUltrasMotorPos, 0, sizeof(CurrUltrasMotorPos));
    memset(PrevUltrasTime, 0, sizeof(PrevUltrasTime));

    for (int i = 0; i < USS_ECHO_INDEX_MAX; i++)
    {
        CurrUltrasMotorPos[i][3] = -10000.0f;
    }
}

static void T2S_updateCCP(void) { return; }
