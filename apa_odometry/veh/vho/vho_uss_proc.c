/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_uss_proc.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO

#include <pa_components/base_type/global_include.h>
#ifdef md_RUN_ON_POSIX
#include <pa_components/debugprintf/debug_log.h>
#include <sys/time.h>
#endif // md_RUN_ON_POSIX

#include <pa_components/mathlib/mathlib_api.h>

#include "vho_api.h"
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#include "vho_uss_proc.h"
// #include <uss/mp_if/mpif_api.h>

#include "vho.h"

#include <string.h>

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef LS_VHO_USSMAP_REAR_SENSOR_ACTIVE
#error ('compiler switch LS_VHO_USSMAP_REAR_SENSOR_ACTIVE is not defined')
#endif

#define md_WrapAroundConditionMS_si32 ((SInt32)32768)

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3892
#endif

VISIBILITY SInt32 m_correctWrapAround_si32(SInt32 f_Time_si32);

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         reset USS ring memory
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOUSSProcessingInit_vd(void)
{
    UInt8 l_I_ui8;

#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
    for (l_I_ui8 = 0; l_I_ui8 < 2; l_I_ui8++)
#else
    for (l_I_ui8 = 0; l_I_ui8 < 4; l_I_ui8++)
#endif
    {
        (void)memset(&g_VHOSensorDataUSS_pst[l_I_ui8], 0,
                     sizeof(gType_VHOSensorDataUSS_st));
        g_VHOSensorDataUSS_pst[l_I_ui8].SensorNumber_ui8 = 0xFF;
    }
    return;
    /*
      (void)
      memset(&g_VHOFirstSensorDataUSS_st,0,sizeof(gType_VHOSensorDataUSS_st));
      (void)
      memset(&g_VHOSecondSensorDataUSS_st,0,sizeof(gType_VHOSensorDataUSS_st));
      g_VHOFirstSensorDataUSS_st.SensorNumber_ui8 = 0xFF;
      g_VHOSecondSensorDataUSS_st.SensorNumber_ui8 = 0xFF;
      return;
    */
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief          get next dist list from FIFO queue
 *                 (causes subsequent deletion)
 * @details
 *
 * @param         f_SensorNumber_ui8
 * @param         f_DistList_ppui16
 * @param         f_StartTime_pui16
 * @param         f_DrivenDistSinceLastEcho_si16
 * @param         f_EstimatedPosition_pst
 * @return        UInt8                                0: call successful,
 * interpolated position, 1: call successful, extrapolated position (forward),
 *                                                     2: call successful,
 * extrapolated position (back), 3: uss data valid, but position estimation
 * failed, 10: FIFO queue for corresponding sensor is empty (no uss data
 * available) or position estimation not possible yet, 11: sensor number not
 * available in storage buffers
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_VHOUSSProcessingGetDistList_ui8(
    UInt8 f_SensorNumber_ui8, UInt16 **f_DistList_ppui16, UInt16 *f_StartTime_pui16,
    SInt16 *f_DrivenDistSinceLastEcho_si16,
    gType_VHOEstimatedPosition_st *f_EstimatedPosition_pst)
{

    UInt16 l_TargetCANTime_ui16;
    UInt8 l_RetVal_ui8 = 0xFF;
    gType_VHOSensorDataUSS_st *l_VHOSensorDataUSS_pst;
    static UInt16 l_TimeBufTCNT_ui16;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3353
#endif

    if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[0].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[0];
    }
    else if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[1].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[1];
    }
#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_ON)
    else if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[2].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[2];
    }
    else if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[3].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[3];
    }
#endif
    else
    {
        // sensor number not available in storage buffers
        l_RetVal_ui8 = 11;
    }

    if (l_RetVal_ui8 == 0xFF)
    {
        if (l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8 == 0)
        {
            (*f_DistList_ppui16) = NULL;

            // FIFO queue for corresponding sensor is empty
            l_RetVal_ui8 = 10;
        }
        else
        {
            // return oldest element
            l_TargetCANTime_ui16 = l_VHOSensorDataUSS_pst->TargetCANTime_pui16
                                       [l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8];

            //|   Results:
            //|     0: position estimation successful (interpolated)
            //|     1: position estimation successful (extrapolated - forward)
            //|     2: position estimation successful (extrapolated - back)
            //|     3: position estimation failed
            l_RetVal_ui8 = g_VHOUSSProcessingGetEstimatedPosition_ui8(
                f_EstimatedPosition_pst, l_TargetCANTime_ui16);

#if (VHO_USSPROC_EXTRAPOLATION_RESTRICTED == SW_ON)
            if (l_RetVal_ui8 == 1)
            {
                // result value is "extrapolation forward". for better precision the
                // data will not be provided until interpolation is possible
                l_RetVal_ui8 = 10;
            }
            else
#endif
            {

                if (l_RetVal_ui8 == 2)
                {
                    l_RetVal_ui8 = 0;
                }

                // read echoes of selected measurement
                (*f_DistList_ppui16) =
                    &l_VHOSensorDataUSS_pst
                         ->DistList_pui16[l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8]
                                         [0];

                // read start time of selected measurement
                (*f_StartTime_pui16) =
                    l_VHOSensorDataUSS_pst->StartTime_pui16
                        [l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8];

                // update index of the oldest element in FIFO queue
                l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8 =
                    (l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8 + 1) %
                    gd_VHOUSS_RM_SIZE_ui8;

                // decrement number of stored dist lists
                l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8--;

                if ((UInt16)g_mtl_Abs_mac(f_EstimatedPosition_pst->DrivenDistanceCM_si16 -
                                          l_VHOSensorDataUSS_pst->DrivenDistBuf_si16) <
                    (UInt16)15000)
                {
                    // no overflow occured => leave on previous value otherwise
                    l_VHOSensorDataUSS_pst->DrivenDistBetweenLastTwoEchoesCM_si16 =
                        f_EstimatedPosition_pst->DrivenDistanceCM_si16 -
                        l_VHOSensorDataUSS_pst->DrivenDistBuf_si16;

                    l_VHOSensorDataUSS_pst->DrivenDistUpdated_bl = TRUE;
                }

                (*f_DrivenDistSinceLastEcho_si16) =
                    l_VHOSensorDataUSS_pst->DrivenDistBetweenLastTwoEchoesCM_si16;

                l_VHOSensorDataUSS_pst->DrivenDistBuf_si16 =
                    f_EstimatedPosition_pst->DrivenDistanceCM_si16;

                l_VHOSensorDataUSS_pst->TimeSinceLastEchoMS_ui16 =
                    l_TargetCANTime_ui16 - l_VHOSensorDataUSS_pst->TimeBuf_ui16;
                l_VHOSensorDataUSS_pst->TimeSinceLastEchoTCNT_ui16 =
                    (*f_StartTime_pui16) - l_TimeBufTCNT_ui16;
                l_VHOSensorDataUSS_pst->TimeBuf_ui16 = l_TargetCANTime_ui16;
                l_TimeBufTCNT_ui16                   = (*f_StartTime_pui16);
            }
        }
    }
    return l_RetVal_ui8;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3353
#endif
}

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3206
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         storage of uss distance lists in ring memory
 *                (works as FIFO queue)
 * @details
 *
 * @param         f_SensorNumber_ui8
 * @param         fc_EchoData_pst
 * @param         f_USSPeakListTime_ui16
 * @param         f_CurrentUSSTime_ui16
 * @param         f_CurrentCANTime_ui16
 * @return        UInt8                    0: distance list successfully stored,
 *                                         1: distance list successfully stored,
 * but oldest element had to be removed due to free memory in FIFO queue 2: uss
 * timestamp already present in ring memory, 3: data was not stored because of
 * invalid data
 *
 * @note
 * @see
 * @warning
 */
#if 0
UInt8 g_VHOUSSProcessingStoreDistList_ui8(UInt8 f_SensorNumber_ui8,
                                          const gType_adiEchoDataSet_st *fc_EchoData_pst,
                                          UInt16 f_USSPeakListTime_ui16,
                                          UInt16 f_CurrentUSSTime_ui16,
                                          UInt16 f_CurrentCANTime_ui16)
{
    UInt8 l_RetVal_ui8;
    UInt8 l_I_ui8;

    gType_VHOSensorDataUSS_st *l_VHOSensorDataUSS_pst;

    l_RetVal_ui8 = 0;

    if (g_VHOSensorDataUSS_pst[0].SensorNumber_ui8 == (UInt8)0xFF)
    {
        g_VHOSensorDataUSS_pst[0].SensorNumber_ui8 = f_SensorNumber_ui8;
    }
    else if ((g_VHOSensorDataUSS_pst[1].SensorNumber_ui8 == (UInt8)0xFF) &&
             (f_SensorNumber_ui8 != g_VHOSensorDataUSS_pst[0].SensorNumber_ui8))
    {
        g_VHOSensorDataUSS_pst[1].SensorNumber_ui8 = f_SensorNumber_ui8;
    }
#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_ON)
    else if ((g_VHOSensorDataUSS_pst[2].SensorNumber_ui8 == 0xFF) &&
             (f_SensorNumber_ui8 != g_VHOSensorDataUSS_pst[0].SensorNumber_ui8) &&
             (f_SensorNumber_ui8 != g_VHOSensorDataUSS_pst[1].SensorNumber_ui8))
    {
        g_VHOSensorDataUSS_pst[2].SensorNumber_ui8 = f_SensorNumber_ui8;
    }
    else if ((g_VHOSensorDataUSS_pst[3].SensorNumber_ui8 == 0xFF) &&
             (f_SensorNumber_ui8 != g_VHOSensorDataUSS_pst[0].SensorNumber_ui8) &&
             (f_SensorNumber_ui8 != g_VHOSensorDataUSS_pst[1].SensorNumber_ui8) &&
             (f_SensorNumber_ui8 != g_VHOSensorDataUSS_pst[2].SensorNumber_ui8))
    {
        g_VHOSensorDataUSS_pst[3].SensorNumber_ui8 = f_SensorNumber_ui8;
    }
#endif
    else
    {
        // QAC
    }

    if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[0].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[0];
    }
    else if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[1].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[1];
    }
#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_ON)
    else if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[2].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[2];
    }
    else if (f_SensorNumber_ui8 == g_VHOSensorDataUSS_pst[3].SensorNumber_ui8)
    {
        l_VHOSensorDataUSS_pst = &g_VHOSensorDataUSS_pst[3];
    }
#endif
    else
    {
        l_VHOSensorDataUSS_pst = NULL;
    }

    if (l_VHOSensorDataUSS_pst != NULL)
    {
        if (l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8 == 0)
        {
            // FIFO queue is empty => store uss data
            for (l_I_ui8 = 0; l_I_ui8 < g_parGetParaVHO_NO_ECHOES_ui8; l_I_ui8++)
            {
                l_VHOSensorDataUSS_pst
                    ->DistList_pui16[l_VHOSensorDataUSS_pst->NextStorageIndex_ui8]
                                    [l_I_ui8] =
                    fc_EchoData_pst->adiEchoDistance_ui16[l_I_ui8] / (UInt16)10;
            }

            l_VHOSensorDataUSS_pst
                ->StartTime_pui16[l_VHOSensorDataUSS_pst->NextStorageIndex_ui8] =
                f_USSPeakListTime_ui16;

            // time base is identical to the time base of the wic signals, mapping is
            // not needed
            l_VHOSensorDataUSS_pst
                ->TargetCANTime_pui16[l_VHOSensorDataUSS_pst->NextStorageIndex_ui8] =
                f_USSPeakListTime_ui16;

            l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8 =
                l_VHOSensorDataUSS_pst->NextStorageIndex_ui8;

            l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8++;

            l_VHOSensorDataUSS_pst->NextStorageIndex_ui8 =
                (l_VHOSensorDataUSS_pst->NextStorageIndex_ui8 + 1) %
                gd_VHOUSS_RM_SIZE_ui8;
        }
        else
        {
            for (l_I_ui8 = 0; l_I_ui8 < gd_VHOUSS_RM_SIZE_ui8; l_I_ui8++)
            {
                // check if timestamp already exists
                if (l_VHOSensorDataUSS_pst->StartTime_pui16[l_I_ui8] ==
                    f_USSPeakListTime_ui16)
                {
                    l_RetVal_ui8 = 2;
                }
            }

            if (l_RetVal_ui8 != 2)
            {
                if (l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8 ==
                    gd_VHOUSS_RM_SIZE_ui8)
                {
                    // ring memory occupied, free memory by removal of the oldest element
                    l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8 =
                        (l_VHOSensorDataUSS_pst->IndexOldestDistList_ui8 + 1) %
                        gd_VHOUSS_RM_SIZE_ui8;
                    l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8--;
                    g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
                    l_RetVal_ui8 = 1;
                }
                else
                {
                    l_RetVal_ui8 = 0;
                }

                // store uss dist list in ring memory
                for (l_I_ui8 = 0; l_I_ui8 < g_parGetParaVHO_NO_ECHOES_ui8; l_I_ui8++)
                {
                    l_VHOSensorDataUSS_pst
                        ->DistList_pui16[l_VHOSensorDataUSS_pst->NextStorageIndex_ui8]
                                        [l_I_ui8] =
                        fc_EchoData_pst->adiEchoDistance_ui16[l_I_ui8] / (UInt16)10;
                }

                l_VHOSensorDataUSS_pst
                    ->StartTime_pui16[l_VHOSensorDataUSS_pst->NextStorageIndex_ui8] =
                    f_USSPeakListTime_ui16;

                // time base is identical to the time base of the wic signals, mapping
                // is not needed
                l_VHOSensorDataUSS_pst
                    ->TargetCANTime_pui16[l_VHOSensorDataUSS_pst->NextStorageIndex_ui8] =
                    f_USSPeakListTime_ui16;

                l_VHOSensorDataUSS_pst->NumStoredDistLists_ui8++;

                l_VHOSensorDataUSS_pst->NextStorageIndex_ui8 =
                    (l_VHOSensorDataUSS_pst->NextStorageIndex_ui8 + 1) %
                    gd_VHOUSS_RM_SIZE_ui8;
            }
        }
    }
    else
    {
        l_RetVal_ui8 = 3;
    }

    return l_RetVal_ui8;
}
#endif
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3206
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         estimate position for a given uss timestamp by interpolation
 *                or extrapolation
 * @details
 *
 * @param         f_EstimatedPosition_pst
 * @param         f_TargetCANTime_ui16
 * @return        UInt8                      0: position estimation successfull
 * (interpolated), 1: position estimation successfull (extrapolated - forward),
 *                                           2: position estimation successfull
 * (extrapolated - back), 3: position estimation failed
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_VHOUSSProcessingGetEstimatedPosition_ui8(
    gType_VHOEstimatedPosition_st *f_EstimatedPosition_pst, UInt16 f_TargetCANTime_ui16)
{
    UInt16 l_DeltaTimeMaxMin_ui16;
    SInt32 l_DeltaTimeTargetMin_si32;
    SInt32 l_DeltaTimeTargetMax_si32;
    UInt8 l_RetVal_ui8;
    UInt8 l_IdNewest_ui8;
    UInt8 l_IdOldest_ui8;

    gType_VHOPosBuffer_st l_VHOPosNew_st;
    gType_VHOPosBuffer_st l_VHOPosOld_st;

    // enter critical section to avoid parameter inconsistance
    //(void)g_DisableInterrupt_mac();

    // copy global vho output values to local structure to handle
    // race conditions (g_VHOWICHistoryUSS_st can be modified by another
    // task)

    l_IdNewest_ui8 = g_VHOWICHistoryUSS_st.IdNewest_ui8;
    l_IdOldest_ui8 = g_VHOWICHistoryUSS_st.IdOldest_ui8;

    l_VHOPosNew_st.XPositionMM_si32 =
        g_VHOWICHistoryUSS_st.XPositionMM_psi32[l_IdNewest_ui8];
    l_VHOPosNew_st.YPositionMM_si32 =
        g_VHOWICHistoryUSS_st.YPositionMM_psi32[l_IdNewest_ui8];
    l_VHOPosNew_st.DrivenDistanceMM_si32 =
        g_VHOWICHistoryUSS_st.DrivenDistanceMM_psi32[l_IdNewest_ui8];
    l_VHOPosNew_st.YawAngleRAD_F22_ui32 =
        g_VHOWICHistoryUSS_st.YawAngleRad_F22_pui32[l_IdNewest_ui8];
    l_VHOPosNew_st.WIC_CanTime_ui16 =
        g_VHOWICHistoryUSS_st.WIC_CanTime_pui16[l_IdNewest_ui8];

    l_VHOPosOld_st.XPositionMM_si32 =
        g_VHOWICHistoryUSS_st.XPositionMM_psi32[l_IdOldest_ui8];
    l_VHOPosOld_st.YPositionMM_si32 =
        g_VHOWICHistoryUSS_st.YPositionMM_psi32[l_IdOldest_ui8];
    l_VHOPosOld_st.DrivenDistanceMM_si32 =
        g_VHOWICHistoryUSS_st.DrivenDistanceMM_psi32[l_IdOldest_ui8];
    l_VHOPosOld_st.YawAngleRAD_F22_ui32 =
        g_VHOWICHistoryUSS_st.YawAngleRad_F22_pui32[l_IdOldest_ui8];
    l_VHOPosOld_st.WIC_CanTime_ui16 =
        g_VHOWICHistoryUSS_st.WIC_CanTime_pui16[l_IdOldest_ui8];

    // exit critical section
    //(void)g_EnableInterrupt_mac();

    // calculate time difference between timestamps of the last
    // two calculated positions
    if (l_VHOPosOld_st.WIC_CanTime_ui16 != l_VHOPosNew_st.WIC_CanTime_ui16)
    {
        // compensate time overflow of wic can time
        // o = WIC_CAN_TIME
        // x = TIME_LAST_WIC_INCREMENT

        // TIME_DELTA = WIC_CAN_TIME - TIME_LAST_WIC_INCREMENT
        //
        //         /
        //        o
        //       /
        //      x
        //     /
        //    /
        //   /
        //  /
        // /

        // TIME_DELTA = WIC_CAN_TIME + (MAX_VALUE - TIME_LAST_WIC_INCREMENT)
        //
        //         /|
        //        / |
        //       /  |
        //      x   |
        //     /    |
        //    /     |   o
        //   /      |  /
        //  /       | /
        // /        |/
        if (l_VHOPosOld_st.WIC_CanTime_ui16 <= l_VHOPosNew_st.WIC_CanTime_ui16)
        {
            l_DeltaTimeMaxMin_ui16 =
                l_VHOPosNew_st.WIC_CanTime_ui16 - l_VHOPosOld_st.WIC_CanTime_ui16;
        }
        else
        {
            l_DeltaTimeMaxMin_ui16 =
                l_VHOPosNew_st.WIC_CanTime_ui16 +
                (UInt16)((UInt32)65536 - (UInt32)l_VHOPosOld_st.WIC_CanTime_ui16);
        }

        // default: position estimation failed
        l_RetVal_ui8 = 3;

        if (l_DeltaTimeMaxMin_ui16 > md_MaxDeltaTimePositionMS_ui16)
        {
            // time difference between two positions too long
            g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
        }
        else
        {
            // calculate time between first and second can timestamp and the target
            // can time for which a position has to be estimated. the target can time
            // can lie I:   between the two timestamps => interpolated position II:
            // before both timestamps     => extrapolated position (past) III: after
            // both timestamps      => extrapolated position (future)
            //
            // I                    t_can_pos1     T_CAN_TARGET           t_can_pos2
            //                      |              |                      |
            //                      xxxxxxxxxxxxxxxx l_DeltaTimeTargetMin (-)
            //                                     xxxxxxxxxxxxxxxxxxxxxxxx
            //                                     l_DeltaTimeTargetMax (+)
            //
            //
            // II                   T_CAN_TARGET        t_can_pos1        t_can_pos2
            //                      |                   |                 |
            //                      xxxxxxxxxxxxxxxxxxxxx l_DeltaTimeTargetMin (-)
            //                      xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
            //                      l_DeltaTimeTargetMax (-)
            //
            //
            // III                  t_can_pos1             t_can_pos2 T_CAN_TARGET
            //                      |                      |               |
            //                      xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
            //                      l_DeltaTimeTargetMin (+)
            //                                             xxxxxxxxxxxxxxxxx
            //                                             l_DeltaTimeTargetMax (+)
            l_DeltaTimeTargetMin_si32 = m_correctWrapAround_si32(
                ((SInt32)f_TargetCANTime_ui16 - (SInt32)l_VHOPosOld_st.WIC_CanTime_ui16));
            l_DeltaTimeTargetMax_si32 = m_correctWrapAround_si32(
                ((SInt32)f_TargetCANTime_ui16 - (SInt32)l_VHOPosNew_st.WIC_CanTime_ui16));

            if ((l_DeltaTimeTargetMin_si32 >= 0) && (l_DeltaTimeTargetMax_si32 <= 0))
            {
                // calculate interpolated position
                l_RetVal_ui8 = 0;
            }
            else if ((l_DeltaTimeTargetMin_si32 < 0) && (l_DeltaTimeTargetMax_si32 < 0))
            {
                if (l_DeltaTimeTargetMin_si32 < l_DeltaTimeTargetMax_si32)
                {
                    g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
                }
                else if (l_DeltaTimeTargetMin_si32 > (-md_MaxToleranceMS_si32))
                {
                    // calculated extrapolated position (back)
                    l_RetVal_ui8 = 2;
                }
                else
                {
                    l_RetVal_ui8 = 3;
                }
            }
            else if ((l_DeltaTimeTargetMax_si32 > 0) && (l_DeltaTimeTargetMin_si32 > 0))
            {
                if (l_DeltaTimeTargetMin_si32 < l_DeltaTimeTargetMax_si32)
                {
                    g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
                }
                else if (l_DeltaTimeTargetMax_si32 < md_MaxToleranceMS_si32)
                {
                    // calculated extrapolated position (forward)
                    l_RetVal_ui8 = 1;
                }
                else
                {
                    l_RetVal_ui8 = 3;
                }
            }
            else
            {
                g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
            }

            if (l_RetVal_ui8 < 3)
            {
                SInt32 l_XEst_si32;
                SInt32 l_YEst_si32;
                SInt32 l_PsiOldRadF10_si32;
                SInt32 l_PsiNewRadF10_si32;
                SInt32 l_PsiEstRadF10_si32;
                SInt32 l_SEst_si32;

                // with a maximum value for l_DeltaTimeTargetMin_si32 of 300ms,
                // an overflow will occur in case that the difference between two
                // consecutive values of x,y,s will exceed ~28m => definitely not
                // possible

                l_XEst_si32 = g_mtl_s32_Div_s32_si32(
                    (l_VHOPosNew_st.XPositionMM_si32 - l_VHOPosOld_st.XPositionMM_si32) *
                        l_DeltaTimeTargetMin_si32,
                    (SInt32)l_DeltaTimeMaxMin_ui16);

                l_YEst_si32 = g_mtl_s32_Div_s32_si32(
                    (l_VHOPosNew_st.YPositionMM_si32 - l_VHOPosOld_st.YPositionMM_si32) *
                        l_DeltaTimeTargetMin_si32,
                    (SInt32)l_DeltaTimeMaxMin_ui16);

                l_PsiOldRadF10_si32 = (SInt32)g_mtl_ReducePrecision_ui32(
                    l_VHOPosOld_st.YawAngleRAD_F22_ui32, (UInt8)12);

                l_PsiNewRadF10_si32 = g_mtl_ReducePrecision_si32(
                    ((SInt32)l_VHOPosNew_st.YawAngleRAD_F22_ui32), (UInt8)12);

                if ((l_PsiOldRadF10_si32 < 1000) && (l_PsiNewRadF10_si32 > 5500))
                {
                    // wrap around occured in psi, psi decreased
                    l_PsiEstRadF10_si32 =
                        ((SInt32)6434 - l_PsiNewRadF10_si32) + l_PsiOldRadF10_si32;

                    l_PsiEstRadF10_si32 *= ((SInt32)-1);
                    // l_PsiEstRadF10_si32 = ((l_PsiNewRadF10_si32 - l_PsiOldRadF10_si32)
                    // * l_DeltaTimeTargetMin_si32) / (SInt32) l_DeltaTimeMaxMin_ui16;
                    if (l_PsiEstRadF10_si32 > 0)
                    {
                        g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
                    }
                }
                else if ((l_PsiOldRadF10_si32 > 5500) && (l_PsiNewRadF10_si32 < 1000))
                {
                    // wrap around occured in psi, psi increased
                    l_PsiEstRadF10_si32 =
                        ((SInt32)6434 - l_PsiOldRadF10_si32) + l_PsiNewRadF10_si32;

                    if (l_PsiEstRadF10_si32 < 0)
                    {
                        g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
                    }
                }
                else
                {
                    // no wrap around
                    l_PsiEstRadF10_si32 = l_PsiNewRadF10_si32 - l_PsiOldRadF10_si32;
                }

                l_PsiEstRadF10_si32 = g_mtl_s32_Div_s32_si32(
                    l_PsiEstRadF10_si32 * l_DeltaTimeTargetMin_si32,
                    (SInt32)l_DeltaTimeMaxMin_ui16);

                l_SEst_si32 =
                    g_mtl_s32_Div_s32_si32((l_VHOPosNew_st.DrivenDistanceMM_si32 -
                                            l_VHOPosOld_st.DrivenDistanceMM_si32) *
                                               l_DeltaTimeTargetMin_si32,
                                           (SInt32)l_DeltaTimeMaxMin_ui16);

                l_XEst_si32         = l_VHOPosOld_st.XPositionMM_si32 + l_XEst_si32;
                l_YEst_si32         = l_VHOPosOld_st.YPositionMM_si32 + l_YEst_si32;
                l_PsiEstRadF10_si32 = l_PsiOldRadF10_si32 + l_PsiEstRadF10_si32;
                l_SEst_si32         = l_VHOPosOld_st.DrivenDistanceMM_si32 + l_SEst_si32;

                l_XEst_si32 = g_mtl_s32_Div_s32_si32(l_XEst_si32, (SInt32)10);
                l_YEst_si32 = g_mtl_s32_Div_s32_si32(l_YEst_si32, (SInt32)10);
                l_SEst_si32 = g_mtl_s32_Div_s32_si32(l_SEst_si32, (SInt32)10);

                // convert output driven distance, shall be in a range
                // of 0..32767cm
                if (l_SEst_si32 >= 0)
                {
                    l_SEst_si32 = (SInt32)(((UInt32)l_SEst_si32) & (UInt32)0x7FFF);
                }
                else
                {
                    l_SEst_si32 = (SInt32)(((UInt32)0x8000 -
                                            (((UInt32)(-l_SEst_si32)) & (UInt32)0x7FFF)) &
                                           (UInt32)0x7FFF);
                }

                if (l_PsiEstRadF10_si32 > (SInt32)6434)
                {
                    l_PsiEstRadF10_si32 -= (SInt32)6434;
                }
                else if (l_PsiEstRadF10_si32 < (SInt32)0)
                {
                    l_PsiEstRadF10_si32 += (SInt32)6434;
                }
                else
                {
                    // do nothing, but be consistent with coding rules
                }

                if ((l_PsiEstRadF10_si32 > 6434) || (l_PsiEstRadF10_si32 < 0))
                {
                    g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
                }

                f_EstimatedPosition_pst->XPositionCM_si16 = (SInt16)l_XEst_si32;
                f_EstimatedPosition_pst->YPositionCM_si16 = (SInt16)l_YEst_si32;

                // value is always positive, abs function is called to avoid
                // qac error
                f_EstimatedPosition_pst->YawAngleRAD_F10_ui16 =
                    (UInt16)g_mtl_Abs_mac(l_PsiEstRadF10_si32);

                f_EstimatedPosition_pst->DrivenDistanceCM_si16 = (SInt16)l_SEst_si32;
            }
        }
    }
    else
    {
        g_vhoLogHdl_vd(VHOLog_USSProc_enm, g_vhoLogStateOn_enm);
        l_RetVal_ui8 = 3;
    }

    return l_RetVal_ui8;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         used to carry out wrap around correction on time value
 * @details
 *
 * @param         f_Time_si32
 * @return        SInt32
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY SInt32 m_correctWrapAround_si32(SInt32 f_Time_si32)
{
    SInt32 l_RetVal_si32;

    if (f_Time_si32 > md_WrapAroundConditionMS_si32)
    {
        l_RetVal_si32 = (f_Time_si32 - (SInt32)0x10000);
    }
    else if (f_Time_si32 < (-md_WrapAroundConditionMS_si32))
    {
        l_RetVal_si32 = (f_Time_si32 + (SInt32)0x10000);
    }
    else
    {
        l_RetVal_si32 = f_Time_si32;
    }
    return l_RetVal_si32;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         store vehicle position (used for interpolation)
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOUSSProcessingBufferPosition_vd(void)
{
    // buffer last position
    g_VHOPosBuffer_st.XPositionMM_si32      = g_VHOVehicleState_st.XPosition_si32;
    g_VHOPosBuffer_st.YPositionMM_si32      = g_VHOVehicleState_st.YPosition_si32;
    g_VHOPosBuffer_st.DrivenDistanceMM_si32 = g_VHOVehicleState_st.DrivenDistance_si32;
    g_VHOPosBuffer_st.YawAngleRAD_F22_ui32  = g_VHOVehicleState_st.YawAngle_ui32;
    g_VHOPosBuffer_st.WIC_CanTime_ui16      = g_VHOOdoBuf_st.WIC_CanTime_buf_ui16;
}

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3892
#endif

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
