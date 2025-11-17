/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_mini.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#define COMPONENT_VHO

/*
  suppress QAC warning message
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 861
#endif

#include "vho_api.h"
#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>

// #include <swcpa_include.h>
// #include <swcpa_api.h>
// #include <vhm_api.h>          // SWCPA Include Structure: ok
// #include <mathlib_api.h>      // SWCPA Include Structure: ok
#include "vho.h"
#include "vho_dir_recog.h" // SWCPA Include Structure: ok
#include "vho_mini.h"
#include "vho_odo_core.h" // SWCPA Include Structure: ok

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 861
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif
#ifndef GS_VHO_RUN_MODE
#error ('compiler switch GS_VHO_RUN_MODE is not defined')
#endif
#ifndef GS_VHO_CONSIDER_USS_DATA
#error ('compiler switch GS_VHO_CONSIDER_USS_DATA is not defined')
#endif
#ifndef GS_VHO_MINI_VHO
#error ('compiler switch GS_VHO_MINI_VHO is not defined')
#endif

#if (GS_VHS_WICRL != SW_ON)
#error ('GS_VHS_WICRL has to be set to ON if VHO is compiled')
#endif

#if (GS_VHS_WICRR != SW_ON)
#error ('GS_VHS_WICRR has to be set to ON if VHO is compiled')
#endif

/*----------------------------------------------------------------*/
/*- local symbolic constants                                     -*/
/*----------------------------------------------------------------*/
#ifndef g_parGetParaVHO_Veh_FacSteerTransmission_ui16
#define g_parGetParaVHO_Veh_FacSteerTransmission_ui16 (UInt16)6000
#endif

UInt16 g_TimeStampOSEK_MiniVHO_tt;
UInt16 g_LastTime_MiniVHO_ui16;
UInt16 g_ElapsedTimeMS_ui16;

/*----------------------------------------------------------------*/
/*- definition of exported functions                             -*/
/*----------------------------------------------------------------*/

VISIBILITY void g_VHOGetKappa_MiniVHO_si16(void)
{
#if (GS_VHS_SWA == SW_ON)

    SInt32 l_Temp_si32;

    // convert 0.04� to RAD F12
    //          1             pi             2^12
    // input * -- [1�] --> * --- [RAD] --> * ---- [F12] = 2.859547446
    //         25            180               1
    //
    // overflow protection: max(SWA[�]) = 720� --> 18000 [0.04�]
    //                      max(SInt32) = 2^31
    //                      max(ScalingVal) = 2^31 / 18000 = 119304
    //                      --> SWA * a / 10^x --> a < 119304 --> a = 28595 -->
    //                      10^4
    //                      --> ScalingVal  = 2.859547446 * 100000 = 285954.74 ~
    //                      285955
    l_Temp_si32 = g_mtl_s32_Div_s32_si32(g_VHOCanSig_st.SWA_si16 * (SInt32)28595,
                                         (SInt32)10000); // steering wheel angle F12
    l_Temp_si32 = g_mtl_s32_Div_s32_si32(
        l_Temp_si32 * (SInt32)g_parGetParaVHO_Veh_FacSteerTransmission_ui16,
        (SInt32)100000); // virtual steering angle F12

    g_VHOVehicleState_st.SAngRadF12_si16 = (SInt16)l_Temp_si32;

    // transform the result steering angle to the
    // curvature at the center of the rear axle
    g_VHOVehicleState_st.KappaF14_si16 =
        g_VHOGetCurvatureFromSteeringAngle_si16((SInt16)l_Temp_si32);

#endif
}

VISIBILITY UInt32 g_VHOGetDrivenDist_MiniVHO_ui32(void)
{
    UInt32 l_DrivenDistanceRearF8_ui32;

#if ((GS_VHS_WICRL == SW_ON) && (GS_VHS_WICRR == SW_ON))

    // caluclation of g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm]

    if (g_VHOCanSig_st.WICRaw_pui16[RightRear_enm] >
        g_VHOOdoBuf_st.WICBuf_pui16[RightRear_enm])
    {
        // wic has been incremented without wrap around
        g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] =
            g_VHOCanSig_st.WICRaw_pui16[RightRear_enm] -
            g_VHOOdoBuf_st.WICBuf_pui16[RightRear_enm];
    }
    else if (g_VHOCanSig_st.WICRaw_pui16[RightRear_enm] <
             g_VHOOdoBuf_st.WICBuf_pui16[RightRear_enm])
    {
        // wic has been incremented, wrap around occured
        g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] =
            ((UInt16)(1) + g_VHOCanSig_st.WICRaw_pui16[RightRear_enm] +
             g_parGetParaVHO_Veh_MaxWICValue_ui16) -
            g_VHOOdoBuf_st.WICBuf_pui16[RightRear_enm];
    }
    else
    {
        // wic value did not change, set delta to zero
        g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] = (UInt16)0;
    }

    // caluclation of g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm]

    if (g_VHOCanSig_st.WICRaw_pui16[LeftRear_enm] >
        g_VHOOdoBuf_st.WICBuf_pui16[LeftRear_enm])
    {
        // wic has been incremented without wrap around
        g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] =
            g_VHOCanSig_st.WICRaw_pui16[LeftRear_enm] -
            g_VHOOdoBuf_st.WICBuf_pui16[LeftRear_enm];
    }
    else if (g_VHOCanSig_st.WICRaw_pui16[LeftRear_enm] <
             g_VHOOdoBuf_st.WICBuf_pui16[LeftRear_enm])
    {
        // wic has been incremented, wrap around occured
        g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] =
            ((UInt16)(1) + g_VHOCanSig_st.WICRaw_pui16[LeftRear_enm] +
             g_parGetParaVHO_Veh_MaxWICValue_ui16) -
            g_VHOOdoBuf_st.WICBuf_pui16[LeftRear_enm];
    }
    else
    {
        // wic value did not change, set delta to zero
        g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] = (UInt16)0;
    }

    g_VHOOdoBuf_st.WICBuf_pui16[RightRear_enm] =
        g_VHOCanSig_st.WICRaw_pui16[RightRear_enm];
    g_VHOOdoBuf_st.WICBuf_pui16[LeftRear_enm] = g_VHOCanSig_st.WICRaw_pui16[LeftRear_enm];

    l_DrivenDistanceRearF8_ui32 = (g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] *
                                   g_parGetParaVHO_Veh_WICLength_ui16) +
                                  (g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] *
                                   g_parGetParaVHO_Veh_WICLength_ui16);

    l_DrivenDistanceRearF8_ui32 =
        g_mtl_ReducePrecision_ui32(l_DrivenDistanceRearF8_ui32, (UInt8)1); //[mm] -> F8

#elif (GS_VHS_VEH_SPEED == SW_ON)

    // 0.1km/h * [ms] --> 1/256mm    Time: [ms]    Velo: [0.1km/h]
    //
    //   Velo           Time
    // -------- [m/s] * ---- [s] * 256 * 1000 [1/256mm]
    // 10 * 3.6         1000
    l_DrivenDistanceRearF8_ui32 = g_mtl_u32_Div_u32_ui32(
        (UInt32)g_VHOCanSig_st.Vel_ui16 * (UInt32)g_ElapsedTimeMS_ui16 * (UInt32)256000,
        (UInt32)36000);

#else
#error ('required input signal(s) to calculate driven distance not enabled')
#endif

    return l_DrivenDistanceRearF8_ui32;
}

VISIBILITY void g_VHOCalcModelDepentPosition_MiniVHO_vd(void)
{

    mType_DeltaValues_st g_DeltaValues_st;

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
        VHO_Direction_Wic_enm)
#endif
    {
        if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_UNKNOWN_enm)
        {
            // rolling direction is unknown; this state is allowed up to a defined
            // driven distance
            g_VHORollRecog_st.ds_buf_MMF8_ui32 +=
                (UInt32)g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;

            // determine incremental changes in x-position, y-position and psi for the
            // assumption of driving forward and update buffers
            m_VHOPosCalcGetPosChanges_vd(
                &g_DeltaValues_st, g_VHOOdoBuf_st.DrivenDistanceMMF8_si32,
                m_VHONormAngleRadF22_ui32((SInt32)g_VHOVehicleState_st.YawAngle_ui32 +
                                          g_VHORollRecog_st.YawAngle_buf_si32));

            g_VHORollRecog_st.dx_buf_MMF8_si32 += g_DeltaValues_st.XF8_mm_si32;

            g_VHORollRecog_st.dy_buf_MMF8_si32 += g_DeltaValues_st.YF8_mm_si32;

            g_VHORollRecog_st.YawAngle_buf_si32 += g_DeltaValues_st.PsiF22_rad_si32;
        }
        else
        {
            // ROLLING DIRECTION IS KNOWN
            // (MOV_FORW, MOV_BACKW, MOV_TURNED_FORW_enm or MOV_TURNED_BACKW_enm)
            if (g_VHORollRecog_st.ds_buf_MMF8_ui32 > (UInt32)0)
            {
                // previous state was VHO_MOV_UNKNOWN_enm and distance has been buffered
                // => update position
                if ((g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm) ||
                    (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_TURNED_FORW_enm))
                {
                    // hypothesis that vehicle was driving forward was wrong
                    // => transformation of x, y, s and phi
                    m_VHOOdoCoreCoordTransform_vd(
                        &g_VHORollRecog_st, &g_DeltaValues_st,
                        (SInt16)g_mtl_ReducePrecision_ui32(
                            g_VHOVehicleState_st.YawAngle_ui32, 12));

                    g_VHOVehicleState_st.DrivenDistance_si32 -=
                        (SInt32)(g_VHORollRecog_st.ds_buf_MMF8_ui32 >> 8);

                    g_DeltaValues_st.PsiF22_rad_si32 =
                        -g_VHORollRecog_st.YawAngle_buf_si32;
                }
                else
                {
                    // rolling direction is VHO_MOV_FORW_enm or VHO_MOV_TURNED_BACKW_enm
                    g_VHOVehicleState_st.DrivenDistance_si32 +=
                        (SInt32)(g_VHORollRecog_st.ds_buf_MMF8_ui32 >> 8);
                    g_DeltaValues_st.XF8_mm_si32 = g_VHORollRecog_st.dx_buf_MMF8_si32;
                    g_DeltaValues_st.YF8_mm_si32 = g_VHORollRecog_st.dy_buf_MMF8_si32;
                    g_DeltaValues_st.PsiF22_rad_si32 =
                        g_VHORollRecog_st.YawAngle_buf_si32;
                }

                // update vho state variables by adding incremental changes
                m_VHOPosCalcUpdateVehicleState_vd(&g_DeltaValues_st,
                                                  g_VHOOdoBuf_st.DrivenDistanceMMF8_si32);
                g_VHORollRecog_st.ds_buf_MMF8_ui32  = (UInt32)0;
                g_VHORollRecog_st.dx_buf_MMF8_si32  = (SInt32)0;
                g_VHORollRecog_st.dy_buf_MMF8_si32  = (SInt32)0;
                g_VHORollRecog_st.YawAngle_buf_si32 = (SInt32)0;
            }

            if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_TURNED_BACKW_enm)
            {
                g_VHOVehicleState_st.RollingDirection_en = VHO_MOV_BACKW_enm;
            }
            else
            {
                if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_TURNED_FORW_enm)
                {
                    g_VHOVehicleState_st.RollingDirection_en = VHO_MOV_FORW_enm;
                }
            }
        }
    }
#endif //(LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC)

    if (g_VHOVehicleState_st.RollingDirection_en != VHO_MOV_UNKNOWN_enm)
    {
        // determine incremental changes in x-position, y-position and psi
        m_VHOPosCalcGetPosChanges_vd(&g_DeltaValues_st,
                                     g_VHOOdoBuf_st.DrivenDistanceMMF8_si32,
                                     g_VHOVehicleState_st.YawAngle_ui32);

        // update vho state variables by adding incremental changes
        m_VHOPosCalcUpdateVehicleState_vd(&g_DeltaValues_st,
                                          g_VHOOdoBuf_st.DrivenDistanceMMF8_si32);
    }
}

void g_VHOMini_vd(void)
{
    // -----------------------------------------------------------------------------------------
    // set internal times
    g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 += (UInt32)10; // VHO-cycle-time (10ms-task)
    g_VHOCanSig_st.WIC_CanTime_ui16 =
        (UInt16)g_VHOCtrlStat_st
            .VHOTotalRuntimeMS_ui32; // just a work-around for DirRecogMainWIC

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3212, 3417, 3426
#endif

    //  g_BswOs1msTimeCounter_en(&g_TimeStampOSEK_MiniVHO_tt);
    g_TimeStampOSEK_MiniVHO_tt = g_GetTimeMs_ui16();
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3212, 3417, 3426
#endif

    g_ElapsedTimeMS_ui16    = (g_TimeStampOSEK_MiniVHO_tt)-g_LastTime_MiniVHO_ui16;
    g_LastTime_MiniVHO_ui16 = g_TimeStampOSEK_MiniVHO_tt;
    // -----------------------------------------------------------------------------------------

    // -----------------------------------------------------------------------------------------
    // calculate curvature
    // --> g_VHOVehicleState_st.KappaF14_si16
    g_VHOGetKappa_MiniVHO_si16();
    // -----------------------------------------------------------------------------------------

    // not required any more because of IF change between VHO
    // and VHS VHS provides now vehicle velocity
    // -----------------------------------------------------------------------------------------
    // set vehicle velocity
    //
    // input: [0.1km/h] --> intern: 1/256[m/s]
    //
    //   Velo
    // -------- [m/s] * 256 [1/256 m/s]
    // 10 * 3.6
    // g_VHOVehicleState_st.VehicleVelocity_ui16 =
    //  (UInt16) g_mtl_u32_Div_u32_ui32((UInt32) g_VHOCanSig_st.Vel_ui16 *
    //  (UInt32)256, (UInt32) 36);
    // -----------------------------------------------------------------------------------------

    if (g_VHOCanSig_st.Vel_ui16 < (SInt32)gd_VHOShutdownVelocitySigRes_ui16)
    {
        // -----------------------------------------------------------------------------------------
        // calculate driven distance
        g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 =
            (SInt32)g_VHOGetDrivenDist_MiniVHO_ui32();
        // -----------------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------------
        // calculate time for detecting vehicle stand still
        g_VHORollRecog_st.WICTimeDeltaMS_ui16 = g_VHOg_VHOCalcWICTimeDeltaMS_ui16();
// -----------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------
// calculate rolling direction
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR)
        g_VHOVehicleState_st.RollingDirection_en = m_VHODirRecogMainGear_en();
#elif (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC)
        g_VHOVehicleState_st.RollingDirection_en = m_VHODirRecogMainWIC_en();
#elif (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
        if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
             VHO_Direction_Wic_enm) ||
            ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
             VHO_Direction_Yaw_Wic_enm))
        {
            g_VHOVehicleState_st.RollingDirection_en = m_VHODirRecogMainWIC_en();
        }
        else
        {
            g_VHOVehicleState_st.RollingDirection_en = m_VHODirRecogMainGear_en();
        }
#else
#error ('Setting for LS_VHO_DIRDETECT_MODE not valid!')
#endif
        // -----------------------------------------------------------------------------------------

        // -----------------------------------------------------------------------------------------
        // rolling direction depending updates
        if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
        {
            // vehicle is moving backward => negate sign of driven distance
            g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 =
                -g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;
        }

        // --> g_VHOVehicleState_st.YawAngle_ui32
        // --> g_VHOVehicleState_st.DrivenDistance_si32
        // --> g_VHOVehicleState_st.XPosition_si32
        // --> g_VHOVehicleState_st.YPosition_si32
        g_VHOCalcModelDepentPosition_MiniVHO_vd();
        // -----------------------------------------------------------------------------------------
    }
}

#endif // if (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
