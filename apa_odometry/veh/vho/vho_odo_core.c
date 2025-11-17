/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_odo_core.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO

#include "vho_api.h"
#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>

#include "vho.h"
#include "vho_odo_core.h" // SWCPA Include Structure: ok
// #include <mathlib_api.h>      // SWCPA Include Structure: ok

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#include "vho_dir_recog.h" // SWCPA Include Structure: ok
#include "vho_kin.h"       // SWCPA Include Structure: ok
#include "vho_sig_proc.h"
#endif

// #include <vho_loghdl.h>       // SMS Include Structure: ok

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef GS_4WS_VEHICLE
#error ('compiler switch GS_4WS_VEHICLE is not defined')
#endif

#ifndef LS_VHO_DIRDETECT_MODE
#error ('compiler switch setting LS_VHO_DIRDETECT_MODE is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_ACCSENS
#error ('compiler switch setting LS_VHO_DIRDETECT_ACCSENS is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_GEAR
#error ('compiler switch setting LS_VHO_DIRDETECT_GEAR is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_WIC
#error ('compiler switch setting LS_VHO_DIRDETECT_WIC is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_YAW_WIC
#error ('compiler switch setting LS_VHO_DIRDETECT_YAW_WIC is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_ALL
#error ('compiler switch setting LS_VHO_DIRDETECT_ALL is not defined')
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#ifndef LS_VHO_AUTOCALIB_SWA_OFFSET
#error ('compiler switch LS_VHO_AUTOCALIB_SWA_OFFSET is not defined')
#endif
#ifndef LS_VHO_EXTRAPOL_ENGINE_RESTART
#error ('compiler switch LS_VHO_EXTRAPOL_ENGINE_RESTART is not defined')
#endif
#ifndef LS_VHO_CALC_VEHICLE_ACCELERATION
#error ('compiler switch LS_VHO_CALC_VEHICLE_ACCELERATION is not defined')
#endif
#endif

#ifndef SW_OFF
#error ('compiler switch SW_OFF is not defined')
#endif
#ifndef SW_ON
#error ('compiler switch SW_ON is not defined')
#endif
#ifndef LS_VHO_VEL_ADAPTIVE_CURVATURE
#error ('compiler switch LS_VHO_VEL_ADAPTIVE_CURVATURE is not defined')
#endif
#ifndef LS_VHO_DIST_FROM_FRONT_WIC
#error ('compiler switch LS_VHO_DIST_FROM_FRONT_WIC is not defined')
#endif
#ifndef LS_VHO_DIST_FROM_FRONT_WIC_AVG
#error ('compiler switch LS_VHO_DIST_FROM_FRONT_WIC_AVG is not defined')
#endif
#ifndef LS_VHO_DIST_FROM_FRONT_WIC_ACKERMANN_STEER
#error ('compiler switch LS_VHO_DIST_FROM_FRONT_WIC_ACKERMANN_STEER is not defined')
#endif
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#ifndef LS_VHO_CORRECTED_SWA_XPG
#error ('compiler switch LS_VHO_CORRECTED_SWA_XPG is not defined')
#endif
#endif

#define md_VHOMaxSteerAng_si16         ((SInt16)4361)  // [2^12 Rad] --> 61�
#define md_VHOMaxSteerWheelAng_si16    ((SInt16)18000) // [0.04�] --> 720�
#define md_VHOMaxSteerWheelAngRad_si16 ((SInt16)25736) // [2^11 Rad] --> 720�
#define md_VHOMaxKappaMF14_si16        ((SInt16)16383) // [2^14  1/m] --> 1

// lower limit: 150�
#define md_VHOSWALowerLimit_si16 ((SInt16)3750) // 150�

// upper limit: 300�
#define md_VHOSWAUpperLimit_si16 ((SInt16)7500) // 300�

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3892, 3219
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
static const SInt32 gc_MonomialScalingFactor_psi32[6] = {10000,    100000,    1000000,
                                                         10000000, 100000000, 1000000000};
#endif

/*--------------------------------------------------------------------------*/
/*- prototypes of local functions                                          -*/
/*--------------------------------------------------------------------------*/

/****************************************************************************/

/****************************************************************************/
VISIBILITY void m_VHOPosCalcBufferLSB_vd(SInt32 f_Delta_si32,
                                         SInt32 *f_VHOInterfaceStateVar_psi32,
                                         SInt16 *f_VHOInterfaceBufferF8_psi16);
/****************************************************************************/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
/****************************************************************************/
#if (GS_4WS_VEHICLE == SW_OFF)
VISIBILITY SInt32 m_VHOGetDeltaPsiFromRearWICDiff_si32(UInt16 f_WICDeltaLeft_ui16,
                                                       UInt16 f_WICDeltaRight_ui16);
#else
VISIBILITY SInt32 m_VHOGetDeltaPsiFromRearWICDiff_si32(UInt16 f_WICDeltaLeft_ui16,
                                                       UInt16 f_WICDeltaRight_ui16,
                                                       SInt16 f_SAF_rad_F12_si16,
                                                       SInt16 f_SAR_rad_F12_si16);
#endif // (GS_4WS_VEHICLE == SW_OFF)
/****************************************************************************/
VISIBILITY SInt32 m_VHOGetDeltaPsiFromFrontWICDiff_si32(UInt16 f_WICDeltaLeft_ui16,
                                                        UInt16 f_WICDeltaRight_ui16);
/****************************************************************************/
#endif

#if (LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON)
/****************************************************************************/
VISIBILITY SInt32 m_VHOGetDeltaPsiFromSteeringAngle_si32(
    SInt32 f_DrivenDistF8_si32, SInt16 f_SteeringAngleRadF12_si16,
    UInt16 f_VehVelMPerSF8_ui16);
/****************************************************************************/
#else
/****************************************************************************/
VISIBILITY SInt32 m_VHOGetDeltaPsiFromSteeringAngle_si32(
    SInt32 f_DrivenDistF8_si32, SInt16 f_SteeringAngleRadF12_si16);
/****************************************************************************/
#endif

#if (LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON)
/****************************************************************************/
VISIBILITY UInt16 m_VHOOdoCoreScaleByVelo_ui16(UInt16 f_X_ui16,
                                               UInt16 f_VehVelMPerSF8_ui16);
/****************************************************************************/
#endif

/****************************************************************************/
VISIBILITY void m_VHOGetXYDeltaApproxStraightLine_vd(SInt32 *f_XDeltaF8_mm_psi32,
                                                     SInt32 *f_YDeltaF8_mm_psi32,
                                                     SInt32 f_DrivenDistF8_si32,
                                                     UInt16 f_PsiF12_rad_ui16);
/****************************************************************************/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY void m_VHOGetXYDeltaApproxCurve_vd(SInt32 *f_XDeltaF8_mm_psi32,
                                              SInt32 *f_YDeltaF8_mm_psi32,
                                              SInt32 f_DrivenDistF8_si32,
                                              SInt16 f_PsiF12_rad_si16,
                                              SInt32 l_DeltaPsiF22_rad_si32);
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/****************************************************************************/

/****************************************************************************/
VISIBILITY SInt16 m_VHOGetAngleFromXY_si16(SInt16 f_X_si16, SInt16 f_Y_si16);
/****************************************************************************/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY UInt32 m_VHOGetYawAngSmooth_ui32(SInt16 f_YawRate_si16, SInt32 f_New_si32,
                                            SInt32 f_Old_si32);
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/****************************************************************************/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
/****************************************************************************/
VISIBILITY SInt32 m_VHODualTrackCompLargeRad_si32(SInt32 f_DeltaPsiDualTrack_si32,
                                                  SInt32 f_DeltaPsiSingleTrack_si32,
                                                  SInt16 f_SteeringWheelAngle_si16);
/****************************************************************************/
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/****************************************************************************/
VISIBILITY SInt32 m_VHOShiftRightSignedInt_si32(SInt32 f_X_si32, UInt8 f_N_ui8);
/****************************************************************************/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
/****************************************************************************/
VISIBILITY void m_VHOCalcYawAngleYRSensorRadF22_vd(void);
/****************************************************************************/
/****************************************************************************/
VISIBILITY void m_VHOCalcYawAngleCorrAtStandStillRadF22_vd(void);
/****************************************************************************/
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*--------------------------------------------------------------------------*/
/*- global data objects                                                    -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- definition of exported functions                                       -*/
/*--------------------------------------------------------------------------*/

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         main odometry function - invokes one position calculation
 * cycle (calculation of incremental changes in position and orientation and
 * update of vho state variables)
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
void g_VHOCalculateModelDependentPosition_vd(void)
{
    mType_DeltaValues_st g_DeltaValues_st;
    UInt8 l_VirtCanMsgRcv_ui8;
    UInt8 l_ForceOutput_ui8                         = 0;
    Boolean l_ErrPresent_MaxUnknownDistDiscarded_bl = FALSE;
    Boolean l_ErrPresent_MaxUnknownDistExceeded_bl  = FALSE;

    // calculate driven distance from the wic difference between last and current
    // call
    g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 = g_VHOPosCalcGetDrivenDistance_si32();

    // calculate time for detecting vehicle stand still
    g_VHORollRecog_st.WICTimeDeltaMS_ui16 = g_VHOg_VHOCalcWICTimeDeltaMS_ui16();

#if (GS_4WS_VEHICLE == SW_ON)
    // store current steering wheel angle and current rear wheel angle
    g_VHOBufferSWAandSAR_vd(g_VHOCanSig_st.SWA_si16, g_VHOCanSig_st.SAR_si16);
#else
    // store current steering wheel angle
    g_VHOBufferSWA_vd(g_VHOCanSig_st.SWA_si16);
#endif

    // get delayed SWA according to driven distance in current cycle (with offset
    // correction): set delayed corrected SWA to delayed SWA --> initialization /
    // standardized input
    g_VHOSWAPreprocSetCorrectedSWAToRawSWA_vd();

#if (GS_4WS_VEHICLE == SW_ON)
    // get delayed steering angle rear according to driven distance in current
    // cycle (with offset correction): set delayed steering angle rear to
    // corrected and deleayed steering angle rear to delayed SWA -->
    // initialization / standardized input
    g_VHOSARPreprocSetCorrectedSARToRawSAR_vd();
    // calculate virtual wheel base according to four wheel steering
    g_VHOCalcVirtualWheelBase_vd();
#endif

// calculate delayed corrected SWA by use of steering wheel angle offset
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
    g_VHOSWAPreprocCalcCorrWithSWAOff_vd();
#endif

#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    if (g_VHOCtrlStat_st.DegMode_en == DegMode_VHO_Extrapolation_enm)
    {
        g_VHOCtrlStat_st.DistPassedDegModeExtrapolMMF8_si32 +=
            g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;
    }
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)

    g_VHORollRecog_st.VirtCan_dsMMF8_si32 += g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;

    if (g_VHOAutocalib_st.WPErrCnt_ui8 == 0)
    {
        // update velocity if no wic extrapolation
        l_VirtCanMsgRcv_ui8 = g_VHOKin_ui8(g_VHOOdoBuf_st.DrivenDistanceMMF8_si32);
    }
    else
    {
        l_VirtCanMsgRcv_ui8 = 0;
    }

    if ((g_VHORollRecog_st.ds_buf_MMF8_ui32 +
         (UInt32)g_VHOOdoBuf_st.DrivenDistanceMMF8_si32) >=
        g_parGetParaVHO_Odo_MaxDistDirUndef_ui32)
    {
        l_ForceOutput_ui8 = 1;
    }

    // determine current rolling direction of the vehicle
    g_VHOVehicleState_st.RollingDirection_en =
        g_VHODirRecogGetCurrentDirection_en(l_VirtCanMsgRcv_ui8, l_ForceOutput_ui8);

    if ((l_ForceOutput_ui8 == 1) &&
        (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_UNKNOWN_enm) &&
        (g_VHOParameters_st.RollRecogMode_en == VHO_Direction_Accsens_enm))
    {
        // rolling direction output is urgent but could not have been determined =>
        // use gear information instead
        if (g_VHOCanSig_st.ReverseGearInserted_bl == FALSE)
        {
            g_VHOVehicleState_st.RollingDirection_en = VHO_MOV_FORW_enm;
        }
        else
        {
            g_VHOVehicleState_st.RollingDirection_en = VHO_MOV_BACKW_enm;
        }
    }

    // determine sign of the driven distance depending on the rolling direction
    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // vehicle is moving backward => negate sign of driven distance
        g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 = -g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;
    }

    if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 > (UInt16)711)
    {
        // vehicle velocity > 10km/h => reset discarded distance
        g_VHORollRecog_st.s_discarded_MMF8_ui32 = (UInt32)0;
    }

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm)
    {
        // VEHICLE IS STANDING STILL; in the case that the current cycle
        // is the first cycle in vehicle stillstand and that the rolling
        // direction of the previous movement could not have been identified
        // the distance increments have to be discarded; this is allowed up
        // to a defined threshold
        if (g_VHORollRecog_st.ds_buf_MMF8_ui32 != 0)
        {
            // distance of the last movement has to be discarded
            if ((
                    // distance of the last movement
                    g_VHORollRecog_st.ds_buf_MMF8_ui32 +
                    // distance of previous movements that has already been discarded
                    g_VHORollRecog_st.s_discarded_MMF8_ui32) >
                g_parGetParaVHO_Odo_MaxDistDiscarded_ui32)
            {
                // pass condition to error handler
                l_ErrPresent_MaxUnknownDistDiscarded_bl = TRUE;

                // reset discarded distance
                g_VHORollRecog_st.s_discarded_MMF8_ui32 = 0;
            }
            else
            {
                // store discarded distance
                g_VHORollRecog_st.s_discarded_MMF8_ui32 +=
                    g_VHORollRecog_st.ds_buf_MMF8_ui32;
            }
            g_VHORollRecog_st.ds_buf_MMF8_ui32             = (UInt32)0;
            g_VHORollRecog_st.dx_buf_MMF8_si32             = (SInt32)0;
            g_VHORollRecog_st.dy_buf_MMF8_si32             = (SInt32)0;
            g_VHORollRecog_st.YawAngle_buf_si32            = (SInt32)0;
            g_VHORollRecog_st.YawAngleDualTrack_buf_si32   = (SInt32)0;
            g_VHORollRecog_st.YawAngleSingleTrack_buf_si32 = (SInt32)0;
        }
    }
    else // VEHICLE IS MOVING
    {
        if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_UNKNOWN_enm)
        {
            // rolling direction is unknown; this state is allowed up to a defined
            // driven distance
            g_VHORollRecog_st.ds_buf_MMF8_ui32 +=
                (UInt32)g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;

            if (g_VHORollRecog_st.ds_buf_MMF8_ui32 <=
                g_parGetParaVHO_Odo_MaxDistDirUndef_ui32)
            {
                // determine incremental changes in x-position, y-position and psi for
                // the assumption of driving forward and update buffers
                m_VHOPosCalcGetPosChanges_vd(
                    &g_DeltaValues_st, g_VHOOdoBuf_st.DrivenDistanceMMF8_si32,
                    m_VHONormAngleRadF22_ui32((SInt32)g_VHOVehicleState_st.YawAngle_ui32 +
                                              g_VHORollRecog_st.YawAngle_buf_si32));

                g_VHORollRecog_st.dx_buf_MMF8_si32 += g_DeltaValues_st.XF8_mm_si32;

                g_VHORollRecog_st.dy_buf_MMF8_si32 += g_DeltaValues_st.YF8_mm_si32;

                g_VHORollRecog_st.YawAngle_buf_si32 += g_DeltaValues_st.PsiF22_rad_si32;

                g_VHORollRecog_st.YawAngleDualTrack_buf_si32 +=
                    g_DeltaValues_st.PsiDualTrackF22_rad_si32;

                g_VHORollRecog_st.YawAngleSingleTrack_buf_si32 +=
                    g_DeltaValues_st.PsiSingleTrackF22_rad_si32;
            }
            else
            {
                // distance driven with unknown direction exceeded =>
                // throw error
                l_ErrPresent_MaxUnknownDistExceeded_bl = TRUE;
            }
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

                    g_DeltaValues_st.PsiDualTrackF22_rad_si32 =
                        -g_VHORollRecog_st.YawAngleDualTrack_buf_si32;

                    g_DeltaValues_st.PsiSingleTrackF22_rad_si32 =
                        -g_VHORollRecog_st.YawAngleSingleTrack_buf_si32;
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
                    g_DeltaValues_st.PsiDualTrackF22_rad_si32 =
                        g_VHORollRecog_st.YawAngleDualTrack_buf_si32;
                    g_DeltaValues_st.PsiSingleTrackF22_rad_si32 =
                        g_VHORollRecog_st.YawAngleSingleTrack_buf_si32;
                }
                // update vho state variables by adding incremental changes
                m_VHOPosCalcUpdateVehicleState_vd(&g_DeltaValues_st, 0);

                // g_VHORollRecog_st.s_discarded_MMF8_ui32 +=
                //  g_VHORollRecog_st.ds_buf_MMF8_ui32;
                g_VHORollRecog_st.ds_buf_MMF8_ui32             = (UInt32)0;
                g_VHORollRecog_st.dx_buf_MMF8_si32             = (SInt32)0;
                g_VHORollRecog_st.dy_buf_MMF8_si32             = (SInt32)0;
                g_VHORollRecog_st.YawAngle_buf_si32            = (SInt32)0;
                g_VHORollRecog_st.YawAngleDualTrack_buf_si32   = (SInt32)0;
                g_VHORollRecog_st.YawAngleSingleTrack_buf_si32 = (SInt32)0;
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

            // determine incremental changes in x-position, y-position and psi
            m_VHOPosCalcGetPosChanges_vd(&g_DeltaValues_st,
                                         g_VHOOdoBuf_st.DrivenDistanceMMF8_si32,
                                         g_VHOVehicleState_st.YawAngle_ui32);

            // update vho state variables by adding incremental changes
            m_VHOPosCalcUpdateVehicleState_vd(&g_DeltaValues_st,
                                              g_VHOOdoBuf_st.DrivenDistanceMMF8_si32);
        }
    }

    // calculate yaw angle by integration of offset corrected yaw rate
    m_VHOCalcYawAngleYRSensorRadF22_vd();

    // calculate correction of VHO yaw angle at vehicle stands still
    m_VHOCalcYawAngleCorrAtStandStillRadF22_vd();

    g_VHOOdoBuf_st.WIC_CanTime_buf_ui16 = g_VHOCanSig_st.WIC_CanTime_ui16;

    // set state of FSV error VhoUnknownDirection_enm in a way that
    //
    // - VhoUnknownDirection_enm is turned to ON if
    //   both VHOLog_MaxUnknownDistDiscarded_enm AND
    //   VHOLog_MaxUnknownDistExceeded_enm are not present any more
    // - VhoUnknownDirection_enm is turned to ON if
    //   at least one of the errors VHOLog_MaxUnknownDistDiscarded_enm AND
    //   VHOLog_MaxUnknownDistExceeded_enm is present
    if ((l_ErrPresent_MaxUnknownDistDiscarded_bl == FALSE) &&
        (l_ErrPresent_MaxUnknownDistExceeded_bl == FALSE))
    {
        // both errors are not present any more => reset
        g_vhoLogHdl_vd(VHOLog_MaxUnknownDistDiscarded_enm, g_vhoLogStateOff_enm);
        g_vhoLogHdl_vd(VHOLog_MaxUnknownDistExceeded_enm, g_vhoLogStateOff_enm);
    }
    else
    {
        // at least one of the errors is present, set corresponding VHO log
        if (l_ErrPresent_MaxUnknownDistDiscarded_bl != FALSE)
        {
            g_vhoLogHdl_vd(VHOLog_MaxUnknownDistDiscarded_enm, g_vhoLogStateOn_enm);
        }

        if (l_ErrPresent_MaxUnknownDistExceeded_bl != FALSE)
        {
            g_vhoLogHdl_vd(VHOLog_MaxUnknownDistExceeded_enm, g_vhoLogStateOn_enm);
        }
    }
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         used to setup wic source and yaw angle computation mode
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
void g_VHOOdoSetup_vd(void)
{
    if (g_VHOOdoState_st.ExternalCtrl == FALSE)
    {
        g_VHOOdoState_st.WeightDualTrackFrontAxle_bf3 =
            (Bitfield)g_parGetParaVHO_Odo_WeightDualTrackFront_ui8;
        g_VHOOdoState_st.WeightDualTrackRearAxle_bf3 =
            (Bitfield)g_parGetParaVHO_Odo_WeightDualTrackRear_ui8;
        g_VHOOdoState_st.WeightSingleTrackModel_bf3 =
            (Bitfield)g_parGetParaVHO_Odo_WeightSingleTrack_ui8;
        g_VHOOdoState_st.WeightDualTrackModel_bf3 =
            (Bitfield)g_parGetParaVHO_Odo_WeightDualTrack_ui8;
        g_VHOOdoState_st.WeightDistanceFront_bf3 =
            (Bitfield)g_parGetParaVHO_Odo_WeightDistanceFront_ui8;
        g_VHOOdoState_st.WeightDistanceRear_bf3 =
            (Bitfield)g_parGetParaVHO_Odo_WeightDistanceRear_ui8;

        switch ((gType_VHODrivenAxle_en)g_VHOParameters_st.VHO_Veh_pst->DrivenAxle_ui8)
        {
            case VHODrivenAxleFront_enm:
                // use wheel pulses of the rear axle only
                g_VHOOdoState_st.WicSource_en = VHOWicSourceTwoWICRear_enm;

                if (g_VHOAutocalibCtrl_st.TTCState_enm == VHOCalibState_STABLE_enm)
                {
                    g_VHOOdoState_st.YawAngCalc_en = VHOYAngCalcSingleXDualTrackRear_enm;
                }
                else
                {
                    g_VHOOdoState_st.YawAngCalc_en = VHOYAngCalcSingleTrack_enm;
                }
                break;

            case VHODrivenAxleRear_enm:
            case VHODrivenAxle4Wheel_enm:
                // use wheel pulses of both axles, front axle and rear axle
                g_VHOOdoState_st.WicSource_en = VHOWicSourceFourWIC_enm;

                if (g_VHOAutocalibCtrl_st.TTCState_enm == VHOCalibState_STABLE_enm)
                {
                    g_VHOOdoState_st.YawAngCalc_en =
                        VHOYAngCalcSingleXDualTrackFrontRear_enm;
                }
                else
                {
                    g_VHOOdoState_st.YawAngCalc_en = VHOYAngCalcSingleTrack_enm;
                }
                break;

            default:
                // do not change configuration
                break;
        }
#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
        if (g_VHOCtrlStat_st.DegMode_en == DegMode_VHO_Extrapolation_enm)
        {
            // do not use combined models in extrapolation mode to prevent large yaw
            // angle errors
            g_VHOOdoState_st.YawAngCalc_en = VHOYAngCalcSingleTrack_enm;
        }
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    }
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*----------------------------------------------------------------*/
/*- definition of local functions                                -*/
/*----------------------------------------------------------------*/

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         calculate incremental changes in x- and y-position and
 *                orientation
 * @details
 *
 * @param         f_DeltaValues_pst
 * @param         f_DrivenDistF8_si32
 * @param         f_YawAngleF22_rad_ui32
 * @return
 *
 * @note
 * @see
 * @warning
 */
void m_VHOPosCalcGetPosChanges_vd(mType_DeltaValues_st *f_DeltaValues_pst,
                                  SInt32 f_DrivenDistF8_si32,
                                  UInt32 f_YawAngleF22_rad_ui32)
{
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_DeltaValues_pst != NULL)
    {
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

        SInt32 l_DeltaPsiSAF22_rad_si32;

        SInt16 l_SteeringAngle_si16;

        // take delayed corrected steering angle as the calculation input
        l_SteeringAngle_si16 = g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16;

//
// CALCULATE YAW ANGLE CHANGE OF THE CURRENT CYCLE
//
#if (LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON)
        l_DeltaPsiSAF22_rad_si32 = m_VHOGetDeltaPsiFromSteeringAngle_si32(
            f_DrivenDistF8_si32, l_SteeringAngle_si16,
            g_VHOVehicleState_st.VehicleVelCanMPerS_ui16);
#else
        l_DeltaPsiSAF22_rad_si32 = m_VHOGetDeltaPsiFromSteeringAngle_si32(
            f_DrivenDistF8_si32, l_SteeringAngle_si16);
#endif

#elif ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) && (GS_VHS_SWA == SW_ON))
        f_DeltaValues_pst->PsiF22_rad_si32 = m_VHOGetDeltaPsiFromSteeringAngle_si32(
            f_DrivenDistF8_si32, g_VHOVehicleState_st.SAngRadF12_si16);
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        switch (g_VHOOdoState_st.YawAngCalc_en)
        {
            case VHOYAngCalcSingleTrack_enm:
                // take driven distance and steering wheel angle as reference
                // for yaw angle delta
                f_DeltaValues_pst->PsiF22_rad_si32 = l_DeltaPsiSAF22_rad_si32;
                break;

            case VHOYAngCalcSingleXDualTrackFrontRear_enm:
            case VHOYAngCalcDualTrackFrontRear_enm:
                // get weighted average both dual track references
                f_DeltaValues_pst->PsiF22_rad_si32 = g_mtl_s32_Div_s32_si32(
                    (((SInt32)g_VHOOdoState_st.WeightDualTrackRearAxle_bf3 *
#if (GS_4WS_VEHICLE == SW_OFF)
                      m_VHOGetDeltaPsiFromRearWICDiff_si32(
                          g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm],
                          g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm])
#else
                      m_VHOGetDeltaPsiFromRearWICDiff_si32(
                          g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm],
                          g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm],
                          g_VHOVehicleState_st.SAngRadF12_si16,
                          g_VHOVehicleState_st.SAngRearRadF12_si16)
#endif
                          ) +
                     ((SInt32)g_VHOOdoState_st.WeightDualTrackFrontAxle_bf3 *
                      m_VHOGetDeltaPsiFromFrontWICDiff_si32(
                          g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm],
                          g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm]))),
                    ((SInt32)g_VHOOdoState_st.WeightDualTrackRearAxle_bf3 +
                     (SInt32)g_VHOOdoState_st.WeightDualTrackFrontAxle_bf3));
                break;

            case VHOYAngCalcSingleXDualTrackRear_enm:
            case VHOYAngCalcDualTrackRearAxle_enm:
                // take difference of the rear axle wics as reference
                // for yaw angle delta
                // if vehicle is equipped with 4WS, delta yaw angle will
                // be calculated according to Ackermann
                f_DeltaValues_pst->PsiF22_rad_si32 =
#if (GS_4WS_VEHICLE == SW_OFF)
                    m_VHOGetDeltaPsiFromRearWICDiff_si32(
                        g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm],
                        g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm]);
#else
                    m_VHOGetDeltaPsiFromRearWICDiff_si32(
                        g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm],
                        g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm],
                        g_VHOVehicleState_st.SAngRadF12_si16,
                        g_VHOVehicleState_st.SAngRearRadF12_si16);
#endif
                break;

            case VHOYAngCalcDualTrackFrontAxle_enm:
            case VHOYAngCalcSingleXDualTrFront_enm:
                // take difference of the front axle wics and steering wheel
                // angle as reference for yaw angle delta
                f_DeltaValues_pst->PsiF22_rad_si32 =
                    m_VHOGetDeltaPsiFromFrontWICDiff_si32(
                        g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm],
                        g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm]);
                break;

#if (LS_VHO_INCLUDE_PROTOTYPES == LS_VHO_YES)
            case VHOYAngCalcYawRate_enm:
                // take yaw rate as the single reference for
                // the yaw angle change of the current cycle
                f_DeltaValues_pst->PsiF22_rad_si32 = g_VHOGetDeltaPsiFromYawRate_si32(
                    g_VHOCorrectedSignals_st.Corrected_YawRate_si32);
                break;
#endif

            default:
                // take driven distance and steering wheel angle as reference
                // for yaw angle delta
                f_DeltaValues_pst->PsiF22_rad_si32 = l_DeltaPsiSAF22_rad_si32;
                break;
        }

        /* save delta Psi from single track model */
        f_DeltaValues_pst->PsiSingleTrackF22_rad_si32 = l_DeltaPsiSAF22_rad_si32;

        /* save delta Psi from dual track model */
        if ((g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcYawRate_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcSingleTrack_enm))
        {
            f_DeltaValues_pst->PsiDualTrackF22_rad_si32 = (SInt32)(0);
        }
        else
        {
            f_DeltaValues_pst->PsiDualTrackF22_rad_si32 =
                f_DeltaValues_pst->PsiF22_rad_si32;
        }

        if ((g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcSingleXDualTrackRear_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en ==
             VHOYAngCalcSingleXDualTrackFrontRear_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcDualTrackFrontAxle_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcDualTrackRearAxle_enm))
        {
            // with a small steering wheel angles (large curve radius) the
            // dual track model does not provide sufficient information on
            // the yaw angle change on small driven distances; the
            // function call is used to compensate this using the
            // single track model information
            f_DeltaValues_pst->PsiF22_rad_si32 = m_VHODualTrackCompLargeRad_si32(
                f_DeltaValues_pst->PsiF22_rad_si32, l_DeltaPsiSAF22_rad_si32,
                g_VHOCanSig_st.SWA_si16);
        }

        if ((g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcSingleXDualTrackRear_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en ==
             VHOYAngCalcSingleXDualTrackFrontRear_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcSingleXDualTrFront_enm))
        {
            // get weighted average of single and dual track model
            f_DeltaValues_pst->PsiF22_rad_si32 = g_mtl_s32_Div_s32_si32(
                (((SInt32)g_VHOOdoState_st.WeightSingleTrackModel_bf3 *
                  l_DeltaPsiSAF22_rad_si32) +
                 ((SInt32)g_VHOOdoState_st.WeightDualTrackModel_bf3 *
                  f_DeltaValues_pst->PsiF22_rad_si32)),
                ((SInt32)g_VHOOdoState_st.WeightSingleTrackModel_bf3 +
                 (SInt32)g_VHOOdoState_st.WeightDualTrackModel_bf3));
        }

        // calculate incremental changes of x- and y-position by choosing linear and
        // circle arc approximation dynamically; linear approximation will be used
        // until a specified threshold in the yaw angle change will exceed
        if ((UInt32)g_mtl_Abs_mac(l_DeltaPsiSAF22_rad_si32) > md_CurveApproxMinDPsi_ui32)
        {
            m_VHOGetXYDeltaApproxCurve_vd(
                &(f_DeltaValues_pst->XF8_mm_si32), &(f_DeltaValues_pst->YF8_mm_si32),
                f_DrivenDistF8_si32,
                (SInt16)g_mtl_ReducePrecision_si32((SInt32)f_YawAngleF22_rad_ui32, 10),
                l_DeltaPsiSAF22_rad_si32);
        }
        else
        {
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
            m_VHOGetXYDeltaApproxStraightLine_vd(
                &(f_DeltaValues_pst->XF8_mm_si32), &(f_DeltaValues_pst->YF8_mm_si32),
                f_DrivenDistF8_si32,
                (UInt16)g_mtl_ReducePrecision_si32((SInt32)f_YawAngleF22_rad_ui32, 10));
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        }
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         compensate dual track model on large curve radius by usage
 *                of single track model
 * @details
 *
 * @param         f_DeltaPsiDualTrack_si32
 * @param         f_DeltaPsiSingleTrack_si32
 * @param         f_SteeringWheelAngle_si16
 * @return        SInt32
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY SInt32 m_VHODualTrackCompLargeRad_si32(SInt32 f_DeltaPsiDualTrack_si32,
                                                  SInt32 f_DeltaPsiSingleTrack_si32,
                                                  SInt16 f_SteeringWheelAngle_si16)
{
    SInt32 l_RetVal_si32;

    f_SteeringWheelAngle_si16 = (SInt16)g_mtl_Abs_mac(f_SteeringWheelAngle_si16);

    if (f_SteeringWheelAngle_si16 < md_VHOSWALowerLimit_si16)
    {
        // replace dual track model information with single track model
        l_RetVal_si32 = f_DeltaPsiSingleTrack_si32;
    }
    else if (f_SteeringWheelAngle_si16 > md_VHOSWAUpperLimit_si16)
    {
        // no compensation needed
        l_RetVal_si32 = f_DeltaPsiDualTrack_si32;
    }
    else
    {
        SInt32 l_WeightDualTrack_si32;
        SInt32 l_WeightSingleTrack_si32;

        l_WeightDualTrack_si32 =
            ((SInt32)100 *
             ((SInt32)(f_SteeringWheelAngle_si16 - md_VHOSWALowerLimit_si16))) /
            ((SInt32)(md_VHOSWAUpperLimit_si16 - md_VHOSWALowerLimit_si16));

        l_WeightSingleTrack_si32 = ((SInt32)100) - l_WeightDualTrack_si32;

        l_RetVal_si32 = ((l_WeightSingleTrack_si32 * f_DeltaPsiSingleTrack_si32) +
                         (l_WeightDualTrack_si32 * f_DeltaPsiDualTrack_si32)) /
                        (SInt32)100;
    }
    return l_RetVal_si32;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         update state variables after position calculation
 * @details
 *
 * @param         fc_DeltaValues_pst
 * @param         f_DrivenDistance_si32
 * @return
 *
 * @note
 * @see
 * @warning
 */
void m_VHOPosCalcUpdateVehicleState_vd(const mType_DeltaValues_st *fc_DeltaValues_pst,
                                       SInt32 f_DrivenDistance_si32)
{
    SInt32 l_PsiF22_si32;

    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (fc_DeltaValues_pst != NULL)
    {
        // update driven distance
        m_VHOPosCalcBufferLSB_vd(f_DrivenDistance_si32,
                                 &g_VHOVehicleState_st.DrivenDistance_si32,
                                 &g_VHOOdoBuf_st.DrivenDistance_bufF8_si16);

        // update x-position
        m_VHOPosCalcBufferLSB_vd(fc_DeltaValues_pst->XF8_mm_si32,
                                 &g_VHOVehicleState_st.XPosition_si32,
                                 &g_VHOOdoBuf_st.XPosition_bufF8_si16);

        // update y-position
        m_VHOPosCalcBufferLSB_vd(fc_DeltaValues_pst->YF8_mm_si32,
                                 &g_VHOVehicleState_st.YPosition_si32,
                                 &g_VHOOdoBuf_st.YPosition_bufF8_si16);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        // update yaw angle from dual track model
        l_PsiF22_si32 = (SInt32)g_VHOVehicleState_st.YawAngleDualTrack_ui32 +
                        fc_DeltaValues_pst->PsiDualTrackF22_rad_si32;

        l_PsiF22_si32 = (SInt32)m_VHONormAngleRadF22_ui32(l_PsiF22_si32);

        g_VHOVehicleState_st.YawAngleDualTrack_ui32 = (UInt32)(l_PsiF22_si32);

        // update yaw angle from single track model
        l_PsiF22_si32 = (SInt32)g_VHOVehicleState_st.YawAngleSingleTrack_ui32 +
                        fc_DeltaValues_pst->PsiSingleTrackF22_rad_si32;

        l_PsiF22_si32 = (SInt32)m_VHONormAngleRadF22_ui32(l_PsiF22_si32);

        g_VHOVehicleState_st.YawAngleSingleTrack_ui32 = (UInt32)(l_PsiF22_si32);

#endif

        // update yaw angle
        l_PsiF22_si32 =
            (SInt32)g_VHOOdoBuf_st.YawAngleRaw_ui32 + fc_DeltaValues_pst->PsiF22_rad_si32;

        l_PsiF22_si32 = (SInt32)m_VHONormAngleRadF22_ui32(l_PsiF22_si32);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

        g_VHOOdoBuf_st.YawAngleRaw_ui32 =
            // g_VHOVehicleState_st.YawAngle_ui32 =
            (UInt32)l_PsiF22_si32;

        g_VHOVehicleState_st.YawAngle_ui32 = m_VHOGetYawAngSmooth_ui32(
            g_VHOVehicleState_st.YawRate_si16, (SInt32)g_VHOOdoBuf_st.YawAngleRaw_ui32,
            (SInt32)g_VHOVehicleState_st.YawAngle_ui32);

#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

        g_VHOOdoBuf_st.YawAngleRaw_ui32    = (UInt32)l_PsiF22_si32;
        g_VHOVehicleState_st.YawAngle_ui32 = (UInt32)l_PsiF22_si32;

#endif
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         smooth yaw angle
 * @details
 *
 * @param         f_YawRate_si16
 * @param         f_New_si32
 * @param         f_Old_si32
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY UInt32 m_VHOGetYawAngSmooth_ui32(SInt16 f_YawRate_si16, SInt32 f_New_si32,
                                            SInt32 f_Old_si32)
{
    UInt16 l_YawRate_ui16;
    UInt8 l_k_ui8;
    const UInt8 lc_YawRateUpperLimit_ui8 = 5;
    UInt32 l_RetVal_ui32;

    if (((SInt32)g_mtl_Abs_mac(g_VHOCanSig_st.SWA_si16) >
         (SInt32)md_VHOSWALowerLimit_si16) &&
        (g_VHOOdoState_st.YawAngCalc_en != VHOYAngCalcSingleTrack_enm))
    {
        l_YawRate_ui16 = (UInt16)(g_mtl_Abs_mac(f_YawRate_si16) / (UInt16)100);
        if (l_YawRate_ui16 > (UInt16)lc_YawRateUpperLimit_ui8)
        {
            l_YawRate_ui16 = (UInt16)lc_YawRateUpperLimit_ui8;
        }
        l_k_ui8 =
            (UInt8)((l_YawRate_ui16 * (UInt16)100) / (UInt16)lc_YawRateUpperLimit_ui8);
        l_k_ui8 =
            (UInt8)(g_parGetParaVHO_Odo_YawAngSmoothKoeffMax_ui8 -
                    (UInt8)(((UInt16)l_k_ui8 *
                             ((UInt16)(g_parGetParaVHO_Odo_YawAngSmoothKoeffMax_ui8 -
                                       g_parGetParaVHO_Odo_YawAngSmoothKoeffMin_ui8))) /
                            (UInt16)100));

        if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 == 0)
        {
            l_RetVal_ui32 = (UInt32)f_Old_si32;
        }
        else
        {
            if ((f_New_si32 < (SInt32)3660220) && (f_Old_si32 > (SInt32)21961324))
            {
                f_New_si32 += (SInt32)26353589;
            }

            if ((f_New_si32 > (SInt32)21961324) && (f_Old_si32 < (SInt32)3660220))
            {
                f_Old_si32 += (SInt32)26353589;
            }

            l_RetVal_ui32 = ((UInt32)l_k_ui8 * (UInt32)f_Old_si32) +
                            (((UInt32)((UInt8)100 - l_k_ui8)) * (UInt32)f_New_si32);

            l_RetVal_ui32 /= (UInt32)100;

            l_RetVal_ui32 = m_VHONormAngleRadF22_ui32((SInt32)l_RetVal_ui32);
        }
    }
    else
    {
        l_RetVal_ui32 = (UInt32)f_New_si32;
    }

    return l_RetVal_ui32;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         used to norm an input angle to 0..2pi
 * @details
 *
 * @param         l_InputAngleRadF22_si32
 * @return        UInt32
 *
 * @note
 * @see
 * @warning
 */
UInt32 m_VHONormAngleRadF22_ui32(SInt32 l_InputAngleRadF22_si32)
{
    // check whether current yaw angle is still in range 0...2pi
    while (l_InputAngleRadF22_si32 > (SInt32)26353589)
    {
        // current yaw angle > 2pi => yaw angle = yaw angle - 2pi
        l_InputAngleRadF22_si32 -= (SInt32)26353589;
    }

    while (l_InputAngleRadF22_si32 < (SInt32)0)
    {
        // current yaw angle < 0 => yaw angle = yaw angle + 2pi
        l_InputAngleRadF22_si32 += (SInt32)26353589;
    }
    return (UInt32)l_InputAngleRadF22_si32;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         used to add increments with resolution of 1/256mm to state
 *                variables with resolution of 1mm; seperate buffers are used
 *                to store the least significant bits
 * @details
 *
 * @param         f_Delta_si32                    delta value in resolution of
 * 1/256mm which has to be added to the state variable
 * @param         f_VHOInterfaceStateVar_psi32    pointer to the state variable
 * that has to be updated
 * @param         f_VHOInterfaceBufferF8_psi16    pointer to the 8 least
 * significant bits of the state variable
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOPosCalcBufferLSB_vd(SInt32 f_Delta_si32,
                                         SInt32 *f_VHOInterfaceStateVar_psi32,
                                         SInt16 *f_VHOInterfaceBufferF8_psi16)
{
    UInt8 l_A_ui8;

    // check whether delta is causing increase or decrease of
    // state variable
    if (f_Delta_si32 > (SInt32)0) // increment
    {
        // truncate least significant byte and add to buffer
        l_A_ui8 = (UInt8)(((UInt32)f_Delta_si32) & (UInt32)0x000000ff);
        *f_VHOInterfaceBufferF8_psi16 += (SInt16)l_A_ui8;

        if (*f_VHOInterfaceBufferF8_psi16 > (SInt16)0xff)
        {
            // length stored in buffer > 1mm
            //    => add 1mm to state variable
            //    => buffer = buffer - 1mm
            *f_VHOInterfaceStateVar_psi32 += (SInt32)1;
            *f_VHOInterfaceBufferF8_psi16 -= (SInt16)0xff;
        }
    }
    else if (f_Delta_si32 < (SInt32)0) // decrement
    {
        // truncate least significant byte and add to buffer
        l_A_ui8 = (UInt8)(((UInt32)(-f_Delta_si32)) & (UInt32)0x000000ff);
        *f_VHOInterfaceBufferF8_psi16 -= (SInt16)l_A_ui8;

        if (*f_VHOInterfaceBufferF8_psi16 < (SInt16)(-0xff))
        {
            // length stored in buffer < -1mm
            //    => subtract 1mm from state variable
            //    => buffer = buffer + 1mm
            *f_VHOInterfaceStateVar_psi32 -= (SInt32)1;
            *f_VHOInterfaceBufferF8_psi16 += (SInt16)0xff;
        }
    }
    else
    {
        // do nothing but be consistent with coding rules
    }

    // add incremental change to state variable by shifting right
    // 8 bits
    *f_VHOInterfaceStateVar_psi32 +=
        m_VHOShiftRightSignedInt_si32(f_Delta_si32, (UInt8)8);
}

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculates delta psi using the steering angle information
 *                optionally the radius which is calculated from the steering
 * angle is scaled with a velocity dependent factor
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON) && \
     (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
VISIBILITY SInt32 m_VHOGetDeltaPsiFromSteeringAngle_si32(
    SInt32 f_DrivenDistF8_si32, SInt16 f_SteeringAngleRadF12_si16,
    UInt16 f_VehVelMPerSF8_ui16)
#else
VISIBILITY SInt32 m_VHOGetDeltaPsiFromSteeringAngle_si32(
    SInt32 f_DrivenDistF8_si32, SInt16 f_SteeringAngleRadF12_si16)
#endif
{
    SInt32 l_TanSteeringAngle_si32;
    SInt32 l_DeltaPsiRadF22_si32;
    SInt32 l_X_si32 = 2;
#if ((LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON) && \
     (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
    SInt32 l_WheelBaseXScalingFactor_si32;
#endif

    // BASICALLY d_phi is calculated as follows
    //
    //          s   s * tan(steering angle)
    // d_phi  = - = -----------------------
    //          r   wheel_base
    //
    //
    // OPTIONALLY the vehicle velocity will be considered for this
    // calculation; a scalar k(v) will be used then to extend the
    // equation as follows:
    //
    //          s           s * tan(steering angle)
    // d_phi  = ------    = -----------------------
    //          r * k(v)    wheel_base * k(v)

    // get tan(steering angle)
    l_TanSteeringAngle_si32 = g_mtl_Tan_si16(f_SteeringAngleRadF12_si16); // F13

    // f_DrivenDistF8 < 0.256m
    // -----------------------
    // s = f_DrivenDistF8; max(s) * max(l_tanSteeringAngle_si32) == 65536 * 32767
    // < 2^31
    //
    // 0.256m <= f_DrivenDistF8 < 65.536m
    // ----------------------------------
    // s = f_DrivenDistF8 / 256; max(s) * max(l_tanSteeringAngle_si32) == 65536 *
    // 32767 < 2^31
    if ((UInt32)g_mtl_Abs_mac(f_DrivenDistF8_si32) > (UInt32)65536)
    {
        // reduce resolution of driven distance to mm
        f_DrivenDistF8_si32 = g_mtl_ReducePrecision_si32(f_DrivenDistF8_si32, (UInt8)8);
        l_X_si32            = (SInt32)512;
    }

    // s * tan(steering angle)
    l_DeltaPsiRadF22_si32 =
        f_DrivenDistF8_si32 * l_TanSteeringAngle_si32; // l_X_si32 == 2^1  => F21
                                                       // l_X_si32 == 2^9  => F13
#if ((LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON) && \
     (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
    // get wheel_base * k(v)
    l_WheelBaseXScalingFactor_si32 = (SInt32)m_VHOOdoCoreScaleByVelo_ui16(
        g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16, f_VehVelMPerSF8_ui16);

    // get d_phi
    l_DeltaPsiRadF22_si32 = // F22
        g_mtl_s32_Div_s32_si32(l_DeltaPsiRadF22_si32, l_WheelBaseXScalingFactor_si32) *
        l_X_si32;
#else
    l_DeltaPsiRadF22_si32 =
        g_mtl_s32_Div_s32_si32(l_DeltaPsiRadF22_si32, // F22
                               (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) *
        l_X_si32;
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    // calculate yaw rate signal in 0.01�/s for further use by
    //
    //                tan(steering_angle)   256 * 8192
    // yaw_rate = v * ------------------- * ----------
    //                wheel_base            1000

    l_X_si32 =
        l_TanSteeringAngle_si32 * (SInt32)g_VHOVehicleState_st.VehicleVelCanMPerS_ui16;
    // avoid overflow in YawRate_si16
    if (l_X_si32 > (SInt32)17990678)
    {
        l_X_si32 = (SInt32)17990678;
    }
    l_X_si32 = g_mtl_s32_Div_s32_si32(
        l_X_si32, (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16);

    g_VHOVehicleState_st.YawRate_si16 =
        (SInt16)((l_X_si32 * (SInt32)5729) / (SInt32)2097);

#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    return l_DeltaPsiRadF22_si32;
}

#if ((LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON) && \
     (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculates scaling factor as a function from vehicle velocity
 * and scales input value
 * @details
 *
 * @param         f_X_ui16
 * @param         f_VehVelMPerSF8_ui16
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY UInt16 m_VHOOdoCoreScaleByVelo_ui16(UInt16 f_X_ui16,
                                               UInt16 f_VehVelMPerSF8_ui16)
{
    UInt32 l_RetVal_ui32;
    UInt16 l_I_ui16;
    const UInt32 lc_MaxScalFac_ui32      = 150000;
    const UInt16 lc_VelThresMperSF2_ui16 = (UInt16)6;

    // convert velocity from 1/256 m/s to 1/4 m/s
    l_I_ui16 = g_mtl_ReducePrecision_ui16(f_VehVelMPerSF8_ui16, (UInt8)6);

    if (l_I_ui16 <= lc_VelThresMperSF2_ui16)
    {
        // as long as the velocity lies below a defined threshold the
        // scaling will NOT be active (radii did not differ in the
        // range 0..5km/h in measurements)
        l_RetVal_ui32 = (UInt32)f_X_ui16;
    }
    else
    {
        //
        // CALCULATE k_3*v^2 + k_2*v + k_1, which is equivalent to
        //
        //  100000 * (k_3*v^2 + k_2*v + k_1)
        //  --------------------------------
        //  100000
        l_RetVal_ui32 = // 4*100000*(k_3*v^2)
            g_mtl_ReducePrecision_ui32(
                (((UInt32)l_I_ui16 * (UInt32)l_I_ui16) * md_VHOPolCoeff_3_ui32),
                (UInt8)2);

        l_RetVal_ui32 += // 4*100000*(k_3*v^2 + k_2*v)
            ((UInt32)l_I_ui16 * md_VHOPolCoeff_2_ui32);

        l_RetVal_ui32 = // 100000*(k_3*v^2 + k_2*v)
            g_mtl_ReducePrecision_ui32(l_RetVal_ui32, (UInt8)2);

        l_RetVal_ui32 += // 100000*(k_3*v^2 + k_2*v + k_1)
            md_VHOPolCoeff_1_ui32;

        if (l_RetVal_ui32 >= lc_MaxScalFac_ui32)
        {
            // maximum scaling factor has been reached or exceeded
            // => limit scaling, throw error condition
            l_RetVal_ui32 = lc_MaxScalFac_ui32;
            g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
        }

        // scale input value
        l_RetVal_ui32 *=      // 100000*(k_3*v^2 + k_2*v + k_1)*
            (UInt32)f_X_ui16; // wheel_base

        // divide by 100000         // (k_3*v^2 + k_2*v + k_1)*wheel_base
        l_RetVal_ui32 /= (UInt32)100000;
    }
    return (UInt16)l_RetVal_ui32;
}
#endif // LS_VHO_VEL_ADAPTIVE_CURVATURE == SW_ON

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculates delta psi using the wic differences
 *                If vehicles are equipt with 4WS deltaPsi will be calculated
 *                according to Ackermann
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_OFF)
VISIBILITY SInt32 m_VHOGetDeltaPsiFromRearWICDiff_si32(UInt16 f_WICDeltaLeft_ui16,
                                                       UInt16 f_WICDeltaRight_ui16)
#else
VISIBILITY SInt32 m_VHOGetDeltaPsiFromRearWICDiff_si32(UInt16 f_WICDeltaLeft_ui16,
                                                       UInt16 f_WICDeltaRight_ui16,
                                                       SInt16 f_SAF_rad_F12_si16,
                                                       SInt16 f_SAR_rad_F12_si16)
#endif
{
    SInt32 l_DeltaPsiF22_rad_si32;

#if (GS_4WS_VEHICLE == SW_ON)
    /* only absolute value of front steering angle is needed */
    const SInt16 l_Abs_SAF_rad_F12_si16 = (SInt16)g_mtl_Abs_mac(f_SAF_rad_F12_si16);
    /* only absolute value of rear steering angle is needed */
    const SInt16 l_Abs_SAR_rad_F12_si16 = (SInt16)g_mtl_Abs_mac(f_SAR_rad_F12_si16);

    /**********************************************************************
     * calculate driven distance of the virtual rear wheel from the single
     * wheel distances of the rear axle for an ideal ackermann steering
     **********************************************************************/
    SInt32 l_DrivenDistRLMM_F8_si32;
    SInt32 l_DrivenDistRRMM_F8_si32;
    SInt32 l_DeltaDrivenDistRearWheelsMM_F8_si32;

    UInt32 l_temp1_ui32;
    UInt32 l_temp2_ui32;
    UInt32 l_temp3_ui32;
    SInt32 l_temp1_si32;
    UInt32 l_Cos2SAF_ui32;
    UInt32 l_Cos2SAR_ui32;
    UInt32 l_B_ui32;
    UInt32 l_C_ui32;
    SInt32 l_L_si32;
    SInt32 l_R_si32;
    SInt32 l_deltaRear_si32;

    // max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) = 8192 = 2^13
    // => max(result) = 2^13 * 2^13 * 2^2 = 2^28 < 2^32      Quant: mm^2
    const UInt32 lc_A_ui32 = (UInt32)(4) *
                             (UInt32)(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) *
                             (UInt32)(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16);
#endif

    if (g_mtl_Abs_mac((SInt16)f_WICDeltaLeft_ui16 - (SInt16)f_WICDeltaRight_ui16) > 5)
    {
        g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
    }

#if (GS_4WS_VEHICLE == SW_OFF)
    // calculate difference between driven distances of both tyres on the rear
    // axle
    l_DeltaPsiF22_rad_si32 =
        (((SInt32)(f_WICDeltaRight_ui16) *
          (SInt32)(g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm])) -
         ((SInt32)(f_WICDeltaLeft_ui16) *
          (SInt32)(g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm])));

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // vehicle is moving backwards
        l_DeltaPsiF22_rad_si32 = -l_DeltaPsiF22_rad_si32;
    }

    l_DeltaPsiF22_rad_si32 *= (SInt32)(16384);

    l_DeltaPsiF22_rad_si32 = g_mtl_s32_Div_s32_si32(
        (SInt32)(l_DeltaPsiF22_rad_si32),
        (SInt32)(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16));
#else

    /* temp1_si32 = 2*Abs(SAF) */
    l_temp1_si32   = (SInt32)(2) * (SInt32)(l_Abs_SAF_rad_F12_si16); // F12
    l_Cos2SAF_ui32 = (UInt32)g_mtl_Cos_si16((SInt16)(l_temp1_si32)); // F14

    /* temp1_si32 = 2*Abs(SAR) */
    l_temp1_si32   = (SInt32)(2) * (SInt32)(l_Abs_SAR_rad_F12_si16); // F12
    l_Cos2SAR_ui32 = (UInt32)g_mtl_Cos_si16((SInt16)(l_temp1_si32)); // F14

    /* temp1_ui32 = 1 - cos(2*saf) */
    l_temp1_ui32 = (UInt32)(16384) - l_Cos2SAF_ui32; // F14

    /* temp2_ui32 = 1 + cos(2*saf) */
    l_temp2_ui32 = (UInt32)(16384) + l_Cos2SAF_ui32; // F14

    /* temp3_ui32 = 1 + cos(2*sar) */
    l_temp3_ui32 = (UInt32)(16384) + l_Cos2SAR_ui32; // F14

    /* (1-cos2saf)*(1+cos2sar)/1+cos2saf) */
    /* (2^14 * 2^14) / 2^14 = 2^14 < 2^32 */
    l_B_ui32 =
        (UInt32)g_mtl_s32_Div_s32_si32((SInt32)(l_temp1_ui32) * (SInt32)(l_temp3_ui32),
                                       (SInt32)(l_temp2_ui32)); // F14
    /* l_B / 2 */
    /* Die Quantisierung bleibt unver�ndert: F14 */
    l_B_ui32 = g_mtl_ReducePrecision_ui32((UInt32)(l_B_ui32), (UInt32)(1)); // F14

    // max(g_VHOParameters_st.VehData_st.AxleBaseRearMM_ui16) = 2048 = 2^11
    // max(l_B_ui32) * max(g_VHOParameters_st.VehData_st.AxleBaseRearMM_ui16) =
    // 2^14 * 2^11 = 2^25 < 2^32
    // => max(result) = 2^(25-4) = 2^21
    l_B_ui32 = g_mtl_ReducePrecision_ui32(
        ((UInt32)(l_B_ui32) *
         (UInt32)(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16)),
        (UInt32)(4)); // F21 => F10
    // max(g_VHOParameters_st.VehData_st.AxleBaseRearMM_ui16) = 2048 = 2^11
    // max(l_C_ui32) * max(g_VHOParameters_st.VehData_st.AxleBaseRearMM_ui16) =
    // 2^21 * 2^11 = 2^32
    // => max(result) = 2^(32-10) = 2^22
    // Quant: mm^2
    l_B_ui32 = g_mtl_ReducePrecision_ui32(
        ((UInt32)(l_B_ui32) *
         (UInt32)(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16)),
        (UInt32)(10)); // F21 => F10

    /* temp1_ui32 = sin(Abs(saf)) */
    l_temp1_ui32 = (UInt32)g_mtl_Sin_si16(l_Abs_SAF_rad_F12_si16); // F14

    /* temp2_ui32 = cos(Abs(saf)) */
    l_temp2_ui32 = (UInt32)g_mtl_Cos_si16(l_Abs_SAF_rad_F12_si16); // F14

    /* sin(saf)*(1+cos2sar)/cossaf) */
    /* (2^14 * 2^14) / 2^14 = 2^14 < 2^32 */
    l_C_ui32 =
        (UInt32)g_mtl_s32_Div_s32_si32((SInt32)(l_temp1_ui32) * (SInt32)(l_temp3_ui32),
                                       (SInt32)(l_temp2_ui32)); // F14

    // max(l_B_si32) = 2^14, max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) =
    // 8192 = 2^13 max(l_B_si32) *
    // max(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) = 2^14 * 2^13 = 2^27 <
    // 2^31
    // => max(result) = 2^(27-6) = 2^21
    l_C_ui32 = g_mtl_ReducePrecision_ui32(
        l_C_ui32 * (UInt32)(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16),
        (UInt8)(6)); // F8

    // max(g_VHOParameters_st.VehData_st.AxleBaseRearMM_ui16) = 2048 = 2^11
    // max(g_VHOParameters_st.VehData_st.AxleBaseRearMM_ui16) *  max(l_C_ui32) =
    // 2^11 * 2^21 = 2^32
    // => max(result) = 2^(32-7) = 2^25
    // (final multiplication with 2 => bit shift of 7 instead of 8)
    l_C_ui32 = g_mtl_ReducePrecision_ui32(
        l_C_ui32 * (UInt32)(g_VHOParameters_st.VHO_Veh_pst->AxleBaseRear_ui16),
        (UInt8)(7)); // F0

    // max(l_R_si32) <= max(l_L_si32) < 2^31
    l_R_si32 = (SInt32)(lc_A_ui32) + (SInt32)(l_B_ui32) + (SInt32)(l_C_ui32); // F0
    l_R_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)(l_R_si32)); // F0   [l_R_si32] = mm

    // max(lc_A_ui32) + max(l_B_ui32) + max(l_C_ui32) = 2^28 +  2^21 +  2^22 <
    // 2^31
    l_L_si32 = ((SInt32)(lc_A_ui32) + (SInt32)(l_B_ui32)) - (SInt32)(l_C_ui32); // F0
    l_L_si32 = (SInt32)g_mtl_Sqrt_u32_ui16((UInt32)(l_L_si32)); // F0   [l_L_si32] = mm

    l_deltaRear_si32 = l_R_si32 - l_L_si32; // F0   [l_deltaRear_si32] = mm

    /* calculate the changes of the arc length rear right: Quantisierung 1mm/256
     * (F8) */
    l_DrivenDistRRMM_F8_si32 =
        ((SInt32)f_WICDeltaRight_ui16 *
         (SInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm]);

    /* calculate the changes of the arc length rear left: : Quantisierung 1mm/256
     * (F8) */
    l_DrivenDistRLMM_F8_si32 =
        ((SInt32)f_WICDeltaLeft_ui16 *
         (SInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm]);

    /* Quantisierung 1mm/256 (F8) */
    l_DeltaDrivenDistRearWheelsMM_F8_si32 =
        l_DrivenDistRRMM_F8_si32 - l_DrivenDistRLMM_F8_si32;

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // vehicle is moving backwards
        l_DeltaDrivenDistRearWheelsMM_F8_si32 = -l_DeltaDrivenDistRearWheelsMM_F8_si32;
    }

    /* temp1_ui32 = sin(Abs(saf)) */
    l_temp1_ui32 = (UInt32)g_mtl_Sin_si16(l_Abs_SAF_rad_F12_si16); // F14

    /* temp2_ui32 = cos(Abs(saf)) */
    l_temp2_ui32 = (UInt32)g_mtl_Cos_si16(l_Abs_SAF_rad_F12_si16); // F14

    /* temp2_ui32 = cos(Abs(sar)) */
    l_temp3_ui32 = (UInt32)g_mtl_Cos_si16(l_Abs_SAR_rad_F12_si16); // F14

    /* 2*sin(Abs(saf))*cos(Abs(sar))/cos(Abs(saf)) */
    l_temp1_si32 = g_mtl_s32_Div_s32_si32((SInt32)(l_temp1_ui32) * (SInt32)(l_temp3_ui32),
                                          (SInt32)(l_temp2_ui32)); // F14

    /* die Quantisierung von F14 bleibt erhalten */
    l_temp1_si32 = (SInt32)(2) * l_temp1_si32;

    /* die Quantisierung wird von F14 auf F22 erh�ht */
    l_temp1_si32 =
        l_temp1_si32 * l_DeltaDrivenDistRearWheelsMM_F8_si32; // 2^14*2^8 = 2^22

    /* 2^22 / 2^0 = 2^22 */
    l_DeltaPsiF22_rad_si32 = g_mtl_s32_Div_s32_si32(l_temp1_si32, l_deltaRear_si32);

#endif

    return l_DeltaPsiF22_rad_si32;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculates delta psi using wic differences of the front axle
 *                and the current steering angle
 * @details
 *
 * @param         f_WICDeltaLeft_ui16
 * @param         f_WICDeltaRight_ui16
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY SInt32 m_VHOGetDeltaPsiFromFrontWICDiff_si32(UInt16 f_WICDeltaLeft_ui16,
                                                        UInt16 f_WICDeltaRight_ui16)
{
    SInt32 l_DeltaPsiF22_rad_si32;
    UInt16 l_Denominator_ui16;
    UInt16 l_AbsKappa_ui16;
    UInt8 l_BaseIndex_ui8;
    const SInt32 lc_DeltaWICLF_si32 = (SInt32)f_WICDeltaLeft_ui16;
    const SInt32 lc_DeltaWICRF_si32 = (SInt32)f_WICDeltaRight_ui16;

    if (g_mtl_Abs_mac((SInt16)f_WICDeltaLeft_ui16 - (SInt16)f_WICDeltaRight_ui16) > 5)
    {
        g_vhoLogHdl_vd(VHOLog_OdoCore_enm, g_vhoLogStateOn_enm);
    }

    l_AbsKappa_ui16 = (UInt16)g_mtl_Abs_mac(g_VHOVehicleState_st.KappaF14_si16);

    if (l_AbsKappa_ui16 >= (UInt16)8178)
    {
        // kappa>=0.4992 (radius<=2m) => invalid!
        l_DeltaPsiF22_rad_si32 = (SInt32)0;
    }
    else
    {
        // calculate difference of driven distances of both tyres at the front axle
        l_DeltaPsiF22_rad_si32 =
            (lc_DeltaWICRF_si32 *
             (SInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm]) -
            (lc_DeltaWICLF_si32 *
             (SInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm]);

        l_BaseIndex_ui8 = (UInt8)(l_AbsKappa_ui16 >> 8);
        l_Denominator_ui16 =
            g_VHOParameters_st.VHO_Veh_pst
                ->DifFrontWheelRadiiTable_pui16[l_BaseIndex_ui8] -
            ((UInt16)(((l_AbsKappa_ui16 & (UInt16)0xFF) *
                       (g_VHOParameters_st.VHO_Veh_pst
                            ->DifFrontWheelRadiiTable_pui16[l_BaseIndex_ui8] -
                        g_VHOParameters_st.VHO_Veh_pst
                            ->DifFrontWheelRadiiTable_pui16[l_BaseIndex_ui8 +
                                                            (UInt8)1])) /
                      (UInt16)0x100));

        l_DeltaPsiF22_rad_si32 = g_mtl_s32_Div_s32_si32(
            l_DeltaPsiF22_rad_si32 * (SInt32)16384, (SInt32)l_Denominator_ui16);

        if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
        {
            // vehicle is moving backwards
            l_DeltaPsiF22_rad_si32 = -l_DeltaPsiF22_rad_si32;
        }
    }
    return l_DeltaPsiF22_rad_si32;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculates x-,y- deltas using straight line approximation
 * @details
 * @param         f_XDeltaF8_mm_psi32
 * @param         f_YDeltaF8_mm_psi32
 * @param         f_DrivenDistF8_si32   driven distance in 1/256mm
 *                                      allowed range: -524288..524287
 * (-2.048..2.04799m)
 * @param         f_PsiF12_rad_ui16     0..25735 (0..2pi)
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOGetXYDeltaApproxStraightLine_vd(SInt32 *f_XDeltaF8_mm_psi32,
                                                     SInt32 *f_YDeltaF8_mm_psi32,
                                                     SInt32 f_DrivenDistF8_si32,
                                                     UInt16 f_PsiF12_rad_ui16)
{
    Boolean l_X_bl = FALSE;

    // f_DrivenDistF8 < 0.512m
    // -----------------------
    // s = f_DrivenDistF8; max(s) * max(cos/sin(f_PsiF12_rad_ui16)*2^14) < 131072
    // * 16384 < 2^31
    //
    // 0.512 <= f_DrivenDistF8 < 131.071m
    // ----------------------------------
    // s = f_DrivenDistF8 / 256; max(s) * max(l_tanSteeringAngle_si32) == 131071 *
    // 16384 < 2^31
    if ((UInt32)g_mtl_Abs_mac(f_DrivenDistF8_si32) > (UInt32)131071)
    {
        f_DrivenDistF8_si32 = g_mtl_ReducePrecision_si32(f_DrivenDistF8_si32, (UInt8)8);
        l_X_bl              = TRUE;
    }

    // calculate delta x/y using current orientation and driven distance
    (*f_XDeltaF8_mm_psi32) =
        (f_DrivenDistF8_si32 * (SInt32)g_mtl_Cos_si16((SInt16)f_PsiF12_rad_ui16));
    (*f_YDeltaF8_mm_psi32) =
        (f_DrivenDistF8_si32 * (SInt32)g_mtl_Sin_si16((SInt16)f_PsiF12_rad_ui16));

    if (l_X_bl == FALSE)
    {
        (*f_XDeltaF8_mm_psi32) = g_mtl_ReducePrecision_si32((*f_XDeltaF8_mm_psi32),
                                                            (UInt8)14); // F22 => F8
        (*f_YDeltaF8_mm_psi32) = g_mtl_ReducePrecision_si32((*f_YDeltaF8_mm_psi32),
                                                            (UInt8)14); // F22 => F8
    }
    else
    {
        (*f_XDeltaF8_mm_psi32) = g_mtl_ReducePrecision_si32((*f_XDeltaF8_mm_psi32),
                                                            (UInt8)6); // F14 => F8
        (*f_YDeltaF8_mm_psi32) = g_mtl_ReducePrecision_si32((*f_YDeltaF8_mm_psi32),
                                                            (UInt8)6); // F14 => F8
    }
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    g_VHOCtrlStat_st.CrdUpdCrv_bl = FALSE;
#endif
}

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief
 * @details       calculates x-,y- deltas using approximation by circle arcs
 *
 * @param         f_XDeltaF8_mm_psi32
 * @param         f_YDeltaF8_mm_psi32
 * @param         f_DrivenDistF8_si32
 * @param         f_PsiF12_rad_si16
 * @param         l_DeltaPsiF22_rad_si32
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY void m_VHOGetXYDeltaApproxCurve_vd(SInt32 *f_XDeltaF8_mm_psi32,
                                              SInt32 *f_YDeltaF8_mm_psi32,
                                              SInt32 f_DrivenDistF8_si32,
                                              SInt16 f_PsiF12_rad_si16,
                                              SInt32 l_DeltaPsiF22_rad_si32)
{
    SInt32 l_S_si32;
    SInt16 l_BetaF12_rad_si16;

    if (l_DeltaPsiF22_rad_si32 != 0)
    {
        // d_phi/2
        l_BetaF12_rad_si16 =
            (SInt16)g_mtl_ReducePrecision_si32(l_DeltaPsiF22_rad_si32, (UInt8)11); // F12

        // calculate the driven distance s of the current cycle
        //
        //         L  * sin(d_phi/2)
        // s = 2 * --------------------
        //         tan(steering_angle)
        //

        l_S_si32 = (SInt32)g_mtl_Sin_si16(l_BetaF12_rad_si16) * (SInt8)2; // F15
        // max(WheelBase)=2^13 --> 2^13 * 2^15 = 2^28
        l_S_si32 *= (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16;
        //(2^28/2^13) * 2^5 = 2^20 < 2^31
        l_S_si32 = g_mtl_s32_Div_s32_si32(
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
                       l_S_si32, (SInt32)g_mtl_Tan_si16(
                                     g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16)
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
                       l_S_si32,
                       (SInt32)g_mtl_Tan_si16(g_VHOVehicleState_st.SAngRadF12_si16)
#endif
                           ) *
                   (SInt8)32; // F6

// consider rear wheel angle at four wheel steering
// rR = rMP/cos(rear wheel angle)
// rR = (WheelBaseVirtual/Tan(front wheel angle))/cos(rear wheel angle)
// ==> sR = s_Momentanpol / cos(rear wheel angle)
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
                              // 2^6 * 2^14 = 2^20 < 2^31
        l_S_si32 *= (SInt32)(16384);
        // 2^20 / 2^14 = 2^6
        l_S_si32 = g_mtl_s32_Div_s32_si32(
            l_S_si32, (SInt32)g_mtl_Cos_si16(
                          g_VHOCorrectedSignals_st.DelayedCorr_SAngRearRadF12_si16));
#endif
#endif

        // calculate beta
        //
        //               d_phi
        // beta = phi - -------
        //                 2

        l_BetaF12_rad_si16 = f_PsiF12_rad_si16 - l_BetaF12_rad_si16; // F12
        // over 360�
        if (l_BetaF12_rad_si16 > (SInt16)25735)
        {
            l_BetaF12_rad_si16 -= (SInt16)25735;
        }
    }
    else
    {
        l_S_si32 = g_mtl_ReducePrecision_si32(f_DrivenDistF8_si32, (UInt8)2); // F6
        l_BetaF12_rad_si16 = f_PsiF12_rad_si16;                               // F12
    }

    if (l_S_si32 > 131072)
    {
        // calculate delta x
        // max(l_S_si32) = 2^20 * 2^10 = 2^30 < 2^31
        (*f_XDeltaF8_mm_psi32) =
            l_S_si32 * g_mtl_ReducePrecision_si32(
                           (SInt32)g_mtl_Cos_si16(l_BetaF12_rad_si16), 4); // F16
        (*f_XDeltaF8_mm_psi32) =
            g_mtl_ReducePrecision_si32((*f_XDeltaF8_mm_psi32), (UInt8)8);
    }
    else
    {
        // calculate delta x
        (*f_XDeltaF8_mm_psi32) =
            l_S_si32 * (SInt32)g_mtl_Cos_si16(l_BetaF12_rad_si16); // F20
        (*f_XDeltaF8_mm_psi32) =
            g_mtl_ReducePrecision_si32((*f_XDeltaF8_mm_psi32), (UInt8)12);
    }

    if (l_S_si32 > 131072)
    {
        // calculate delta y
        // max(l_S_si32) = 2^20 * 2^10 = 2^30 < 2^31
        (*f_YDeltaF8_mm_psi32) =
            l_S_si32 * g_mtl_ReducePrecision_si32(
                           (SInt32)g_mtl_Sin_si16(l_BetaF12_rad_si16), 4); // F16
        (*f_YDeltaF8_mm_psi32) =
            g_mtl_ReducePrecision_si32(*f_YDeltaF8_mm_psi32, (UInt8)8);
    }
    else
    {
        // calculate delta y
        (*f_YDeltaF8_mm_psi32) =
            l_S_si32 * (SInt32)g_mtl_Sin_si16(l_BetaF12_rad_si16); // F20
        (*f_YDeltaF8_mm_psi32) =
            g_mtl_ReducePrecision_si32(*f_YDeltaF8_mm_psi32, (UInt8)12);
    }

    // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) -> complete function not
    // used in Mini_VHO
    g_VHOCtrlStat_st.CrdUpdCrv_bl = TRUE;
    // #endif
}
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         performs transformation from steering wheel angle to steering
angle
 * @details
 *
 * @param         f_SteeringWheelAngle_si16  steering wheel angle that has to be
converted;
 *                                           Scaling: 1/25 � [0.04�] ;
 *                                           Range:   ]-720�...720�[
 * @param         f_RollingForward_bl
 * @return        SInt16                     steering wheel angle;
|                                            Scaling: 1/4096rad ;
|                                            Range: ]-1.1rad(63�)..1.1rad(63�)[
 *
 * @note
 * @see
 * @warning
*/
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
SInt16 g_VHOConvertSWA2SA_si16(SInt16 f_SteeringWheelAngle_si16,
                               Boolean f_RollingForward_bl)
{
    SInt32 l_monomialA_si32;
    SInt32 l_monomialB_si32;
    SInt32 l_monomialC_si32;
    SInt32 l_SteeringWheelAngle_si32;
    SInt32 l_SteeringWheelAngleHighRes_si32;
    SInt16 l_RetVal_si16;

    if (f_SteeringWheelAngle_si16 > md_VHOMaxSteerWheelAng_si16)
    {
        // steering out of defined range (>720�) => set to maximum
        f_SteeringWheelAngle_si16 = md_VHOMaxSteerWheelAng_si16;
    }

    if (f_SteeringWheelAngle_si16 < -md_VHOMaxSteerWheelAng_si16)
    {
        // steering wheel angle is out of normal range (<-720�) => set to allowed
        // minimum
        f_SteeringWheelAngle_si16 = (SInt16)-md_VHOMaxSteerWheelAng_si16;
    }

    // F4=16
    l_SteeringWheelAngle_si32 =
        g_mtl_s32_Div_s32_si32(((SInt32)f_SteeringWheelAngle_si16 * (SInt32)16),
                               (SInt32)25); // SWA F4[�] (max = 11520)

    // 25
    l_SteeringWheelAngleHighRes_si32 =
        (SInt32)f_SteeringWheelAngle_si16; //[1/25�] (max = 18000)

    // 25
    l_monomialC_si32 = l_SteeringWheelAngleHighRes_si32; //(max = 18000)
    // 1
    l_monomialB_si32 =
        g_mtl_s32_Div_s32_si32(l_SteeringWheelAngle_si32 * l_SteeringWheelAngle_si32,
                               (SInt32)256); // SWA^2 F1[�] (max = 518400)
    // 1
    l_monomialA_si32 =
        l_monomialB_si32 *
        g_mtl_s32_Div_s32_si32(l_SteeringWheelAngle_si32,
                               (SInt32)16); // SWA^3 F1[�] (max = 373248000)

    if (f_RollingForward_bl != FALSE)
    {
        //--forward driving--
        // SWA^3 * VHOMultMonomA_ForwSWA2SA_si32(max. 1000) < 2^31, therefore SWA
        // smaller than threshold
        if ((UInt32)g_mtl_Abs_mac(l_SteeringWheelAngle_si32) > (UInt32)129)
        {
            // calculate first summand
            l_monomialA_si32 /= (SInt32)1000; // (max = 373248)
            // (max = 163482624)
            l_monomialA_si32 *=
                (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSWA2SA_si16;
        }
        else
        {
            // calculate first summand
            l_monomialA_si32 *=
                (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSWA2SA_si16;
            l_monomialA_si32 /= (SInt32)1000;
        }
        l_monomialA_si32 /=
            (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                ->ScalingMonomA_SWA2SA_bf3] /
             (SInt32)4); // F14Rad (max = 6539)

        // calculate second summand
        l_monomialB_si32 *= (SInt32)g_VHOParameters_st.VHO_Veh_pst
                                ->MultMonomB_ForwSWA2SA_si16; // (max = 224467200)
        l_monomialB_si32 /=
            (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                ->ScalingMonomB_SWA2SA_bf3] /
             (SInt32)4); // F14Rad (max = 89)

        // calculate third summand + fourth summand
        // input: max(CoeffC) = 100000  max(l_monomialC) = 18000
        l_monomialC_si32 *= g_VHOParameters_st.VHO_Veh_pst
                                ->CoeffC_ForwSWA2SA_si32; // (max = 1800000000 < 2^31)
        l_monomialC_si32 = g_mtl_s32_Div_s32_si32(l_monomialC_si32,
                                                  (SInt32)25); // (max = 72000000 < 2^27)
        l_monomialC_si32 +=
            g_VHOParameters_st.VHO_Veh_pst->CoeffD_ForwSWA2SA_si32; // (max = 38137600)
        l_monomialC_si32 = g_mtl_s32_Div_s32_si32(
            l_monomialC_si32,
            (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                ->ScalingMonomCD_SWA2SA_bf3] /
             (SInt32)4)); // F14Rad
    }
    else
    {
        //--backward driving--
        // SWA^3 * VHOMultMonomA_ForwSWA2SA_si32(max. 1000) < 2^31, therefore SWA
        // smaller than threshold
        if (g_mtl_Abs_mac(l_SteeringWheelAngle_si32) > 129)
        {
            // calculate first summand
            l_monomialA_si32 /= (SInt32)1000;
            l_monomialA_si32 *=
                (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSWA2SA_si16;
        }
        else
        {
            // calculate first summand
            l_monomialA_si32 *=
                (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSWA2SA_si16;
            l_monomialA_si32 /= (SInt32)1000;
        }
        l_monomialA_si32 = g_mtl_s32_Div_s32_si32(
            l_monomialA_si32,
            gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                               ->ScalingMonomA_SWA2SA_bf3] /
                (SInt32)4); // F14Rad

        // calculate second summand
        l_monomialB_si32 *=
            (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_BackSWA2SA_si16;
        l_monomialB_si32 = g_mtl_s32_Div_s32_si32(
            l_monomialB_si32,
            gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                               ->ScalingMonomB_SWA2SA_bf3] /
                (SInt32)4); // F14Rad

        // calculate third summand + fourth summand
        l_monomialC_si32 *= g_VHOParameters_st.VHO_Veh_pst->CoeffC_BackSWA2SA_si32;
        l_monomialC_si32 = g_mtl_s32_Div_s32_si32(l_monomialC_si32, (SInt32)25);
        l_monomialC_si32 += g_VHOParameters_st.VHO_Veh_pst->CoeffD_BackSWA2SA_si32;
        l_monomialC_si32 = g_mtl_s32_Div_s32_si32(
            l_monomialC_si32,
            gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                               ->ScalingMonomCD_SWA2SA_bf3] /
                (SInt32)4); // F14Rad
    }

    // add all monomials --> result of polynomial
    l_monomialA_si32 = g_mtl_s32_Div_s32_si32(
        l_monomialA_si32 + l_monomialB_si32 + l_monomialC_si32, (SInt32)4);

    if (l_monomialA_si32 > (SInt32)md_VHOMaxSteerAng_si16)
    {
        // resulting steering angle is bigger than 61� (defined max)
        // => set to defined max
        l_RetVal_si16 = md_VHOMaxSteerAng_si16;
    }
    else if (l_monomialA_si32 < (SInt32)-md_VHOMaxSteerAng_si16)
    {
        // resulting steering angle is bigger than 61� (defined min)
        // => set to defined min
        l_RetVal_si16 = (SInt16)-md_VHOMaxSteerAng_si16;
    }
    else
    {
        // resulting steering angle is valid
        l_RetVal_si16 = (SInt16)l_monomialA_si32;
    }
    return l_RetVal_si16;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         performs transformation from steering wheel angle to steering
 * angle
 * @details
 *
 * @param         f_SteeringAngle_si16   steering angle that has to be
 * converted; Scaling: 1/4096rad ; Range: ]-1.1rad(63�)..1.1rad(63�)[
 * @param         f_RollingForward_bl
 * @return        SInt16                 calculated steering wheel angle with
 * the resolution 0.04
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
SInt16 g_VHOConvertSA2SWA_si16(SInt16 f_SteeringAngle_si16, Boolean f_RollingForward_bl)
{
    SInt32 l_monomialA_si32;
    SInt32 l_monomialB_si32;
    SInt32 l_monomialC_si32;
    SInt16 l_RetVal_si16;

    // input over physical maximum ]-61�...+61�[ --> invalid
    if (f_SteeringAngle_si16 > md_VHOMaxSteerAng_si16)
    {
        // set current steering angle to positive maximum +61�
        f_SteeringAngle_si16 = md_VHOMaxSteerAng_si16;
    }

    if (f_SteeringAngle_si16 < (SInt16)-md_VHOMaxSteerAng_si16)
    {
        // set current steering angle to negative maximum -61�
        f_SteeringAngle_si16 = (SInt16)-md_VHOMaxSteerAng_si16;
    }

    l_monomialC_si32 = (SInt32)f_SteeringAngle_si16;
    l_monomialB_si32 =
        g_mtl_s32_Div_s32_si32(l_monomialC_si32 * (SInt32)f_SteeringAngle_si16,
                               (SInt32)16); // 1^10 rad

    l_monomialA_si32 =
        l_monomialB_si32 * g_mtl_s32_Div_s32_si32((SInt32)f_SteeringAngle_si16,
                                                  (SInt32)4); // 1^10 rad

    if (f_RollingForward_bl != FALSE)
    {
        if (f_SteeringAngle_si16 >= g_VHOParameters_st.VHO_Veh_pst->ClippingAngle1_si16)
        {
            if (f_SteeringAngle_si16 <=
                g_VHOParameters_st.VHO_Veh_pst->ClippingAngle2_si16)
            {
                // range around zero

                // SA^3 * md_VHOMultMonomA_ForwSA2SWAM_si32(max. 1000) < 2^31
                if ((UInt16)g_mtl_Abs_mac(f_SteeringAngle_si16) > (UInt16)129)
                {
                    l_monomialA_si32 /= (SInt32)1000;
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSA2SWAC_si16;
                }
                else
                {
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSA2SWAC_si16;
                    l_monomialA_si32 /= (SInt32)1000;
                }
                l_monomialA_si32 /= (SInt32)100; // avoid overflow
                l_monomialA_si32 *=
                    (SInt32)64; // 4^3 consequence of changed input resolution
                l_monomialA_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomA_SA2SWAC_bf3] /
                     (SInt32)100); //[Scaling = 0.04�]

                l_monomialB_si32 *=
                    (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_ForwSA2SWAC_si16;
                l_monomialB_si32 /= (SInt32)10; // avoid overflow
                l_monomialB_si32 *=
                    (SInt32)16; // 4^2 consequence of changed input resolution
                l_monomialB_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomB_SA2SWAC_bf3] /
                     (SInt32)10); //[Scaling = 0.04�]

                l_monomialC_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffC_ForwSA2SWAC_si32;
                l_monomialC_si32 +=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffD_ForwSA2SWAC_si32;
                l_monomialC_si32 /=
                    gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                       ->ScalingMonomCD_SA2SWAC_bf3];
            }
            else
            {
                // range right

                // SA^3 * md_VHOMultMonomA_ForwSA2SWAL_si32(max. 1000) < 2^31
                if ((UInt16)g_mtl_Abs_mac(f_SteeringAngle_si16) > (UInt16)129)
                {
                    l_monomialA_si32 /= (SInt32)1000;
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSA2SWAR_si16;
                }
                else
                {
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSA2SWAR_si16;
                    l_monomialA_si32 /= (SInt32)1000;
                }

                l_monomialA_si32 /= (SInt32)100; // avoid overflow
                l_monomialA_si32 *=
                    (SInt32)64; // 4^3 consequence of changed input resolution
                l_monomialA_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomA_SA2SWAR_bf3] /
                     (SInt32)100); //[Scaling = 0.04�]

                l_monomialB_si32 *=
                    (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_ForwSA2SWAR_si16;
                l_monomialB_si32 /= (SInt32)10; // avoid overflow
                l_monomialB_si32 *=
                    (SInt32)16; // 4^2 consequence of changed input resolution
                l_monomialB_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomB_SA2SWAR_bf3] /
                     (SInt32)10); //[Scaling = 0.04�]

                l_monomialC_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffC_ForwSA2SWAR_si32;
                l_monomialC_si32 +=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffD_ForwSA2SWAR_si32;
                l_monomialC_si32 /=
                    gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                       ->ScalingMonomCD_SA2SWAR_bf3];
            }
        } // close f_SteeringAngle_si16 >= md_VHOClippingAngle1_ForwSA2SWA_si16
        else
        {
            // range left

            // SA^3 * md_VHOMultMonomA_ForwSA2SWAL_si32(max. 1000) < 2^31
            if ((UInt16)g_mtl_Abs_mac(f_SteeringAngle_si16) > (UInt16)129)
            {
                l_monomialA_si32 /= 1000;
                l_monomialA_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSA2SWAL_si16;
            }
            else
            {
                l_monomialA_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->MultMonomA_ForwSA2SWAL_si16;
                l_monomialA_si32 /= 1000;
            }

            l_monomialA_si32 /= 100; // avoid overflow
            l_monomialA_si32 *= 64;  // 4^3 consequence of changed input resolution
            l_monomialA_si32 /=
                (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                    ->ScalingMonomA_SA2SWAL_bf3] /
                 (SInt32)100); //[Scaling = 0.04�]

            l_monomialB_si32 *=
                (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_ForwSA2SWAL_si16;
            l_monomialB_si32 /= (SInt32)10; // avoid overflow
            l_monomialB_si32 *= (SInt32)16; // 4^2 consequence of changed input resolution
            l_monomialB_si32 /=
                (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                    ->ScalingMonomB_SA2SWAL_bf3] /
                 (SInt32)10); //[Scaling = 0.04�]

            l_monomialC_si32 *= g_VHOParameters_st.VHO_Veh_pst->CoeffC_ForwSA2SWAL_si32;
            l_monomialC_si32 += g_VHOParameters_st.VHO_Veh_pst->CoeffD_ForwSA2SWAL_si32;
            l_monomialC_si32 /=
                gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                   ->ScalingMonomCD_SA2SWAL_bf3];
        }
    }
    else
    {
        // reverse movement

        if (f_SteeringAngle_si16 >= g_VHOParameters_st.VHO_Veh_pst->ClippingAngle1_si16)
        {
            if (f_SteeringAngle_si16 <=
                g_VHOParameters_st.VHO_Veh_pst->ClippingAngle2_si16)
            {
                // range around zero

                // SA^3 * md_VHOMultMonomA_ForwSA2SWAM_si32(max. 1000) < 2^31
                if ((UInt16)g_mtl_Abs_mac(f_SteeringAngle_si16) > (UInt16)129)
                {
                    l_monomialA_si32 /= (SInt32)1000;
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSA2SWAC_si16;
                }
                else
                {
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSA2SWAC_si16;
                    l_monomialA_si32 /= (SInt32)1000;
                }
                l_monomialA_si32 /= (SInt32)100; // avoid overflow
                l_monomialA_si32 *=
                    (SInt32)64; // 4^3 consequence of changed input resolution
                l_monomialA_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomA_SA2SWAC_bf3] /
                     (SInt32)100); //[Scaling = 0.04�]

                l_monomialB_si32 *=
                    (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_BackSA2SWAC_si16;
                l_monomialB_si32 /= (SInt32)10; // avoid overflow
                l_monomialB_si32 *=
                    (SInt32)16; // 4^2 consequence of changed input resolution
                l_monomialB_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomB_SA2SWAC_bf3] /
                     (SInt32)10); //[Scaling = 0.04�]

                l_monomialC_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffC_BackSA2SWAC_si32;
                l_monomialC_si32 +=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffD_BackSA2SWAC_si32;
                l_monomialC_si32 /=
                    gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                       ->ScalingMonomCD_SA2SWAC_bf3];
            }
            else
            {
                // range right

                // SA^3 * md_VHOMultMonomA_ForwSA2SWAL_si32(max. 1000) < 2^31
                if ((UInt16)g_mtl_Abs_mac(f_SteeringAngle_si16) > (UInt16)129)
                {
                    l_monomialA_si32 /= (SInt32)1000;
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSA2SWAR_si16;
                }
                else
                {
                    l_monomialA_si32 *=
                        (SInt32)
                            g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSA2SWAR_si16;
                    l_monomialA_si32 /= (SInt32)1000;
                }

                l_monomialA_si32 /= (SInt32)100; // avoid overflow
                l_monomialA_si32 *=
                    (SInt32)64; // 4^3 consequence of changed input resolution
                l_monomialA_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomA_SA2SWAR_bf3] /
                     (SInt32)100); //[Scaling = 0.04�]

                l_monomialB_si32 *=
                    (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_BackSA2SWAR_si16;
                l_monomialB_si32 /= (SInt32)10; // avoid overflow
                l_monomialB_si32 *=
                    (SInt32)16; // 4^2 consequence of changed input resolution
                l_monomialB_si32 /=
                    (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                        ->ScalingMonomB_SA2SWAR_bf3] /
                     (SInt32)10); //[Scaling = 0.04�]

                l_monomialC_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffC_BackSA2SWAR_si32;
                l_monomialC_si32 +=
                    g_VHOParameters_st.VHO_Veh_pst->CoeffD_BackSA2SWAR_si32;
                l_monomialC_si32 /=
                    gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                       ->ScalingMonomCD_SA2SWAR_bf3];
            }
        } // close f_SteeringAngle_si16 >= md_VHOClippingAngle1_ForwSA2SWA_si16
        else
        {
            // range left

            // SA^3 * md_VHOMultMonomA_ForwSA2SWAL_si32(max. 1000) < 2^31
            if ((UInt16)g_mtl_Abs_mac(f_SteeringAngle_si16) > (UInt16)129)
            {
                l_monomialA_si32 /= 1000;
                l_monomialA_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSA2SWAL_si16;
            }
            else
            {
                l_monomialA_si32 *=
                    g_VHOParameters_st.VHO_Veh_pst->MultMonomA_BackSA2SWAL_si16;
                l_monomialA_si32 /= 1000;
            }

            l_monomialA_si32 /= 100; // avoid overflow
            l_monomialA_si32 *= 64;  // 4^3 consequence of changed input resolution
            l_monomialA_si32 /=
                (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                    ->ScalingMonomA_SA2SWAL_bf3] /
                 (SInt32)100); //[Scaling = 0.04�]

            l_monomialB_si32 *=
                (SInt32)g_VHOParameters_st.VHO_Veh_pst->MultMonomB_BackSA2SWAL_si16;
            l_monomialB_si32 /= (SInt32)10; // avoid overflow
            l_monomialB_si32 *= (SInt32)16; // 4^2 consequence of changed input resolution
            l_monomialB_si32 /=
                (gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                    ->ScalingMonomB_SA2SWAL_bf3] /
                 (SInt32)10); //[Scaling = 0.04�]

            l_monomialC_si32 *= g_VHOParameters_st.VHO_Veh_pst->CoeffC_BackSA2SWAL_si32;
            l_monomialC_si32 += g_VHOParameters_st.VHO_Veh_pst->CoeffD_BackSA2SWAL_si32;
            l_monomialC_si32 /=
                gc_MonomialScalingFactor_psi32[g_VHOParameters_st.VHO_Veh_pst
                                                   ->ScalingMonomCD_SA2SWAL_bf3];
        }
    }

    // add all monomials --> result of polynomial
    l_monomialA_si32 = (l_monomialA_si32 + l_monomialB_si32 + l_monomialC_si32);

    if (l_monomialA_si32 > (SInt32)md_VHOMaxSteerWheelAng_si16)
    {
        // resulting steering wheel angle is bigger than 720� - physical invalid
        l_RetVal_si16 =
            md_VHOMaxSteerWheelAng_si16; // set return value to max. defined value
    }
    else if (l_monomialA_si32 < (SInt32)-md_VHOMaxSteerWheelAng_si16)
    {
        // resulting steering wheel angle is smaller than -720� - physical invalid
        l_RetVal_si16 = (SInt16)-md_VHOMaxSteerWheelAng_si16; // set return value to
                                                              // max. defined value
    }
    else
    {
        // resulting steering wheel angle is valid
        l_RetVal_si16 = (SInt16)l_monomialA_si32;
    }
    return l_RetVal_si16;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         convert steering angle (wheel) to kappa
 *                (reciprocal value of the rear axle center radius)
 * @details
 *
 * @param         f_SteeringAngleRadF12_si16   angle of the virtual middle front
 * wheel
 * @return        SInt16                       unit:       1/m;
 *                                             resolution: 1/16384
 *
 * @note
 * @see
 * @warning
 */
SInt16 g_VHOGetCurvatureFromSteeringAngle_si16(SInt16 f_SteeringAngleRadF12_si16)
{
    SInt32 l_X_si32;

    if (f_SteeringAngleRadF12_si16 > md_VHOMaxSteerAng_si16)
    {
        f_SteeringAngleRadF12_si16 = md_VHOMaxSteerAng_si16;
    }

    if (f_SteeringAngleRadF12_si16 < (SInt16)-md_VHOMaxSteerAng_si16)
    {
        f_SteeringAngleRadF12_si16 = (SInt16)-md_VHOMaxSteerAng_si16;
    }

    // calculate curvature (kappa) and curve radius (cm)
    l_X_si32 = // F13
        (SInt32)g_mtl_Tan_si16(f_SteeringAngleRadF12_si16);
    l_X_si32 *= (SInt32)2; // F14
    l_X_si32 =             // F14
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        g_mtl_s32_Div_s32_si32((l_X_si32 * (SInt32)1000),
                               (SInt32)g_VHOParameters_st.VHO_Veh_pst->WheelBase_ui16);
#else
        g_mtl_s32_Div_s32_si32((l_X_si32 * (SInt32)1000),
                               (SInt32)g_parGetParaVHO_Veh_WheelBase_ui16);
#endif

    if (g_mtl_Abs_mac(l_X_si32) > (SInt32)md_VHOMaxKappaMF14_si16)
    {
        // kappa out of range
        l_X_si32 = (SInt32)md_VHOMaxKappaMF14_si16;
    }

    return (SInt16)l_X_si32;
}

#if (GS_4WS_VEHICLE == SW_ON)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         convert steering angle (wheel) and measured sterring angle
 * rear to kappa.
 * @details
 *
 * @param         f_SteeringAngleRadF12_si16 angle of the virtual middle front
 * wheel
 * @return        SInt16                     unit:       1/m;
 *                                           resolution: 1/16384
 *
 * @note
 * @see
 * @warning
 */
SInt16 g_VHOGetCurvatureFrPSCctualSteeringAngle_si16(SInt16 f_SteeringAngleRadF12_si16)
{
    SInt32 l_X_si32;

    if (f_SteeringAngleRadF12_si16 > md_VHOMaxSteerAng_si16)
    {
        f_SteeringAngleRadF12_si16 = md_VHOMaxSteerAng_si16;
    }

    if (f_SteeringAngleRadF12_si16 < (SInt16)-md_VHOMaxSteerAng_si16)
    {
        f_SteeringAngleRadF12_si16 = (SInt16)-md_VHOMaxSteerAng_si16;
    }

    // calculate curvature (kappa) and curve radius (cm)
    l_X_si32 = // F13
        (SInt32)g_mtl_Tan_si16(f_SteeringAngleRadF12_si16);
    l_X_si32 *= (SInt32)2; // F14
    l_X_si32 =             // F14
        g_mtl_s32_Div_s32_si32((l_X_si32 * (SInt32)1000),
                               (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16);

    if (g_mtl_Abs_mac(l_X_si32) > (SInt32)md_VHOMaxKappaMF14_si16)
    {
        // kappa out of range
        l_X_si32 = (SInt32)md_VHOMaxKappaMF14_si16;
    }

    return (SInt16)l_X_si32;
}
#endif

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         transform coordinates x,y, phi
 * @details
 *
 * @param         fc_VHORollRecogBuf_pst
 * @param         f_DeltaValues_pst
 * @param         f_YawAngleRadF10_si16
 * @return
 *
 * @note
 * @see
 * @warning
 */
void m_VHOOdoCoreCoordTransform_vd(const gType_VHORollRecog_st *fc_VHORollRecogBuf_pst,
                                   mType_DeltaValues_st *f_DeltaValues_pst,
                                   SInt16 f_YawAngleRadF10_si16)
{
    SInt16 l_X_si16;
    SInt16 l_Y_si16;

    SInt16 l_AngleRadF10_si16;
    UInt16 l_RadiusMM_ui16;

    const SInt16 lc_AngleRadF10_180_si16 = (SInt16)3217;
    const SInt16 lc_AngleRadF10_360_si16 = (SInt16)6434;

    // scale inputs down to mm
    l_X_si16 =
        (SInt16)g_mtl_ReducePrecision_si32(fc_VHORollRecogBuf_pst->dx_buf_MMF8_si32, 8);

    l_Y_si16 =
        (SInt16)g_mtl_ReducePrecision_si32(fc_VHORollRecogBuf_pst->dy_buf_MMF8_si32, 8);

    // calculate distance between point of origin and (x,y)
    l_RadiusMM_ui16 = g_mtl_Sqrt_u32_ui16((UInt32)((SInt32)l_X_si16 * (SInt32)l_X_si16) +
                                          (UInt32)((SInt32)l_Y_si16 * (SInt32)l_Y_si16));

    // calculate angle between the x-axis and the line
    // between (x,y) and the point of origin
    l_AngleRadF10_si16 = m_VHOGetAngleFromXY_si16(l_X_si16, l_Y_si16);

    // calculate the difference between the returned value and the yaw angle
    l_AngleRadF10_si16 -= f_YawAngleRadF10_si16;

    // in case that difference is big an overflow is assumed
    if (l_AngleRadF10_si16 < -lc_AngleRadF10_180_si16)
    {
        l_AngleRadF10_si16 += lc_AngleRadF10_360_si16;
    }
    else
    {
        if (l_AngleRadF10_si16 > lc_AngleRadF10_180_si16)
        {
            l_AngleRadF10_si16 -= lc_AngleRadF10_360_si16;
        }
    }
    l_AngleRadF10_si16 =
        (f_YawAngleRadF10_si16 + lc_AngleRadF10_180_si16) - l_AngleRadF10_si16;

    if (l_AngleRadF10_si16 > lc_AngleRadF10_360_si16)
    {
        l_AngleRadF10_si16 -= lc_AngleRadF10_360_si16;
    }

    f_DeltaValues_pst->XF8_mm_si32 =
        (SInt32)g_mtl_Cos_si16(l_AngleRadF10_si16 * (SInt16)4); // F14
    f_DeltaValues_pst->XF8_mm_si32 =
        g_mtl_ReducePrecision_si32(f_DeltaValues_pst->XF8_mm_si32, 6); // F8
    f_DeltaValues_pst->XF8_mm_si32 *= (SInt32)l_RadiusMM_ui16;         // F8

    f_DeltaValues_pst->YF8_mm_si32 =
        (SInt32)g_mtl_Sin_si16(l_AngleRadF10_si16 * (SInt16)4); // F14
    f_DeltaValues_pst->YF8_mm_si32 =
        g_mtl_ReducePrecision_si32(f_DeltaValues_pst->YF8_mm_si32, 6); // F8
    f_DeltaValues_pst->YF8_mm_si32 *= (SInt32)l_RadiusMM_ui16;         // F8
}

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         provides the angle between the x-axis and the line between a
 * given point (x,y) and the point of origin
 * @details
 *
 * @param         f_X_si16
 * @param         f_Y_si16
 * @return        SInt16     angle between 0..2pi; rad F10
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY SInt16 m_VHOGetAngleFromXY_si16(SInt16 f_X_si16, SInt16 f_Y_si16)
{
    SInt16 l_AngleRadF10_si16;
    const SInt16 lc_AngleRadF10_90_si16  = (SInt16)1608;
    const SInt16 lc_AngleRadF10_180_si16 = (SInt16)3217;
    const SInt16 lc_AngleRadF10_270_si16 = (SInt16)4825;
    // const SInt16 lc_AngleRadF10_360_si16 = (SInt16) 6434;

    if ((f_X_si16 >= 0) && (f_Y_si16 == 0))
    {
        // angle = 0�;
        l_AngleRadF10_si16 = (SInt16)0;
    }
    else if ((f_X_si16 == 0) && (f_Y_si16 > 0))
    {
        // angle = 90�
        l_AngleRadF10_si16 = lc_AngleRadF10_90_si16;
    }
    else if ((f_X_si16 < 0) && (f_Y_si16 == 0))
    {
        // angle = 180�
        l_AngleRadF10_si16 = lc_AngleRadF10_180_si16;
    }
    else if ((f_X_si16 == 0) && (f_Y_si16 < 0))
    {
        // angle = 270�
        l_AngleRadF10_si16 = lc_AngleRadF10_270_si16;
    }
    else
    {
        // arctan2 --> return 2^22 * 2pi
        l_AngleRadF10_si16 = (SInt16)g_mtl_ReducePrecision_si32(
            (SInt32)g_mtl_ArcTan2_ui32((SInt32)f_Y_si16, (SInt32)f_X_si16),
            (UInt8)12); // F22-F12=F10
    }
    return l_AngleRadF10_si16;
}

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         mapping of a steering wheel angle to a corresponding kappa
 *                with consideration of the rolling direction
 * @details
 *
 * @param         f_SWARadF11_si16       steering wheel angle [1/2^11 rad];
 *                                       (RIGHT-, LEFT+)
 * @param         f_RollingForward_bl    TRUE: vehicle is rolling forward;
 *                                       FALSE: vehicle is rolling backwards
 * @return        SInt16                 unit:       1/m;
 *                                       resolution: 1/16384
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
SInt16 g_VHOGetKappaFromSWA_si16(SInt16 f_SWARadF11_si16, Boolean f_RollingForward_bl)
{
    SInt32 l_X_si32;

    if (f_SWARadF11_si16 > md_VHOMaxSteerWheelAngRad_si16)
    {
        f_SWARadF11_si16 = md_VHOMaxSteerWheelAngRad_si16;
    }

    if (f_SWARadF11_si16 < (SInt16)-md_VHOMaxSteerWheelAngRad_si16)
    {
        f_SWARadF11_si16 = (SInt16)-md_VHOMaxSteerWheelAngRad_si16;
    }

    // convert input resolution to 0.04� resolution
    //                180       1
    //  = inputres  * --- * ----------
    //                PI    0.04 * 2^11
    // max(f_SWARadF11_si16) = md_VHOMaxSteerWheelAngRad_si16 [F11Rad] * 69941 <
    // 2^31
    l_X_si32 = g_mtl_s32_Div_s32_si32((SInt32)f_SWARadF11_si16 * (SInt32)69941,
                                      (SInt32)100000); // max(l_X_si16) = 18000

    // correct SWA in cause of an mechanical offset at the steering wheel

#if ((LS_VHO_CORRECTED_SWA_XPG == SW_ON) && (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
    if (g_VHOAutocalibCtrl_st.SWAState_enm == VHOCalibState_STABLE_enm)
    {
        l_X_si32 = l_X_si32 - (SInt32)g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
    }
#endif

    // calculate steering angle
    // map swa to sa [radF12]; max = +/-4361 --> SA=+/-61� (SWA=+/-720�)
    l_X_si32 = g_VHOConvertSWA2SA_si16((SInt16)l_X_si32, f_RollingForward_bl);

    // calculate kappa
    l_X_si32 = g_VHOGetCurvatureFromSteeringAngle_si16((SInt16)l_X_si32);

    return (SInt16)l_X_si32;
}
#endif

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         mapping of a steering wheel angle to a corresponding kappa
 *                with consideration of the rolling direction
 * @details
 *
 * @param         f_KappaMF14_si16       kappa [1/2^14 1/m];
 *                                       (RIGHT-, LEFT+)
 * @param         f_RollingForward_bl    TRUE: vehicle is rolling forward;
 *                                       FALSE: vehicle is rolling backwards
 * @return        SInt16                 calculated steering wheel angle [1/2^11
 * rad]
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
SInt16 g_VHOGetSWAFromKappa_si16(SInt16 f_KappaMF14_si16, Boolean f_RollingForward_bl)
{
    SInt32 l_temp_si32;
    UInt32 l_temp_ui32;

    if (f_KappaMF14_si16 > md_VHOMaxKappaMF14_si16)
    {
        f_KappaMF14_si16 = md_VHOMaxKappaMF14_si16;
    }

    if (f_KappaMF14_si16 < (SInt16)-md_VHOMaxKappaMF14_si16)
    {
        f_KappaMF14_si16 = (SInt16)-md_VHOMaxKappaMF14_si16;
    }

    /* ----------------------------------------------
    |           tan(steering_angle)    1
    |   kappa = ------------------- = ---
    |                   L              r
    |
    |   steering_angle = arctan(kappa * L / 1000)
    -------------------------------------------------*/
    l_temp_si32 = (SInt32)f_KappaMF14_si16 *
                  (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16; // F14

    // input of arctan is 2^8
    l_temp_ui32 =
        g_mtl_ArcTan2_ui32(l_temp_si32, (SInt32)16384000); // 2^12rad --> steering angle

    // transform to 16 bit resolution
    f_KappaMF14_si16 = (SInt16)(l_temp_ui32 >> (gd_MTL_ANGLE_SCALE_BITS_u32_ui8 -
                                                gd_MTL_ANGLE_SCALE_BITS_u16_ui8));

    // transform angle range [0, pi/2], [3/2pi, 2pi] -> [-pi/2, pi/2]
    if (f_KappaMF14_si16 > ((SInt32)gd_MTL_PI_ui16))
    {
        f_KappaMF14_si16 -= ((SInt16)gd_MTL_2_PI_ui16);
    }

    // input of g_VHOConvertSA2SWA_si16 for steering angle is 2^12
    f_KappaMF14_si16 =
        g_VHOConvertSA2SWA_si16(f_KappaMF14_si16, f_RollingForward_bl); //[0.04�]

// correct SWA in cause of an mechanical offset at the steering wheel
#if ((LS_VHO_CORRECTED_SWA_XPG == SW_ON) && (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
    if (g_VHOAutocalibCtrl_st.SWAState_enm == VHOCalibState_STABLE_enm)
    {
        f_KappaMF14_si16 = f_KappaMF14_si16 + g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
    }
#endif

    // convert [0.04�] resolution to 1/2^11 rad
    //                    PI
    // outputres = 0.04 * ---  * 2^11 = 1.429773723 ~ 1.4298
    //                    180
    f_KappaMF14_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
        (SInt32)f_KappaMF14_si16 * (SInt32)14298, (SInt32)10000);
    return f_KappaMF14_si16;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         mapping of a given radius to a corresponding steering
 *                wheel angle with consideration of the rolling direction
 * @details
 *
 * @param         f_RadiusCM_si16        radius that is driven at the center;
 *                                       of the rear axle [cm] - (RIGHT-, LEFT+)
 * @param         f_RollingForward_bl    TRUE: vehicle is rolling forward;
 *                                       FALSE: vehicle is rolling backwards
 * @return        SInt16                 calculated steering wheel angle [1/2^11
 * rad]
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
SInt16 g_VHOGetSWAFromRadius_si16(SInt16 f_RadiusCM_si16, Boolean f_RollingForward_bl)
{
    SInt32 l_X_si32;
    UInt32 l_temp_ui32;

    // map radius to steering angle
    if ((UInt16)g_mtl_Abs_mac(f_RadiusCM_si16) <= (UInt16)3)
    {
        // invalid parameter value
        l_X_si32 = 0; // --> invalid
    }
    else
    {
        if ((SInt16)g_mtl_Abs_mac(f_RadiusCM_si16) == (SInt16)32767)
        {
            // vehicle is driving straight
            l_X_si32 = 0;
        }
        else // vehicle is not driving straight
        {
            // map radius to steering angle by calculation of
            //                          wheel_base
            // steering_angle = arctan  ----------
            //                          radius
            //

            // input of arctan is 2^8
            l_temp_ui32 =
                g_mtl_ArcTan2_ui32((SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16,
                                   (SInt32)f_RadiusCM_si16 * (SInt32)10); // F22

            // transform to 16 bit resolution
            l_X_si32 = (SInt16)(l_temp_ui32 >> (gd_MTL_ANGLE_SCALE_BITS_u32_ui8 -
                                                gd_MTL_ANGLE_SCALE_BITS_u16_ui8)); // F16

            // transform angle range [0, pi/2], [3/2pi, 2pi] -> [-pi, pi]
            if (l_X_si32 > ((SInt32)gd_MTL_PI_ui16))
            {
                l_X_si32 -= ((SInt16)gd_MTL_2_PI_ui16);
            }

            /* in case of negative radii the resulting angle is shifted by pi --> back
             * shifting */
            if (f_RadiusCM_si16 < (SInt16)0)
            {
                l_X_si32 -= (SInt16)gd_MTL_PI_ui16;
            }
        }

        // map steering angle to steering wheel angle in 0.04�
        // input resolution F12
        l_X_si32 = (SInt32)g_VHOConvertSA2SWA_si16(
            (SInt16)l_X_si32, f_RollingForward_bl); // [0.04�] - max = 1600

// correct SWA in cause of an mechanical offset at the steering wheel
#if ((LS_VHO_CORRECTED_SWA_XPG == SW_ON) && (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
        if (g_VHOAutocalibCtrl_st.SWAState_enm == VHOCalibState_STABLE_enm)
        {
            l_X_si32 = l_X_si32 + g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
        }
#endif

        if ((SInt16)g_mtl_Abs_mac((SInt16)l_X_si32) <= md_VHOMaxSteerWheelAng_si16)
        {
            // result is valid => steering wheel angle is in the range [-720�...+720�]
            // convert [0.04�] resolution to 1/2^11 rad
            //                    PI
            // outputres = 0.04 * ---  * 2^11 = 1.429773723 ~ 1.4298
            //                    180
            l_X_si32 = g_mtl_s32_Div_s32_si32(l_X_si32 * (SInt32)14298, (SInt32)10000);
        }
        else
        {
            // resulting steering wheel angle is out of range ]-720�...+720�[
            // (should normaly not possible because SA-->SWA eliminate this kind of
            // errors)
            l_X_si32 = 0; // --> invalid
        }
    }

    return (SInt16)l_X_si32;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         mapping of a given steering wheel angle to a
 *                corresponding radius with consideration of the rolling
 * direction
 * @details
 *
 * @param         f_SWARadF11_si16         steering wheel angle [1/2^11 rad];
 *                                         (RIGHT-, LEFT+)
 * @param         f_RollingForward_bl      TRUE: vehicle is rolling forward;
 *                                         FALSE: vehicle is rolling backwards
 * @return        SInt16                   radius that is driven at the center
 *                                         of the rear axle [cm] - (RIGHT-,
 * LEFT+); 0x7FFF: infinite radius (driving straight) 0: result invalid
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
SInt16 g_VHOGetRadiusFromSWA_si16(SInt16 f_SWARadF11_si16, Boolean f_RollingForward_bl)
{
    SInt32 l_X_si32;

    if (f_SWARadF11_si16 > md_VHOMaxSteerWheelAngRad_si16)
    {
        f_SWARadF11_si16 = md_VHOMaxSteerWheelAngRad_si16;
    }

    if (f_SWARadF11_si16 < (SInt16)-md_VHOMaxSteerWheelAngRad_si16)
    {
        f_SWARadF11_si16 = (SInt16)-md_VHOMaxSteerWheelAngRad_si16;
    }

    // convert input resolution to 0.04� resolution
    //                180       1
    //  = inputres  * --- * ----------
    //                PI    0.04 * 2^11
    // max(f_SWARadF11_si16) = md_VHOMaxSteerWheelAngRad_si16 [F11Rad] * 69941 <
    // 2^31
    l_X_si32 = g_mtl_s32_Div_s32_si32((SInt32)f_SWARadF11_si16 * (SInt32)69941,
                                      (SInt32)100000); // max(l_X_si16) = 18000

// correct SWA in cause of an mechanical offset at the steering wheel
#if ((LS_VHO_CORRECTED_SWA_XPG == SW_ON) && (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))
    if (g_VHOAutocalibCtrl_st.SWAState_enm == VHOCalibState_STABLE_enm)
    {
        l_X_si32 = l_X_si32 - g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
    }
#endif

    // map steering wheel angle to steering angle [radF12]
    l_X_si32 = g_VHOConvertSWA2SA_si16(
        (SInt16)l_X_si32,
        f_RollingForward_bl); // max = +/-4361 --> SA=+/-61� (SWA=+/-720�)

    if (l_X_si32 == 0)
    {
        // steering angle is zero => infinite radius
        l_X_si32 = 0x7FFF;
    }
    else
    {
        // convert steering angle to radius
        // calculate tan value of steering angle

        // max_input =5325 --> 1.3 rad        max_output = 27497
        // min_input =   1 --> 0.000244 rad   min_output = 2
        l_X_si32 = (SInt32)g_mtl_Tan_si16((SInt16)l_X_si32); // F13
        l_X_si32 *= (SInt32)2;                               // F14 max = 54994

        //
        // g_VHOParameters_st.VehData_st.WheelBaseMM_ui16[mm] * 2^19
        // ------------------------------------------------- = RadiusCM
        // 10 * tan(f_SteeringAngleRadF12_si16) * 2^14 * 2^5
        //
        // max (g_VHOParameters_st.VehData_st.WheelBaseMM_ui16) <= 4096 = 2^12
        // => max(g_VHOParameters_st.VehData_st.WheelBaseMM_ui16 * 2^19) = 2^31
        //
        // --> 32Bit Division --> cast in 16Bit value
        //

        l_X_si32 = g_mtl_s32_Div_s32_si32(
            (SInt32)g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 * (SInt32)524288,
            l_X_si32 * (SInt32)320);

        if (l_X_si32 > (SInt32)32767)
        {
            // almost infinite radius or rather the radius is bigger than 327.68m
            l_X_si32 = 0x7FFF;
        }

        if (l_X_si32 < (SInt32)-32767)
        {
            // almost infinite radius or rather the radius is bigger than -327.68m
            l_X_si32 = -0x7FFF;
        }
    }
    return (SInt16)l_X_si32;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/

/**
 * @brief         (calculate l_X_si32/2^n without rounding)
 *                (using >> may lead to logical OR arithmetic shift)
 * @details
 *
 * @param         f_X_si32   original value
 * @param         f_N_ui8    number of bits to be shifted
 * @return        SInt32
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY SInt32 m_VHOShiftRightSignedInt_si32(SInt32 f_X_si32, UInt8 f_N_ui8)
{
    UInt32 l_X_ui32;
    SInt32 l_RetVal_si32;

    if ((f_N_ui8 == 0) || (f_N_ui8 > 16))
    {
        // number of bits to be reduced out of range
        l_RetVal_si32 = (SInt32)0;
    }
    else
    {
        if (f_X_si32 < (SInt32)0)
        {
            l_X_ui32      = (UInt32)(-f_X_si32);
            l_RetVal_si32 = -((SInt32)(l_X_ui32 >> f_N_ui8));
        }
        else
        {
            l_X_ui32      = (UInt32)f_X_si32;
            l_RetVal_si32 = (SInt32)(l_X_ui32 >> f_N_ui8);
        }
    }
    return l_RetVal_si32;
}

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief
 * @details       calculation of steering angle rear correspondeding to steering
 * wheel angle, maximum steering wheel angle that is reachable by EPS - it
 *                depends on customer - and maximum steering angle rear - it
 * depends on customer too -.
 *
 *                For instance:
 *                SWAMax =   525� ==> SWAMaxdeg25 =  13125
 *                SARearMax =  0� ==> SARearMaxdeg50 =   0  (Renault)
 *                SARearMax = 10� ==> SARearMaxdeg50 = 500  (VW)
 *
 * @param         f_SWAdeg25_si16              SWA [-SWAMax*(25/�) ...
 * +SWAMax*(25/�)] [SWA] = � (RIGHT-, LEFT+)
 * @param         f_SWAMaxdeg25_si16           SWAMax*(25/�) [SWAMax] = �
 * @param         f_SARearMaxdeg50_si16        SARearMax*(50/�) [SARearMax] = �
 * @return        SInt16                       calculated steering angle rear
 * [SARear] = �
 *                                             [-SARearMax*(50/�) ...
 * +SARearMax*(50/�)]
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
SInt16 g_VHOGetSARearFromSWA_si16(SInt16 f_SWAdeg25_si16, SInt16 f_SWAMaxdeg25_si16,
                                  SInt16 f_SARearMaxdeg50_si16)
{
    SInt16 l_SAFrontRadF12_si16;
    SInt16 l_SARearRadF12_si16;
    SInt16 l_SAFrontMaxRadF12_si16;
    SInt16 l_SARearMaxRadF12_si16;
    SInt16 l_TanSAFrontRadF13_si16;
    SInt16 l_TanSARearRadF13_si16;
    SInt16 l_TanSAFrontMaxRadF13_si16;
    SInt16 l_TanSARearMaxRadF13_si16;
    SInt16 l_SteeringRatio_si16; // 1.0 physikalisch entspricht 1000 binary

    SInt16 l_SAReardeg50_si16;

    SInt32 l_temp_si32;

    if (f_SARearMaxdeg50_si16 <= (SInt16)(5))
    // ramp steering angle rear down to zero
    // =====================================
    {
        // The high gradient for ramp down is allowed if vehicle is faster than 0,1
        // km/h VehicleVelCanMPerS = VehicleVelovity_physikalisch * (256/(m/s)) (0,1
        // km/h/ 3.6) = 0.027778; 0.027778 * 256 = 7.111 ~ 7 high gradient is 10�/s
        // = 0,1�/(10 ms) ==> binary gradient = 0,1*50 = 5
        if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 >= (UInt16)(7))
        {
            if (g_VHOCanSig_st.SAR_si16 > (SInt16)(5))
            {
                l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 - (SInt16)(5);
            }
            else if (g_VHOCanSig_st.SAR_si16 < (SInt16)(-5))
            {
                l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 + (SInt16)(5);
            }
            else
            {
                l_SAReardeg50_si16 = (SInt16)(0);
            }
        }
        else
        // low gradient is 3�/s = 0,03�/(10 ms) ==> binary gradient = 0,03*50 = 1
        {
            if (g_VHOCanSig_st.SAR_si16 > (SInt16)(1))
            {
                l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 - (SInt16)(1);
            }
            else if (g_VHOCanSig_st.SAR_si16 < (SInt16)(-1))
            {
                l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 + (SInt16)(1);
            }
            else
            {
                l_SAReardeg50_si16 = (SInt16)(0);
            }
        }
    }
    else
    // f_SARearMaxdeg50_si16 is greater than 0.1�, calculation is allowed
    {
        // below 0,1 km/h the steereing angle rear has to be ramped to 0
        if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 <= (UInt16)(7))
        {
            // middle gradient is 4�/s = 0,04�/(10 ms) ==> binary gradient = 0,04*50 =
            // 2
            {
                if (g_VHOCanSig_st.SAR_si16 > (SInt16)(2))
                {
                    l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 - (SInt16)(2);
                }
                else if (g_VHOCanSig_st.SAR_si16 < (SInt16)(-2))
                {
                    l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 + (SInt16)(2);
                }
                else
                {
                    l_SAReardeg50_si16 = (SInt16)(0);
                }
            }
        }
        else
        {
            // calculate steering angle rear
            // =============================

            // map SARearMaxdeg50 to SARearMaxRadF12
            // SARearMaxdeg50 = SARearMaxdeg_physikalisch * (50/1�)
            // SARearMaxRadF12 = SARearMaxdeg_physikalisch * (Pi/180�)*2^12
            // SARearMaxRadF12 = SARearMaxdeg50 * ( (Pi*2�12) / ((50/1�) * 180�) )
            // SARearMaxRadF12 = SARearMaxdeg50 * (12868 / 9000)
            l_temp_si32            = (SInt32)(f_SARearMaxdeg50_si16) * (SInt32)(12868);
            l_temp_si32            = l_temp_si32 / ((SInt32)(9000));
            l_SARearMaxRadF12_si16 = (SInt16)(l_temp_si32);

            // fetch SAFrontRadF12 out of the steering wheel look up table
            if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
            {
                // calculate steering angle for a reverse movement
                l_SAFrontRadF12_si16 = g_VHOConvertSWA2SA_si16(f_SWAdeg25_si16, FALSE);
            }
            else
            {
                // calculate steering angle
                l_SAFrontRadF12_si16 = g_VHOConvertSWA2SA_si16(f_SWAdeg25_si16, TRUE);
            }

            // fetch SAFrontMaxRadF12 out of the steering wheel look up table
            if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
            {
                // calculate steering angle for a reverse movement
                l_SAFrontMaxRadF12_si16 =
                    g_VHOConvertSWA2SA_si16(f_SWAMaxdeg25_si16, FALSE);
            }
            else
            {
                // calculate steering angle
                l_SAFrontMaxRadF12_si16 =
                    g_VHOConvertSWA2SA_si16(f_SWAMaxdeg25_si16, TRUE);
            }

            // get tan(max steering angle front); Scaling F13 = 1/2^13
            l_TanSAFrontMaxRadF13_si16 = g_mtl_Tan_si16(l_SAFrontMaxRadF12_si16);

            // get tan(max steering angle rear); Scaling F13 = 1/2^13
            l_TanSARearMaxRadF13_si16 = g_mtl_Tan_si16(l_SARearMaxRadF12_si16);

            // calc steering ratio for steering rear multiplied with (+-)1000
            // if steering in opposite direction is required, than mulitply with -1000
            // if steering in the same direction is required, than mulitply with +1000
            // Steeringratio = -1000 * (Tan(SARearMax)/Tan(SAFrontMax)
            l_temp_si32          = (SInt32)(-1000) * l_TanSARearMaxRadF13_si16;
            l_temp_si32          = l_temp_si32 / ((SInt32)(l_TanSAFrontMaxRadF13_si16));
            l_SteeringRatio_si16 = (SInt16)l_temp_si32;

            // get tan(steering angle front); Scaling F13 = 1/2^13
            l_TanSAFrontRadF13_si16 = g_mtl_Tan_si16(l_SAFrontRadF12_si16);

            // calculate TanSARearRadF13_si16 depending on
            // TanSAFrontRadF13_si16 and steering ratio
            l_temp_si32 =
                (SInt32)(l_SteeringRatio_si16) * (SInt32)(l_TanSAFrontRadF13_si16);
            l_temp_si32            = l_temp_si32 / ((SInt32)(1000));
            l_TanSARearRadF13_si16 = (SInt16)(l_temp_si32);

            l_SARearRadF12_si16 =
                g_mtl_ArcTan_si16(l_TanSARearRadF13_si16, (UInt8)13); // F12 rad

            // transform SARearRadF12 to SAReardeg50
            // SA rear physical [�] = (180�/(Pi*2^12)) * SARearRadF12
            // SAReardeg50 = (50/1�)* SA rear physical [�]
            // SAReardeg50 = (50/1�)*(180�)/(Pi*2^12) *  SARearRadF12
            // SAReardeg50 = ( (9000)/(12868) ) *  SARearRadF12
            l_temp_si32        = (SInt32)(9000) * (SInt32)(l_SARearRadF12_si16);
            l_temp_si32        = l_temp_si32 / ((SInt32)(12868));
            l_SAReardeg50_si16 = (SInt16)(l_temp_si32);

            // limit l_SAReardeg50_si16 to (+-)f_SARearMaxdeg50_si16
            if (l_SAReardeg50_si16 > f_SARearMaxdeg50_si16)
            {
                l_SAReardeg50_si16 = f_SARearMaxdeg50_si16;
            }
            else if (l_SAReardeg50_si16 < -f_SARearMaxdeg50_si16)
            {
                l_SAReardeg50_si16 = -f_SARearMaxdeg50_si16;
            }
            else
            {
                // just fulfill QAC-MISRA-rules
            }

            // do not increase the highest allowed steering angle rear gradient !!!
            // high gradient is 10�/s = 0,1�/(10 ms) ==> binary gradient = 0,1*50 = 5
            // if difference between l_SAReardeg50_si16 and g_VHOCanSig_st.SAR_si16 is
            // less than -0,1�/(10 ms), or greater than 0,1�/(10 ms) fast ramping is
            // recommended.
            if (l_SAReardeg50_si16 > (g_VHOCanSig_st.SAR_si16 + (SInt16)(5)))
            {
                l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 + (SInt16)(5);
            }
            else if (l_SAReardeg50_si16 < (g_VHOCanSig_st.SAR_si16 - (SInt16)(5)))
            {
                l_SAReardeg50_si16 = g_VHOCanSig_st.SAR_si16 - (SInt16)(5);
            }
            else
            {
                // just fulfill QAC-MISRA-rules
            }
        }
    }

    return (l_SAReardeg50_si16);
}
#endif
#endif

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculation of YawAngle per cycle time by using YawRate sensor
 signal ratio: 1.0 rad / 2^22
 * @details
 *
 * @param
 * @return        UInt32       calculated YawAngle
 *
 * @note
 * @see
 * @warning
*/
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
/****************************************************************************/
VISIBILITY void m_VHOCalcYawAngleYRSensorRadF22_vd(void)
{
    SInt16 l_YawRateOffCorr_si16;
    SInt32 l_YawAngleYRSens_si32;
    SInt32 l_deltaYawAngleYRSens_si32;

    // store actual yaw angle calculated by integrating of measured yaw rate
    g_VHOVehicleState_st.YawAngleYRSensK1_ui32 = g_VHOVehicleState_st.YawAngleYRSens_ui32;

    // Integrate deltaYawAngleYrSens only if YR auto calibration is finished
    if (g_VHOAutocalibCtrl_st.YRState_enm == VHOCalibState_STABLE_enm)
    {
        // stop calculation of YawAngleYrSens if vehicle stands still
        // and no huge steering wheel angle changes are done
        if (g_VHOAutocalib_st.YawRateOffsetCalcNotAllowed_bl != FALSE)
        {
            // scaling with 2^22 is done because of scaling of VHO internal yaw angle
            // YawRate [0.01�/s], CycleTime = g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16
            // [ms] YawAngle_real [�] = YawRate_real*CycleTime/1000 = YawRate_VHO *
            // 0.01 * CycleTime/1000
            // => deltaYawAngle_VHO [F22 rad] = YawRate_VHO * (2^22 * CycleTime *
            // pi)/(180 * 100 * 1000) Scaling is necessary: YawRate max = 100�/s ==>
            // YawRate_VHO = 100 * 100 mit CycleTime = 200 ms (worstcase) ==> 100 *
            // 100 * 2^22 * CycleTime * pi > 2^31 100 * 100 * (2^22/20000) * CycleTime
            // * pi < 2^31 (ok.) also folgt daraus
            // => deltaYawAngle_VHO [F22 rad] = YawRate_VHO * (2^22/20000 * pi *
            // CycleTime)/(180 * 100000/20000)
            // => deltaYawAngle_VHO [F22 rad] = YawRate_VHO * (659 * CycleTime / 900)

            l_YawRateOffCorr_si16 =
                g_VHOCanSig_st.YawRate_si16 - g_VHOAutocalibCtrl_st.YROff_si16;
            l_deltaYawAngleYRSens_si32 = (SInt32)(l_YawRateOffCorr_si16) * (SInt32)(659);
            l_deltaYawAngleYRSens_si32 = l_deltaYawAngleYRSens_si32 *
                                         (SInt32)(g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16);
            l_deltaYawAngleYRSens_si32 =
                g_mtl_s32_Div_s32_si32(l_deltaYawAngleYRSens_si32, (SInt32)(900));

            // YawAngleYRSens(k) = YawAngleYRSens(k-1) + deltaYawAngleYrSens
            l_YawAngleYRSens_si32 = (SInt32)(g_VHOVehicleState_st.YawAngleYRSens_ui32) +
                                    l_deltaYawAngleYRSens_si32;
            // mormalize local variable within range [0 ... 2pi] and store it
            g_VHOVehicleState_st.YawAngleYRSens_ui32 =
                m_VHONormAngleRadF22_ui32(l_YawAngleYRSens_si32);
        }
        else
        {
            // stop integration
        }
    }
    else
    {
        // if YR auto calibration is not finished set YawAngleYRSens to VHO YawAgle
        g_VHOVehicleState_st.YawAngleYRSens_ui32 = g_VHOVehicleState_st.YawAngle_ui32;
    }

    return;
}
/****************************************************************************/
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/******************************************************************************
|------------------------------------------------------------------------------
| F U N C T I O N D E S C R I P T I O N
|------------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         calculate correction of YawAngle. Because of huge steering
 * wheel angle changes vehicle turns, even at vehicle stands still ratio: 1.0
 * rad / 2^22
 * @details
 *
 * @param
 * @return        UInt16           corrected YawAngle
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
/****************************************************************************/
VISIBILITY void m_VHOCalcYawAngleCorrAtStandStillRadF22_vd(void)
{
    // temporary variables
    SInt32 l_YawAngleVHOCorrected_si32;
    SInt32 l_deltaYawAngleYRSensAtStandStill_si32;
    UInt32 l_deltaYawAngleCheck2PiOverflow_ui32;
    UInt32 l_YawAngleCorrMinMaxAbsolute_ui32;

    // temporary constants
    const UInt32 m_1PiRadF22_ui32 = (UInt32)(13176794);
    const UInt32 m_2PiRadF22_ui32 = (UInt32)(26353589);

    if (
        // vehicle is standing still
        (g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm) &&
        // huge steering wheel changes are done while standing still
        (g_VHOAutocalib_st.YawRateOffsetCalcNotAllowed_bl != FALSE))
    {
        // calc difference of yaw angle changes
        l_deltaYawAngleYRSensAtStandStill_si32 =
            (SInt32)(g_VHOVehicleState_st.YawAngleYRSens_ui32) -
            (SInt32)(g_VHOVehicleState_st.YawAngleYRSensK1_ui32);

        // check if possible 2 Pi Overflow of YawAngleYRSens has bee given
        l_deltaYawAngleCheck2PiOverflow_ui32 =
            (UInt32)(g_mtl_Abs_mac(l_deltaYawAngleYRSensAtStandStill_si32));

        // 2 Pi overflow has been detected
        if (l_deltaYawAngleCheck2PiOverflow_ui32 >= m_1PiRadF22_ui32)
        {
            if (g_VHOVehicleState_st.YawAngleYRSens_ui32 >=
                g_VHOVehicleState_st.YawAngleYRSensK1_ui32)
            {
                l_deltaYawAngleYRSensAtStandStill_si32 -= (SInt32)(m_2PiRadF22_ui32);
            }
            else
            {
                l_deltaYawAngleYRSensAtStandStill_si32 += (SInt32)(m_2PiRadF22_ui32);
            }
        }
        else
        // 2 Pi overflow has not been detected
        {
            // do nothing
        }

        // calculate maximum value of YawAngle correction
        g_VHOVehicleState_st.YawAngleCorrection_si32 +=
            l_deltaYawAngleYRSensAtStandStill_si32;

        // calculate absolute value of difference between SWA and SWAStartStandstill
        l_YawAngleCorrMinMaxAbsolute_ui32 =
            (UInt32)(g_mtl_Abs_mac(g_VHOVehicleState_st.YawAngleCorrection_si32));

        // calculate correction of VHO yaw angle only if |YawAngleCorrMinMax| is
        // less or equal than application param.
        if (l_YawAngleCorrMinMaxAbsolute_ui32 <=
            g_parGetParaVHO_Odo_MaxYawAngCorrAtStandstill_ui32)
        {
            l_YawAngleVHOCorrected_si32 = (SInt32)(g_VHOVehicleState_st.YawAngle_ui32) +
                                          (l_deltaYawAngleYRSensAtStandStill_si32);
        }
        else
        {
            // no additional changes allowed, just use old value
            l_YawAngleVHOCorrected_si32 = (SInt32)(g_VHOVehicleState_st.YawAngle_ui32);
        }
        // mormalize local variable within range [0 ... 2pi] and store it
        g_VHOVehicleState_st.YawAngle_ui32 =
            m_VHONormAngleRadF22_ui32(l_YawAngleVHOCorrected_si32);
    }
    else
    // correction of YawAngle is not allowed if vehicle is moving or moving
    // direction is unknown or calculation of YawRate offset is allowed
    {
        g_VHOVehicleState_st.YawAngleCorrection_si32 = (SInt32)(0);
    }

    // If vehicle stands still, it is necessary to synchronize the buffer
    // YawAngleRaw and it makes sense to synchronize the dual model and the single
    // track model too.
    if (g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm)
    {
        g_VHOOdoBuf_st.YawAngleRaw_ui32 = g_VHOVehicleState_st.YawAngle_ui32;
        g_VHOVehicleState_st.YawAngleSingleTrack_ui32 =
            g_VHOVehicleState_st.YawAngle_ui32;

        // dual track model required  ?
        if ((g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcYawRate_enm) ||
            (g_VHOOdoState_st.YawAngCalc_en == VHOYAngCalcSingleTrack_enm))
        // if dual track model is not required, set dual track model to zero
        {
            g_VHOVehicleState_st.YawAngleDualTrack_ui32 = ((UInt32)(0));
        }
        else
        // if dual track model is required, synchronize dual track model to
        // YawAngleCorrected
        {
            g_VHOVehicleState_st.YawAngleDualTrack_ui32 =
                g_VHOVehicleState_st.YawAngle_ui32;
        }
    }
    else
    { /* do nothing */
    }

    return;
}
/****************************************************************************/
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3892, 3219
#endif
