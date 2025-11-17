/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_dir_recog.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO

#include "vho_api.h"
#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>
#include <veh/vhs/vhs_api.h>

// #include <swcpa_include.h>
// #include <vhm_api.h>       // SWCPA Include Structure: ok
#include "vho_dir_recog.h" // SWCPA Include Structure: ok
// #include <mathlib_api.h>   // SWCPA Include Structure: ok
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))
#include <vho_kin.h> // SWCPA Include Structure: ok
#endif
#include "vho.h" // SWCPA Include Structure: ok
// #include <vhm_loghdl.h>    // SWCPA Include Structure: ok
#include <string.h>

#ifdef QAC_MSG_OFF

#pragma PRQA_MESSAGES_OFF 3892
#endif

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_MODE
#error ('compiler switch LS_VHO_DIRDETECT_MODE is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_ACCSENS
#error ('compiler switch LS_VHO_DIRDETECT_ACCSENS is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_GEAR
#error ('compiler switch LS_VHO_DIRDETECT_GEAR is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_WIC
#error ('compiler switch LS_VHO_DIRDETECT_WIC is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_YAW_WIC
#error ('compiler switch LS_VHO_DIRDETECT_YAW_WIC is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_ACCSENS
#error ('compiler switch LS_VHO_DIRDETECT_ACCSENS is not defined')
#endif
#ifndef LS_VHO_DIRDETECT_ALL
#error ('compiler switch LS_VHO_DIRDETECT_ALL is not defined')
#endif
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#ifndef LS_VHO_WIC_FILTER_ELEPHANT
#error ('Switch LS_VHO_WIC_FILTER_ELEPHANT not defined!');
#endif
#endif

#define VHO_WICDIR_PENDINGTIME_MS_ui16 ((UInt16)60)

// 248 => 0.97m/s ~ 3.5km/h
#define VHO_WICDIR_MAX_VELO_MPERSF8_ui16 ((UInt16)248)

// -20 => -0.2 m/s^2
// #define VHO_WICDIR_MAXACC_si16              ((SInt16)-20)

typedef enum
{
    VHO_INDIR_FORW,
    VHO_INDIR_REV,
    VHO_INDIR_CONFLICT,
    VHO_INDIR_UNKNOWN
} mType_VHOWICDirInSig_en;

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogYawRate_en(void);
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) && ...

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) && \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS))
VISIBILITY void m_VHODirRecogGetAxZeroPoint_vd(UInt16 f_VehVelMperSF8_ui16,
                                               UInt16 f_S_CurrentCycleMM_ui16,
                                               SInt16 f_Ax_si16);
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogGetDirByIntegration_en(
    SInt16 f_Ax_si16, gType_VHORollRecogState_en f_RollDir_en);
VISIBILITY gType_VHORollRecogState_en
m_VHODirRecogFindChangePattern_en(SInt16 f_AccWic_si16);
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogGear_en(void);
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogGetDirByGradient_en(
    SInt16 f_NewestAxGrad_si16, SInt16 f_NewestAccWICGrad_si16);
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogMainInt_en(UInt8 f_VirtCanMsgRcv_ui8,
                                                              UInt8 f_ForceOutput_ui8);
#endif // #if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) && ... )

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
#if (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
VISIBILITY void m_ElephantTestInit_vd(void);
VISIBILITY void m_ElephantTest_vd(gType_VHORollRecogState_en f_MovDir_en);
#endif // (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) && ... )

#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS)
static SInt16 m_DirByGradientOldAccWICGrad_si16;
static UInt8 m_DirRecogGearCycles_ui8;
#endif

/*----------------------------------------------------------------*/
/*- definition of exported functions                             -*/
/*----------------------------------------------------------------*/

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         initialisation function for direction detection module
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHODirRecogInit_vd(void)
{
    (void)memset(&g_VHORollRecog_st, 0, sizeof(gType_VHORollRecog_st));

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    g_VHORollRecog_st.LookingForMin_bl              = TRUE;
    g_VHORollRecog_st.MaximaReached_bl              = FALSE;
    g_VHORollRecog_st.ZeroPointValid_bl             = FALSE;
    g_VHORollRecog_st.RollDirRefYawRate_en          = VHO_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.RollDirRefGear_en             = VHO_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.RollDirRefAccInteg_en         = VHO_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.RollDirRefAccGrad_en          = VHO_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.RollDirRefAccChangePattern_en = VHO_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.RollDirRefWIC_en              = VHO_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.RollDirReference_ui8          = (UInt8)0xFF;
#endif

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
#if (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
    g_VHORollRecog_st.WIC_MovingIndication_bl = FALSE;
    g_VHORollRecog_st.ExecuteElephant_bl      = FALSE;
#endif // (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&
       // ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) || ... )

    g_VHORollRecog_st.VHOWICDirState_en            = VHO_WICDIR_MOV_UNKNOWN_enm;
    g_VHORollRecog_st.EntryTimePendingStateMS_ui16 = 0;
    g_VHORollRecog_st.TimeLastWICIncrement_ui16    = 0;

#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS)
    m_DirByGradientOldAccWICGrad_si16 = 0;
    m_DirRecogGearCycles_ui8          = 0;
#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         main function for direction recognition
 * @details
 *
 * @param         f_VirtCanMsgRcv_ui8
 * @param         f_ForceOutput_ui8
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
// QAC warning off: The parameters f_VirtCanMsgRcv_ui8 and f_ForceOutput_ui8 are
// not used in this function
// --> depending on switch for driving direction estimation
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3206
#endif

gType_VHORollRecogState_en g_VHODirRecogGetCurrentDirection_en(UInt8 f_VirtCanMsgRcv_ui8,
                                                               UInt8 f_ForceOutput_ui8)
{
    gType_VHORollRecogState_en l_RetVal_en;

    switch (g_VHOParameters_st.RollRecogMode_en)
    {

#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS)
        case VHO_Direction_Accsens_enm:
            l_RetVal_en = m_VHODirRecogMainInt_en(f_VirtCanMsgRcv_ui8, f_ForceOutput_ui8);
            break;
#endif

        case VHO_Direction_Gear_enm:
            l_RetVal_en = m_VHODirRecogMainGear_en();
            break;

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))

        case VHO_Direction_Yaw_Wic_enm:
        case VHO_Direction_Wic_enm:
            l_RetVal_en = m_VHODirRecogMainWIC_en();

            g_VHORollRecog_st.RollDirRefWIC_en     = l_RetVal_en;
            g_VHORollRecog_st.RollDirReference_ui8 = 6;

// YAW RATE CRITERIA - ROLLING DIRECTION RECOGNITION
// ------------------------------- niv1lr - 26/10/2010
//
// motivation:  faster detection of moving direction
//              --> depending on the ESP and the mounted wheel impulse counter
//                  the moving direction will be provided late
//              --> yaw rate indicates early and robust the rolling direction
//              --> yaw rate method is well known and still in series production
//                  in systems where no intelligent wheel impulse counter
//                  (provide moving direction) are available --> GM/Opel
//                  Insignia --> PSI
// ----------------------------------------------------------------------------------

// if yaw rate is used in addition to WICs
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
            if ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
                VHO_Direction_Yaw_Wic_enm)
#endif
            {

                // do not use yaw rate for detection of vehicle moving direction
                // while yaw rate offset calculation is in initialisation mode
                if (g_VHOAutocalibCtrl_st.YRState_enm != VHOCalibState_INITIAL_enm)
                {
                    g_VHORollRecog_st.RollDirRefYawRate_en = m_VHODirRecogYawRate_en();
                }
                else
                {
                    g_VHORollRecog_st.RollDirRefYawRate_en = VHO_MOV_UNKNOWN_enm;
                }

                if ((g_VHORollRecog_st.RollDirRefYawRate_en != VHO_MOV_UNKNOWN_enm) &&
                    (g_VHORollRecog_st.RollDirRefWIC_en == VHO_MOV_UNKNOWN_enm))
                {
                    l_RetVal_en = g_VHORollRecog_st.RollDirRefYawRate_en;
                    g_VHORollRecog_st.RollDirReference_ui8 = 2;
                }

                // ----------------------------------------------------------------------------------

                // ----------------------------------------------------------------------------------
                // Test-Description:
                // - yaw criteria will be active if steering wheel angle >20�
                // - required time to determine vehicle moving direction 400ms
                //      --> Test if 400ms works fine --> <400ms in case of WIC-MOV-DIR
                //      could increase performance
                // - yaw rate (offset compensated) must be > 0.45�/s (3*ZeroLevelNoise)
                //
                // --> compare: g_VHORollRecog_st.RollDirRefWIC_en and
                // g_VHORollRecog_st.RollDirRefYawRate_en
                //              --> if WICRef is UNKNOWN and YawRef is ~UNKNOWN the output
                //              of the VHO should also be ~UNKNOWN
                //              --> additionally the VHO output should occur an update
                //              (position, ...)
                //              --> g_VHORollRecog_st.RollDirReference_ui8 indicates which
                //              reference was used to
                //                  calculate rolling direction
                //
                // --> maneuvers: STOP --> FORW   (left steering)   extreme slow .....
                // fast acceleration
                //                STOP --> FORW   (right steering)  extreme slow .....
                //                fast acceleration STOP --> BACKW  (left steering)
                //                extreme slow ..... fast acceleration STOP --> BACKW
                //                (right steering)  extreme slow ..... fast acceleration
                //                FORW --> STOP   (left steering)   extreme slow .....
                //                fast acceleration FORW --> STOP   (right steering)
                //                extreme slow ..... fast acceleration BACKW --> STOP
                //                (left steering)   extreme slow ..... fast acceleration
                //                BACKW --> STOP  (right steering)  extreme slow .....
                //                fast acceleration
                //
                // --> system task: determine time and driven distance delta between
                // RefWIC and RefYaw
                //       --> system understanding parameters (could be used for
                //       requirements/system design,
                //           calculation of system error, interface description ...)
                // ----------------------------------------------------------------------------------------
            }
#endif // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC)

#if (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
            if (l_RetVal_en == VHO_STOP_enm)
            {
                g_VHORollRecog_st.ExecuteElephant_bl = TRUE;
            }
            else if ((l_RetVal_en == VHO_MOV_FORW_enm) ||
                     (l_RetVal_en == VHO_MOV_BACKW_enm))
            {
                g_VHORollRecog_st.ExecuteElephant_bl = FALSE;
            }
            else // i.e. (l_RetVal_en == VHO_MOV_UNKNOWN_enm)
            {
                // just for QAC
                // g_VHORollRecog_st.ExecuteElephant_bl =
                // g_VHORollRecog_st.ExecuteElephant_bl;
            }
            m_ElephantTest_vd(l_RetVal_en);
#endif // (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)

            break;

#endif // ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||
       // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC)
       // || (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))

        default:
            l_RetVal_en = m_VHODirRecogMainGear_en();
            break;
    }
    return l_RetVal_en;
}

// QAC warning off: The parameters f_VirtCanMsgRcv_ui8 and f_ForceOutput_ui8 are
// not used in this function
// --> depending on switch for driving direction estimation
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3206
#endif
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         get direction from gear signal
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) ||     \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR) && \
      (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)) ||            \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL) &&  \
      (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)))
gType_VHORollRecogState_en m_VHODirRecogMainGear_en(void)
{
    gType_VHORollRecogState_en l_RetVal_en;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 == 0)
#else
    if (g_VHOCanSig_st.Vel_ui16 == 0)
#endif
    {
        // vehicle is not moving
        l_RetVal_en = VHO_STOP_enm;
    }
    else
    {
        // vehicle is moving
        if (g_VHOCanSig_st.ReverseGearInserted_bl != FALSE)
        {
            l_RetVal_en = VHO_MOV_BACKW_enm;
        }
        else
        {
            l_RetVal_en = VHO_MOV_FORW_enm;
        }
    }
    return l_RetVal_en;
}
#endif // #if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Initialization function for elephant test
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
#if (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)

VISIBILITY void m_ElephantTestInit_vd(void)
{
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        // Initialization of variables

        g_VHORollRecog_st.OpenWindows_ui8         = 0;
        g_VHORollRecog_st.WIC_MovingIndication_bl = FALSE;
        g_VHORollRecog_st.Saved_Distance_si32     = 0;
        if (g_VHORollRecog_st.RollDirRefWIC_en == VHO_MOV_UNKNOWN_enm)
        {
            g_VHORollRecog_st.ExecuteElephant_bl = FALSE;
        }

        (void)memset(&g_VHORollRecog_st.StartTimeXx_pui16, 0, sizeof(UInt16) * 4);
        (void)memset(&g_VHORollRecog_st.WICDeltaXx_pui32, 0, sizeof(UInt32) * 4);
        (void)memset(&g_VHORollRecog_st.MovingTireXx_pui8, 0, sizeof(UInt8) * 4);
    }
}
#endif // (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&
       // ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||
       // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         Detection if vehicle is moving if just several WICs were
 * detected
 * @details
 *
 * @param         f_MovDir_en    actual moving direction from
 * m_VHODirRecogMainWIC_en()
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
#if (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
VISIBILITY void m_ElephantTest_vd(gType_VHORollRecogState_en f_MovDir_en)
{
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        UInt8 l_I_ui8;                    // counter variable
        UInt8 l_MovingTiresTotal_ui8 = 0; // total number of moving tires per each cycle
        UInt16 l_DiffStartTimeCurrentCANTime_ui16; // difference between start time
                                                   // where a new window was opened
                                                   // and actual CAN time

        // elephant test should just run if moving direction is STOP or if moving
        // direction is UNKNOWN if there was a transition from STOP -> UNKNOWN
        if (g_VHORollRecog_st.ExecuteElephant_bl == FALSE)
        {
            // Initialization of variables
            m_ElephantTestInit_vd();
        }
        else
        // i.e. if ( (f_MovDir_en == VHO_STOP_enm) ||
        // ( (f_MovDir_en == VHO_MOV_UNKNOWN_enm) in case of a transition from STOP
        // -> UNKNOWN )
        {
            for (l_I_ui8 = 0; l_I_ui8 < 4;
                 l_I_ui8++) // for each tire Xx out of {RR, RL, FR, FL}
            {
                // If WIC change was detected -> open time window, set start time to
                // current CAN time, set moving indication to TRUE
                if ((g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8] > 0) &&
                    (g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] == 0))
                {
                    g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] =
                        g_VHOCanSig_st.WIC_CanTime_ui16;
                    g_VHORollRecog_st.OpenWindows_ui8++;
                    g_VHORollRecog_st.WIC_MovingIndication_bl = TRUE;
                }

                // If start time > 0 and WICDelta < MaxWICDelta add WIC change to
                // WICDelta
                if ((g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8] > 0) &&
                    (g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] > 0))
                {
                    if (g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8] <
                        (UInt32)md_VHO_MaxWICDelta_ui8)
                    {
                        g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8] +=
                            (UInt32)g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8];
                    }
                    if ((g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8] >=
                         (UInt32)md_VHO_MaxWICDelta_ui8) &&
                        (g_VHORollRecog_st.MovingTireXx_pui8[l_I_ui8] == 0))
                    {
                        // elseif WICDelta >= MaxWICDelta set MovingTireXx = 1
                        g_VHORollRecog_st.MovingTireXx_pui8[l_I_ui8] = 1;
                    }
                } // end if ( (g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8] > 0) &&
                  // (g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] > 0) )

                // compensate time overflow of wic can time
                // o = WIC_CAN_TIME
                // x = StartTimeXx
                // DiffStartTimeCurrentCANTime = WIC_CAN_TIME - StartTimeXx
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

                // DiffStartTimeCurrentCANTime = WIC_CAN_TIME + (MAX_VALUE -
                // StartTimeXx)
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

                if (g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] <=
                    g_VHOCanSig_st.WIC_CanTime_ui16)
                {
                    l_DiffStartTimeCurrentCANTime_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16 -
                        g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8];
                }
                else
                {
                    l_DiffStartTimeCurrentCANTime_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16 +
                        (UInt16)((UInt32)65536 -
                                 (UInt32)g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8]);
                }

                // if time window has reached 1s and WICDelta < MaxWICDelta then close
                // open window and re-initialize StartTimeXx and WICDelta and set moving
                // tire = 0; if WICDelta > MaxWICDelta hold window for 1,5s instead of
                // just 1s to guarantee not to close the window too early before having
                // certainty that there will be no overlapping window of another tire
                // that would result in detecting a vehicle movement
                if ((g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] > 0) &&
                    (l_DiffStartTimeCurrentCANTime_ui16 >= md_VHO_TimeThreshold1_ui16))
                {
                    if (g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8] <
                        (UInt32)md_VHO_MaxWICDelta_ui8)
                    {
                        g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] = 0;
                        g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8]  = 0;
                        g_VHORollRecog_st.MovingTireXx_pui8[l_I_ui8] = 0;
                        g_VHORollRecog_st.OpenWindows_ui8--;
                    }
                    else if ((g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8] >=
                              (UInt32)md_VHO_MaxWICDelta_ui8) &&
                             (l_DiffStartTimeCurrentCANTime_ui16 >=
                              md_VHO_TimeThreshold2_ui16))
                    {
                        g_VHORollRecog_st.StartTimeXx_pui16[l_I_ui8] = 0;
                        g_VHORollRecog_st.WICDeltaXx_pui32[l_I_ui8]  = 0;
                        g_VHORollRecog_st.MovingTireXx_pui8[l_I_ui8] = 0;
                        g_VHORollRecog_st.OpenWindows_ui8--;
                    }
                    else
                    {
                        // QAC
                    }
                }
            } // end for

            // after having checked each tire within each cycle decide whether the
            // vehicle is moving or if it was a "FALSE alarm"

            for (l_I_ui8 = 0; l_I_ui8 < 4; l_I_ui8++) // check how many tires have moved
            {
                l_MovingTiresTotal_ui8 += g_VHORollRecog_st.MovingTireXx_pui8[l_I_ui8];
            }

            // vehicle movement has been detected if number of moving tires >=
            // MinNumberMovTires
            // => move state is pending and re-initialize

            if (l_MovingTiresTotal_ui8 >= md_VHO_MinNumberMovTires_ui8)
            {
                if (f_MovDir_en == VHO_STOP_enm)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                }
                // Initialization of variables
                m_ElephantTestInit_vd();
            }

            // save driven distance if there is an indication that vehicle has moved
            // to be able to subtract driven distance again if there was no movement
            if (g_VHORollRecog_st.WIC_MovingIndication_bl != FALSE)
            {
                g_VHORollRecog_st.Saved_Distance_si32 +=
                    g_VHOOdoBuf_st.DrivenDistanceMMF8_si32;
            }

            // if MovingTiresTotal != 0 but MovingTiresTotal < MinNumberMovTires
            // nothing should happen because it might be possible that in the next
            // cycle an overlapping time window will be opened
            if ((g_VHORollRecog_st.OpenWindows_ui8 == 0) &&
                (l_MovingTiresTotal_ui8 == 0) &&
                (g_VHORollRecog_st.WIC_MovingIndication_bl != FALSE))
            {
                if (f_MovDir_en == VHO_STOP_enm)
                {
                    // correction of discarded distance because MovDir == STOP and
                    // re-initialization
                    if (g_VHORollRecog_st.s_discarded_MMF8_ui32 >=
                        (UInt32)g_mtl_Abs_mac(g_VHORollRecog_st.Saved_Distance_si32))
                    {
                        g_VHORollRecog_st.s_discarded_MMF8_ui32 -=
                            (UInt32)g_mtl_Abs_mac(g_VHORollRecog_st.Saved_Distance_si32);
                    }
                    else if ((g_VHORollRecog_st.s_discarded_MMF8_ui32 <
                              (UInt32)g_mtl_Abs_mac(
                                  g_VHORollRecog_st.Saved_Distance_si32)) &&
                             (g_VHORollRecog_st.s_discarded_MMF8_ui32 != (UInt32)0))
                    // set discarded distance to zero if discarded distance != 0 and saved
                    // distance > discarded distance
                    {
                        g_VHORollRecog_st.s_discarded_MMF8_ui32 = (UInt32)0;
                    }
                    else // i.e. discarded distance == 0 => do nothing
                    {
                        // else just for QAC
                    }
                    m_ElephantTestInit_vd();
                }
                else // i.e. MovDir == UNKNOWN
                {
                    // no correction of driven distance because vehicle movement was
                    // indicated and MovDir = UNKNOWN and re-initialization of variables
                    m_ElephantTestInit_vd();
                }
            }
        } // end else, i.e. if ( g_VHORollRecog_st.ExecuteElephant_bl != FALSE )

        // just for testing (because measure g_adiCarMoveDir does not reflect adi
        // output of VHO)
        if (g_VHORollRecog_st.WIC_MovingIndication_bl != FALSE)
        {
            g_VHORollRecog_st.MovingDirection_en = VHO_STOP_enm;
        }
        else
        {
            g_VHORollRecog_st.MovingDirection_en = f_MovDir_en;
        }
    }
}
#endif // (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
#endif //( (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&
       //  ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) || ... ) )

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         get direction from direction sensitive wheel pulse counters
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) ||    \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) && \
      (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)) ||           \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL) && \
      (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)))
gType_VHORollRecogState_en m_VHODirRecogMainWIC_en(void)
{
    UInt8 l_I_ui8, l_NumFORW_ui8 = 0, l_NumREV_ui8 = 0;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt8 l_MinWICDirForMovDir_ui8;
#endif

    mType_VHOWICDirInSig_en l_InDir_en;
    gType_VHORollRecogState_en l_RetVal_en;

    // count number of input signals that indicate either FORWARD
    // or REVERSE movement
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    if (g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8 < (UInt8)(1))
    {
        l_MinWICDirForMovDir_ui8 = (UInt8)(1);
    }
    else if (g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8 > (UInt8)(4))
    {
        l_MinWICDirForMovDir_ui8 = (UInt8)(4);
    }
    else
    {
        l_MinWICDirForMovDir_ui8 = g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8;
    }
    for (l_I_ui8 = 0; l_I_ui8 < 4; l_I_ui8++)
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    /*
    if (g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8 < (UInt8)(1))
    {
      l_MinWICDirForMovDir_ui8 = (UInt8)(1);
    }
    else if (g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8 > (UInt8)(2))
    {
      l_MinWICDirForMovDir_ui8 = (UInt8)(2);
    }
    else
    {
      l_MinWICDirForMovDir_ui8 = g_parGetParaVHO_Odo_MinWICDirForMovDir_ui8;
    }
    */
    for (l_I_ui8 = 0; l_I_ui8 < 2; l_I_ui8++)
#endif
    {
        if (g_VHOCanSig_st.WICState_pen[l_I_ui8] == VHO_MOV_FORW_enm)
        {
            l_NumFORW_ui8++;
        }
        else
        {
            if (g_VHOCanSig_st.WICState_pen[l_I_ui8] == VHO_MOV_BACKW_enm)
            {
                l_NumREV_ui8++;
            }
        }
    }

    // derive cumulated input signal for direction determination
    if ((l_NumREV_ui8 > 0) && (l_NumFORW_ui8 > 0))
    {
        // both forward and reverse movement are indicated at the same time
        if (l_NumREV_ui8 == l_NumFORW_ui8)
        {
            // set input signal to "CONFLICT"
            l_InDir_en = VHO_INDIR_CONFLICT;
        }
        else
        {
            if (l_NumFORW_ui8 > l_NumREV_ui8)
            {
                // set input signal to "FORWARD"
                l_InDir_en = VHO_INDIR_FORW;
            }
            else
            {
                // set input signal to "REVERSE"
                l_InDir_en = VHO_INDIR_REV;
            }
        }
    }
    else
    // if premium VHO is required the number of used wheel directions
    // for the decision of car moving direction is tunable by application,
    // but not if mini VHO is required
    {
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        if (l_NumREV_ui8 > (l_MinWICDirForMovDir_ui8 - (UInt8)(1)))
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
        if (l_NumREV_ui8 > (UInt8)(0))
#endif
        {
            // set input signal to "REVERSE"
            l_InDir_en = VHO_INDIR_REV;
        }
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        else if (l_NumFORW_ui8 > (l_MinWICDirForMovDir_ui8 - (UInt8)(1)))
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
        else if (l_NumFORW_ui8 > (UInt8)(0))
#endif
        {
            // set input signal to "FORWARD"
            l_InDir_en = VHO_INDIR_FORW;
        }
        else
        {
            // set input signal to "UNKNOWN"
            l_InDir_en = VHO_INDIR_UNKNOWN;
        }
    }

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    if (((g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 == 0) &&
         (g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 == 0)) &&
        (g_VHORollRecog_st.WICTimeDeltaMS_ui16 > gd_VHOMinTimeBeforeSTOPMS_ui16))
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    if (((g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] == 0) &&
         (g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] == 0)) &&
        (g_VHORollRecog_st.WICTimeDeltaMS_ui16 > gd_VHOMinTimeBeforeSTOPMS_ui16))
#endif
    {
        // STATE STOP
        g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_STOP_enm;
    }
    else
    {
        // SUPERSTATE MOVING
        switch (g_VHORollRecog_st.VHOWICDirState_en)
        {
            case VHO_WICDIR_STOP_enm:
                // vehicle has stopped and starts moving now
                if (l_InDir_en == VHO_INDIR_FORW)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_FORW_PENDING_enm;
                    g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16;
                }
                else if (l_InDir_en == VHO_INDIR_REV)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_REV_PENDING_enm;
                    g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16;
                }
                else
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                }
                break;

            case VHO_WICDIR_MOV_UNKNOWN_enm:
                if (l_InDir_en == VHO_INDIR_FORW)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_FORW_PENDING_enm;
                    g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16;
                }
                else
                {
                    if (l_InDir_en == VHO_INDIR_REV)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en =
                            VHO_WICDIR_MOV_REV_PENDING_enm;
                        g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                            g_VHOCanSig_st.WIC_CanTime_ui16;
                    }
                }
                break;

            case VHO_WICDIR_MOV_FORW_PENDING_enm:
                if (l_InDir_en == VHO_INDIR_CONFLICT)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                }
                else if (l_InDir_en == VHO_INDIR_REV)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_REV_PENDING_enm;
                    g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16;
                }
                else
                {
                    if (((UInt16)(g_VHOCanSig_st.WIC_CanTime_ui16 -
                                  g_VHORollRecog_st.EntryTimePendingStateMS_ui16)) >=
                        VHO_WICDIR_PENDINGTIME_MS_ui16)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_FORW_enm;
                    }
                }
                break;

            case VHO_WICDIR_MOV_REV_PENDING_enm:
                if (l_InDir_en == VHO_INDIR_CONFLICT)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                }
                else if (l_InDir_en == VHO_INDIR_FORW)
                {
                    g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_FORW_PENDING_enm;
                    g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                        g_VHOCanSig_st.WIC_CanTime_ui16;
                }
                else
                {
                    if (((UInt16)(g_VHOCanSig_st.WIC_CanTime_ui16 -
                                  g_VHORollRecog_st.EntryTimePendingStateMS_ui16)) >=
                        VHO_WICDIR_PENDINGTIME_MS_ui16)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_REV_enm;
                    }
                }
                break;

            case VHO_WICDIR_MOV_FORW_enm:
                if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 <
                    VHO_WICDIR_MAX_VELO_MPERSF8_ui16)
                {
                    if (l_InDir_en == VHO_INDIR_REV)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en =
                            VHO_WICDIR_MOV_REV_PENDING_enm;
                        g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                            g_VHOCanSig_st.WIC_CanTime_ui16;
                    }
                    else if (l_InDir_en == VHO_INDIR_CONFLICT)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                    }
                    else
                    {
                        if (l_InDir_en == VHO_INDIR_UNKNOWN)
                        {
                            g_VHORollRecog_st.VHOWICDirState_en =
                                VHO_WICDIR_MOV_UNKNOWN_enm;
                        }
                    }
                }
                break;
            case VHO_WICDIR_MOV_REV_enm:
                if (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 <
                    VHO_WICDIR_MAX_VELO_MPERSF8_ui16)
                {
                    if (l_InDir_en == VHO_INDIR_FORW)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en =
                            VHO_WICDIR_MOV_FORW_PENDING_enm;
                        g_VHORollRecog_st.EntryTimePendingStateMS_ui16 =
                            g_VHOCanSig_st.WIC_CanTime_ui16;
                    }
                    else if (l_InDir_en == VHO_INDIR_CONFLICT)
                    {
                        g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                    }
                    else
                    {
                        if (l_InDir_en == VHO_INDIR_UNKNOWN)
                        {
                            g_VHORollRecog_st.VHOWICDirState_en =
                                VHO_WICDIR_MOV_UNKNOWN_enm;
                        }
                    }
                }
                break;
            default:
                g_VHORollRecog_st.VHOWICDirState_en = VHO_WICDIR_MOV_UNKNOWN_enm;
                break;
        }
    }

    switch (g_VHORollRecog_st.VHOWICDirState_en)
    {
        case VHO_WICDIR_STOP_enm:
            l_RetVal_en = VHO_STOP_enm;
            break;
        case VHO_WICDIR_MOV_UNKNOWN_enm:
            l_RetVal_en = VHO_MOV_UNKNOWN_enm;
            break;
        case VHO_WICDIR_MOV_FORW_PENDING_enm:
            l_RetVal_en = VHO_MOV_UNKNOWN_enm;
            break;
        case VHO_WICDIR_MOV_REV_PENDING_enm:
            l_RetVal_en = VHO_MOV_UNKNOWN_enm;
            break;
        case VHO_WICDIR_MOV_FORW_enm:
            l_RetVal_en = VHO_MOV_FORW_enm;
            break;
        case VHO_WICDIR_MOV_REV_enm:
            l_RetVal_en = VHO_MOV_BACKW_enm;
            break;
        default:
            l_RetVal_en = VHO_STOP_enm;
            break;
    }

    return l_RetVal_en;
}
#endif // #if ( (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) || (
       // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC)
       // && (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) ) ... )

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) && \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS))
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 *
 * @brief         derive driving direction from vehicle dynamic signals
 *                from ESP system: longitudinal acceleration, yaw rate and
 *                wheel pulses
 * @details
 *
 * @param         f_VirtCanMsgRcv_ui8
 * @param         f_ForceOutput_ui8
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogMainInt_en(UInt8 f_VirtCanMsgRcv_ui8,
                                                              UInt8 f_ForceOutput_ui8)
{
    const gType_VHORollRecogState_en lc_LastState_en =
        g_VHOVehicleState_st.RollingDirection_en;
    gType_VHORollRecogState_en l_RetVal_en = lc_LastState_en;

    UInt16 l_temp_ui16;
    SInt16 l_temp_si16;

    Boolean l_VehicleStandstill_bl;

    l_VehicleStandstill_bl = g_VHOKinVehicleStandstill_bl();

    // in case of no standstill force rolling direction to unknown to avoid
    // loosing a driven distance
    if ((g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm) &&
        (f_VirtCanMsgRcv_ui8 == 0) && (l_VehicleStandstill_bl == FALSE))
    {
        l_RetVal_en                            = VHO_MOV_UNKNOWN_enm;
        g_VHORollRecog_st.RollDirReference_ui8 = 0;
    }

    if (f_VirtCanMsgRcv_ui8 != 0)
    // message has been received on virtual can
    {
        g_VHORollRecog_st.TimeAfterDirChange_ui16 +=
            g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16;

        // read Ax signal, synchronized to calculated wic acceleration
        g_VHORollRecog_st.AxDelayed_si16 = m_VHOKinRMGetAxSync_si16();

        l_temp_si16 =
            (SInt16)g_mtl_ReducePrecision_si32(g_VHORollRecog_st.VirtCan_dsMMF8_si32, 8);
        l_temp_ui16 = (UInt16)g_mtl_Abs_mac(l_temp_si16);

        // calculate zero point of Ax signal in case of vehicle standstill of very
        // slow movement
        m_VHODirRecogGetAxZeroPoint_vd(g_VHOVehicleState_st.VehicleVelocity_ui16,
                                       l_temp_ui16, g_VHORollRecog_st.AxDelayed_si16);

        g_VHORollRecog_st.VirtCan_dsMMF8_si32 = (SInt32)0;

        g_VHORollRecog_st.RollDirRefGear_en = m_VHODirRecogGear_en();

        // do not use yaw rate for detection of vehicle moving direction
        // while yaw rate offset calculation is in initialisation mode
        if (g_VHOAutocalibCtrl_st.YRState_enm != VHOCalibState_INITIAL_enm)
        {
            g_VHORollRecog_st.RollDirRefYawRate_en = m_VHODirRecogYawRate_en();
        }
        else
        {
            g_VHORollRecog_st.RollDirRefYawRate_en = VHO_MOV_UNKNOWN_enm;
        }

        g_VHORollRecog_st.RollDirRefAccChangePattern_en =
            m_VHODirRecogFindChangePattern_en(
                g_VHOVehicleState_st.VehicleAcceleration_si16);

        g_VHORollRecog_st.RollDirRefAccInteg_en = m_VHODirRecogGetDirByIntegration_en(
            g_VHOKinRMGetAcc_si16(1), lc_LastState_en);

        g_VHORollRecog_st.RollDirRefAccGrad_en = m_VHODirRecogGetDirByGradient_en(
            g_VHOKinRMGetAxGrad_si16(md_RM_ELEM_OLDEST_ui8),
            g_VHORollRecog_st.GradAccWIC_si16);

        if ((g_VHOVehicleState_st.VehicleVelocity_ui16 <
             md_VHODirVelThres1MperSF8_ui16) ||
            (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_UNKNOWN_enm))
        {
            l_VehicleStandstill_bl = g_VHOKinVehicleStandstill_bl();

            if (l_VehicleStandstill_bl != FALSE)
            {
                l_RetVal_en                            = VHO_STOP_enm;
                g_VHORollRecog_st.RollDirReference_ui8 = 0;
            }
            else
            {
                if (g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm)
                {
                    l_RetVal_en = VHO_MOV_UNKNOWN_enm;
                }

                if (g_VHORollRecog_st.RollDirRefGear_en != VHO_MOV_UNKNOWN_enm)
                {
                    l_RetVal_en = g_VHORollRecog_st.RollDirRefGear_en;
                    g_VHORollRecog_st.RollDirReference_ui8 = 1;
                }
                else if (g_VHORollRecog_st.RollDirRefYawRate_en != VHO_MOV_UNKNOWN_enm)
                {
                    l_RetVal_en = g_VHORollRecog_st.RollDirRefYawRate_en;
                    g_VHORollRecog_st.RollDirReference_ui8 = 2;
                }
                else
                {
                    if ((UInt16)g_mtl_Abs_mac((SInt16)g_VHOCanSig_st.SWA_si16) <
                        md_VHODirCtrlSWAThres_ui16)
                    {
                        if ((g_VHORollRecog_st.RollDirRefAccChangePattern_en !=
                             VHO_MOV_UNKNOWN_enm) &&
                            (g_VHORollRecog_st.RollDirRefAccChangePattern_en !=
                             VHO_MOV_UNDER_INVESTIGATION_enm))
                        {
                            l_RetVal_en = g_VHORollRecog_st.RollDirRefAccChangePattern_en;
                            g_VHORollRecog_st.TimeAfterDirChange_ui16 = 0;
                            g_VHORollRecog_st.RollDirReference_ui8    = 3;
                        }
                        else
                        {
                            if ((g_VHORollRecog_st.RollDirRefAccChangePattern_en !=
                                 VHO_MOV_UNDER_INVESTIGATION_enm) &&
                                (g_VHORollRecog_st.TimeAfterDirChange_ui16 >
                                 md_VHODirCtrlAccGradLockTimeMS_ui16) &&
                                ((g_VHORollRecog_st.RollDirRefAccGrad_en !=
                                  VHO_MOV_UNKNOWN_enm) &&
                                 (g_VHORollRecog_st.RollDirRefAccGrad_en !=
                                  VHO_MOV_UNDER_INVESTIGATION_enm)))

                            {
                                l_RetVal_en = g_VHORollRecog_st.RollDirRefAccGrad_en;
                                g_VHORollRecog_st.RollDirReference_ui8 = 4;
                            }
                        }
                    }
                }
            }
        }
        else
        {
            if (g_VHOVehicleState_st.VehicleVelocity_ui16 >
                md_VHODirVelThres2MperSF8_ui16)
            {
                // velocity is above speed threshold 2 => gear information is used for
                // direction output
                if (g_VHOCanSig_st.ReverseGearInserted_bl != FALSE)
                {
                    l_RetVal_en = VHO_MOV_BACKW_enm;
                }
                else
                {
                    l_RetVal_en = VHO_MOV_FORW_enm;
                }
            }
        }
    }

    if (
        // output of rolling direction is urgent and no valid information
        // available
        ((f_ForceOutput_ui8 == 1) && (l_RetVal_en == VHO_MOV_UNKNOWN_enm)) ||
        // vehicle has stopped and no valid information is available
        ((l_RetVal_en == VHO_STOP_enm) && (lc_LastState_en == VHO_MOV_UNKNOWN_enm)))
    {
        // try to determine direction by integration of acceleration signal
        if (g_VHORollRecog_st.RollDirRefAccInteg_en != VHO_MOV_UNKNOWN_enm)
        {
            l_RetVal_en = g_VHORollRecog_st.RollDirRefAccInteg_en;
            g_VHORollRecog_st.RollDirReference_ui8 = 5;
        }
    }
    return l_RetVal_en;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         derive current rolling direction from gear; the gear is
 considered reliable if the acceleration gradient is very high for at least a
 specified time
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
*/
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogGear_en(void)
{
    UInt16 l_CurrentAxGradient_ui16;
    SInt16 l_temp_si16;
    gType_VHORollRecogState_en l_RetVal_enm = VHO_MOV_UNKNOWN_enm;

    // get current gradient on Ax signal
    l_temp_si16              = g_VHOKinRMGetAxGrad_si16(1);
    l_CurrentAxGradient_ui16 = (UInt16)g_mtl_Abs_mac(l_temp_si16);
    if ((l_CurrentAxGradient_ui16 > md_VHODirGearAxGradThres_ui16) &&
        (g_VHORollRecog_st.GradAccWIC_si16 > md_VHODirGearAccWICGradThres_si16) &&
        (g_VHOVehicleState_st.VehicleAcceleration_si16 > md_VHODirGearAccWICThres_si16))
    {
        m_DirRecogGearCycles_ui8++;
    }
    else
    {
        m_DirRecogGearCycles_ui8 = 0;
    }

    if (m_DirRecogGearCycles_ui8 == md_VHODirGearNumCycles_ui8)
    {
        l_RetVal_enm             = (g_VHOCanSig_st.ReverseGearInserted_bl == FALSE)
                                       ? VHO_MOV_FORW_enm
                                       : VHO_MOV_BACKW_enm;
        m_DirRecogGearCycles_ui8 = 0;
    }

    return l_RetVal_enm;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function used to determine the rolling direction by
 * integration of the longitudinal acceleration
 * @details
 *
 * @param         f_Ax_si16
 * @param         f_RollDir_en
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogGetDirByIntegration_en(
    SInt16 f_Ax_si16, gType_VHORollRecogState_en f_RollDir_en)
{
    UInt8 l_I_ui8;
    UInt32 l_ThresSumAcc_ui32;
    gType_VHORollRecogState_en l_RetVal_en;

    const UInt32 lc_ThresSumAccMax_ui32 =
        (UInt32)md_VHODirIntegVelThresMax_ui16 *
        ((UInt32)1000 / (UInt32)g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16);
    const UInt8 lc_MaxZeroPointQuality_ui8 = 0xFF;

    l_RetVal_en = VHO_MOV_UNKNOWN_enm;

    if ((f_RollDir_en == VHO_MOV_FORW_enm) || (f_RollDir_en == VHO_MOV_BACKW_enm))
    {
        // vehicle is moving and rolling direction by integration is no longer
        // needed => reset variables
        g_VHORollRecog_st.SumAx_si32 = 0;
    }
    else
    {
        if (g_VHORollRecog_st.ds_buf_MMF8_ui32 <=
            md_VHODirIntegMaxDistAfterStandstillMMF8_ui32)
        {
            // in case that the rolling direction is already known
            // this component shall not become active
            if ((g_VHORollRecog_st.SumAx_si32 == 0) &&
                (g_VHOVehicleState_st.StandstillCycleCnt_ui16 == 0) &&
                (g_VHORollRecog_st.ZeroPointValid_bl != FALSE))
            {
                // vehicle has started to move and values in ring memory have not been
                // integrated yet
                for (l_I_ui8 = 1; l_I_ui8 < 4; l_I_ui8++)
                {
                    // subtract acceleration offset from acceleration in ring memory and
                    // add to sum
                    g_VHORollRecog_st.SumAx_si32 +=
                        (SInt32)(g_VHOKinRMGetAcc_si16(l_I_ui8) -
                                 g_VHORollRecog_st.AxZeroPoint_si16);
                }
            }
            else if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 > 1)
            {
                // vehicle has been standing for at least two cycles => reset
                g_VHORollRecog_st.SumAx_si32 = 0;
            }
            else
            {
                if ((g_VHORollRecog_st.SumAx_si32 != 0) &&
                    (g_VHORollRecog_st.ZeroPointValid_bl != FALSE))
                {
                    g_VHORollRecog_st.SumAx_si32 +=
                        (SInt32)(f_Ax_si16 - g_VHORollRecog_st.AxZeroPoint_si16);

                    // calculate threshold for the output of a driving direction
                    // it depends on the quality of the calculated zero point and
                    // lies in [0.5*l_ThresSumAccMax_ui32,l_ThresSumAccMax_ui32]
                    l_ThresSumAcc_ui32 =
                        lc_ThresSumAccMax_ui32 -
                        (((lc_ThresSumAccMax_ui32 / (UInt32)2) *
                          (UInt32)g_VHORollRecog_st.ZeroPointQuality_ui8) /
                         (UInt32)lc_MaxZeroPointQuality_ui8);

                    if (g_VHORollRecog_st.SumAx_si32 < -((SInt32)l_ThresSumAcc_ui32))
                    {
                        l_RetVal_en = VHO_MOV_FORW_enm;
                    }
                    else
                    {
                        if (g_VHORollRecog_st.SumAx_si32 > (SInt32)l_ThresSumAcc_ui32)
                        {
                            l_RetVal_en = VHO_MOV_BACKW_enm;
                        }
                    }
                }
            }
        }
        else
        {
            // reset
            g_VHORollRecog_st.SumAx_si32 = 0;
        }
    }
    return l_RetVal_en;
}

/*****************************************************************************
| F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         function used to determine the rolling direction by comparison
 *               the gradients of the sensor acceleration and the calculated
 *                acceleration based on the wic signal
 * @details
 *
 * @param         f_NewestAxGrad_si16
 * @param         f_NewestAccWICGrad_si16
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogGetDirByGradient_en(
    SInt16 f_NewestAxGrad_si16, SInt16 f_NewestAccWICGrad_si16)
{
    gType_VHORollRecogState_en l_DirecctionFromGradient_en;

    /* search for global maximum in current peak */
    if ((g_mtl_Abs_mac(f_NewestAccWICGrad_si16) >
         g_mtl_Abs_mac(g_VHORollRecog_st.Maxima_si16)) &&
        (g_VHORollRecog_st.MaximaReached_bl != FALSE))
    {
        g_VHORollRecog_st.MaximaReached_bl = FALSE;
    }

    /* detect intersection with abscissa --> restart driving direction detection
     */
    if (((m_DirByGradientOldAccWICGrad_si16 < (SInt16)0) &&
         (f_NewestAccWICGrad_si16 > (SInt16)0)) ||
        ((m_DirByGradientOldAccWICGrad_si16 > (SInt16)0) &&
         (f_NewestAccWICGrad_si16 < (SInt16)0)))
    {
        /* reset status variables */
        g_VHORollRecog_st.Maxima_si16          = (SInt16)0;
        g_VHORollRecog_st.AxGradPosVal_ui8     = (UInt8)0;
        g_VHORollRecog_st.AccWICGradPosVal_ui8 = (UInt8)0;
        g_VHORollRecog_st.AccTotalValues_ui8   = (UInt8)0;
        g_VHORollRecog_st.MaximaReached_bl     = (Boolean)FALSE;
        // l_DirecctionFromGradient_en             = VHO_MOV_UNKNOWN_enm;
    }

    /* start algorithm if values above/below threshold to estimate driving
     * direction */
    if (((UInt16)g_mtl_Abs_mac(f_NewestAccWICGrad_si16) >=
         (UInt16)md_VHODirAccGradDetectThresh_ui8) &&
        (g_VHORollRecog_st.MaximaReached_bl == FALSE))
    {
        l_DirecctionFromGradient_en = VHO_MOV_UNDER_INVESTIGATION_enm;
        g_VHORollRecog_st.AccTotalValues_ui8++;

        if (f_NewestAccWICGrad_si16 >= (SInt16)md_VHODirAccGradDetectThresh_ui8)
        {
            g_VHORollRecog_st.AccWICGradPosVal_ui8++;
        }

        /*
        l_Gradient_si16 = g_VHODivide_si32( (SInt32) (
        (SInt32)f_NewestAccWICGrad_si16 - (SInt32)m_DirByGradientOldAccWICGrad_si16)
        * (SInt32) 1000, (SInt32) md_VIRTUAL_LSCAN_TCYC_MS_ui16
                                           ); //gradient [0.01]
        */

        /* search maximum --> comparison of data points */
        if (g_mtl_Abs_mac(m_DirByGradientOldAccWICGrad_si16) >
            g_mtl_Abs_mac(f_NewestAccWICGrad_si16))
        {
            /* local maximum was found */
            g_VHORollRecog_st.MaximaReached_bl = TRUE;
            g_VHORollRecog_st.Maxima_si16      = m_DirByGradientOldAccWICGrad_si16;
        }

        if (f_NewestAxGrad_si16 > (SInt16)md_VHODirAccGradDetectThresh_ui8)
        {
            g_VHORollRecog_st.AxGradPosVal_ui8++;
        }
        else
        {
            if ((UInt16)g_mtl_Abs_mac(f_NewestAxGrad_si16) <
                (UInt16)md_VHODirAccGradDetectThresh_ui8)
            {
                /*reinit --> sensor values below threshold, no gradient comparison
                 * possible */
                g_VHORollRecog_st.AxGradPosVal_ui8     = (UInt8)0;
                g_VHORollRecog_st.AccWICGradPosVal_ui8 = (UInt8)0;
                g_VHORollRecog_st.AccTotalValues_ui8   = (UInt8)0;
                l_DirecctionFromGradient_en            = VHO_MOV_UNKNOWN_enm;
            }
        }

        /* detect direction */
        if (g_VHORollRecog_st.AccTotalValues_ui8 >= md_VHODirAccGradDetectWindow_ui8)
        {
            /* gradients of the sensor acc and the wic based acc moving in same
             * direction --> backward */
            if (((g_VHORollRecog_st.AccWICGradPosVal_ui8 ==
                  g_VHORollRecog_st.AccTotalValues_ui8) &&
                 (g_VHORollRecog_st.AxGradPosVal_ui8 ==
                  g_VHORollRecog_st.AccTotalValues_ui8)) ||
                ((g_VHORollRecog_st.AccWICGradPosVal_ui8 == (UInt8)0) &&
                 (g_VHORollRecog_st.AxGradPosVal_ui8 == (UInt8)0)))

            {
                /* gradients of the sensor acc and the wic based acc moving in same
                 * direction --> backward */
                g_VHORollRecog_st.AxGradPosVal_ui8     = (UInt8)0;
                g_VHORollRecog_st.AccWICGradPosVal_ui8 = (UInt8)0;
                g_VHORollRecog_st.AccTotalValues_ui8   = (UInt8)0;
                l_DirecctionFromGradient_en            = VHO_MOV_BACKW_enm;
            }
            else if (((g_VHORollRecog_st.AccWICGradPosVal_ui8 ==
                       g_VHORollRecog_st.AccTotalValues_ui8) &&
                      (g_VHORollRecog_st.AxGradPosVal_ui8 == (UInt8)0)) ||
                     ((g_VHORollRecog_st.AccWICGradPosVal_ui8 == (UInt8)0) &&
                      (g_VHORollRecog_st.AxGradPosVal_ui8 ==
                       g_VHORollRecog_st.AccTotalValues_ui8)))
            {
                /* gradients of the sensor acc and the wic based acc moving in different
                 * direction --> forward */
                g_VHORollRecog_st.AxGradPosVal_ui8     = (UInt8)0;
                g_VHORollRecog_st.AccWICGradPosVal_ui8 = (UInt8)0;
                g_VHORollRecog_st.AccTotalValues_ui8   = (UInt8)0;
                l_DirecctionFromGradient_en            = VHO_MOV_FORW_enm;
            }
            else
            {
                /* error: reinit => direction unknown */
                g_VHORollRecog_st.AxGradPosVal_ui8     = (UInt8)0;
                g_VHORollRecog_st.AccWICGradPosVal_ui8 = (UInt8)0;
                g_VHORollRecog_st.AccTotalValues_ui8   = (UInt8)0;
                l_DirecctionFromGradient_en            = VHO_MOV_UNKNOWN_enm;
            }
        }
    }
    else
    {
        /* reset value */
        if ((UInt16)g_mtl_Abs_mac(f_NewestAccWICGrad_si16) <
            (UInt16)md_VHODirAccGradDetectThresh_ui8)
        {
            g_VHORollRecog_st.Maxima_si16      = (SInt16)0;
            g_VHORollRecog_st.MaximaReached_bl = (Boolean)FALSE;
        }
        g_VHORollRecog_st.AxGradPosVal_ui8     = (UInt8)0;
        g_VHORollRecog_st.AccWICGradPosVal_ui8 = (UInt8)0;
        g_VHORollRecog_st.AccTotalValues_ui8   = (UInt8)0;
        l_DirecctionFromGradient_en            = VHO_MOV_UNKNOWN_enm;
    }

    m_DirByGradientOldAccWICGrad_si16 = f_NewestAccWICGrad_si16;

    return l_DirecctionFromGradient_en;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function used to determine the zero point of the longitudinal
 *                acceleration
 * @details
 *
 * @param         f_VehVelMperSF8_ui16
 * @param         f_S_CurrentCycleMM_ui16
 * @param         f_Ax_si16
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHODirRecogGetAxZeroPoint_vd(UInt16 f_VehVelMperSF8_ui16,
                                               UInt16 f_S_CurrentCycleMM_ui16,
                                               SInt16 f_Ax_si16)
{
    const UInt8 lc_CycleThresInitial_ui8 =
        (UInt8)(md_VHODirZeroPointInitialTimeMS_ui16 /
                g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16);

    const UInt8 lc_CycleThresMinCalib_ui8 =
        (UInt8)(md_VHODirZeroMinCalibTimeMS_ui16 /
                g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16);

    const UInt8 lc_CycleThresBasePointEval_ui8 =
        (UInt8)(md_VHODirZeroPointTimeBasePointEvalMS_ui16 /
                g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16);

    const UInt8 lc_WeightBaseValue_ui8 =
        (UInt8)(md_VHODirZeroPointWeightBaseValueMS_ui16 /
                g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16);

    const UInt16 lc_ThresDistCountMM_ui16 = (UInt16)60000;

    if (f_VehVelMperSF8_ui16 > md_VHODirZeroPointMaxVel_ui16)
    {
        if (g_VHORollRecog_st.ZeroPointCycleCount_ui8 > 0)
        {
            g_VHORollRecog_st.ZeroPointCycleCount_ui8         = (UInt8)0;
            g_VHORollRecog_st.DistPassedWithoutRefreshMM_ui16 = (UInt16)0;
        }

        /* if-clause acts as overflow protection */
        if (g_VHORollRecog_st.DistPassedWithoutRefreshMM_ui16 < lc_ThresDistCountMM_ui16)
        {
            g_VHORollRecog_st.DistPassedWithoutRefreshMM_ui16 += f_S_CurrentCycleMM_ui16;
        }

        if (g_VHORollRecog_st.DistPassedWithoutRefreshMM_ui16 >
            md_VHODirZeroPointMaxDistBeforeDiscardMM_ui16)
        {
            g_VHORollRecog_st.ZeroPointValid_bl = FALSE;
            g_VHORollRecog_st.AxZeroPoint_si16  = (SInt16)0;
        }
        else if (g_VHORollRecog_st.DistPassedWithoutRefreshMM_ui16 >
                 md_VHODirZeroPointMaxDistCycleMM_ui16)
        {
            g_VHORollRecog_st.ZeroPointMeasCount_ui16 = (UInt16)0;
        }
        else
        {
            // do nothing
        }
    }
    else
    {

        g_VHORollRecog_st.ZeroPointCycleCount_ui8++;

        if (g_VHORollRecog_st.ZeroPointMeasCount_ui16 > 0)
        {
            if ((f_VehVelMperSF8_ui16 == 0) ||
                (g_VHORollRecog_st.ZeroPointValid_bl == FALSE))
            {
                // measurement has already been started
                g_VHORollRecog_st.ZeroPointMeasCount_ui16++;
                g_VHORollRecog_st.ZeroPointAccMeasSum_si32 += (SInt32)f_Ax_si16;

                if (g_VHORollRecog_st.ZeroPointMeasCount_ui16 >
                    (UInt16)lc_CycleThresMinCalib_ui8)
                {
                    g_VHORollRecog_st.AxZeroPoint_si16 =
                        (SInt16)(g_VHORollRecog_st.ZeroPointAccMeasSum_si32 /
                                 (SInt32)g_VHORollRecog_st.ZeroPointMeasCount_ui16);

                    g_VHORollRecog_st.ZeroPointValid_bl = TRUE;

                    // possibility to add a quality metric later to adjust
                    // threshold for integration
                    g_VHORollRecog_st.ZeroPointQuality_ui8 = (UInt8)0xFF;

                    g_VHORollRecog_st.DistPassedWithoutRefreshMM_ui16 = (UInt16)0;
                }
            }
        }
        else
        {
            if (g_VHORollRecog_st.ZeroPointCycleCount_ui8 >= lc_CycleThresInitial_ui8)
            {
                // measurement has not been started yet => get base value for zero point
                // calibration
                if (g_VHORollRecog_st.ZeroPointValid_bl != FALSE)
                {
                    // take last valid zero point as the base value
                    g_VHORollRecog_st.ZeroPointMeasCount_ui16 = lc_WeightBaseValue_ui8;
                    g_VHORollRecog_st.ZeroPointAccMeasSum_si32 =
                        (SInt32)lc_WeightBaseValue_ui8 *
                        (SInt32)g_VHORollRecog_st.AxZeroPoint_si16;
                }
                else
                {
                    // take average between minimum and maximum in a window of
                    // md_VHODirZeroPointTimeBasePointEvalMS_ui16 milliseconds
                    // as base value
                    if (g_VHORollRecog_st.ZeroPointCycleCount_ui8 ==
                        lc_CycleThresInitial_ui8)
                    {
                        g_VHORollRecog_st.AxMin_si16 = f_Ax_si16;
                        g_VHORollRecog_st.AxMax_si16 = f_Ax_si16;
                    }
                    else if (g_VHORollRecog_st.ZeroPointCycleCount_ui8 ==
                             (UInt8)(lc_CycleThresInitial_ui8 +
                                     lc_CycleThresBasePointEval_ui8))
                    {
                        // calculate average and set as the base value
                        g_VHORollRecog_st.ZeroPointMeasCount_ui16 =
                            lc_WeightBaseValue_ui8;
                        g_VHORollRecog_st.ZeroPointAccMeasSum_si32 =
                            (SInt32)lc_WeightBaseValue_ui8 *
                            ((SInt32)(g_VHORollRecog_st.AxMax_si16 +
                                      g_VHORollRecog_st.AxMin_si16) /
                             (SInt32)2);

                        g_VHORollRecog_st.AxMin_si16 = 0;
                        g_VHORollRecog_st.AxMax_si16 = 0;
                    }
                    else
                    {
                        if (f_Ax_si16 > g_VHORollRecog_st.AxMax_si16)
                        {
                            g_VHORollRecog_st.AxMax_si16 = f_Ax_si16;
                        }
                        else
                        {
                            if (f_Ax_si16 < g_VHORollRecog_st.AxMin_si16)
                            {
                                g_VHORollRecog_st.AxMin_si16 = f_Ax_si16;
                            }
                        }
                    }
                }
            }
        }
    }

    if (g_VHORollRecog_st.ZeroPointMeasCount_ui16 == (UInt16)0)
    {
        g_VHORollRecog_st.ZeroPointDebug_si16 = (SInt16)0;
    }
    else
    {
        g_VHORollRecog_st.ZeroPointDebug_si16 =
            (SInt16)(g_VHORollRecog_st.ZeroPointAccMeasSum_si32 /
                     (SInt32)g_VHORollRecog_st.ZeroPointMeasCount_ui16);
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function used to find a direction change without a defined
 *                stillstand in between
 * @details
 *
 * @param         f_AccWic_si16
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY gType_VHORollRecogState_en
m_VHODirRecogFindChangePattern_en(SInt16 f_AccWic_si16)
{
    const UInt8 lc_DirChangeCycleThres_ui8 =
        (UInt8)(md_VHODirChangeTimeThres_ui16 /
                g_parGetParaVHO_Odo_VIRTUAL_LSCAN_TCYC_MS_ui16);

    gType_VHORollRecogState_en l_RetVal_en;

    const UInt8 lc_WindowSize_ui8              = (UInt8)3;
    const UInt16 lc_LowerVelThresMPerSF8_ui16  = (UInt16)14;
    const SInt16 lc_AxThres_si16               = ((SInt16)30);
    const UInt16 lc_MinQuotdAccWICdAxHigh_ui16 = ((UInt16)150);
    const UInt8 lc_AxGradCountThres_ui8        = ((UInt8)2);

    const SInt16 l_AxSyncAvg_si16  = g_VHOKinRMGetAxSyncAvg_si16(lc_WindowSize_ui8);
    const SInt16 l_AxGradSync_si16 = g_VHOKinRMGetAxGrad_si16(md_RM_ELEM_OLDEST_ui8);

    SInt16 l_temp_si16;

    Boolean l_GearIndicator_bl = FALSE;

    l_RetVal_en = VHO_MOV_UNKNOWN_enm;

    if (g_VHOVehicleState_st.VehicleVelocity_ui16 == 0)
    {
        g_VHORollRecog_st.LookingForMin_bl = TRUE;
        l_GearIndicator_bl                 = FALSE;
    }
    else
    {
        if (((g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_FORW_enm) &&
             (g_VHOCanSig_st.ReverseGearInserted_bl != FALSE)) ||
            ((g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm) &&
             (g_VHOCanSig_st.ReverseGearInserted_bl == FALSE)))
        {
            l_GearIndicator_bl = TRUE;
        }
        else
        {
            l_GearIndicator_bl = FALSE;
        }
    }

    if (g_VHORollRecog_st.LookingForMin_bl != FALSE)
    {
        // initial state that is kept until AccWic
        // falls below a defined neg. threshold
        if (f_AccWic_si16 < md_VHOAccWicNegThres_si16)
        {
            // look for maximum
            g_VHORollRecog_st.LookingForMin_bl        = FALSE;
            g_VHORollRecog_st.CountAxGradNeg_ui8      = 0;
            g_VHORollRecog_st.CountAxGradPos_ui8      = 0;
            g_VHORollRecog_st.AccWicMin_si16          = f_AccWic_si16;
            g_VHORollRecog_st.AccWicMax_si16          = g_VHORollRecog_st.AccWicMin_si16;
            g_VHORollRecog_st.DirChangeCycleCount_ui8 = (UInt8)0;
            g_VHORollRecog_st.AxBase_si16             = l_AxSyncAvg_si16;
            g_VHORollRecog_st.VeloBaseMPerSF8_ui16 =
                g_VHOVehicleState_st.VehicleVelocity_ui16;
        }
    }
    else
    {
        // minimum has already been found
        l_RetVal_en = VHO_MOV_UNDER_INVESTIGATION_enm;

        if (g_VHOVehicleState_st.VehicleVelocity_ui16 < lc_LowerVelThresMPerSF8_ui16)
        {
            g_VHORollRecog_st.DirChangeCycleCount_ui8++;
        }

        if (g_VHOVehicleState_st.VehicleVelocity_ui16 <
            g_VHORollRecog_st.VeloBaseMPerSF8_ui16)
        {
            g_VHORollRecog_st.VeloBaseMPerSF8_ui16 =
                g_VHOVehicleState_st.VehicleVelocity_ui16;
        }

        if ((f_AccWic_si16 < (SInt16)0) &&
            (g_VHORollRecog_st.GradAccWIC_si16 < (SInt16)0))
        {
            if (l_AxGradSync_si16 > lc_AxThres_si16)
            {
                if (g_VHORollRecog_st.CountAxGradPos_ui8 < (UInt8)0xFF)
                {
                    g_VHORollRecog_st.CountAxGradPos_ui8++;
                }
                else
                {
                    if (g_VHORollRecog_st.CountAxGradNeg_ui8 > 0)
                    {
                        g_VHORollRecog_st.CountAxGradNeg_ui8--;
                    }
                }
            }

            if (l_AxGradSync_si16 < (-lc_AxThres_si16))
            {
                if (g_VHORollRecog_st.CountAxGradNeg_ui8 < (UInt8)0xFF)
                {
                    g_VHORollRecog_st.CountAxGradNeg_ui8++;
                }
                else
                {
                    if (g_VHORollRecog_st.CountAxGradPos_ui8 > 0)
                    {
                        g_VHORollRecog_st.CountAxGradPos_ui8--;
                    }
                }
            }
        }

        // minimum of AccWic has already been found, look for maximum now
        if (f_AccWic_si16 < g_VHORollRecog_st.AccWicMin_si16)
        {
            // new minimum has been found
            g_VHORollRecog_st.AccWicMin_si16          = f_AccWic_si16;
            g_VHORollRecog_st.AccWicMax_si16          = g_VHORollRecog_st.AccWicMin_si16;
            g_VHORollRecog_st.DirChangeCycleCount_ui8 = (UInt8)0;
            g_VHORollRecog_st.AxBase_si16             = l_AxSyncAvg_si16;
            g_VHORollRecog_st.VeloBaseMPerSF8_ui16 =
                g_VHOVehicleState_st.VehicleVelocity_ui16;
        }
        else if (

            // termination criteria
            (g_VHORollRecog_st.DirChangeCycleCount_ui8 > lc_DirChangeCycleThres_ui8) ||
            // AccWic has decreased too far below a previously found maximum
            ((((SInt16)(g_VHORollRecog_st.AccWicMax_si16 - f_AccWic_si16)) >
              md_VHOAccWicMaxFallback_si16) &&
             (l_GearIndicator_bl == FALSE)))
        {
            // Reset and look for minimum again
            g_VHORollRecog_st.LookingForMin_bl        = TRUE;
            g_VHORollRecog_st.CountAxGradNeg_ui8      = 0;
            g_VHORollRecog_st.CountAxGradPos_ui8      = 0;
            g_VHORollRecog_st.AccWicMin_si16          = (SInt16)0;
            g_VHORollRecog_st.AccWicMax_si16          = (SInt16)0;
            g_VHORollRecog_st.DirChangeCycleCount_ui8 = (UInt8)0;
        }
        else
        {
            UInt16 l_dAccWic_ui16;
            UInt16 l_dAx_ui16;
            UInt16 l_QuotdAccWICdAx_ui16 = 0;

            if (f_AccWic_si16 > g_VHORollRecog_st.AccWicMax_si16)
            {
                g_VHORollRecog_st.AccWicMax_si16 = f_AccWic_si16;
            }

            if ((UInt16)g_mtl_Abs_mac(f_AccWic_si16 - g_VHORollRecog_st.AccWicMin_si16) <
                md_VHOMaxDistAccFromMin_ui16)
            {
                g_VHORollRecog_st.DirChangeCycleCount_ui8 = (UInt8)0;
            }

            l_dAccWic_ui16 = (UInt16)(f_AccWic_si16 - g_VHORollRecog_st.AccWicMin_si16);

            l_dAx_ui16 =
                (UInt16)g_mtl_Abs_mac(l_AxSyncAvg_si16 - g_VHORollRecog_st.AxBase_si16);

            if (l_dAx_ui16 > (UInt16)0)
            {
                l_QuotdAccWICdAx_ui16 =
                    (UInt16)(((UInt32)l_dAccWic_ui16 * (UInt32)100) / (UInt32)l_dAx_ui16);
            }

            if ((f_AccWic_si16 > md_VHOAccWicPosThres_si16) &&
                (l_dAccWic_ui16 > md_VHOAccWicMinDelta_ui16) &&
                (
                    //(l_QuotdAccWICdAx_ui16 > lc_MinQuotdAccWICdAxHigh_ui16) ||
                    (l_GearIndicator_bl != FALSE)))
            {

                l_temp_si16 = (SInt16)(((SInt16)g_VHORollRecog_st.CountAxGradPos_ui8) -
                                       ((SInt16)g_VHORollRecog_st.CountAxGradNeg_ui8));
                if ((UInt16)g_mtl_Abs_mac(l_temp_si16) > (UInt16)lc_AxGradCountThres_ui8)
                {
                    if (g_VHORollRecog_st.CountAxGradPos_ui8 >
                        g_VHORollRecog_st.CountAxGradNeg_ui8)
                    {
                        l_RetVal_en = VHO_MOV_TURNED_BACKW_enm;
                    }
                    else
                    {
                        l_RetVal_en = VHO_MOV_TURNED_FORW_enm;
                    }
                    if (g_VHORollRecog_st.VeloBaseMPerSF8_ui16 >
                        md_VHODirChangeMaxVel_ui16)
                    {
                        l_RetVal_en = VHO_MOV_UNKNOWN_enm;
                    }
                }
                else
                {
                    l_RetVal_en = VHO_MOV_UNKNOWN_enm;
                }
                g_VHORollRecog_st.LookingForMin_bl        = TRUE;
                g_VHORollRecog_st.AccWicMin_si16          = (SInt16)0;
                g_VHORollRecog_st.AccWicMax_si16          = (SInt16)0;
                g_VHORollRecog_st.DirChangeCycleCount_ui8 = (UInt8)0;
                g_VHORollRecog_st.CountAxGradNeg_ui8      = 0;
                g_VHORollRecog_st.CountAxGradPos_ui8      = 0;
            }
        }
    }

    return l_RetVal_en;
}

#endif
// #if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) && (LS_VHO_DIRDETECT_MODE
// == LS_VHO_DIRDETECT_ACCSENS))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         derive current rolling direction from the comparison of the
 *                sense of rotation (clockwise/anticlockwise) and the steering
 *                wheel angle (turned to the right or to the left)
 * @details
 *
 * @param
 * @return       gType_VHORollRecogState_en      0: VHO_STOP_enm;
 *                                               1: VHO_MOV_FORW_enm;
 *                                               2: VHO_MOV_BACKW_enm
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
VISIBILITY gType_VHORollRecogState_en m_VHODirRecogYawRate_en(void)
{
    // static UInt8 l_SWA_Cnt_ui8 = (UInt8) 0; -> not allowed any more, replaced
    // by g_VHORollRecog_st.SWA_Cnt_ui8 static SInt16 l_SWAngBuf_si16 = 0; -> not
    // allowed any more for, replaced by g_VHORollRecog_st.SWAngBuf_si16
    // => variables are initialized by the compiler during startup
    // this is not allowed in PA projects (software reset may not be correctly
    // performed) and therefore all variables shall be initialized by the
    // component init-function "g_CompNameInit_vd"

    SInt32 l_YawRateOffsetCorr_si32;
    UInt16 l_SWAngDegAbsRes_ui16;

    // determine rolling direction
    gType_VHORollRecogState_en l_RetVal_en;

    // default: no direction output
    l_RetVal_en = VHO_MOV_UNKNOWN_enm;

    l_YawRateOffsetCorr_si32 =
        g_VHOAutocalib_st.YawRateFil_si32 - (SInt32)g_VHOAutocalibCtrl_st.YROff_si16;

    // set SWA counter to zero if sign of steering angle changes
    if ((((SInt32)g_VHOCanSig_st.SWA_si16) * ((SInt32)g_VHORollRecog_st.SWAngBuf_si16)) <
        (SInt32)0)
    {
        g_VHORollRecog_st.SWA_Cnt_ui8 = 0;
    }
    // store current SWA value for next iteration
    g_VHORollRecog_st.SWAngBuf_si16 = g_VHOCanSig_st.SWA_si16;
    // get absolute value of current steering angle
    l_SWAngDegAbsRes_ui16 = (UInt16)g_mtl_Abs_mac(g_VHOCanSig_st.SWA_si16);

    if (l_SWAngDegAbsRes_ui16 >= md_VHOYawRateDirSWAThreshold_ui16)
    {
        UInt32 l_AbsYawRateOffsetCorr_ui32;

        // current steering angle is big enough
        if (g_VHORollRecog_st.SWA_Cnt_ui8 < (UInt8)0xFF)
        {
            g_VHORollRecog_st.SWA_Cnt_ui8++;
        }
        l_AbsYawRateOffsetCorr_ui32 = (UInt32)g_mtl_Abs_mac(l_YawRateOffsetCorr_si32);

        if ((g_VHORollRecog_st.SWA_Cnt_ui8 > md_VHOYawRateDirSWACntThres_ui16) &&
            (l_AbsYawRateOffsetCorr_ui32 >= md_VHOYawRateDirYRThres_ui32) &&
            (g_VHOCanSig_st.Vel_ui16 >= md_VHOYawRateDirVeloThres_ui16))
        {
            // the vehicle must drive faster than md_VHOYawRateDirVeloThres_ui16 to
            // realize this YawRate no left <=> right turn during the last
            // swa_counter_thres cycles and absolute value of yaw rate is higher than
            // specified threshold
            l_RetVal_en =
                ((((SInt32)g_VHOCanSig_st.SWA_si16 * l_YawRateOffsetCorr_si32) > 0)
                     ? VHO_MOV_FORW_enm
                     : VHO_MOV_BACKW_enm);
        }
    }
    else
    {
        g_VHORollRecog_st.SWA_Cnt_ui8 = (UInt8)0;
    }

    return l_RetVal_en;
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&
       //     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) ||
       //     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS)...)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         WICTimeDeltaMS_ui16 is needed to detect vehicle stand still
 *                WICTimeDeltaMS_ui16 is needed in premium VHO and in mini VHO
 * @details
 *
 * @param
 * @return        UInt16       WICTimeDeltaMS_ui16
 *
 * @note
 * @see
 * @warning
 */
UInt16 g_VHOg_VHOCalcWICTimeDeltaMS_ui16(void)
{
    UInt16 l_RetVal_ui16;

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
    if (g_mtl_Abs_mac(g_VHOOdoBuf_st.DrivenDistanceMMF8_si32) > (SInt32)(0))
    {
        // a wic increment has occured since last call
        g_VHORollRecog_st.TimeLastWICIncrement_ui16 = g_VHOCanSig_st.WIC_CanTime_ui16;
    }

    if (g_VHORollRecog_st.TimeLastWICIncrement_ui16 <= g_VHOCanSig_st.WIC_CanTime_ui16)
    {
        l_RetVal_ui16 =
            g_VHOCanSig_st.WIC_CanTime_ui16 - g_VHORollRecog_st.TimeLastWICIncrement_ui16;
    }
    else
    {
        l_RetVal_ui16 =
            g_VHOCanSig_st.WIC_CanTime_ui16 +
            (UInt16)((UInt32)65536 - (UInt32)g_VHORollRecog_st.TimeLastWICIncrement_ui16);
    }

    // in case of long stop cycles, it could be happen that (after 65s) that the
    // timestamp delta is < gd_VHOMinTimeBeforeSTOPMS_ui16 thats why the timestamp
    // delta must be freeze
    if ((g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm) &&
        (g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 == (SInt32)(0)) &&
        (l_RetVal_ui16 < gd_VHOMinTimeBeforeSTOPMS_ui16))
    {
        l_RetVal_ui16 = gd_VHOMinTimeBeforeSTOPMS_ui16 + (UInt16)(1);
    }

    return l_RetVal_ui16;
}

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3892
#endif
