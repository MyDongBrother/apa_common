/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho_sig_proc.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO

#include "vho_api.h"
#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#include "vho.h"
#include "vho_sig_proc.h"
// #include <mathlib_api.h>
#include "vho_odo_core.h" // SWCPA Include Structure: ok
// #include <appl_vhm.h>   // SWCPA Include Structure: nok
// #include <vhm_param.h>  // SWCPA Include Structure: ok
// #include <vhm_loghdl.h> // SWCPA Include Structure: ok
#include <string.h> // SWCPA Include Structure: ok

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef GS_4WS_VEHICLE
#error ('compiler switch GS_4WS_VEHICLE is not defined')
#endif

#ifndef LS_VHO_SPP_WIC
#error ('compiler switch LS_VHO_SPP_WIC is not defined')
#endif

#ifndef SW_ON
#error ('compiler switch SW_ON is not defined')
#endif

#ifndef LS_VHO_AUTOCALIB_SWA_OFFSET
#error ('compiler switch LS_VHO_AUTOCALIB_SWA_OFFSET is not defined')
#endif

#ifndef LS_VHO_AUTOCALIB_TYRE_RADII
#error ('compiler switch LS_VHO_AUTOCALIB_TYRE_RADII is not defined')
#endif

#ifndef LS_VHO_AUTOCALIB_TTC
#error ('compiler switch LS_VHO_AUTOCALIB_TTC is not defined')
#endif

#ifndef LS_VHO_EXTRAPOL_ENGINE_RESTART
#error ('compiler switch LS_VHO_EXTRAPOL_ENGINE_RESTART is not defined')
#endif

#ifndef LS_VHO_YES
#error ('compiler switch LS_VHO_YES is not defined')
#endif

#define md_VHOWICErrThres_ui8 ((UInt8)5)

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3892
#endif

#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)

VISIBILITY SInt16 m_VHOSignalPreProcEstimateSWA_si16(SInt16 f_YawRateDegPerSec_si16,
                                                     UInt16 f_VeloRearAxleCenterKmh_ui16,
                                                     UInt16 f_WheelBaseMM_ui16);

VISIBILITY void m_VHOSignalPreProcSWAOffsetCalc_vd(void);

VISIBILITY UInt16 m_VHOGetRelError_ui16(SInt16 f_SteeringAngleRadF12_si16);

VISIBILITY void m_VHOSignalPreprocCalcCorrWithSWAOff_vd(void);

VISIBILITY void m_VHOSignalPreProcSetSWAThresh(
    UInt16 *f_MaxSWAToBeDecreased_pui16, UInt16 *f_MinSWAToBeIncreased_pui16,
    UInt16 *f_MaxSWAFullRange_pui16, UInt16 *f_MinSWAFullRange_pui16,
    const UInt16 *fc_AvgErrToBeDecreased_pui16,
    const UInt16 *fc_AvgErrToBeIncreased_pui16);

VISIBILITY void m_VHOResetSWAOffsetCalib_vd(void);

#endif
/*----------------------------------------------------------------*/
#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)

VISIBILITY UInt16 m_VHOSignalPreProcEstimateVelo_ui16(SInt16 f_YawRateDegPerSec_si16,
                                                      SInt16 f_SteeringAngleRadF12_si16,
                                                      UInt16 f_WheelBaseMM_ui16);

VISIBILITY void m_VHOSignalPreProcTyreRadiiCalc_vd(void);

VISIBILITY void m_VHOResetTyreRadiiCalib_vd(void);

#endif
/*----------------------------------------------------------------*/
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)

VISIBILITY void m_VHOSignalPreprocTTC_VEL_vd(void);

VISIBILITY UInt8 m_VHOSignalPreprocDevAboveThres_ui8(UInt16 f_FirstValue_ui16,
                                                     UInt16 f_SecondValue_ui16,
                                                     UInt8 f_MaxDevPercentF4_ui8);

VISIBILITY Boolean m_VHOSignalPreprocTTC_AdmCondFulfilled_bl(
    UInt16 f_VehicleVelocityMperSF2_ui16, SInt16 f_VehicleAccelerationMperSPow2F4_si16,
    SInt16 f_YawRateFiltered_si16, SInt16 f_YawRateOffset_si16,
    gType_VHORollRecogState_en f_RollingDirection_en);

VISIBILITY void m_VHOSignalPreprocTTCEvalRadDiff_vd(UInt8 *l_DevPercentMinMax_pui8,
                                                    Boolean *l_SpareTyreMounted_pbl);

VISIBILITY UInt16 m_VHOSignalPreprocTTCGetScalingFactor_ui16(UInt32 l_A_ui32,
                                                             UInt32 l_B_ui32);

VISIBILITY void m_VHOResetTTC_vd(void);

#endif
/*----------------------------------------------------------------*/
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))

VISIBILITY void m_VHOSignalPreprocUpdateVehicleState_vd(Boolean f_WriteToEEPROM_bl);

VISIBILITY void m_VHOSignalPreprocYawRate_vd(void);

VISIBILITY Boolean m_VHOSignalPreProcYROffsetCalc_bl(void);

VISIBILITY Boolean m_VHOValidateAutocalibCtrl_bl(void);

#endif

VISIBILITY void m_VHOSignalPreprocWIC_vd(Boolean f_ForceExtrapolation_bl);
VISIBILITY void m_VHOSignalPreprocSetCorrectedSignalsToRawSignals_vd(void);

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         reset variables of input validation and calibration
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOSignalPreprocInit_vd(void)
{
    (void)memset(&g_VHOAutocalib_st, 0, sizeof(gType_VHOAutocalib_st));
    (void)memset(&g_VHOAutocalibCtrl_st, 0, sizeof(gType_VHOAutocalibCtrl_st));
    g_VHOAutocalib_st.BypassWP_bl = FALSE;

    // set state of all autocalib modules to 'initial' - just to be sure ;-)
    g_VHOAutocalibCtrl_st.SWAState_enm      = VHOCalibState_INITIAL_enm;
    g_VHOAutocalibCtrl_st.TyreRadState_enm  = VHOCalibState_INITIAL_enm;
    g_VHOAutocalibCtrl_st.TTCState_enm      = VHOCalibState_INITIAL_enm;
    g_VHOAutocalibCtrl_st.YRState_enm       = VHOCalibState_INITIAL_enm;
    g_VHOAutocalibCtrl_st.AxleBaseState_enm = VHOCalibState_INITIAL_enm;

    // set ttc deviations to 'no deviation' for all tyres
    g_VHOAutocalibCtrl_st.ScalFacRR_ui16 = (UInt16)51200;
    g_VHOAutocalibCtrl_st.ScalFacLF_ui16 = (UInt16)51200;
    g_VHOAutocalibCtrl_st.ScalFacRF_ui16 = (UInt16)51200;

    // set average wic length of the rear axle to default length
    g_VHOAutocalibCtrl_st.AvgWICLengthRearAxle_ui16 =
        g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

    g_VHOAutocalib_st.YawRateRMSize_ui8 =
        (UInt8)(gd_TIME_WINDOW_YR_FILT_MS_ui16 /
                (UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8);

    g_VHOAutocalibCtrl_st.AxleBase_ui16 = g_parGetParaVHO_Veh_AxleBaseRear_ui16;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         reset autocalibration data from eeprom, function has to be
 *                called at system startup as soon as values are read from
 *                eeprom
 * @details
 *
 * @param         fc_VHOAutocalibData_pst
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
void g_InitAutocalibFromEEPROM_ui8(
    const gType_VHOParaAutocalib_st *fc_VHOAutocalibData_pst)
{
    // set ttc state
    g_VHOAutocalibCtrl_st.ScalFacRR_ui16 = fc_VHOAutocalibData_pst->ScalFacRR_ui16;
    g_VHOAutocalibCtrl_st.ScalFacLF_ui16 = fc_VHOAutocalibData_pst->ScalFacLF_ui16;
    g_VHOAutocalibCtrl_st.ScalFacRF_ui16 = fc_VHOAutocalibData_pst->ScalFacRF_ui16;
    g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 =
        fc_VHOAutocalibData_pst->NumStableTTCCycles_ui8;
    if (fc_VHOAutocalibData_pst->NumStableTTCCycles_ui8 >=
        gd_VCALTTCMinCyclesUntilStable_ui8)
    {
        g_VHOAutocalibCtrl_st.TTCState_enm = VHOCalibState_STABLE_enm;
    }
    else if (fc_VHOAutocalibData_pst->NumStableTTCCycles_ui8 == (UInt8)0)
    {
        g_VHOAutocalibCtrl_st.TTCState_enm = VHOCalibState_INITIAL_enm;
    }
    else
    {
        g_VHOAutocalibCtrl_st.TTCState_enm = VHOCalibState_UNSTABLE_enm;
    }

    //
    // set state of steering wheel angle offset calibration
    //
    g_VHOAutocalibCtrl_st.SWOffDegRes_si16 = fc_VHOAutocalibData_pst->SWOffDegRes_si16;
    g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 =
        fc_VHOAutocalibData_pst->NumStableSWACycles_ui8;
    if (fc_VHOAutocalibData_pst->NumStableSWACycles_ui8 >=
        gd_VCALSWAMinCyclesUntilStable_ui8)
    {
        g_VHOAutocalibCtrl_st.SWAState_enm = VHOCalibState_STABLE_enm;
    }
    else if (fc_VHOAutocalibData_pst->NumStableSWACycles_ui8 == (UInt8)0)
    {
        g_VHOAutocalibCtrl_st.SWAState_enm = VHOCalibState_INITIAL_enm;
    }
    else
    {
        g_VHOAutocalibCtrl_st.SWAState_enm = VHOCalibState_UNSTABLE_enm;
    }

    //
    // set state of tyre radii calibration
    //
    g_VHOAutocalibCtrl_st.AvgWICLengthRearAxle_ui16 =
        fc_VHOAutocalibData_pst->AvgWICLengthRearAxle_ui16;
    g_VHOAutocalibCtrl_st.NumStableTyreRadCycles_ui8 =
        fc_VHOAutocalibData_pst->NumStableTyreRadCycles_ui8;
    if (fc_VHOAutocalibData_pst->NumStableTyreRadCycles_ui8 >=
        gd_VCALTyreMinCyclesUntilStable_ui8)
    {
        g_VHOAutocalibCtrl_st.TyreRadState_enm = VHOCalibState_STABLE_enm;
    }
    else if (fc_VHOAutocalibData_pst->NumStableTyreRadCycles_ui8 == (UInt8)0)
    {
        g_VHOAutocalibCtrl_st.TyreRadState_enm = VHOCalibState_INITIAL_enm;
    }
    else
    {
        g_VHOAutocalibCtrl_st.TyreRadState_enm = VHOCalibState_UNSTABLE_enm;
    }

    //
    // yaw rate offset is not read from EEPROM => set to initial
    //
    g_VHOAutocalibCtrl_st.YRState_enm = VHOCalibState_INITIAL_enm;

    // save autocalibration state
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.AvgWICLengthRearAxle_ui16 =
        g_VHOAutocalibCtrl_st.AvgWICLengthRearAxle_ui16;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.NumStableSWACycles_ui8 =
        g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.NumStableTTCCycles_ui8 =
        g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.NumStableTyreRadCycles_ui8 =
        g_VHOAutocalibCtrl_st.NumStableTyreRadCycles_ui8;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.ScalFacLF_ui16 =
        g_VHOAutocalibCtrl_st.ScalFacLF_ui16;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.ScalFacRF_ui16 =
        g_VHOAutocalibCtrl_st.ScalFacRF_ui16;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.ScalFacRR_ui16 =
        g_VHOAutocalibCtrl_st.ScalFacRR_ui16;
    g_VHOAutocalib_st.AutocalibStateEEPROM_st.SWOffDegRes_si16 =
        g_VHOAutocalibCtrl_st.SWOffDegRes_si16;

    // call vehicle state update to calculate wic lengths according to the
    // state variables read from eeprom
    g_VHOAutocalibCtrl_st.StateChangedInCurrentCycle_bl = TRUE;
    m_VHOSignalPreprocUpdateVehicleState_vd(FALSE);
}
#endif // #if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC ==
       //  SW_ON) ||
//   (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOSignalPreproc_vd(void)
{
    // static Boolean g_VHOAutocalib_st.BypassWP_bl = FALSE;

    // set corrected signals to raw data --> initialization / standardized input
    m_VHOSignalPreprocSetCorrectedSignalsToRawSignals_vd();

//
// CALCULATE CORRECTIONS BY USE OF STEERING WHEEL ANGLE OFFSET
// AND YAW RATE OFFSET
//
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
    m_VHOSignalPreprocCalcCorrWithSWAOff_vd();
#endif

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
    // caculate moving average of the yaw rate provided by can
    m_VHOSignalPreprocYawRate_vd();
#endif

    //
    // PREPROCESSING OF WHEEL IMPULSES
    //
    if ((g_VHOAutocalib_st.WPTimeCounterStart_ui8 <
         ((UInt32)md_VHODIAG_RESET_TIME_MS_ui16 /
          g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8)) &&
        (g_VHOAutocalib_st.BypassWP_bl == FALSE))
    {
        g_VHOAutocalib_st.WPNoErrCnt_ui8 = 0;

#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
        if (g_VHOCtrlStat_st.DegModeExtrapolForceExtrapol_bl != FALSE)
        {
            // perform preprocessing of WIC signals
            m_VHOSignalPreprocWIC_vd(TRUE);
        }
        else if (g_VHOParameters_st.WICPreprocActive_bl != FALSE)
        {
            // perform preprocessing of WIC signals
            m_VHOSignalPreprocWIC_vd(FALSE);
        }
        else
        {
            // in case of a not provided pre-processing of the wic signal, the wic
            // specific failure must debounced. The reason therefore is that system
            // DTCs will not debounce until all connecting failures will be debounced
            // themselves.
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOff_enm);
        }
#else
        if (g_VHOParameters_st.WICPreprocActive_bl != FALSE)
        {
            // perform preprocessing of WIC signals
            m_VHOSignalPreprocWIC_vd(FALSE);
        }
        else
        {
            // in case of a not provided pre-processing of the wic signal, the wic
            // specific failure must debounced. The reason therefore is that system
            // DTCs will not debounce until all connecting failures will be debounced
            // themselves.
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOff_enm);
        }
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    }
    else
    {
        g_VHOAutocalib_st.BypassWP_bl = TRUE;

        if ((g_VHOAutocalib_st.WPTimeCounterStart2_ui8 != 0) &&
            (g_VHOAutocalib_st.WPTimeCounterStart2_ui8 <=
             (md_VHODIAG_MAX_TIME_FAULT_PRESENT_ui16 /
              g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8)))
        {
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOn_enm);
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8++;
        }
        else
        {
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOff_enm);
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8 = 0;
        }

        g_VHOAutocalib_st.WPErrCnt_ui8           = 0;
        g_VHOAutocalib_st.ContinuousWPErrCnt_ui8 = 0;
        g_VHOAutocalib_st.WPTimeCounterStart_ui8 = 0;

        if (g_VHOAutocalib_st.WPNoErrCnt_ui8 >=
            (UInt8)(md_VHODIAG_MAX_BYPASS_WIC_TIME_MS_ui16 /
                    g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8))
        {
            g_VHOAutocalib_st.BypassWP_bl = FALSE;
        }

        g_VHOAutocalib_st.WPNoErrCnt_ui8++;
    }

//
// AUTOCALIBRATION
//
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
    if (g_VHOCanSig_st.YawRate_ValidFlag_bl != FALSE)
    {
        Boolean l_AutocalibAmdissionCondition1_bl;
        Boolean l_AutocalibAmdissionCondition2_bl;

        l_AutocalibAmdissionCondition1_bl = m_VHOSignalPreProcYROffsetCalc_bl();
        l_AutocalibAmdissionCondition2_bl = g_VHPSMxIdle_bl();

        if ((l_AutocalibAmdissionCondition1_bl != FALSE) &&
            (l_AutocalibAmdissionCondition2_bl != FALSE))
        {
            //
            // valid offset for yaw rate has been determined
            // => carry on with determination of steering wheel offset and tyre radii
            //

            // calculate velocity on center of the rear axle
            g_VHOAutocalib_st.VeloRearAxleCenter_ui16 = g_mtl_ReducePrecision_ui16(
                g_VHOOdoBuf_st.WheelVelokmh_pui16[LeftRear_enm] +
                    g_VHOOdoBuf_st.WheelVelokmh_pui16[RightRear_enm],
                (UInt8)1);

// calculate velocity on center of the rear axle by given steering angle rear
#if (GS_4WS_VEHICLE == SW_ON)
            {
                // get veloRearZeroSAR
                const UInt16 lc_veloRearZeroSAR_ui16 =
                    g_VHOAutocalib_st.VeloRearAxleCenter_ui16;

                // get steering angle rear
                const SInt16 lc_SARF12_si16 = g_VHOVehicleState_st.SAngRearRadF12_si16;

                // only absolute value of rear steering angle is needed    F12
                const SInt16 lc_Abs_SAR_rad_F12_si16 =
                    (SInt16)g_mtl_Abs_mac(lc_SARF12_si16);

                // calculate cos of absolute value of steering angle rear  F14
                const UInt16 lc_CosAbsSAR_ui16 =
                    (UInt16)(g_mtl_Cos_si16(lc_Abs_SAR_rad_F12_si16));

                // weitere Berechnungen nur innerhalb sinnvoller Winkelbereiche
                // durchf�hren. Winkelbereiche, die kleiner als 25� sind.
                // Pi*(25�/180�)*2^12 = 1787 l_temp_ui32 =  VeloRearAxleCenter_ui16 *
                // l_CosAbsSAR_si16
                //                        2^16            *      2^14 = 2^30 < 2^32
                if (lc_Abs_SAR_rad_F12_si16 < (SInt16)(1787))
                {
                    const UInt32 lc_a_Mul_b_ui32 =
                        (UInt32)(lc_veloRearZeroSAR_ui16 * lc_CosAbsSAR_ui16);

                    g_VHOAutocalib_st.VeloRearAxleCenter_ui16 =
                        (UInt16)(g_mtl_ReducePrecision_ui32(lc_a_Mul_b_ui32,
                                                            (UInt8)(14)));
                }
            }
#endif

#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
            m_VHOSignalPreProcSWAOffsetCalc_vd();
#endif

#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)
            m_VHOSignalPreProcTyreRadiiCalc_vd();
#endif

#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
            if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm) ||
                (g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICRear_enm) ||
                (g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm))
            {
                // use wheel velocities for tyre tolerance compensation
                m_VHOSignalPreprocTTC_VEL_vd();
            }
#endif
        }
    }
#endif

#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
    m_VHOSignalPreprocUpdateVehicleState_vd(TRUE);
#endif

/*
  suppress nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 2217
#endif

    // Christian Pampus: Hier handelt es sich um einen speziellen BMW-Code, der
    // immer berechnet werden soll.
    {
        UInt32 l_TyreCircumference_pui32[4];
        UInt32 l_TeethCount_ui32;

        l_TeethCount_ui32 = (UInt32)g_parGetParaVHO_Veh_WICTeethCount_ui16;

        l_TyreCircumference_pui32[LeftRear_enm] =
            (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm] *
            l_TeethCount_ui32;
        l_TyreCircumference_pui32[RightRear_enm] =
            (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm] *
            l_TeethCount_ui32;
        l_TyreCircumference_pui32[LeftFront_enm] =
            (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm] *
            l_TeethCount_ui32;
        l_TyreCircumference_pui32[RightFront_enm] =
            (UInt32)g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm] *
            l_TeethCount_ui32;

        if (m_VHOPluginSetExternalCircumference_bl(l_TyreCircumference_pui32) != FALSE)
        {
            g_VHOAutocalibCtrl_st.TTCState_enm = VHOCalibState_STABLE_enm;
            g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm] =
                (UInt16)(l_TyreCircumference_pui32[LeftRear_enm] / l_TeethCount_ui32);
            g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm] =
                (UInt16)(l_TyreCircumference_pui32[RightRear_enm] / l_TeethCount_ui32);
            g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm] =
                (UInt16)(l_TyreCircumference_pui32[LeftFront_enm] / l_TeethCount_ui32);
            g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm] =
                (UInt16)(l_TyreCircumference_pui32[RightFront_enm] / l_TeethCount_ui32);
            g_VHOVehicleState_st.WICLengthMMF8_pui16[AvgLeftRight_enm] =
                g_mtl_ReducePrecision_ui16(
                    g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm] +
                        g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm],
                    1);
        }
    }
/*
  reactivate nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 2217
#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         set corrected signal to raw signals --> independent of
 *                auto-calibration functions
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOSignalPreprocSetCorrectedSignalsToRawSignals_vd(void)
{

    SInt16 l_SWAAngDegRes_buf_si16;

    // set corrected yaw rate to raw signal
    g_VHOCorrectedSignals_st.Corrected_YawRate_si32 = g_VHOCanSig_st.YawRate_si16;

    // 03.11.2011, shi2lr: This part is necessary for the calculation of kappa
    // based on the current steering wheel angle and not on the delayed offset
    // corrected SWA. This part is replaced by
    // g_VHOSWAPreprocSetCorrectedSWAToRawSWA_vd for the delayed offset corrected
    // SWA.

    l_SWAAngDegRes_buf_si16 = g_VHOOdoBuf_st.SWAngDegRes_buf_si16;

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // calculate steering angle for a reverse movement
        g_VHOCorrectedSignals_st.Corrected_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            (SInt16)g_mtl_ReducePrecision_si32(
                (SInt32)g_VHOCanSig_st.SWA_si16 + ((SInt32)l_SWAAngDegRes_buf_si16), 1),
            FALSE);
    }
    else
    {
        // calculate steering angle
        g_VHOCorrectedSignals_st.Corrected_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            (SInt16)g_mtl_ReducePrecision_si32(
                (SInt32)g_VHOCanSig_st.SWA_si16 + ((SInt32)l_SWAAngDegRes_buf_si16), 1),
            TRUE);
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         set corrected signal to raw signals --> independent of
 *                auto-calibration functions
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOSWAPreprocSetCorrectedSWAToRawSWA_vd(void)
{
    // set corrected steering wheel angle to delayed SWA signal
    g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 = g_VHOGetSWAFromBuf_si16();

    if (g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 >= g_VHOCanSig_st.SWA_si16)
    {
        if ((g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 -
             g_VHOCanSig_st.SWA_si16) > g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16)
        {
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 =
                g_VHOCanSig_st.SWA_si16 + g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16;
        }
    }
    else
    {
        if ((g_VHOCanSig_st.SWA_si16 -
             g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16) >
            g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16)
        {
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 =
                g_VHOCanSig_st.SWA_si16 - g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16;
        }
    }

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // calculate steering angle for a reverse movement
        g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16, FALSE);
    }
    else
    {
        // calculate steering angle
        g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16, TRUE);
    }
}

#if (GS_4WS_VEHICLE == SW_ON)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         set corrected signal to raw signals --> independent of
 *                auto-calibration functions
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOSARPreprocSetCorrectedSARToRawSAR_vd(void)
{
    // Zahlenwert als Ersatz f�r solange bis Parameterdefinition
    // nur solange bis g_parGetParaVHO_Odo_SARMaxDeviationDeg_si16 funktioniert
    const SInt16 lc_SAngRearDegGradient_si16 = (SInt16)(25);

    // set corrected steering angle rear to delayed SAR signal
    g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16 = g_VHOGetSARFromBuf_si16();

    if (g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16 >= g_VHOCanSig_st.SAR_si16)
    {
        if ((g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16 -
             g_VHOCanSig_st.SAR_si16) > (lc_SAngRearDegGradient_si16))
        {
            g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16 =
                g_VHOCanSig_st.SAR_si16 + lc_SAngRearDegGradient_si16;
        }
    }
    else
    {
        if ((g_VHOCanSig_st.SAR_si16 -
             g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16) >
            (lc_SAngRearDegGradient_si16))
        {
            g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16 =
                g_VHOCanSig_st.SAR_si16 - lc_SAngRearDegGradient_si16;
        }
    }

    // CAN norming of steering angle rear in degrees: 1 LSB = 1�/50
    // VHO norming of steering angle rear in radiand: 1 LSB = 1rad/2^12
    // transformation of steering angle rear in degrees to radiand:
    // SAR_rad = SAR_deg * (2^12 * pi)/(50 * 180) = SAR_deg * (12868/9000)
    g_VHOCorrectedSignals_st.DelayedCorr_SAngRearRadF12_si16 =
        g_mtl_s16_Mul_s16_Div_s16_si16(
            (SInt16)(g_VHOCorrectedSignals_st.DelayedCorr_SAngRearDeg_si16),
            (SInt16)(12868), (SInt16)(9000));
}
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOSignalPreprocCalcCorrWithSWAOff_vd(void)
{
    SInt16 l_SWAAngDegRes_buf_si16;

    // set corrected yaw rate to raw signal
    g_VHOCorrectedSignals_st.Corrected_YawRate_si32 = g_VHOCanSig_st.YawRate_si16;

    if (g_VHOAutocalibCtrl_st.YRState_enm == VHOCalibState_STABLE_enm)
    {
        // calculate offset corrected yaw rate
        g_VHOCorrectedSignals_st.Corrected_YawRate_si32 -=
            (SInt32)g_VHOAutocalibCtrl_st.YROff_si16;
    }

    //  This part is necessary for the calculation of kappa
    // based on the current steering wheel angle and not on the delayed offset
    // corrected SWA. The delayed offset corrected SWA is calculated by
    // g_VHOSWAPreprocCalcCorrWithSWAOff_vd

    // set corrected steering wheel angle to SWA signal
    g_VHOCorrectedSignals_st.Corrected_SWAngDeg_si16 = g_VHOCanSig_st.SWA_si16;

    l_SWAAngDegRes_buf_si16 = g_VHOOdoBuf_st.SWAngDegRes_buf_si16;

#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
    if (g_VHOAutocalibCtrl_st.SWAState_enm == VHOCalibState_STABLE_enm)
    {
        // calculate offset corrected steering wheel angle
        g_VHOCorrectedSignals_st.Corrected_SWAngDeg_si16 -=
            g_VHOAutocalibCtrl_st.SWOffDegRes_si16;

        l_SWAAngDegRes_buf_si16 -= g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
    }
#endif

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // calculate offset corrected steering angle for a reverse
        // movement

        g_VHOCorrectedSignals_st.Corrected_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            (SInt16)g_mtl_ReducePrecision_si32(
                (SInt32)g_VHOCorrectedSignals_st.Corrected_SWAngDeg_si16 +
                    ((SInt32)l_SWAAngDegRes_buf_si16),
                1),
            FALSE);
    }
    else
    {
        // calculate offset corrected steering angle
        g_VHOCorrectedSignals_st.Corrected_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            (SInt16)g_mtl_ReducePrecision_si32(
                (SInt32)g_VHOCorrectedSignals_st.Corrected_SWAngDeg_si16 +
                    ((SInt32)l_SWAAngDegRes_buf_si16),
                1),
            TRUE);
    }
}
/*
  suppress QAC warning messages 3204
  The variable 'l_AvgWicLengthRearAxle_ui16' is only set once -
  This depends on the compiler switches used in this function
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3204
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
void g_VHOSWAPreprocCalcCorrWithSWAOff_vd(void)
{
    // set corrected steering wheel angle to delayed SWA signal
    g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 = g_VHOGetSWAFromBuf_si16();

    if (g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 >= g_VHOCanSig_st.SWA_si16)
    {
        if ((g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 -
             g_VHOCanSig_st.SWA_si16) > g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16)
        {
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 =
                g_VHOCanSig_st.SWA_si16 + g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16;
        }
    }
    else
    {
        if ((g_VHOCanSig_st.SWA_si16 -
             g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16) >
            g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16)
        {
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 =
                g_VHOCanSig_st.SWA_si16 - g_parGetParaVHO_Odo_SWAMaxDeviationDeg_si16;
        }
    }

    if (g_VHOAutocalibCtrl_st.SWAState_enm == VHOCalibState_STABLE_enm)
    {
        // calculate offset corrected steering wheel angle
        g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16 -=
            g_VHOAutocalibCtrl_st.SWOffDegRes_si16;
    }

    if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
    {
        // calculate offset corrected steering angle for a reverse
        // movement

        g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16, FALSE);
    }
    else
    {
        // calculate offset corrected steering angle
        g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
            g_VHOCorrectedSignals_st.DelayedCorr_SWAngDeg_si16, TRUE);
    }
}
#endif // #if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details       this function is used to set the autocalibration state in
 *                g_VHOVehicleState_st according to the results of the
 *                the signal preprocessing module
 *                IMPORTANT: the variables MUST NOT be modified by ANY OTHER
 *                           function than this one !!!
 *
 *                These variables are:
 *                  g_VHOAutocalibCtrl_st.YROff_si16
 *                  g_VHOVehicleState_st.WICLengthMMF8_pui16
 *
 * @param         f_WriteToEEPROM_bl
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
VISIBILITY void m_VHOSignalPreprocUpdateVehicleState_vd(Boolean f_WriteToEEPROM_bl)
{
    // this function is called before autocalib values are set active; this values
    // can be read by EEPROM or calculated by the component; to prevent that
    // corrupted EEPROM data is used the data is validated before
    Boolean l_AutocalibCtrlValidated_bl = m_VHOValidateAutocalibCtrl_bl();

    // update odo setup (yaw angle calculation depends on autocalib state and
    // driven axle)
    g_VHOOdoSetup_vd();

    if (f_WriteToEEPROM_bl != FALSE)
    {
        if ((((UInt32)(g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 -
                       g_VHOAutocalib_st.TimeLastEEPROMaccess_ui32) >
              md_VHOMinTimeBetweenEEPROMWriteOpMS_ui32) ||
             (g_VHOAutocalib_st.TimeLastEEPROMaccess_ui32 == 0)) &&
            (g_VHOAutocalib_st.l_EEPROMWriteREQ_bl != FALSE) &&
            (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 <
             md_VHOMaxVeloEEPROMWriteOpMS_ui16))
        {
/*
  suppress nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 2217
#endif

            g_VHOParameters_st.AutocalibData_st.AvgWICLengthRearAxle_ui16 =
                g_VHOAutocalibCtrl_st.AvgWICLengthRearAxle_ui16;
            g_VHOParameters_st.AutocalibData_st.NumStableSWACycles_ui8 =
                g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8;
            g_VHOParameters_st.AutocalibData_st.NumStableTTCCycles_ui8 =
                g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8;
            g_VHOParameters_st.AutocalibData_st.NumStableTyreRadCycles_ui8 =
                g_VHOAutocalibCtrl_st.NumStableTyreRadCycles_ui8;
            g_VHOParameters_st.AutocalibData_st.ScalFacLF_ui16 =
                g_VHOAutocalibCtrl_st.ScalFacLF_ui16;
            g_VHOParameters_st.AutocalibData_st.ScalFacRF_ui16 =
                g_VHOAutocalibCtrl_st.ScalFacRF_ui16;
            g_VHOParameters_st.AutocalibData_st.ScalFacRR_ui16 =
                g_VHOAutocalibCtrl_st.ScalFacRR_ui16;
            g_VHOParameters_st.AutocalibData_st.SWOffDegRes_si16 =
                g_VHOAutocalibCtrl_st.SWOffDegRes_si16;

/*
  reactivate nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 2217
#endif

            m_VHOSetEepromRequestModeToWrite_vd();

/*
  suppress nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 2217
#endif

            // save autocalibration state
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.AvgWICLengthRearAxle_ui16 =
                g_VHOAutocalibCtrl_st.AvgWICLengthRearAxle_ui16;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.NumStableSWACycles_ui8 =
                g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.NumStableTTCCycles_ui8 =
                g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.NumStableTyreRadCycles_ui8 =
                g_VHOAutocalibCtrl_st.NumStableTyreRadCycles_ui8;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.ScalFacLF_ui16 =
                g_VHOAutocalibCtrl_st.ScalFacLF_ui16;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.ScalFacRF_ui16 =
                g_VHOAutocalibCtrl_st.ScalFacRF_ui16;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.ScalFacRR_ui16 =
                g_VHOAutocalibCtrl_st.ScalFacRR_ui16;
            g_VHOAutocalib_st.AutocalibStateEEPROM_st.SWOffDegRes_si16 =
                g_VHOAutocalibCtrl_st.SWOffDegRes_si16;

            g_VHOAutocalib_st.TimeLastEEPROMaccess_ui32 =
                g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32;
            g_VHOAutocalib_st.l_EEPROMWriteREQ_bl = FALSE;

/*
  reactivate nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 2217
#endif
        }
    }

    if ((g_VHOAutocalibCtrl_st.StateChangedInCurrentCycle_bl != FALSE) &&
        (l_AutocalibCtrlValidated_bl != FALSE))
    {
        // an autocalib component has produced new result values
        UInt16 l_WICLengthMMF8_pui16[5];
        UInt16 l_AvgWicLengthRearAxle_ui16 =
            g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)
        if (g_VHOAutocalibCtrl_st.TyreRadState_enm == VHOCalibState_STABLE_enm)
        {
            l_AvgWicLengthRearAxle_ui16 = g_VHOAutocalibCtrl_st.AvgWICLengthRearAxle_ui16;
        }
        // else default will be used
#endif

        // set all wic lengths to the same value, will only be changed in case that
        // ttc produced stable output
        l_WICLengthMMF8_pui16[LeftRear_enm]     = l_AvgWicLengthRearAxle_ui16;
        l_WICLengthMMF8_pui16[RightRear_enm]    = l_AvgWicLengthRearAxle_ui16;
        l_WICLengthMMF8_pui16[LeftFront_enm]    = l_AvgWicLengthRearAxle_ui16;
        l_WICLengthMMF8_pui16[RightFront_enm]   = l_AvgWicLengthRearAxle_ui16;
        l_WICLengthMMF8_pui16[AvgLeftRight_enm] = l_AvgWicLengthRearAxle_ui16;

#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
        if (g_VHOAutocalibCtrl_st.TTCState_enm == VHOCalibState_STABLE_enm)
        {
            // get the wic length of the reference wheel from the average wic length
            // on the rear axle and the results of tyre tolerance compensation
            //
            // wl_avg_rear = 0.5 * (wl_left_rear + scal_fac_right_rear * wl_left_rear)
            //
            //                    51200* (2 * wl_avg_rear)
            //  => wl_left_rear = -----------------------------
            //                    (51200 + scal_fac_right_rear)
            //
            // (NOTE: scal_fac_right_rear is itself scaled with 51200)

            l_WICLengthMMF8_pui16[LeftRear_enm] =
                (UInt16)((
                             // wic length is assumed to be smaller than 6cm =>
                             // l_AvgWicLengthRearAxle_ui16 < 15360 < 41943 = 2^32/102400
                             (UInt32)102400 * (UInt32)l_AvgWicLengthRearAxle_ui16) /
                         ((UInt32)51200 + (UInt32)g_VHOAutocalibCtrl_st.ScalFacRR_ui16));

            // get wic length of other three tyres by scaling the wic length
            // of the reference wheel with the factor generated by ttc;
            // the scaling facor is itself scaled with 51200, so the result value
            // is calculated with
            //
            //         wl_lr * scal_fac_x
            //  wl_x = ------------------
            //         51200

            l_WICLengthMMF8_pui16[RightRear_enm] =
                (UInt16)(((UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
                          (UInt32)g_VHOAutocalibCtrl_st.ScalFacRR_ui16) /
                         (UInt32)51200);

            l_WICLengthMMF8_pui16[LeftFront_enm] =
                (UInt16)(((UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
                          (UInt32)g_VHOAutocalibCtrl_st.ScalFacLF_ui16) /
                         (UInt32)51200);

            l_WICLengthMMF8_pui16[RightFront_enm] =
                (UInt16)(((UInt32)l_WICLengthMMF8_pui16[LeftRear_enm] *
                          (UInt32)g_VHOAutocalibCtrl_st.ScalFacRF_ui16) /
                         (UInt32)51200);
            // in case that the wic source provides average wheel impulses
            // from the rear axle it has to be set to the estimated wic
            // length from the tyre radii calibration; in case that it
            // provides average wheel impulses from the front axle the
            // average of the two front wic lengths is used

            if (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgFront_enm)
            {
                l_WICLengthMMF8_pui16[AvgLeftRight_enm] =
                    g_mtl_ReducePrecision_ui16(l_WICLengthMMF8_pui16[LeftFront_enm] +
                                                   l_WICLengthMMF8_pui16[RightFront_enm],
                                               1);
            }
        }
#endif // #if (LS_VHO_AUTOCALIB_TTC == SW_ON)

        // copy calculated wic lengths to global structures
        g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm] =
            l_WICLengthMMF8_pui16[LeftRear_enm];
        g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm] =
            l_WICLengthMMF8_pui16[RightRear_enm];
        g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm] =
            l_WICLengthMMF8_pui16[LeftFront_enm];
        g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm] =
            l_WICLengthMMF8_pui16[RightFront_enm];
        g_VHOVehicleState_st.WICLengthMMF8_pui16[AvgLeftRight_enm] =
            l_WICLengthMMF8_pui16[AvgLeftRight_enm];
    }
    g_VHOAutocalibCtrl_st.StateChangedInCurrentCycle_bl = FALSE;
}

/*
  reactivate QAC warning messages 3204
  The variable 'l_AvgWicLengthRearAxle_ui16' is only set once -
  This depends on the compiler switches used in this function
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3204
#endif

#endif // #if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) ||
// (LS_VHO_AUTOCALIB_TTC == SW_ON) ||  (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
*****************************************************************************/
/**
 * @brief         used to validate the content of the autocalibration structure
 * @details
 *
 * @param
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
VISIBILITY Boolean m_VHOValidateAutocalibCtrl_bl(void)
{
    Boolean l_RetVal_bl = TRUE;

    const UInt16 lc_MinScalFac_ui16   = (UInt16)48640; // 0.95
    const UInt16 lc_MaxScalFac_ui16   = (UInt16)53760; // 1.05
    const SInt16 lc_MinMaxSWAOff_si16 = (SInt16)375;   // 15�

    if ((g_VHOAutocalibCtrl_st.SWOffDegRes_si16 > lc_MinMaxSWAOff_si16) ||
        (g_VHOAutocalibCtrl_st.SWOffDegRes_si16 < -lc_MinMaxSWAOff_si16))
    {
        g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 = 0;
        g_VHOAutocalibCtrl_st.SWAState_enm           = VHOCalibState_INITIAL_enm;
        g_VHOAutocalibCtrl_st.SWOffDegRes_si16       = 0;
        l_RetVal_bl                                  = FALSE;
    }

    if ((g_VHOAutocalibCtrl_st.ScalFacRR_ui16 < lc_MinScalFac_ui16) ||
        (g_VHOAutocalibCtrl_st.ScalFacRR_ui16 > lc_MaxScalFac_ui16))
    {
        g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 = 0;
        g_VHOAutocalibCtrl_st.TTCState_enm           = VHOCalibState_INITIAL_enm;
        g_VHOAutocalibCtrl_st.ScalFacLF_ui16         = (UInt16)51200;
        g_VHOAutocalibCtrl_st.ScalFacRF_ui16         = (UInt16)51200;
        g_VHOAutocalibCtrl_st.ScalFacRR_ui16         = (UInt16)51200;
        l_RetVal_bl                                  = FALSE;
    }

    if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm) ||
        (g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm))
    {
        if ((g_VHOAutocalibCtrl_st.ScalFacLF_ui16 < lc_MinScalFac_ui16) ||
            (g_VHOAutocalibCtrl_st.ScalFacLF_ui16 > lc_MaxScalFac_ui16) ||
            (g_VHOAutocalibCtrl_st.ScalFacRF_ui16 < lc_MinScalFac_ui16) ||
            (g_VHOAutocalibCtrl_st.ScalFacRF_ui16 > lc_MaxScalFac_ui16))
        {
            g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 = 0;
            g_VHOAutocalibCtrl_st.TTCState_enm           = VHOCalibState_INITIAL_enm;
            g_VHOAutocalibCtrl_st.ScalFacLF_ui16         = (UInt16)51200;
            g_VHOAutocalibCtrl_st.ScalFacRF_ui16         = (UInt16)51200;
            g_VHOAutocalibCtrl_st.ScalFacRR_ui16         = (UInt16)51200;
            l_RetVal_bl                                  = FALSE;
        }
    }

    if ((g_VHOAutocalibCtrl_st.YROff_si16 > md_VCALMinMaxYROffset_si16) ||
        (g_VHOAutocalibCtrl_st.YROff_si16 < -md_VCALMinMaxYROffset_si16))
    {
        // do not reset YROff, just change YRstate_enm to INITIAL
        // g_VHOAutocalibCtrl_st.YROff_si16 = (SInt16) 0;
        g_VHOAutocalibCtrl_st.YRState_enm = VHOCalibState_INITIAL_enm;
        l_RetVal_bl                       = FALSE;
    }

    return l_RetVal_bl;
}
#endif // #if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) ||
// (LS_VHO_AUTOCALIB_TTC == SW_ON) ||  (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         calculation of yaw rate offset
 * @details
 *
 * @param
 * @return        BOOL       valid zero point of yaw rate present
 *
 * @note
 * @see
 * @warning
 */
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
VISIBILITY Boolean m_VHOSignalPreProcYROffsetCalc_bl(void)
{
    UInt16 l_SWADifferenceAbsolute_ui16;
    SInt16 l_SWAStandstillRedThres_si16;
    Boolean l_RefreshYawRateOffset_bl;
    Boolean l_RetVal_bl;

    const UInt16 lc_StandstillCycleThres_ui16 =
        md_VCALYRStillstTimeThresMS_ui16 /
        (UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8;

    const UInt16 lc_TimeFrameMinCycles_ui16 =
        md_VCALYRMinTimeFrameMS_ui16 / (UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8;

    const UInt16 lc_TimeFrameMaxCycles_ui16 =
        md_VCALYRMaxTimeFrameMS_ui16 / (UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8;

    // default: do not update yaw rate offset
    l_RefreshYawRateOffset_bl = FALSE;

    // default: yaw rate offset calculation is not allowed
    g_VHOAutocalib_st.YawRateOffsetCalcNotAllowed_bl = TRUE;

    if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 == 0)
    {
        // vehicle is not standing still
        if (g_VHOAutocalib_st.YawRateCountedValues_ui16 > lc_TimeFrameMinCycles_ui16)
        {
            // vehicle was standing still one call cycle before and
            // time frame for offset calculation long enough
            l_RefreshYawRateOffset_bl = TRUE;
        }
        // reset SWAStartStandstill if vehicle does not stand still
        g_VHOAutocalib_st.SWAStartStandstill_si16 = 0;
    }
    else if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 < lc_StandstillCycleThres_ui16)
    {
        // because YROff calculation deppends on steering changes while
        // vehicle stands still store SWA at the beginning of stand still
        if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 < (UInt16)(3))
        {
            g_VHOAutocalib_st.SWAStartStandstill_si16 = g_VHOCanSig_st.SWA_si16;

            // set yaw angle calculated by integration of yaw rate to yaw rate
            // calculated by odometry
            g_VHOVehicleState_st.YawAngleYRSens_ui32 = g_VHOVehicleState_st.YawAngle_ui32;
            g_VHOVehicleState_st.YawAngleYRSensK1_ui32 =
                g_VHOVehicleState_st.YawAngle_ui32;
        }

        // vehicle is standing still, but not long enough yet
        g_VHOAutocalib_st.YawRateSum_si32           = 0;
        g_VHOAutocalib_st.YawRateCountedValues_ui16 = 0;
    }
    else
    {
        // calculate absolute value of difference between SWA and SWAStartStandstill
        l_SWADifferenceAbsolute_ui16 = (UInt16)(g_mtl_Abs_mac(
            g_VHOAutocalib_st.SWAStartStandstill_si16 - g_VHOCanSig_st.SWA_si16));

        // calculate reduction threshold depending of the absolute value of the
        // difference itself if |SWA - SWAStartStandstill| > 500� reduce with +-20�
        // every cyle
        if (l_SWADifferenceAbsolute_ui16 > md_VCALVeryHighSWADifferenceAbs_ui16)
        {
            l_SWAStandstillRedThres_si16 = md_VCALVeryHighSWAStandstillRedThres_si16;
        }
        // if |SWA - SWAStartStandstill| > 250� reduce with +- 10� every cyle
        else if (l_SWADifferenceAbsolute_ui16 > md_VCALHighSWADifferenceAbs_ui16)
        {
            l_SWAStandstillRedThres_si16 = md_VCALHighSWAStandstillRedThres_si16;
        }
        // if |SWA - SWAStartStandstill| > 125� reduce with +- 5� every cyle
        else if (l_SWADifferenceAbsolute_ui16 > md_VCALMidSWADifferenceAbs_ui16)
        {
            l_SWAStandstillRedThres_si16 = md_VCALMidSWAStandstillRedThres_si16;
        }
        // if |SWA - SWAStartStandstill| > md_VCALMaxSWAChangesStandingStill_ui16
        // reduce with +- 3� every cyle
        else if (l_SWADifferenceAbsolute_ui16 > md_VCALMaxSWAChangesStandingStill_ui16)
        {
            l_SWAStandstillRedThres_si16 = md_VCALLowSWAStandstillRedThres_si16;
        }
        // if |SWA - SWAStartStandstill| <= md_VCALMaxSWAChangesStandingStill_ui16
        // reduce with +-0.2� every cycle
        else
        {
            l_SWAStandstillRedThres_si16 = md_VCALVeryLowSWAStandstillRedThres_si16;
        }

        // steeering wheel angle changes should not be considered for a longer time
        // than 1 second
        if (g_VHOCanSig_st.SWA_si16 >
            (g_VHOAutocalib_st.SWAStartStandstill_si16 + l_SWAStandstillRedThres_si16))
        {
            g_VHOAutocalib_st.SWAStartStandstill_si16 += l_SWAStandstillRedThres_si16;
        }
        else if (g_VHOCanSig_st.SWA_si16 < (g_VHOAutocalib_st.SWAStartStandstill_si16 -
                                            l_SWAStandstillRedThres_si16))
        {
            g_VHOAutocalib_st.SWAStartStandstill_si16 -= l_SWAStandstillRedThres_si16;
        }
        else
        {
            // do nothing
        }

        // vehicle has been standing still long enough to evaluate yaw rate zero
        // point
        if ((UInt16)g_mtl_Abs_mac(g_VHOCanSig_st.YawRate_si16) <
            (UInt16)(md_VCALMaxYRWhileStandingStill_si16))
        {
            // calculate actualized absolute value of difference between SWA and
            // SWAStartStandstill
            l_SWADifferenceAbsolute_ui16 = (UInt16)(g_mtl_Abs_mac(
                g_VHOAutocalib_st.SWAStartStandstill_si16 - g_VHOCanSig_st.SWA_si16));

            // yaw rate offset calculation depends on SWA changes
            if (l_SWADifferenceAbsolute_ui16 < md_VCALMaxSWAChangesStandingStill_ui16)
            {
                // yaw rate offset calculation is allowed
                g_VHOAutocalib_st.YawRateOffsetCalcNotAllowed_bl = FALSE;

                // accumulate yaw rate
                // max yaw rate in stillstand with worst case absolute zero point error
                // is +-3�/s (MM3 PH-Spec) => +-6�/s assumed here => overflow occurs
                // after 2^31/600 cycles
                g_VHOAutocalib_st.YawRateSum_si32 += (SInt32)g_VHOCanSig_st.YawRate_si16;

                g_VHOAutocalib_st.YawRateCountedValues_ui16++;

                if (g_VHOAutocalib_st.YawRateCountedValues_ui16 >=
                    lc_TimeFrameMaxCycles_ui16)
                {
                    l_RefreshYawRateOffset_bl = TRUE;
                }
            }
            else
            {
                // yaw rate offset calculation is not allowed
                g_VHOAutocalib_st.YawRateOffsetCalcNotAllowed_bl = TRUE;
            }
        }
        else
        {
            // yaw rate seems to be invalid => reset structures for zero point
            // calculation and call error handler
            g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
            g_VHOAutocalib_st.YawRateSum_si32           = 0;
            g_VHOAutocalib_st.YawRateCountedValues_ui16 = 0;
            l_RefreshYawRateOffset_bl                   = FALSE;
        }
    }

    if (l_RefreshYawRateOffset_bl != FALSE)
    {
        SInt16 l_YawRateZeroPoint_si16;

        // calculate new zero point for yaw rate
        l_YawRateZeroPoint_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            g_VHOAutocalib_st.YawRateSum_si32,
            (SInt32)g_VHOAutocalib_st.YawRateCountedValues_ui16);

        if (g_VHOAutocalibCtrl_st.YRState_enm == VHOCalibState_INITIAL_enm)
        {
            // if absolute amount of yaw rate offset is within maximum reliable range
            // the use of yaw rate for detection of vehicle moving direction is
            // allowed.
            if ((g_VHOAutocalibCtrl_st.YROff_si16 < md_VCALMinMaxYROffset_si16) &&
                (g_VHOAutocalibCtrl_st.YROff_si16 > -md_VCALMinMaxYROffset_si16))
            {
                if (g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 >
                    md_VCALYRInitialBreakinPhaseMS_ui32)
                // initial break-in phase of yaw rate sensor completed => start yaw rate
                // offset calculation in STABLE mode
                {
                    // yaw rate offset has not been initialized yet correctly =>
                    // take calculated value as initial value
                    g_VHOAutocalibCtrl_st.YROff_si16  = l_YawRateZeroPoint_si16;
                    g_VHOAutocalibCtrl_st.YRState_enm = VHOCalibState_STABLE_enm;

// submodules can be started => reset data structures
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
                    m_VHOResetSWAOffsetCalib_vd();
#endif
#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)
                    m_VHOResetTyreRadiiCalib_vd();
#endif
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
                    m_VHOResetTTC_vd();
#endif
                }
                else // i.e. (g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 <=
                     // md_VCALYRInitialBreakinPhaseMS_ui32) initial break-in phase of
                     // yaw rate sensor not completed => start yaw rate offset
                     // calculation in UNSTABLE mode
                {
                    // yaw rate offset has not been initialized yet correctly =>
                    // take calculated value as initial value
                    g_VHOAutocalibCtrl_st.YROff_si16  = l_YawRateZeroPoint_si16;
                    g_VHOAutocalibCtrl_st.YRState_enm = VHOCalibState_UNSTABLE_enm;
                }
            }
            else
            {
                // if absolute amount of yaw rate offset is outside maximum reliable
                // range the use of yaw rate is for detection of vehicle moving
                // direction is not allowed. YRState_enm remains in
                // VHOCalibState_INITIAL_enm
                g_VHOAutocalibCtrl_st.YROff_si16 = l_YawRateZeroPoint_si16;
            }
        }
        else if ((g_VHOAutocalibCtrl_st.YRState_enm == VHOCalibState_UNSTABLE_enm) &&
                 (g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 >
                  md_VCALYRInitialBreakinPhaseMS_ui32))
        // initial break-in phase of yaw rate sensor completed => start yaw rate
        // offset calculation in STABLE mode
        {
            // yaw rate offset has not been initialized yet correctly =>
            // take calculated value as initial value
            g_VHOAutocalibCtrl_st.YROff_si16  = l_YawRateZeroPoint_si16;
            g_VHOAutocalibCtrl_st.YRState_enm = VHOCalibState_STABLE_enm;

// submodules can be started => reset data structures
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
            m_VHOResetSWAOffsetCalib_vd();
#endif
#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)
            m_VHOResetTyreRadiiCalib_vd();
#endif
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
            m_VHOResetTTC_vd();
#endif
        }
        else // i.e. (g_VHOAutocalibCtrl_st.YRState_enm ==
             // VHOCalibState_STABLE_enm)
             // || ((g_VHOAutocalibCtrl_st.YRState_enm ==
             // VHOCalibState_UNSTABLE_enm)
             // && (g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 <=
             // md_VCALYRInitialBreakinPhaseMS_ui32))
        {
            UInt8 l_Multiplier_ui8;

            if (g_VHOAutocalib_st.YawRateCountedValues_ui16 != 0)
            {
                // calculate new yaw rate zero point as a weighted average of new
                // and old zero point (for a time frame that is >= .5 *
                // lc_TimeFrameMaxCycles_ui16 the chosen setting will not be influenced,
                // for a time frame < .5 * lc_TimeFrameMaxCycles_ui16 the new value will
                // be trusted less by reducing the chosen weight)
                l_Multiplier_ui8 = (UInt8)(lc_TimeFrameMaxCycles_ui16 /
                                           g_VHOAutocalib_st.YawRateCountedValues_ui16);
            }
            else
            {
                l_Multiplier_ui8 = (UInt8)1;
            }

            g_VHOAutocalibCtrl_st.YROff_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
                (SInt32)((md_VCALYRWeightOldValue_si16 *
                          g_VHOAutocalibCtrl_st.YROff_si16 * (SInt16)l_Multiplier_ui8) +
                         (md_VCALYRWeightNewValue_si16 * l_YawRateZeroPoint_si16)),
                ((SInt16)l_Multiplier_ui8 * md_VCALYRWeightOldValue_si16) +
                    md_VCALYRWeightNewValue_si16);
        }
        // reset accumulated yaw rate and counter
        g_VHOAutocalib_st.YawRateSum_si32           = 0;
        g_VHOAutocalib_st.YawRateCountedValues_ui16 = 0;
    }

    if ((g_VHOAutocalibCtrl_st.YRState_enm == VHOCalibState_INITIAL_enm) ||
        (g_VHOAutocalibCtrl_st.YRState_enm == VHOCalibState_UNSTABLE_enm))
    {
        l_RetVal_bl = FALSE;
    }
    else
    {
        l_RetVal_bl = TRUE;
    }

    return l_RetVal_bl;
}
#endif // #if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC ==
       //  SW_ON)
// ||  (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         calculation of steering wheel angle offset
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
VISIBILITY void m_VHOSignalPreProcSWAOffsetCalc_vd(void)
{
    const Boolean lc_SWAOffsetCalibConstraintsFulfilled_bl = (Boolean)(
/*
  suppress nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 2217
#endif

        ((UInt32)g_mtl_Abs_mac(g_VHOCorrectedSignals_st.Corrected_YawRate_si32) <
         (UInt32)md_VCALSWAYawRateHighThres_si32) &&
        (g_VHOCorrectedSignals_st.Corrected_YawRate_si32 != 0) &&
        (g_VHOAutocalib_st.VeloRearAxleCenter_ui16 < md_VCALSWAVeloHighThres_ui16) &&
        (g_VHOAutocalib_st.VeloRearAxleCenter_ui16 > md_VCALSWAVeloLowThres_ui16) &&
        (g_VHOCanSig_st.SWA_si16 >
         -((SInt16)g_VHOAutocalib_st.SWA_st.MaxSWARight_ui16)) &&
        (g_VHOCanSig_st.SWA_si16 < (SInt32)g_VHOAutocalib_st.SWA_st.MaxSWALeft_ui16) &&
        ((g_VHOCanSig_st.SWA_si16 <
          -((SInt32)g_VHOAutocalib_st.SWA_st.MinSWARight_ui16)) ||
         (g_VHOCanSig_st.SWA_si16 > (SInt32)g_VHOAutocalib_st.SWA_st.MinSWALeft_ui16)) &&
        (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_FORW_enm)

/*
  reactivate nonsense QAC warning 2217: Line length exceeds 130 characters.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 2217
#endif
    );

    if (lc_SWAOffsetCalibConstraintsFulfilled_bl != FALSE)
    // calculate steering angle estimation
    {
        SInt16 l_SWADiff_si16;
        // calculate difference
        /* Attention: The calculation of VeloRearAxleCenter considers steering */
        /*            angle rear. So the virtual wheel base has to be used.    */
        l_SWADiff_si16 = g_VHOCanSig_st.SWA_si16 -
                         m_VHOSignalPreProcEstimateSWA_si16(
                             (SInt16)g_VHOCorrectedSignals_st.Corrected_YawRate_si32,
                             g_VHOAutocalib_st.VeloRearAxleCenter_ui16,
                             g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16);

        // difference not too big => accumulate difference
        if ((UInt16)g_mtl_Abs_mac(l_SWADiff_si16) <=
            (UInt16)md_VCALSWAMaxDiffMeasEst_si16)
        {
            UInt16 l_RelErr_ui16;

            // map current steering angle to relative error
            l_RelErr_ui16 = m_VHOGetRelError_ui16(g_VHOVehicleState_st.SAngRadF12_si16)
                            << 2; // F12

            if (l_RelErr_ui16 != (UInt16)0xFFFF)
            {
                // return value is valid
                if (g_VHOCanSig_st.SWA_si16 < 0)
                {
                    // adding relative error for right side
                    g_VHOAutocalib_st.SWA_st.RelErrSumRight_ui32 += l_RelErr_ui16;

                    // adding differences to structure for right side
                    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstSumRight_si32 +=
                        (SInt32)l_SWADiff_si16;
                    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16++;
                }
                else
                {
                    // adding relative error for left side
                    g_VHOAutocalib_st.SWA_st.RelErrSumLeft_ui32 += l_RelErr_ui16;

                    // adding differences to structure for left side
                    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstSumLeft_si32 +=
                        (SInt32)l_SWADiff_si16;
                    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16++;
                }
            }

            // calculate deviation between avg rel errors
            if ((g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16 != 0) &&
                (g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16 != 0))
            {
                // neither g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16 nor
                // g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16 is zero
                g_VHOAutocalib_st.SWA_st.AvgRelErr_Left_ui16 =
                    (UInt16)(g_VHOAutocalib_st.SWA_st.RelErrSumLeft_ui32 /
                             (UInt32)g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16);

                g_VHOAutocalib_st.SWA_st.AvgRelErr_Right_ui16 =
                    (UInt16)(g_VHOAutocalib_st.SWA_st.RelErrSumRight_ui32 /
                             (UInt32)
                                 g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16);

                g_VHOAutocalib_st.SWA_st.DiffRelErrLeftRightPercent_si8 =
                    (SInt8)((SInt16)100 -
                            (SInt16)g_mtl_s32_Div_s32_si32(
                                (SInt32)100 *
                                    (SInt32)g_VHOAutocalib_st.SWA_st.AvgRelErr_Left_ui16,
                                (SInt32)g_VHOAutocalib_st.SWA_st.AvgRelErr_Right_ui16));

                g_VHOAutocalib_st.SWA_st.SWAOffsetTemp_si16 = g_mtl_ReducePrecision_si16(
                    (SInt16)(g_mtl_s32_Div_s32_si32(
                                 g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstSumLeft_si32,
                                 (SInt32)g_VHOAutocalib_st.SWA_st
                                     .DiffSWAMeasEstCntLeft_ui16) +
                             g_mtl_s32_Div_s32_si32(
                                 g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstSumRight_si32,
                                 (SInt32)g_VHOAutocalib_st.SWA_st
                                     .DiffSWAMeasEstCntRight_ui16)),
                    (UInt8)1);
            }
        }

        // measurement data will not be evaluated until a specified time has been
        // driven
        if ((UInt32)(g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 -
                     g_VHOAutocalib_st.SWA_st.SWAOffCalibStartTimeMS_ui32) >=
            md_VCALSWAOffsMinTimeUntilEvalMS_ui32)
        {
            if ((g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16 >
                 md_VCALSWAOffsMinNumMeas_ui16) &&
                (g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16 <
                 md_VCALSWAOffsMinNumMeas_ui16))
            {
                // enough values have been counted for the left side but values for the
                // right side are missing => do not accept measurements for left side
                // any more
                g_VHOAutocalib_st.SWA_st.MaxSWALeft_ui16 = (UInt16)0;
            }
            else if ((g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16 <
                      md_VCALSWAOffsMinNumMeas_ui16) &&
                     (g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16 >
                      md_VCALSWAOffsMinNumMeas_ui16))
            {
                // enough values have been counted for the right side but values for the
                // left side are missing => do not accept measurements for right side
                // any more
                g_VHOAutocalib_st.SWA_st.MaxSWARight_ui16 = (UInt16)0;
            }
            else if ((g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16 >
                      md_VCALSWAOffsMinNumMeas_ui16) &&
                     (g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16 >
                      md_VCALSWAOffsMinNumMeas_ui16))
            {
                // enough values for both sides have been gathered => evaluate whether
                // symmetry is given
                if ((g_VHOAutocalib_st.SWA_st.DiffRelErrLeftRightPercent_si8 >
                     -((SInt8)md_VCALSWAOffsMaxDiffPercent_ui8)) &&
                    (g_VHOAutocalib_st.SWA_st.DiffRelErrLeftRightPercent_si8 <
                     (SInt8)md_VCALSWAOffsMaxDiffPercent_ui8))
                {
                    UInt16 l_X_ui16;

                    l_X_ui16 = (UInt16)g_mtl_Abs_mac(
                        g_VHOAutocalibCtrl_st.SWOffDegRes_si16 -
                        g_VHOAutocalib_st.SWA_st.SWAOffsetTemp_si16);

                    // difference between the relative errors is small enough =>
                    // final evaluation
                    if ((g_VHOAutocalibCtrl_st.SWAState_enm ==
                         VHOCalibState_INITIAL_enm) ||
                        (l_X_ui16 > md_VCALSWAOffsMaxJitter_ui16))
                    {
                        // steering wheel angle offset has not been calibrated yet or
                        // difference between last and new value too big =>
                        // set state to UNSTABLE
                        g_VHOAutocalibCtrl_st.SWOffDegRes_si16 =
                            g_VHOAutocalib_st.SWA_st.SWAOffsetTemp_si16;
                        g_VHOAutocalibCtrl_st.SWAState_enm = VHOCalibState_UNSTABLE_enm;
                        g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 = (UInt8)1;
                        g_VHOAutocalib_st.l_EEPROMWriteREQ_bl        = TRUE;
                    }
                    else
                    {
                        // increment number of stable cycles
                        g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 =
                            (g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 < (UInt8)0xFF)
                                ? (g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 +
                                   (UInt8)1)
                                : ((UInt8)0xFF);

                        if (g_VHOAutocalibCtrl_st.SWAState_enm ==
                            VHOCalibState_UNSTABLE_enm)
                        {
                            g_VHOAutocalib_st.l_EEPROMWriteREQ_bl = TRUE;
                        }

                        g_VHOAutocalibCtrl_st.SWAState_enm =
                            (g_VHOAutocalibCtrl_st.NumStableSWACycles_ui8 >=
                             gd_VCALSWAMinCyclesUntilStable_ui8)
                                ? VHOCalibState_STABLE_enm
                                : VHOCalibState_UNSTABLE_enm;

                        if (!((g_VHOAutocalibCtrl_st.SWOffDegRes_si16 == 0) &&
                              (g_VHOAutocalib_st.SWA_st.SWAOffsetTemp_si16 == 0)))
                        {
                            // set new offset as weighted average of old and currently
                            // estimated offset
                            g_VHOAutocalibCtrl_st
                                .SWOffDegRes_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
                                (SInt32)((md_VCALSWAOffsWeightOldValue_si16 *
                                          g_VHOAutocalibCtrl_st.SWOffDegRes_si16) +
                                         (md_VCALSWAOffsWeightNewValue_si16 *
                                          g_VHOAutocalib_st.SWA_st.SWAOffsetTemp_si16)),
                                (SInt32)(md_VCALSWAOffsWeightOldValue_si16 +
                                         md_VCALSWAOffsWeightNewValue_si16));
                        } // else: offset will remain zero
                        if (g_mtl_Abs_mac(g_VHOAutocalibCtrl_st.SWOffDegRes_si16 -
                                          g_VHOAutocalib_st.AutocalibStateEEPROM_st
                                              .SWOffDegRes_si16) >
                            md_VCALSWAOffsMinDevEEPROMaccess_ui16)
                        {
                            g_VHOAutocalib_st.l_EEPROMWriteREQ_bl = TRUE;
                        }
                    }
                    m_VHOResetSWAOffsetCalib_vd();
                }
                else // symmetry criterion is NOT fulfilled
                {
                    // limit min/max steering angles to equal rel. errors
                    if (g_VHOAutocalib_st.SWA_st.DiffRelErrLeftRightPercent_si8 < 0)
                    {
                        // avg rel error on left side much bigger than on right side
                        m_VHOSignalPreProcSetSWAThresh(
                            &g_VHOAutocalib_st.SWA_st.MaxSWALeft_ui16,
                            &g_VHOAutocalib_st.SWA_st.MinSWARight_ui16,
                            &g_VHOAutocalib_st.SWA_st.MaxSWARight_ui16,
                            &g_VHOAutocalib_st.SWA_st.MinSWALeft_ui16,
                            &g_VHOAutocalib_st.SWA_st.AvgRelErr_Right_ui16,
                            &g_VHOAutocalib_st.SWA_st.AvgRelErr_Left_ui16);
                    }
                    else // g_VHOAutocalib_st.SWA_st.DiffRelErrLeftRightPercent_si8 > 0
                    {
                        // avg rel. error on right side much bigger than on left side
                        m_VHOSignalPreProcSetSWAThresh(
                            &g_VHOAutocalib_st.SWA_st.MaxSWARight_ui16,
                            &g_VHOAutocalib_st.SWA_st.MinSWALeft_ui16,
                            &g_VHOAutocalib_st.SWA_st.MaxSWALeft_ui16,
                            &g_VHOAutocalib_st.SWA_st.MinSWARight_ui16,
                            &g_VHOAutocalib_st.SWA_st.AvgRelErr_Right_ui16,
                            &g_VHOAutocalib_st.SWA_st.AvgRelErr_Left_ui16);
                    }
                }
            }
            else
            {
                // do not get QAC errors
            }
        }
    }
}
#endif // #if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function to set thresholds for the steering wheel angle
 *                used to control the average relative error
 * @details
 *
 * @param         f_MinSWAToBeIncreased_pui16
 * @param         f_MaxSWAFullRange_pui16
 * @param         f_MinSWAFullRange_pui16
 * @param         fc_AvgErrToBeDecreased_pui16
 * @param         fc_AvgErrToBeIncreased_pui16
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
VISIBILITY void m_VHOSignalPreProcSetSWAThresh(UInt16 *f_MaxSWAToBeDecreased_pui16,
                                               UInt16 *f_MinSWAToBeIncreased_pui16,
                                               UInt16 *f_MaxSWAFullRange_pui16,
                                               UInt16 *f_MinSWAFullRange_pui16,
                                               const UInt16 *fc_AvgErrToBeDecreased_pui16,
                                               const UInt16 *fc_AvgErrToBeIncreased_pui16)
{
    SInt16 l_temp_si16;

    // SET THE ANGLES THAT DO NOT HAVE TO BE LIMITED
    (*f_MaxSWAFullRange_pui16) = md_VCALSWA_SWAHighThres_ui16;
    (*f_MinSWAFullRange_pui16) = md_VCALSWA_SWALowThres_ui16;

    // SET THE ANGLE FOR THE SIDE WHOSE REL ERROR SHALL BE DECREASED
    l_temp_si16 = g_VHOConvertSA2SWA_si16(
        (SInt16)((*fc_AvgErrToBeDecreased_pui16) / (UInt16)54), TRUE);
    (*f_MaxSWAToBeDecreased_pui16) = (UInt16)g_mtl_Abs_mac(l_temp_si16);

    (*f_MaxSWAToBeDecreased_pui16) -=
        ((*f_MaxSWAToBeDecreased_pui16) >
         (UInt16)(md_VCALSWAAddtlSWA_ui16 + md_VCALSWA_SWALowThres_ui16))
            ? md_VCALSWAAddtlSWA_ui16
            : (UInt16)0;

    // SET THE ANGLE FOR THE SIDE WHOSE REL ERROR SHALL BE INCREASED
    l_temp_si16 = g_VHOConvertSA2SWA_si16(
        (SInt16)((*fc_AvgErrToBeIncreased_pui16) / (UInt16)54), TRUE);
    (*f_MinSWAToBeIncreased_pui16) =
        (UInt16)g_mtl_Abs_mac(l_temp_si16) + md_VCALSWAAddtlSWA_ui16;
}
#endif // #if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         reset data structures used for swa offset calibration
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
VISIBILITY void m_VHOResetSWAOffsetCalib_vd(void)
{
    // reset counters
    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntLeft_ui16  = (UInt16)0;
    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstCntRight_ui16 = (UInt16)0;

    // reset sum
    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstSumLeft_si32  = (SInt32)0;
    g_VHOAutocalib_st.SWA_st.DiffSWAMeasEstSumRight_si32 = (SInt32)0;

    // reset relative errors
    g_VHOAutocalib_st.SWA_st.RelErrSumLeft_ui32             = (UInt32)0;
    g_VHOAutocalib_st.SWA_st.RelErrSumRight_ui32            = (UInt32)0;
    g_VHOAutocalib_st.SWA_st.AvgRelErr_Left_ui16            = (UInt16)0;
    g_VHOAutocalib_st.SWA_st.AvgRelErr_Right_ui16           = (UInt16)0;
    g_VHOAutocalib_st.SWA_st.DiffRelErrLeftRightPercent_si8 = (SInt8)0;

    // set start time to current runtime
    g_VHOAutocalib_st.SWA_st.SWAOffCalibStartTimeMS_ui32 =
        g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32;

    // reset max steering wheel angles
    g_VHOAutocalib_st.SWA_st.MaxSWALeft_ui16  = md_VCALSWA_SWAHighThres_ui16;
    g_VHOAutocalib_st.SWA_st.MaxSWARight_ui16 = md_VCALSWA_SWAHighThres_ui16;
    g_VHOAutocalib_st.SWA_st.MinSWALeft_ui16  = md_VCALSWA_SWALowThres_ui16;
    g_VHOAutocalib_st.SWA_st.MinSWARight_ui16 = md_VCALSWA_SWALowThres_ui16;

    // reset offset
    g_VHOAutocalib_st.SWA_st.SWAOffsetTemp_si16 = (SInt16)0;
}
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         reset data structures used for swa offset calibration
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)
VISIBILITY void m_VHOResetTyreRadiiCalib_vd(void)
{
    // init variables needed for tyre radii calibration
    g_VHOAutocalib_st.Tyre_st.TyreRadiiCalibState_ui8        = 0;
    g_VHOAutocalib_st.Tyre_st.TyreRadiiCalibStartTimeMS_ui32 = (UInt32)0;

    g_VHOAutocalib_st.Tyre_st.VeloScalFacRight_pui32[0] = (UInt32)0;
    g_VHOAutocalib_st.Tyre_st.VeloScalFacRight_pui32[1] = (UInt32)0;

    g_VHOAutocalib_st.Tyre_st.VeloScalFacLeft_pui32[0] = (UInt32)0;
    g_VHOAutocalib_st.Tyre_st.VeloScalFacLeft_pui32[1] = (UInt32)0;

    g_VHOAutocalib_st.Tyre_st.VeloScalFacRightCnt_pui16[0] = (UInt16)0;
    g_VHOAutocalib_st.Tyre_st.VeloScalFacRightCnt_pui16[1] = (UInt16)0;

    g_VHOAutocalib_st.Tyre_st.VeloScalFacLeftCnt_pui16[0] = (UInt16)0;
    g_VHOAutocalib_st.Tyre_st.VeloScalFacLeftCnt_pui16[1] = (UInt16)0;

    g_VHOAutocalib_st.Tyre_st.VeloScalFacTotalCntRight_ui16 = (UInt16)0;
    g_VHOAutocalib_st.Tyre_st.VeloScalFacTotalCntLeft_ui16  = (UInt16)0;

    g_VHOAutocalib_st.Tyre_st.SlotsOpenRight_ui8 = 0x3; // all slots open 00011111
    g_VHOAutocalib_st.Tyre_st.SlotsOpenLeft_ui8  = 0x3; // all slots open 00011111

    g_VHOAutocalib_st.Tyre_st.ImplausibleEstimationCnt_ui16 = (UInt16)0;
}
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         estimate steering wheel angle from vehicle velocity and
 *                yaw rate
 * @details
 *
 * @param         f_YawRateDegPerSec_si16         yaw rate, unit: 0.01�/s
 * @param         f_VeloRearAxleCenterKmh_ui16    velocity at rear axle center:
 * 0.01kmh
 * @param         f_WheelBaseMM_ui16              wheel base, unit: MM
 * @return        SInt16
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
VISIBILITY SInt16 m_VHOSignalPreProcEstimateSWA_si16(SInt16 f_YawRateDegPerSec_si16,
                                                     UInt16 f_VeloRearAxleCenterKmh_ui16,
                                                     UInt16 f_WheelBaseMM_ui16)
{
    SInt16 l_X_si16;

    // calculate (L*PSI/v)*256
    // (*256 due to input resolution of g_VHOConvertSA2SWA_si16)
    //
    // normation of input values:
    // f_WheelBaseMM_ui16           [mm       -> m]     =>  * 0.001
    // f_VeloRearAxleCenterKmh_ui16 [0.01km/h -> m/s]   =>  * (0.01/3.6)
    // f_YawRateDegPerSec_si16      [0.01�/s  -> rad/s] =>  * (0.01*pi/180)
    //
    //                                    0.01*(pi/180) * f_YawRateDegPerSec_si16
    //  256 * 0.001 f_WheelBaseMM_ui16 * -----------------------------------------
    //                                    (0.01/3.6) *
    //                                    f_VeloRearAxleCenterKmh_ui16
    //
    //                          0.9216*pi*f_YawRateDegPerSec_si16
    //                          f_WheelBaseMM_ui16*f_YawRateDegPerSec_si16
    //  = f_WheelBaseMM_ui16 *  -------------------------------- =
    //  ------------------------------------------
    //                          180*f_VeloRearAxleCenterKmh_ui16   62.17 *
    //                          f_VeloRearAxleCenterKmh_ui16

    l_X_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
        (SInt32)f_WheelBaseMM_ui16 * (SInt32)f_YawRateDegPerSec_si16 * (SInt32)256,
        (SInt32)f_VeloRearAxleCenterKmh_ui16 * (SInt32)15915);

    // get estimated steering angle
    l_X_si16 = g_mtl_ArcTan_si16(l_X_si16, (UInt8)8); // F12 rad

    if ((UInt16)g_mtl_Abs_mac(l_X_si16) >= (UInt16)4289)
    {
        // estimated steering angle bigger than 4096*60*pi/180
        // == 4289.32 => error handling
        l_X_si16 = (SInt16)0;
    }
    else
    {
        // map to steering wheel angle
        // input: F12 Rad - resolution of result: 0.04�
        l_X_si16 = g_VHOConvertSA2SWA_si16(l_X_si16, TRUE);
    }
    return l_X_si16;
}
#endif // #if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         get relative error for a certain steering angle
 * @details
 *
 * @param         f_SteeringAngleRadF12_si16          steering angle of virtual
 * front wheel, unit: radF12
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
VISIBILITY UInt16 m_VHOGetRelError_ui16(SInt16 f_SteeringAngleRadF12_si16)
{
    // in the steering wheel angle offset correction the steering angle is mapped
    // to a relative error the average relative error for the right and the left
    // side is used as a metric for symmetry unit of values provided: F10,
    // stepsize: 1/128rad, input range is 0..42/128rad
    static const UInt16 lc_VCAL_RelErrTable_pui16[43] = {
        0,     458,   917,   1375,  1833,  2290,  2746,  3202,  3657,  4112,  4565,
        5016,  5467,  5916,  6364,  6809,  7254,  7696,  8136,  8574,  9010,  9443,
        9874,  10302, 10728, 11151, 11571, 11988, 12402, 12812, 13220, 13623, 14024,
        14420, 14813, 15202, 15587, 15968, 16345, 16718, 17086, 17450, 17809};

    UInt8 l_BaseIndex_ui8;
    UInt16 l_A_ui16;
    UInt16 l_X_ui16;

    // get absolute value of steering angle
    l_A_ui16 = (UInt16)g_mtl_Abs_mac(f_SteeringAngleRadF12_si16);

    // input range is 0..42/128 rad = 0..1344/4096rad
    if (l_A_ui16 >= (UInt16)1344)
    {
        g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
        l_X_ui16 = (UInt16)0xFFFF;
    }
    else
    {
        l_BaseIndex_ui8 = (UInt8)(l_A_ui16 >> 5);

        // get difference between the values for base index and subsequent index
        l_X_ui16 = lc_VCAL_RelErrTable_pui16[l_BaseIndex_ui8 + 1] -
                   lc_VCAL_RelErrTable_pui16[l_BaseIndex_ui8];

        l_A_ui16 =
            g_mtl_ReducePrecision_ui16(l_X_ui16 * (l_A_ui16 & (UInt16)0x1F), (UInt8)5);

        // calculating delta
        l_X_ui16 = lc_VCAL_RelErrTable_pui16[l_BaseIndex_ui8] + l_A_ui16;
    }
    return l_X_ui16;
}
#endif // #if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         WIC error correction
 * @details
 *
 * @param         f_ForceExtrapolation_bl
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOSignalPreprocWIC_vd(Boolean f_ForceExtrapolation_bl)
{
    UInt16 l_TimeMS_ui16;
    UInt16 l_AccFraction_ui16;
    UInt16 l_VelFraction_ui16;
    UInt16 l_MinWICDiff_ui16;
    UInt16 l_MaxWICDiff_ui16;
    Boolean l_WICCorrected_bl;
    UInt16 l_Tolerance_ui16;
    UInt16 l_MulFac_ui16;
    UInt16 lc_A_UI16;
    static UInt8 l_CntNoError_ui8;

    const UInt16 lc_TimeOneMsgLostMS_UI16 = (UInt16) // 1.5* wic cycletime
        ((g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 * (UInt16)3) / (UInt16)2);

    const UInt16 lc_TimeTwoMsgLostMS_UI16 = // 2.5* wic cycletime
        ((g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 * (UInt16)5) / (UInt16)2);

    const UInt16 lc_TimeThreeMsgLostMS_UI16 = // 3.5* wic cycletime
        ((g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 * (UInt16)7) / (UInt16)2);

/*
  suppress QAC warning message 584: division by zero is not possible here
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 584
#endif

    // calculate constant (max veh. acc. * cycletime) / (2*wic length)
    lc_A_UI16 = (UInt16) // F4
        ((md_VCALWicMaxAcc_ui32 * g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 *
          (UInt32)4096) /
         ((UInt32)((UInt16)2 * g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16)));

    l_WICCorrected_bl = FALSE;

    // detect possible loss of WIC messages
    if (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 < lc_TimeOneMsgLostMS_UI16)
    {
        // standard case, no loss assumed: time difference between reception
        // of last two wic messages in [0,1.5* wic cycletime[
        l_TimeMS_ui16      = ((UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8);
        l_AccFraction_ui16 = lc_A_UI16 * l_TimeMS_ui16;
        l_MulFac_ui16      = (UInt16)1;
    }
    else if (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 < lc_TimeTwoMsgLostMS_UI16)
    {
        // loss of one wic message assumed: time difference between receiption
        // of last two wic messages in [1.5* wic cycletime,2.5*wic cycletime[
        l_TimeMS_ui16 = ((UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 * (UInt16)2);
        l_AccFraction_ui16 = lc_A_UI16 * l_TimeMS_ui16;
        l_MulFac_ui16      = (UInt16)2;
    }
    else if (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 < lc_TimeThreeMsgLostMS_UI16)
    {
        // loss of two wic messages assumed: time difference between receiption
        // of last two wic messages in [2.5* wic cycletime,3.5*wic cycletime[
        l_TimeMS_ui16 = ((UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8 * (UInt16)3);
        l_AccFraction_ui16 = lc_A_UI16 * l_TimeMS_ui16;
        l_MulFac_ui16      = (UInt16)3;
    }
    else
    {
        // loss of more than three messages assumed, calculate with time
        // difference between receiption of last two wic messages
        l_TimeMS_ui16      = g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16;
        l_AccFraction_ui16 = (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 /
                              (UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8);
        l_AccFraction_ui16 *= lc_A_UI16;
        l_AccFraction_ui16 *= (UInt16)g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8;
        l_MulFac_ui16 = (UInt16)5;
    }

    l_AccFraction_ui16 = l_AccFraction_ui16 >> 4; // F0
    l_AccFraction_ui16 =
        (UInt16)g_mtl_s32_Div_s32_si32((SInt32)l_AccFraction_ui16, (SInt32)1000); //[s]
    l_AccFraction_ui16 = ((l_AccFraction_ui16 == 0) ? (UInt16)1 : l_AccFraction_ui16);

    // determine tolerance range for wic difference with respect
    // to vehicle velocity and max. vehicle acceleration
    l_VelFraction_ui16 = (UInt16)g_mtl_u32_Div_u32_ui32(
        (UInt32)g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 * (UInt32)l_TimeMS_ui16,
        (UInt32)g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16);

/*
  reactivate QAC warning message 584: division by zero is not possible here
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 584
#endif

    l_Tolerance_ui16 =
        l_AccFraction_ui16 + (g_parGetParaVHO_Odo_VCALPhaseTol_ui16 * l_MulFac_ui16) +
        (
            // convert velocity from 0.1km/h to 1/256m/s
            // vel_conv = (vel * 256) / (3.6 *10)
            (UInt16)((g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 >> 6) / (UInt8)9) *
            g_parGetParaVHO_Odo_VCALVeloDependentPhaseTol_ui16);

    if (g_VHOAutocalib_st.WPErrCnt_ui8 > 0)
    {
        // corrected wic values have an influence on the calculated velocity
        // which is the basis for the wic jitter correction - to avoid that one
        // wic error causes subsequent errors the window will be enlarged with every
        // corrected wic value
        l_Tolerance_ui16 *= ((UInt16)g_VHOAutocalib_st.WPErrCnt_ui8 + (UInt16)1);
    }

    if ((g_parGetParaVHO_Odo_VCALMinDeltaZeroAllowed_bl != FALSE) ||
        (l_VelFraction_ui16 <= l_Tolerance_ui16))
    {
        // always set minimal wic delta to zero
        l_MinWICDiff_ui16 = 0;
    }
    else
    {
        l_MinWICDiff_ui16 = l_VelFraction_ui16 - l_Tolerance_ui16;
    }

    l_MaxWICDiff_ui16 = l_VelFraction_ui16 + l_Tolerance_ui16;

    if (f_ForceExtrapolation_bl != FALSE)
    {
        g_VHOOdoBuf_st.WICDelta_pui16[AvgLeftRight_enm] = l_VelFraction_ui16;
        g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm]     = l_VelFraction_ui16;
        g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm]    = l_VelFraction_ui16;
        g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm]    = l_VelFraction_ui16;
        g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm]   = l_VelFraction_ui16;
        l_WICCorrected_bl                               = TRUE;
    }
    else
    {
        // force extrapolation != TRUE
        if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgFront_enm) ||
            (g_VHOOdoState_st.WicSource_en == VHOWicSourceAvgRear_enm))
        {
            if ((g_VHOOdoBuf_st.WICDelta_pui16[AvgLeftRight_enm] < l_MinWICDiff_ui16) ||
                (g_VHOOdoBuf_st.WICDelta_pui16[AvgLeftRight_enm] > l_MaxWICDiff_ui16))
            {
                // replace wic delta with extrapolated value
                g_VHOOdoBuf_st.WICDelta_pui16[AvgLeftRight_enm] = l_VelFraction_ui16;

                l_WICCorrected_bl = TRUE;
            }
        }
        else
        {
            if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICRear_enm) ||
                ((g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm) &&
                 (g_VHOOdoState_st.WeightDistanceRear_bf3 != 0)))
            {
                if ((g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] < l_MinWICDiff_ui16) ||
                    (g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] > l_MaxWICDiff_ui16))
                {
                    // replace wic delta with extrapolated value
                    g_VHOOdoBuf_st.WICDelta_pui16[LeftRear_enm] = l_VelFraction_ui16;
                    l_WICCorrected_bl                           = TRUE;
                }

                if ((g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] < l_MinWICDiff_ui16) ||
                    (g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] > l_MaxWICDiff_ui16))
                {
                    // replace wic delta with extrapolated value
                    g_VHOOdoBuf_st.WICDelta_pui16[RightRear_enm] = l_VelFraction_ui16;
                    l_WICCorrected_bl                            = TRUE;
                }
            }

            if ((g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm) ||
                ((g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm) &&
                 (g_VHOOdoState_st.WeightDistanceFront_bf3 != 0)))
            {
                if ((g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm] < l_MinWICDiff_ui16) ||
                    (g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm] > l_MaxWICDiff_ui16))
                {
                    // replace wic delta with extrapolated value
                    g_VHOOdoBuf_st.WICDelta_pui16[LeftFront_enm] = l_VelFraction_ui16;
                    l_WICCorrected_bl                            = TRUE;
                }

                if ((g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm] < l_MinWICDiff_ui16) ||
                    (g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm] > l_MaxWICDiff_ui16))
                {
                    // replace wic delta with extrapolated value
                    g_VHOOdoBuf_st.WICDelta_pui16[RightFront_enm] = l_VelFraction_ui16;
                    l_WICCorrected_bl                             = TRUE;
                }
            }
        }
    }
    if (l_WICCorrected_bl != FALSE)
    {
        l_CntNoError_ui8 = 0;

        if (g_VHOAutocalib_st.ContinuousWPErrCnt_ui8 == 0)
        {
            // start time counter for window of max. size 1,5 seconds
            g_VHOAutocalib_st.WPTimeCounterStart_ui8 = 0;
        }
        g_VHOAutocalib_st.WPTimeCounterStart_ui8++;

        // counter of value of existing wic jitter correction
        g_VHOAutocalib_st.ContinuousWPErrCnt_ui8++;

        if (g_VHOAutocalib_st.ContinuousWPErrCnt_ui8 >
            (((UInt32)(md_VHODIAG_RESET_TIME_MS_ui16 * 25) / ((UInt32)100)) /
             g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8))
        {
            // set failure for 2 seconds
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8 = (UInt8)1;
            g_VHOAutocalib_st.BypassWP_bl             = TRUE;
        }
        else
        {
            g_VHOAutocalib_st.BypassWP_bl = FALSE;
        }

        if ((g_VHOAutocalib_st.WPTimeCounterStart2_ui8 != 0) &&
            (g_VHOAutocalib_st.WPTimeCounterStart2_ui8 <=
             (md_VHODIAG_MAX_TIME_FAULT_PRESENT_ui16 /
              g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8)))
        {
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOn_enm);
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8++;
        }
        else
        {
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOff_enm);
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8 = 0;
        }

        if (g_VHOAutocalib_st.WPErrCnt_ui8 < md_VHOWICErrThres_ui8)
        {
            // counter value to increase or decrease window size of allowed wic change
            // per cycle
            g_VHOAutocalib_st.WPErrCnt_ui8++;
        }
    }
    else
    {
/*
  suppress QA-C error messages due to violation of MISRA-C:2004 Rule 19.10
  reason: Mentioned objections (QAC) are right but a logical overflow is
  undesired --> logical flow 3355 -> The result of this logical operation is
  always 'TRUE' 3358 -> The value of this 'if' control expression is always
  'TRUE'.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3355, 3358
#endif

        if (l_CntNoError_ui8 <= (UInt8)255)
        {
            l_CntNoError_ui8++;
        }

/*
  reactivate QA-C error messages due to violation of MISRA-C:2004 Rule 19.10
  reason: Mentioned objections (QAC) are right but a logical overflow is
  undesired --> logical flow 3355 -> The result of this logical operation is
  always 'TRUE' 3358 -> The value of this 'if' control expression is always
  'TRUE'.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3355, 3358
#endif

        if (l_CntNoError_ui8 >= (UInt8)((UInt32)md_VHODIAG_MAX_TIME_WP_NO_ERROR_MS_ui16 /
                                        g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8))
        {
            // reset timer counter of detected error because of 500ms stable system
            // behaviour
            g_VHOAutocalib_st.WPTimeCounterStart_ui8 = 0;
            g_VHOAutocalib_st.ContinuousWPErrCnt_ui8 = 0;
        }

        if (g_VHOAutocalib_st.ContinuousWPErrCnt_ui8 > (UInt8)0)
        {
/*
  suppress QA-C error messages due to violation of MISRA-C:2004 Rule 19.10
  reason: Mentioned objections (QAC) are right but a logical overflow is
  undesired --> logical flow 3355 -> The result of this logical operation is
  always 'TRUE' 3358 -> The value of this 'if' control expression is always
  'TRUE'.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3355, 3358
#endif

            if (g_VHOAutocalib_st.WPTimeCounterStart_ui8 <= (UInt8)255)
            {
                g_VHOAutocalib_st.WPTimeCounterStart_ui8++;
            }

/*
  reactivate QA-C error messages due to violation of MISRA-C:2004 Rule 19.10
  reason: Mentioned objections (QAC) are right but a logical overflow is
  undesired --> logical flow 3355 -> The result of this logical operation is
  always 'TRUE' 3358 -> The value of this 'if' control expression is always
  'TRUE'.
*/
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3355, 3358
#endif
        }

        if (g_VHOAutocalib_st.WPErrCnt_ui8 > 0)
        {
            g_VHOAutocalib_st.WPErrCnt_ui8--;
        }

        if ((g_VHOAutocalib_st.WPTimeCounterStart2_ui8 != 0) &&
            (g_VHOAutocalib_st.WPTimeCounterStart2_ui8 <=
             (md_VHODIAG_MAX_TIME_FAULT_PRESENT_ui16 /
              g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8)))
        {
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOn_enm);
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8++;
        }
        else
        {
            g_vhoLogHdl_vd(VHOLog_InSigFailureWIC_enm, g_vhoLogStateOff_enm);
            g_VHOAutocalib_st.WPTimeCounterStart2_ui8 = 0;
            g_VHOAutocalib_st.BypassWP_bl             = FALSE;
        }
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         calculates moving average of the yaw rate read from CAN
 *                as well as yaw rate offset (in stillstand)
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
VISIBILITY void m_VHOSignalPreprocYawRate_vd(void)
{

    SInt32 l_Sum_si32;
    Boolean l_YawRateValueTooLarge_bl = FALSE;
    UInt8 l_I_ui8                     = 0;

    const UInt32 lc_YawThres_ui32 =
        (UInt32)0x80000000 / ((UInt32)g_VHOAutocalib_st.YawRateRMSize_ui8 + (UInt32)1);

    // const UInt16 lc_StandstillCycleThres_ui16 =
    //    md_VCALYRStillstTimeThresMS_ui16 / (UInt16)
    //    gd_VHO_WIC_CYCLETIME_MS_ui32;

    l_Sum_si32 = g_VHOCanSig_st.YawRate_si16;

    while ((l_I_ui8 < g_VHOAutocalib_st.YawRateRMSize_ui8) &&
           (l_YawRateValueTooLarge_bl == FALSE))
    {
        if ((UInt32)g_mtl_Abs_mac(g_VHOAutocalib_st.YawRate_si32[l_I_ui8]) <
            lc_YawThres_ui32)
        {
            l_Sum_si32 += g_VHOAutocalib_st.YawRate_si32[l_I_ui8];
        }
        else
        {
            l_YawRateValueTooLarge_bl = TRUE;
        }
        l_I_ui8++;
    }

    if (l_YawRateValueTooLarge_bl == FALSE)
    {
        g_VHOAutocalib_st.YawRateFil_si32 = g_mtl_s32_Div_s32_si32(
            l_Sum_si32, (g_VHOAutocalib_st.YawRateRMSize_ui8 + (UInt8)1));
    }
    else
    {
        // to avoid overflow, the original yaw rate signal is used instead
        g_VHOAutocalib_st.YawRateFil_si32 = g_VHOCanSig_st.YawRate_si16;
    }

    // store current yaw rate to ring memory
    g_VHOAutocalib_st.YawRate_si32[g_VHOAutocalib_st.NextIndex_ui8] =
        g_VHOCanSig_st.YawRate_si16;

    g_VHOAutocalib_st.NextIndex_ui8++;
    g_VHOAutocalib_st.NextIndex_ui8 %= g_VHOAutocalib_st.YawRateRMSize_ui8;

    /* 22.01.2011, shi2lr: Yaw rate offset calculation will be replaced by offset
                           calculation m_VHOSignalPreProcYROffsetCalc_bl
      // calibrate yaw rate offset in vehicle standstill (needed by direction
      detection) if (g_VHOVehicleState_st.StandstillCycleCnt_ui16 >=
      lc_StandstillCycleThres_ui16)
      {
        // max yaw rate in stillstand: -6�/s...+6�/s (OPTIONAL: validation)
        // => in res. 0.0001�/s -60000...60000
        // 100*60000 = 6000000 < 2^31

      //  g_VHOAutocalib_st.YawRateOffsetStillSt_si32 =
      //    g_VHODivide_si32(
      //    ((SInt32) 99*g_VHOAutocalib_st.YawRateOffsetStillSt_si32) +
      //    (g_VHOCanSig_st.YawRate_st.ValCor_si32*100L),100L);
        //
        g_VHOAutocalib_st.YawRateOffsetStillSt_si32 =
          g_mtl_s32_Div_s32_si32(
          ((SInt32) 98*g_VHOAutocalib_st.YawRateOffsetStillSt_si32) +
          (g_VHOCanSig_st.YawRate_st.ValCor_si32*(SInt32)100*(SInt32)2),(SInt32)100);
      }
    */
}

#endif // #if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC ==
       //  SW_ON)
// ||  (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         tyre tolerance algorithm based on wheel velocities, used to
 *                detect relative deviations in the tyre radii
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
VISIBILITY void m_VHOSignalPreprocTTC_VEL_vd(void)
{
    if (m_VHOSignalPreprocTTC_AdmCondFulfilled_bl(
            (g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 >>
             6), // current vehicle velocity m/s   F2
            0,   // current acceleration     m/s^2 F4
            (SInt16)g_VHOAutocalib_st.YawRateFil_si32, // filtered yaw rate 0.01�/s
            g_VHOAutocalibCtrl_st.YROff_si16,          // yaw rate offset
            g_VHOVehicleState_st.RollingDirection_en   // rolling direction
            ) != FALSE)
    {
        UInt8 l_I_ui8;

        // add current wheel velocities to the corresponding sum
        for (l_I_ui8 = 0; l_I_ui8 < 4; l_I_ui8++)
        {
            g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_I_ui8] +=
                g_VHOOdoBuf_st.WheelVelokmh_pui16[l_I_ui8];
        }

        g_VHOAutocalib_st.TTC_st.WheelVelCnt_ui16++;
        if (g_VHOAutocalib_st.TTC_st.WheelVelCnt_ui16 >=
            md_VCALTTCMinNumberOfMeasurements_ui16)
        {
            Boolean l_SpareTyreMounted_bl;
            UInt8 l_DevPercentMinMax_ui8;

            m_VHOSignalPreprocTTCEvalRadDiff_vd(&l_DevPercentMinMax_ui8,
                                                &l_SpareTyreMounted_bl);

            if (l_DevPercentMinMax_ui8 > md_VCALTTCMaxDevMinMaxVelPercent_ui8)
            {
                // the difference between the largest and the smallest tyre radius is
                // bigger than the specified maximum => enables handling of a spare
                // wheel
                g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
                m_VHOResetTTC_vd();
                g_VHOAutocalibCtrl_st.TTCState_enm           = VHOCalibState_INITIAL_enm;
                g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 = (UInt8)0;
                g_VHOAutocalibCtrl_st.StateChangedInCurrentCycle_bl = TRUE;
                g_VHOAutocalib_st.l_EEPROMWriteREQ_bl               = TRUE;
            }
            else
            {
                //
                // difference between max and min wic length is <=
                // md_VCALTTCMaxDevMinMaxVelPercent_ui8
                // => CALCULATE ESTIMATIONS FOR DEVIATIONS OF TYRE RADII
                //

                UInt16 l_ScalFacLRRR_ui16;
                UInt16 l_ScalFacRFLF_ui16      = (UInt16)51200;
                UInt16 l_ScalFacRFLF_HIST_ui16 = (UInt16)51200;
                UInt16 l_ScalFacLRLF_ui16      = (UInt16)51200;
                UInt16 l_ScalFacLRRF_ui16      = (UInt16)51200;
                UInt8 l_X_ui8;

                l_ScalFacLRRR_ui16 = m_VHOSignalPreprocTTCGetScalingFactor_ui16(
                    g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[LeftRear_enm],
                    g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[RightRear_enm]);

                l_X_ui8 = m_VHOSignalPreprocDevAboveThres_ui8(
                    l_ScalFacLRRR_ui16, g_VHOAutocalibCtrl_st.ScalFacRR_ui16,
                    md_VCALTTCMaxDevOldNewWicLengthPercentF4_ui8);

                if (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm)
                {
                    l_ScalFacLRLF_ui16 = m_VHOSignalPreprocTTCGetScalingFactor_ui16(
                        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[LeftRear_enm],
                        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[LeftFront_enm]);

                    l_ScalFacLRRF_ui16 = m_VHOSignalPreprocTTCGetScalingFactor_ui16(
                        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[LeftRear_enm],
                        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[RightFront_enm]);

                    l_ScalFacRFLF_ui16 = m_VHOSignalPreprocTTCGetScalingFactor_ui16(
                        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[RightFront_enm],
                        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[LeftFront_enm]);

                    l_ScalFacRFLF_HIST_ui16 = m_VHOSignalPreprocTTCGetScalingFactor_ui16(
                        g_VHOAutocalibCtrl_st.ScalFacLF_ui16,
                        g_VHOAutocalibCtrl_st.ScalFacRF_ui16);

                    l_X_ui8 += m_VHOSignalPreprocDevAboveThres_ui8(
                        l_ScalFacRFLF_ui16, l_ScalFacRFLF_HIST_ui16,
                        md_VCALTTCMaxDevOldNewWicLengthPercentF4_ui8);
                }

                if ((g_VHOAutocalibCtrl_st.TTCState_enm == VHOCalibState_INITIAL_enm) ||
                    (l_X_ui8 > 0))
                {
                    // tyre radii have not been calibrated yet or differences between last
                    // and new values are too big for at least one tyre =>
                    // set state to UNSTABLE
                    g_VHOAutocalibCtrl_st.TTCState_enm = VHOCalibState_UNSTABLE_enm;
                    g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 = (UInt8)1;
                    g_VHOAutocalibCtrl_st.ScalFacRR_ui16         = l_ScalFacLRRR_ui16;
                    g_VHOAutocalibCtrl_st.ScalFacLF_ui16         = l_ScalFacLRLF_ui16;
                    g_VHOAutocalibCtrl_st.ScalFacRF_ui16         = l_ScalFacLRRF_ui16;
                    m_VHOResetTTC_vd();
                    g_VHOAutocalibCtrl_st.StateChangedInCurrentCycle_bl = TRUE;
                    g_VHOAutocalib_st.l_EEPROMWriteREQ_bl               = TRUE;
                }
                else
                {
                    // increment cycle counter (used to count the number of ttc cycles
                    // with stable results
                    g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 =
                        (g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 == (UInt8)0xFF)
                            ? (UInt8)0xFF
                            : (g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 + (UInt8)1);

                    // difference between new and former wic length lies within the
                    // specified range for all values
                    if ((g_VHOAutocalibCtrl_st.TTCState_enm ==
                         VHOCalibState_UNSTABLE_enm) &&
                        (g_VHOAutocalibCtrl_st.NumStableTTCCycles_ui8 >=
                         gd_VCALTTCMinCyclesUntilStable_ui8))
                    {
                        g_VHOAutocalibCtrl_st.TTCState_enm    = VHOCalibState_STABLE_enm;
                        g_VHOAutocalib_st.l_EEPROMWriteREQ_bl = TRUE;
                    }

                    g_VHOAutocalibCtrl_st.ScalFacRR_ui16 = (UInt16)g_mtl_s32_Div_s32_si32(
                        (md_VCALTTCWeightNewValue_si32 * (SInt32)l_ScalFacLRRR_ui16) +
                            (md_VCALTTCWeightOldValue_si32 *
                             (SInt32)g_VHOAutocalibCtrl_st.ScalFacRR_ui16),
                        md_VCALTTCWeightNewValue_si32 + md_VCALTTCWeightOldValue_si32);

                    g_VHOAutocalibCtrl_st.ScalFacLF_ui16 = (UInt16)g_mtl_s32_Div_s32_si32(
                        (md_VCALTTCWeightNewValue_si32 * (SInt32)l_ScalFacLRLF_ui16) +
                            (md_VCALTTCWeightOldValue_si32 *
                             (SInt32)g_VHOAutocalibCtrl_st.ScalFacLF_ui16),
                        md_VCALTTCWeightNewValue_si32 + md_VCALTTCWeightOldValue_si32);

                    g_VHOAutocalibCtrl_st.ScalFacRF_ui16 = (UInt16)g_mtl_s32_Div_s32_si32(
                        (md_VCALTTCWeightNewValue_si32 * (SInt32)l_ScalFacLRRF_ui16) +
                            (md_VCALTTCWeightOldValue_si32 *
                             (SInt32)g_VHOAutocalibCtrl_st.ScalFacRF_ui16),
                        md_VCALTTCWeightNewValue_si32 + md_VCALTTCWeightOldValue_si32);

                    l_X_ui8 = m_VHOSignalPreprocDevAboveThres_ui8(
                        l_ScalFacLRRR_ui16, g_VHOAutocalibCtrl_st.ScalFacRR_ui16,
                        md_VCALTTCMinDevOldNewWicLengthForEEPROMaccessPercentF4_ui8);

                    if (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm)
                    {
                        l_X_ui8 += m_VHOSignalPreprocDevAboveThres_ui8(
                            l_ScalFacRFLF_ui16, l_ScalFacRFLF_HIST_ui16,
                            md_VCALTTCMinDevOldNewWicLengthForEEPROMaccessPercentF4_ui8);
                    }

                    if (l_X_ui8 > 0)
                    {
                        g_VHOAutocalib_st.l_EEPROMWriteREQ_bl = TRUE;
                    }

                    m_VHOResetTTC_vd();
                    g_VHOAutocalibCtrl_st.StateChangedInCurrentCycle_bl = TRUE;
                }
            }
        }
    }
}
#endif // LS_VHO_AUTOCALIB_TTC

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function used to compare the deviations between the radii of
 *                the used tyres
 * @details
 *
 * @param         l_DevPercentMinMax_pui8
 * @param         l_SpareTyreMounted_pbl
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
VISIBILITY void m_VHOSignalPreprocTTCEvalRadDiff_vd(UInt8 *l_DevPercentMinMax_pui8,
                                                    Boolean *l_SpareTyreMounted_pbl)
{
    UInt32 *l_Min_pui32;
    UInt32 *l_Max_pui32;

    UInt8 l_iStart_ui8;
    UInt8 l_iEnd_ui8;
    UInt8 l_I_ui8;

    if (g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICRear_enm)
    {
        l_iStart_ui8 = LeftRear_enm;
        l_iEnd_ui8   = RightRear_enm;
    }
    else if (g_VHOOdoState_st.WicSource_en == VHOWicSourceFourWIC_enm)
    {
        l_iStart_ui8 = LeftRear_enm;
        l_iEnd_ui8   = RightFront_enm;
    }
    else if (g_VHOOdoState_st.WicSource_en == VHOWicSourceTwoWICFront_enm)
    {
        l_iStart_ui8 = LeftFront_enm;
        l_iEnd_ui8   = RightFront_enm;
    }
    else
    {
        g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
        l_iStart_ui8 = LeftRear_enm;
        l_iEnd_ui8   = l_iStart_ui8;
    }

    l_Min_pui32 = &g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_iStart_ui8];
    l_Max_pui32 = &g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_iStart_ui8];

    for (l_I_ui8 = (l_iStart_ui8 + 1); l_I_ui8 <= l_iEnd_ui8; l_I_ui8++)
    {
        l_Min_pui32 =
            (g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_I_ui8] < (*l_Min_pui32))
                ? (&g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_I_ui8])
                : l_Min_pui32;

        l_Max_pui32 =
            (g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_I_ui8] > (*l_Max_pui32))
                ? (&g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_I_ui8])
                : l_Max_pui32;
    }

    if (*l_Max_pui32 != 0)
    {
        (*l_DevPercentMinMax_pui8) =
            ((UInt8)100 - (UInt8)((*l_Min_pui32 * (UInt32)100) / *l_Max_pui32));
    }
    else
    {
        (*l_DevPercentMinMax_pui8) = (UInt8)100;
    }
    (*l_SpareTyreMounted_pbl) = FALSE;

    return;
}
#endif // LS_VHO_AUTOCALIB_TTC

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         get relative radii size of right rear, left front and right
 * front wheels (compared to left rear wheel)
 * @details
 *
 * @param         l_A_ui32
 * @param         l_B_ui32
 * @return        UInt16
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
VISIBILITY UInt16 m_VHOSignalPreprocTTCGetScalingFactor_ui16(UInt32 l_A_ui32,
                                                             UInt32 l_B_ui32)
{
    UInt16 l_RetVal_ui16;

    if (l_B_ui32 != 0)
    {
        while ((UInt32)g_mtl_Abs_mac((SInt32)l_A_ui32 - (SInt32)l_B_ui32) > (UInt32)41943)
        {
            // reduce precision to prevent overflow
            l_A_ui32 = l_A_ui32 >> 1;
            l_B_ui32 = l_B_ui32 >> 1;
        }

        // get deviation of the wheel indicated by f_Tyre_en to the reference wheel
        l_RetVal_ui16 =
            (UInt16)(g_mtl_s32_Div_s32_si32(
                         ((SInt32)51200 * ((SInt32)l_A_ui32 - (SInt32)l_B_ui32)),
                         (SInt32)l_B_ui32) +
                     (SInt32)51200);
    }
    else
    {
        // invalid argument
        l_RetVal_ui16 = 51200;
    }
    return l_RetVal_ui16;
}
#endif // LS_VHO_AUTOCALIB_TTC

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function used to check whether the difference between two
 *                input values is bigger than x percent
 * @details
 *
 * @param         f_FirstValue_ui16                         first value
 * @param         f_SecondValue_ui16                        second value
 * @param         f_MaxDevPercentF4_ui8                     max dev (1/16%)
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
VISIBILITY UInt8 m_VHOSignalPreprocDevAboveThres_ui8(UInt16 f_FirstValue_ui16,
                                                     UInt16 f_SecondValue_ui16,
                                                     UInt8 f_MaxDevPercentF4_ui8)
{
    UInt8 l_DevPercentF4_ui8;
    UInt8 l_RetVal_ui8;

    if (f_SecondValue_ui16 > 0)
    {
        // calculate deviation in % and multiply with 2^4
        l_DevPercentF4_ui8 = (UInt8)g_mtl_Abs_mac(
            (SInt16)1600 - ((SInt16)(((SInt32)f_FirstValue_ui16 * (SInt32)1600) /
                                     (SInt32)f_SecondValue_ui16)));
    }
    else
    {
        l_DevPercentF4_ui8 = 0;
    }

    if (l_DevPercentF4_ui8 > f_MaxDevPercentF4_ui8)
    {
        // deviation is bigger than the specified threshold
        l_RetVal_ui8 = (UInt8)1;
    }
    else
    {
        l_RetVal_ui8 = (UInt8)0;
    }

    return l_RetVal_ui8;
}
#endif // LS_VHO_AUTOCALIB_TTC

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         tyre tolerance algorithm based on wheel velocities, used to
 *                reset function for tyre tolerance compensation
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
VISIBILITY void m_VHOResetTTC_vd(void)
{
    UInt8 l_I_ui8;

    // initialize wheel velocities
    g_VHOAutocalib_st.TTC_st.WheelVelCnt_ui16 = (UInt16)0;

    for (l_I_ui8 = 0; l_I_ui8 < 4; l_I_ui8++)
    {
        g_VHOAutocalib_st.TTC_st.WheelVelSum_pui32[l_I_ui8] = (UInt32)0;
    }

    g_VHOAutocalib_st.TTC_st.TimeAdmissionConditionFulfilledMS_ui16 = (UInt16)0;
}
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         used for the tyre tolerance compensation to determine whether
 *                admission condition is fulfilled
 * @details
 *
 * @param         f_VehicleVelocityMperSF2_ui16
 * @param         f_VehicleAccelerationMperSPow2F4_si16
 * @param         f_YawRateFiltered_si16
 * @param         f_YawRateOffset_si16
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
#if (LS_VHO_AUTOCALIB_TTC == SW_ON)
VISIBILITY Boolean m_VHOSignalPreprocTTC_AdmCondFulfilled_bl(
    UInt16 f_VehicleVelocityMperSF2_ui16, SInt16 f_VehicleAccelerationMperSPow2F4_si16,
    SInt16 f_YawRateFiltered_si16, SInt16 f_YawRateOffset_si16,
    gType_VHORollRecogState_en f_RollingDirection_en)
{
    // default: ttc admission condition is not fulfilled
    Boolean l_RetVal_bl = FALSE;
    const UInt16 lc_VehicleAccelerationMperSPow2F4_ui16 =
        (UInt16)g_mtl_Abs_mac(f_VehicleAccelerationMperSPow2F4_si16);
    const UInt16 lc_YawRateOffCorr_ui16 =
        (UInt16)g_mtl_Abs_mac(f_YawRateFiltered_si16 - f_YawRateOffset_si16);

    if ((f_VehicleVelocityMperSF2_ui16 > md_VCALTTCMinVelocityMPERSF2_ui16) &&
        (f_VehicleVelocityMperSF2_ui16 < md_VCALTTCMaxVelocityMPERSF2_ui16) &&
        (lc_VehicleAccelerationMperSPow2F4_ui16 < md_VCALTTCMaxAccAbsMPERSPOW2F4_ui16) &&
        (f_RollingDirection_en == VHO_MOV_FORW_enm) &&
        (lc_YawRateOffCorr_ui16 < md_VCALTTCMaxYawRateAbs_ui16))
    {
        g_VHOAutocalib_st.TTC_st.TimeAdmissionConditionFulfilledMS_ui16 +=
            g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16;

        if (g_VHOAutocalib_st.TTC_st.TimeAdmissionConditionFulfilledMS_ui16 >=
            md_VCALTTCMinTimeMSAdmissionCond_ui16)
        {
            l_RetVal_bl = TRUE;
        }
    }
    else
    {
        g_VHOAutocalib_st.TTC_st.TimeAdmissionConditionFulfilledMS_ui16 = (UInt16)0;
    }

    return l_RetVal_bl;
}
#endif

#if (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         calibration of absolute tyre radii
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOSignalPreProcTyreRadiiCalc_vd(void)
{

    // steering angle thresholds in RadF12 defining the slots
    // {1072,1215,1501,1930,2502} ~ {15�,17�,21�,27�,35�}
    // const UInt16 lc_SAThresRadF12_pui16[5] = {1072,1215,1501,1930,2502};
    // {1072,1215,1501,1930,2502} ~ {13�,20�}
    const UInt16 lc_SAThresRadF12_pui16[2] = {929, 1430};

    const Boolean lc_TyreRadiiCalibConstraintsFulfilled_bl =
        (Boolean)(((UInt32)g_mtl_Abs_mac(
                       g_VHOCorrectedSignals_st.Corrected_YawRate_si32) <
                   (UInt32)md_VCALTyreYawRateHighthres_si32) &&
                  (g_VHOAutocalib_st.VeloRearAxleCenter_ui16 >
                   md_VCALTyreVeloLowThres_ui16) &&
                  (g_VHOAutocalib_st.VeloRearAxleCenter_ui16 <
                   md_VCALTyreVeloHighThres_ui16) &&
                  ((g_VHOVehicleState_st.SAngRadF12_si16 >
                    ((SInt16)lc_SAThresRadF12_pui16[0])) ||
                   (g_VHOVehicleState_st.SAngRadF12_si16 <
                    -((SInt16)lc_SAThresRadF12_pui16[0]))) &&
                  (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_FORW_enm));

    if (lc_TyreRadiiCalibConstraintsFulfilled_bl != FALSE)
    // calculate estimation for velocity scaling factor
    {
        UInt16 l_VeloScalFac_ui16;

        // get estimated velocity
        // Attention: Even steering angle rear is != 0
        // the real WheelBase instead of the virtual WheelBase have have to be used,
        // because the estimated velocity of the center of the rear axle is used in
        // calulation
        g_VHOAutocalib_st.Tyre_st.EstimatedVelo_ui16 =
            m_VHOSignalPreProcEstimateVelo_ui16(
                (SInt16)g_VHOCorrectedSignals_st.Corrected_YawRate_si32,
                g_VHOVehicleState_st.SAngRadF12_si16,
                g_VHOParameters_st.VHO_Veh_pst->WheelBase_ui16);

        if (g_VHOAutocalib_st.Tyre_st.EstimatedVelo_ui16 > 0)
        {
            // calculate (v_estimated / v_measured) * 1000
            // e.g. v_e = 10.1km/h, v_m = 10km/h =>
            // l_VeloScalFac_ui16 = 1010
            l_VeloScalFac_ui16 =
                (UInt16)(((UInt32)g_VHOAutocalib_st.VeloRearAxleCenter_ui16 *
                          (UInt32)1000) /
                         //(((UInt32) ((double) g_VHOVehicleState_st.VehicleVelocity_ui16
                         //* (double)1.40625) * (UInt32) 1000) /
                         (UInt32)g_VHOAutocalib_st.Tyre_st.EstimatedVelo_ui16);
        }
        else
        {
            l_VeloScalFac_ui16 = (UInt16)3000;
        }

        if ((l_VeloScalFac_ui16 < 1200) && (l_VeloScalFac_ui16 > 800))
        {
            // deviation smaller than 20% => take into account
            UInt8 l_Index_ui8;
            const UInt16 lc_AbsSteeringAngleRadF12_ui16 =
                (UInt16)g_mtl_Abs_mac(g_VHOVehicleState_st.SAngRadF12_si16);

            // get slot index to store measurement
            if ((lc_AbsSteeringAngleRadF12_ui16 >= lc_SAThresRadF12_pui16[0]) &&
                (lc_AbsSteeringAngleRadF12_ui16 < lc_SAThresRadF12_pui16[1]))
            {
                // 13� <= sa < 20�
                l_Index_ui8 = 0;
            }
            else
            {
                // sa >= 20�
                l_Index_ui8 = 1;
            }

            /*
                    if (lc_AbsSteeringAngleRadF12_ui16 >= lc_SAThresRadF12_pui16[1] &&
                           lc_AbsSteeringAngleRadF12_ui16 < lc_SAThresRadF12_pui16[2])
                  {
                    // sa > 20�
                    l_Index_ui8 = 1;
                  }
                  else if (lc_AbsSteeringAngleRadF12_ui16 >= lc_SAThresRadF12_pui16[2]
               && lc_AbsSteeringAngleRadF12_ui16 < lc_SAThresRadF12_pui16[3])
                  {
                    // 21� <= sa < 27�
                    l_Index_ui8 = 2;
                  }
                  else if (lc_AbsSteeringAngleRadF12_ui16 >= lc_SAThresRadF12_pui16[3]
               && lc_AbsSteeringAngleRadF12_ui16 < lc_SAThresRadF12_pui16[4])
                  {
                    // 27� <= sa < 35�
                    l_Index_ui8 = 3;
                  }
                  else
                  {
                    // sa >= 35�
                    l_Index_ui8 = 4;
                  }
            */

            if ((g_VHOVehicleState_st.SAngRadF12_si16 > 0) &&
                ((UInt8)((g_VHOAutocalib_st.Tyre_st.SlotsOpenLeft_ui8 >> l_Index_ui8) &
                         (UInt8)0x1) == (UInt8)0x1))
            {
                // add quotient to the corresponding slot for the left side
                g_VHOAutocalib_st.Tyre_st.VeloScalFacLeft_pui32[l_Index_ui8] +=
                    l_VeloScalFac_ui16;
                ++g_VHOAutocalib_st.Tyre_st.VeloScalFacLeftCnt_pui16[l_Index_ui8];
                ++g_VHOAutocalib_st.Tyre_st.VeloScalFacTotalCntLeft_ui16;
            }
            else if ((g_VHOVehicleState_st.SAngRadF12_si16 < 0) &&
                     ((UInt8)((g_VHOAutocalib_st.Tyre_st.SlotsOpenRight_ui8 >>
                               l_Index_ui8) &
                              (UInt8)0x1) == (UInt8)0x1))
            {
                // add quotient to the corresponding slot for the right side
                g_VHOAutocalib_st.Tyre_st.VeloScalFacRight_pui32[l_Index_ui8] +=
                    l_VeloScalFac_ui16;
                ++g_VHOAutocalib_st.Tyre_st.VeloScalFacRightCnt_pui16[l_Index_ui8];
                ++g_VHOAutocalib_st.Tyre_st.VeloScalFacTotalCntRight_ui16;
            }
            else
            {
                // do nothing, but do not get QAC error
            }

            // measurement data will not be evaluated until a specified time has been
            // driven
            if ((UInt16)(g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 -
                         g_VHOAutocalib_st.Tyre_st.TyreRadiiCalibStartTimeMS_ui32) >=
                md_VCALTyreMinTimeUntilEvalMS_ui32)
            {
                switch (g_VHOAutocalib_st.Tyre_st.TyreRadiiCalibState_ui8)
                {
                    case (UInt8)0:
                        // not enough measurements on neither right nor left side since
                        // last cycle
                        if ((g_VHOAutocalib_st.Tyre_st.VeloScalFacTotalCntRight_ui16 >
                             md_VCALTyreMinNumberOfMeas_ui16) ||
                            (g_VHOAutocalib_st.Tyre_st.VeloScalFacTotalCntLeft_ui16 >
                             md_VCALTyreMinNumberOfMeas_ui16))
                        {
                            UInt8 l_I_ui8;

                            // clear slots with very small number of measurements
                            for (l_I_ui8 = 0; l_I_ui8 < 2; l_I_ui8++)
                            {
                                if ((UInt16)(g_VHOAutocalib_st.Tyre_st
                                                 .VeloScalFacLeftCnt_pui16[l_I_ui8] +
                                             g_VHOAutocalib_st.Tyre_st
                                                 .VeloScalFacRightCnt_pui16[l_I_ui8]) <
                                    md_VCALTyreMinNumMeasPerSlot_ui16)
                                {
                                    // close slots on both sides
                                    g_VHOAutocalib_st.Tyre_st.SlotsOpenLeft_ui8 &=
                                        (UInt8)((UInt8)0xFF - ((UInt8)1 << l_I_ui8));
                                    g_VHOAutocalib_st.Tyre_st.SlotsOpenRight_ui8 &=
                                        (UInt8)((UInt8)0xFF - ((UInt8)1 << l_I_ui8));

                                    // clear status variables
                                    g_VHOAutocalib_st.Tyre_st
                                        .VeloScalFacLeft_pui32[l_I_ui8] = 0;
                                    g_VHOAutocalib_st.Tyre_st
                                        .VeloScalFacLeftCnt_pui16[l_I_ui8] = 0;
                                    g_VHOAutocalib_st.Tyre_st
                                        .VeloScalFacRight_pui32[l_I_ui8] = 0;
                                    g_VHOAutocalib_st.Tyre_st
                                        .VeloScalFacRightCnt_pui16[l_I_ui8] = 0;
                                }
                            }
                            g_VHOAutocalib_st.Tyre_st.TyreRadiiCalibState_ui8 = 1;
                        }
                        break;

                    case (UInt8)1:
                        // enough measurements on one side
                        if (g_VHOAutocalib_st.Tyre_st
                                .VeloScalFacLeftCnt_pui16[l_Index_ui8] <
                            g_VHOAutocalib_st.Tyre_st
                                .VeloScalFacRightCnt_pui16[l_Index_ui8])
                        {
                            g_VHOAutocalib_st.Tyre_st.SlotsOpenRight_ui8 &=
                                (UInt8)((UInt8)0xFF - ((UInt8)1 << l_Index_ui8));
                        }
                        else if (g_VHOAutocalib_st.Tyre_st
                                     .VeloScalFacRightCnt_pui16[l_Index_ui8] <
                                 g_VHOAutocalib_st.Tyre_st
                                     .VeloScalFacLeftCnt_pui16[l_Index_ui8])
                        {
                            g_VHOAutocalib_st.Tyre_st.SlotsOpenLeft_ui8 &=
                                (UInt8)((UInt8)0xFF - ((UInt8)1 << l_Index_ui8));
                        }
                        else if (g_VHOAutocalib_st.Tyre_st
                                     .VeloScalFacRightCnt_pui16[l_Index_ui8] ==
                                 g_VHOAutocalib_st.Tyre_st
                                     .VeloScalFacLeftCnt_pui16[l_Index_ui8])
                        {
                            // close slots on both sides
                            g_VHOAutocalib_st.Tyre_st.SlotsOpenLeft_ui8 &=
                                (UInt8)((UInt8)0xFF - ((UInt8)1 << l_Index_ui8));
                            g_VHOAutocalib_st.Tyre_st.SlotsOpenRight_ui8 &=
                                (UInt8)((UInt8)0xFF - ((UInt8)1 << l_Index_ui8));
                        }
                        else
                        {
                            // do not get QAC error
                        }
                        break;
                    default:
                        // invalid state => reset calibration
                        g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
                        m_VHOResetTyreRadiiCalib_vd();
                        break;
                } // switch

                if ((g_VHOAutocalib_st.Tyre_st.SlotsOpenLeft_ui8 |
                     g_VHOAutocalib_st.Tyre_st.SlotsOpenRight_ui8) == 0)
                {
                    UInt8 l_Count_ui8;
                    const UInt8 lc_DENOMINATOR_ui8 = 100;
                    const UInt8 lc_MAXCOUNT_ui8    = 63;

                    l_Count_ui8 = 0;

                    g_VHOAutocalib_st.Tyre_st.LastVeloScalFac_ui16 = 0;

                    // all slots closed => final calculation of scaling factor
                    for (l_Index_ui8 = 0; l_Index_ui8 < 2; l_Index_ui8++)
                    {
                        if (g_VHOAutocalib_st.Tyre_st
                                .VeloScalFacLeftCnt_pui16[l_Index_ui8] > 0)
                        {
                            UInt8 l_Multiplier_ui8;

                            g_VHOAutocalib_st.Tyre_st
                                .VeloScalFacLeftCnt_pui16[l_Index_ui8] +=
                                g_VHOAutocalib_st.Tyre_st
                                    .VeloScalFacRightCnt_pui16[l_Index_ui8];

                            // set weight for the current slot (the more measurements the
                            // higher the weight)
                            l_Multiplier_ui8 =
                                (UInt8)(g_VHOAutocalib_st.Tyre_st
                                            .VeloScalFacLeftCnt_pui16[l_Index_ui8] /
                                        (UInt16)lc_DENOMINATOR_ui8) +
                                (UInt8)1;

                            g_VHOAutocalib_st.Tyre_st.LastVeloScalFac_ui16 +=
                                (UInt16)((UInt32)l_Multiplier_ui8 *
                                         ((g_VHOAutocalib_st.Tyre_st
                                               .VeloScalFacLeft_pui32[l_Index_ui8] +
                                           g_VHOAutocalib_st.Tyre_st
                                               .VeloScalFacRight_pui32[l_Index_ui8]) /
                                          (UInt32)g_VHOAutocalib_st.Tyre_st
                                              .VeloScalFacLeftCnt_pui16[l_Index_ui8]));

                            l_Count_ui8 += l_Multiplier_ui8;
                        }
                    }

                    if (l_Count_ui8 <= lc_MAXCOUNT_ui8)
                    {
                        if (l_Count_ui8 > 0)
                        {
                            // get new estimation for scaling factor
                            g_VHOAutocalib_st.Tyre_st.LastVeloScalFac_ui16 /=
                                (UInt16)l_Count_ui8;

                            g_VHOAutocalibCtrl_st.VeloScalFac_ui16 =
                                (UInt16)g_mtl_s32_Div_s32_si32(
                                    (SInt32)((md_VCALTyreOffsWeightOldValue_si16 *
                                              (SInt16)g_VHOAutocalibCtrl_st
                                                  .VeloScalFac_ui16) +
                                             (md_VCALTyreOffsWeightNewValue_si16 *
                                              (SInt16)g_VHOAutocalib_st.Tyre_st
                                                  .LastVeloScalFac_ui16)),
                                    (SInt32)(md_VCALTyreOffsWeightOldValue_si16 +
                                             md_VCALTyreOffsWeightNewValue_si16));
                        }
                        else
                        {
                            g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
                        }
                    }
                    else
                    {
                        // overflow possible => reset data structures and throw error
                        g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
                    }
                    // reset data structures
                    m_VHOResetTyreRadiiCalib_vd();
                }
            }
        }
        else
        {
            g_VHOAutocalib_st.Tyre_st.ImplausibleEstimationCnt_ui16++;
        }
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         estimate velocity from steering angle and yaw rate
 * @details
 *
 * @param         f_YawRateDegPerSec_si16           yaw rate, unit: 0.01�/s
 * @param         f_SteeringAngleRadF12_si16        steering angle of virtual
 * front wheel RadF12
 * @param         f_WheelBaseMM_ui16                wheel base, unit: MM
 * @return        UInt16
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY UInt16 m_VHOSignalPreProcEstimateVelo_ui16(SInt16 f_YawRateDegPerSec_si16,
                                                      SInt16 f_SteeringAngleRadF12_si16,
                                                      UInt16 f_WheelBaseMM_ui16)
{
    SInt16 l_X_si16;
    SInt32 l_Y_si32;

    // calculate (L*PSI/tand(steering_angle))
    // normation of input values:
    // f_WheelBaseMM_ui16              [mm       -> m]     =>  * 0.001
    // tan(f_SteeringAngleRadF12_si16) [1/8192   -> 1]     =>  / 8192
    // f_YawRateDegPerSec_si16         [0.01�/s  -> rad/s] =>  * (0.01*pi/180)
    //
    //                                    0.01*(pi/180) * f_YawRateDegPerSec_si16
    //                                    * 8192
    //  360 * 0.001 f_WheelBaseMM_ui16 *
    //  ----------------------------------------------
    //                                    tan(f_SteeringAngleRadF12_si16)
    //
    //                          f_YawRateDegPerSec_si16 * 100
    //  = f_WheelBaseMM_ui16 *  -------------------------------------------
    //                          194.280936 * tan(f_SteeringAngleRadF12_si16)

    l_X_si16 = 0x7FFF; // set default result to INVALID
    l_Y_si32 = (SInt32)f_WheelBaseMM_ui16 * (SInt32)f_YawRateDegPerSec_si16;

    if ((UInt32)g_mtl_Abs_mac(l_Y_si32) < (UInt32)21474836)
    {
        l_X_si16 = (SInt16)g_mtl_s32_Div_s32_si32(
            l_Y_si32 * (SInt32)100,
            (SInt32)194 * (SInt32)g_mtl_Tan_si16(f_SteeringAngleRadF12_si16));

        if (l_X_si16 < 0)
        {
            g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
        }
    }
    else
    {
        g_vhoLogHdl_vd(VHOLog_Autocalib_enm, g_vhoLogStateOn_enm);
    }
    return (UInt16)g_mtl_Abs_mac(l_X_si16);
}

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3892
#endif

#endif // (LS_VHO_AUTOCALIB_TYRE_RADII == SW_OFF)

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
