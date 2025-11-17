/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 *
 * @file      vho_bas.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_BAS_H

#define VHO_BAS_H
/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include "vho_param.h"

// include of sig/vsm. this is needed to use platform wide type definitions
// within XPG (gear, move direction) and to prevent definition of redundant
// types

#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>
#include <pa_components/parameter/par_api.h>
// #include <veh/vhs/vhs_api.h>

// selectable values for direction recognition mode
#define LS_VHO_DIRDETECT_GEAR    1
#define LS_VHO_DIRDETECT_ACCSENS 2
#define LS_VHO_DIRDETECT_WIC     3
#define LS_VHO_DIRDETECT_YAW_WIC 4
#define LS_VHO_DIRDETECT_ALL     5

#define GS_VHO_MINI_VHO          1
#define GS_VHO_CONSIDER_USS_DATA 2

// window size for buffering of acceleration signal
#define gd_RM_SIZE_Ax_ui8 ((UInt8)10)

#define gd_RM_SIZE_YAW_ui8 ((UInt8)(gd_TIME_WINDOW_YR_FILT_MS_ui16 / (UInt16)10))

#define VHO_USSPROC_EXTRAPOLATION_RESTRICTED SW_ON

#ifndef LS_VHO_ENABLEFAULTINIT
#define LS_VHO_ENABLEFAULTINIT SW_OFF
#endif

#define gd_VHO_YANGHIST_STEPSIZE_MM_ui16 ((UInt16)1000)

//
// #define md_VHODIAG_MIN_TIME_BETW_GEAR_CHANGES_MS_ui16 ((UInt16)   80)
// #define md_VHODIAG_MAX_CORRECTION_COUNT_ui8           ( (UInt8)    3)
// #define md_VHODIAG_MAX_SIGNAL_CHANGE_FACTOR_ui8       ( (UInt8)   15)
// #define md_VHODIAG_MAX_ELAPSED_CAN_SIG_TIME_ui8       ( (UInt8)  150)
#define md_VHODIAG_RESET_TIME_MS_ui16           ((UInt16)1500)
#define md_VHODIAG_MAX_BYPASS_WIC_TIME_MS_ui16  ((UInt16)500)
#define md_VHODIAG_MAX_TIME_WP_NO_ERROR_MS_ui16 ((UInt16)500)
#define md_VHODIAG_MAX_TIME_FAULT_PRESENT_ui16  ((UInt16)2000)
// tyre identification
typedef enum
{
    LeftRear_enm     = 0,
    RightRear_enm    = 1,
    LeftFront_enm    = 2,
    RightFront_enm   = 3,
    AvgLeftRight_enm = 4
} gType_VHOTyre_en;

typedef enum
{
    VHO_WICDIR_STOP_enm             = 1,
    VHO_WICDIR_MOV_UNKNOWN_enm      = 2,
    VHO_WICDIR_MOV_FORW_PENDING_enm = 3,
    VHO_WICDIR_MOV_REV_PENDING_enm  = 4,
    VHO_WICDIR_MOV_FORW_enm         = 5,
    VHO_WICDIR_MOV_REV_enm          = 6
} gType_VHOWICDirState_en;

typedef enum
{
    VHODrivenAxleRear_enm   = 1,
    VHODrivenAxleFront_enm  = 2,
    VHODrivenAxle4Wheel_enm = 3
} gType_VHODrivenAxle_en;

typedef enum
{
    VHO_Direction_Gear_enm    = 0,
    VHO_Direction_Wic_enm     = 1,
    VHO_Direction_Yaw_Wic_enm = 2,
    VHO_Direction_Accsens_enm = 3
} gType_VHORollRecogMode_en;

typedef enum
{
    VHO_MOV_FORW_enm                = 0,
    VHO_MOV_BACKW_enm               = 1,
    VHO_MOV_TURNED_FORW_enm         = 2,
    VHO_MOV_TURNED_BACKW_enm        = 3,
    VHO_MOV_UNKNOWN_enm             = 4,
    VHO_STOP_enm                    = 5,
    VHO_MOV_UNDER_INVESTIGATION_enm = 6
} gType_VHORollRecogState_en;

typedef enum
{
    VHOCalibState_INITIAL_enm  = 0,
    VHOCalibState_UNSTABLE_enm = 1,
    VHOCalibState_STABLE_enm   = 2
} gType_VHOCalibState_en;

typedef struct
{
    UInt8 NumStableTTCCycles_ui8;
    UInt8 NumStableSWACycles_ui8;
    UInt8 NumStableTyreRadCycles_ui8;
    UInt8 NumStableAxlebaseCycles_ui8;
    UInt8 IndexOfTyreRadCluster_ui8;
    SInt16 SWOffDegRes_si16;
    UInt16 AvgWICLengthRearAxle_ui16;
    UInt16 AxlebaseRear_ui16;
    UInt16 ScalFacRR_ui16;
    UInt16 ScalFacLF_ui16;
    UInt16 ScalFacRF_ui16;
} gType_VHOParaAutocalib_st;

// None Volatile Memory (NVM)
typedef enum
{
    NVM_REQ_OK_enm,
    NVM_REQ_NOT_OK_enm,
    NVM_REQ_PENDING_enm,
    NVM_REQ_INTEGRITY_FAILED_enm,
    NVM_REQ_BLOCK_SKIPPED_enm,
    NVM_REQ_NV_INVALIDATED_enm
} gType_NvM_RequestResultType_en;

#ifdef COMPONENT_VHO

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

typedef struct
{

    const gType_parVHO_Veh_st *VHO_Veh_pst;
    const gType_parVHO_Odo_st *VHO_Odo_pst;
    gType_VHOParaAutocalib_st AutocalibData_st;
    gType_VHORollRecogMode_en RollRecogMode_en;
    Boolean WICPreprocActive_bl;
} gType_VHOParameters_st;

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/* shi2lr: Entfall, da zus�tzliche �berwachung der CAN-Signale in vho_diag.c ab
Rel. 5.10 entf�llt #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) typedef
struct
{
     UInt8 CorrectionCount_ui8 : 3;
     Boolean  Initialized_bl      : 1;
     Boolean  Corrected_bl        : 1;

  Boolean  SIGValidFlag_bl     : 1;  // Valid Flag as provided by SW Component
SIG
}
gType_VHOSigCtrl_st;
#endif
*/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
typedef struct
{
    UInt16 MaxChangePerSec_ui16;
    UInt16 BaseTolerance_ui16;
    SInt32 SignalThreshMin_si32;
    SInt32 SignalThreshMax_si32;
} gType_VHOSigLimits_st;
#endif

typedef struct
{
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 WICRaw_pui16[5]; // raw WIC values (not preprocessed!)
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    UInt16 WICRaw_pui16[2]; // raw WIC values (not preprocessed!)
#endif

    UInt16 WIC_CanTime_ui16; // can time [ms] that corresponds to values stored in
                             // WICRaw

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt8 WIC_Invalid_ui8; // indicates the number of invalid detected wic
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    gType_VHORollRecogState_en
        WICState_pen[5]; // state information of wheel impulse counters
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    gType_VHORollRecogState_en
        WICState_pen[2]; // state information of wheel impulse counters
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt16 SWA_si16;
    Boolean SWA_ValidFlag_bl;
    SInt16 YawRate_si16; // yaw rate                   [0.01�/s]
    Boolean YawRate_ValidFlag_bl;
#if (GS_4WS_VEHICLE == SW_ON)
    SInt16 SAR_si16;
    Boolean SAR_ValidFlag_bl;
#endif
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
#if (GS_VHS_SWA == SW_ON)
    SInt16 SWA_si16;
#elif (GS_VHS_YAW_RATE == SW_ON)
    SInt16 YawRate_si16; // yaw rate                   [0.01�/s]
#endif
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 SWA_Tick_ui16;
#endif

    UInt16 Vel_ui16; // vehicle velocity (0.1km/h)
    Boolean Vel_ValidFlag_bl;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt16 Acc_si16; // longitudinal acceleration  [0.01m/s^2] from acceleration
                     // sensor
    Boolean Acc_ValidFlag_bl;
#endif

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) ||      \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&              \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR)) || \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&              \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
    Boolean ReverseGearInserted_bl; // backward gear inserted or not    (INS 1,NOT
                                    // INS 0)
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 WheelRev_pui16[4]; // wheel revolutions, unit depending on compiler
                              // switch Unit [0.5 RPM]
#endif
} gType_VHOCanSig_st;

typedef struct
{

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    gType_VHORollRecogState_en RollDirRefYawRate_en;
    gType_VHORollRecogState_en RollDirRefGear_en;
    gType_VHORollRecogState_en RollDirRefAccInteg_en;
    gType_VHORollRecogState_en RollDirRefAccGrad_en;
    gType_VHORollRecogState_en RollDirRefAccChangePattern_en;
    gType_VHORollRecogState_en RollDirRefWIC_en;
    UInt16 TimeAfterDirChange_ui16;
    UInt8 RollDirReference_ui8;
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    // variables needed for direction recognition based on
    // a direction signal provided by wheel pulse counters
    gType_VHOWICDirState_en VHOWICDirState_en;
    UInt16 EntryTimePendingStateMS_ui16;
    UInt16 TimeLastWICIncrement_ui16;
    UInt16 WICTimeDeltaMS_ui16;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    // variables needed for the detection of direction changes
    SInt16 AccWicMin_si16;
    SInt16 AccWicMax_si16;
    SInt16 AxBase_si16;
    SInt16 GradAccBase_si16;
    UInt16 VeloBaseMPerSF8_ui16;
    Boolean LookingForMin_bl;
    UInt8 DirChangeCycleCount_ui8;
    SInt16 AxBuf_si16;
    SInt16 AxDelayed_si16;
    UInt8 CountAxGradNeg_ui8;
    UInt8 CountAxGradPos_ui8;

    // varibles needed for the detection of direction changes based on yaw rate
    // and SWA
    UInt8 SWA_Cnt_ui8;
    SInt16 SWAngBuf_si16;

    // variables needed for zero point calibration of the
    // acceleration
    UInt8 ZeroPointQuality_ui8; // metric indicating the quality of the zero point
                                // calibration
    UInt8 ZeroPointCycleCount_ui8;
    Boolean ZeroPointValid_bl;
    SInt16 AxZeroPoint_si16;
    SInt16 ZeroPointDebug_si16;
    UInt16 DistPassedWithoutRefreshMM_ui16;
    UInt16 ZeroPointMeasCount_ui16;
    SInt32 ZeroPointAccMeasSum_si32;
    SInt16 AxMin_si16;
    SInt16 AxMax_si16;
    SInt16 GradAx_si16;
    SInt16 GradAccWIC_si16;

    // variables needed for the recognition based on
    // integration
    SInt16 RMAx_psi16[gd_RM_SIZE_Ax_ui8];
    UInt8 RMAxNextElem_ui8;
    UInt8 RMAxNumStoredElem_ui8;
    SInt32 SumAx_si32;

#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    // buffers for position and orientation that is used
    // when vehicle is moving without a known driving direction
    SInt32 dx_buf_MMF8_si32;
    SInt32 dy_buf_MMF8_si32;
    UInt32 ds_buf_MMF8_ui32;
    SInt32 YawAngle_buf_si32;
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt32 YawAngleDualTrack_buf_si32;
    SInt32 YawAngleSingleTrack_buf_si32;
#endif
    UInt32 s_discarded_MMF8_ui32;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    SInt32 VirtCan_dsMMF8_si32;
    SInt16 Maxima_si16;
    UInt8 AxGradPosVal_ui8;
    UInt8 AccWICGradPosVal_ui8;
    UInt8 AccTotalValues_ui8;
    Boolean MaximaReached_bl;

#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

// Variables needed for elephant test
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&        \
     ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
#if (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)

    Boolean WIC_MovingIndication_bl;
    Boolean ExecuteElephant_bl;
    gType_VHORollRecogState_en MovingDirection_en; // just for testing
    UInt8 OpenWindows_ui8;
    UInt8 MovingTireXx_pui8[4];
    UInt16 StartTimeXx_pui16[4];
    UInt32 WICDeltaXx_pui32[4];
    SInt32 Saved_Distance_si32;

#endif // (LS_VHO_WIC_FILTER_ELEPHANT == SW_ON)
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) &&
       // ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) || (LS_VHO_DIRDETECT_MODE
       // == LS_VHO_DIRDETECT_YAW_WIC)...)

} gType_VHORollRecog_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
// structure for USS data storage and
// information needed for position
// interpolation / extrapolation
typedef struct
{
    UInt16 DistList_pui16[gd_VHOUSS_RM_SIZE_ui8][g_parGetParaVHO_NO_ECHOES_ui8];
    UInt16 TargetCANTime_pui16[gd_VHOUSS_RM_SIZE_ui8];
    UInt16 StartTime_pui16[gd_VHOUSS_RM_SIZE_ui8];
    UInt8 SensorIndex_pui8[gd_VHOUSS_RM_SIZE_ui8];
    UInt8 NumStoredDistLists_ui8;
    UInt8 NextStorageIndex_ui8;
    UInt8 IndexOldestDistList_ui8;
    SInt16 DrivenDistBuf_si16;
    SInt16 DrivenDistBetweenLastTwoEchoesCM_si16;
    UInt16 TimeSinceLastEchoMS_ui16;
    UInt16 TimeBuf_ui16;
    UInt16 TimeSinceLastEchoTCNT_ui16;
    Boolean DrivenDistUpdated_bl;
    UInt8 SensorNumber_ui8;
} gType_VHOSensorDataUSS_st;
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

//
//  STRUCTURE WITH SIGNALS AFTER PREPROCESSING
//

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
typedef struct
{
    SInt32 Corrected_YawRate_si32;
#if (GS_4WS_VEHICLE == SW_ON)
    SInt16 DelayedCorr_SWAngDeg_si16;
    SInt16 DelayedCorr_SAngRadF12_si16;
    SInt16 DelayedCorr_SAngRearDeg_si16;
    SInt16 DelayedCorr_SAngRearRadF12_si16;
#else
    SInt16 DelayedCorr_SWAngDeg_si16;
    SInt16 DelayedCorr_SAngRadF12_si16;
#endif
    // 03.11.2011, shil2r: only used for calculation of kappa based on the current
    // SWA
    SInt16 Corrected_SWAngDeg_si16;
    SInt16 Corrected_SAngRadF12_si16;
} gType_VHOCorrectedSignals_st;
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
// data structure used for tyre tolerance compensation
typedef struct
{
    UInt32 WheelVelSum_pui32[4]; // added wheel velocities for single wheels
    UInt16 WheelVelCnt_ui16;
    UInt16 TimeAdmissionConditionFulfilledMS_ui16;
} gType_VHOAutocalibTTC_st;

typedef struct
{
    // variables needed for tyre radii calibration
    UInt32 TyreRadiiCalibStartTimeMS_ui32;
    UInt32 VeloScalFacRight_pui32[2];
    UInt16 VeloScalFacRightCnt_pui16[2];
    UInt16 VeloScalFacTotalCntRight_ui16;
    UInt32 VeloScalFacLeft_pui32[2];
    UInt16 VeloScalFacLeftCnt_pui16[2];
    UInt16 VeloScalFacTotalCntLeft_ui16;
    UInt8 SlotsOpenLeft_ui8;
    UInt8 SlotsOpenRight_ui8;
    UInt8 TyreRadiiCalibState_ui8;
    UInt16 LastVeloScalFac_ui16;
    UInt16 VeloScalFacsLeftTemp_pui16[2];
    UInt16 VeloScalFacsRightTemp_pui16[2];
    UInt16 VeloScalFacsTemp_pui16[2];
    UInt16 ImplausibleEstimationCnt_ui16;
    UInt16 EstimatedVelo_ui16;
} gType_VHOAutocalibTyre_st;

typedef struct
{
    // variables needed for steering wheel offset calibration
    UInt16 MaxSWALeft_ui16;
    UInt16 MaxSWARight_ui16;
    UInt16 MinSWALeft_ui16;
    UInt16 MinSWARight_ui16;
    UInt32 SWAOffCalibStartTimeMS_ui32;
    SInt32 DiffSWAMeasEstSumLeft_si32;
    SInt32 DiffSWAMeasEstSumRight_si32;
    UInt16 DiffSWAMeasEstCntLeft_ui16;
    UInt16 DiffSWAMeasEstCntRight_ui16;
    UInt32 RelErrSumRight_ui32;
    UInt32 RelErrSumLeft_ui32;
    UInt16 AvgRelErr_Right_ui16;
    UInt16 AvgRelErr_Left_ui16;
    SInt8 DiffRelErrLeftRightPercent_si8;
    SInt16 SWAOffsetTemp_si16;
} gType_VHOAutocalibSWA_st;

// data structure for signal preprocessing and autocalibration
typedef struct
{
    // variables needed for steering wheel offset calibration
    gType_VHOAutocalibSWA_st SWA_st;

    // variables needed for tyre calibration
    gType_VHOAutocalibTyre_st Tyre_st;

    // variables needed for tyre tolerance calibration
    gType_VHOAutocalibTTC_st TTC_st;

    // variables needed for yaw rate offset calibration
    SInt16 SWAStartStandstill_si16;
    SInt32 YawRateSum_si32;
    UInt16 YawRateCountedValues_ui16;
    Boolean YawRateOffsetCalcNotAllowed_bl; // is TRUE if steering wheel changes
                                            // at stand still are huge

    // other buffers and variables
    UInt16 VeloRearAxleCenter_ui16; // mapping from single wheel velocities to
                                    // velocity on the center of the rear axle
    SInt32 YawRate_si32[gd_RM_SIZE_YAW_ui8];
    UInt8 YawRateRMSize_ui8;
    UInt8 NextIndex_ui8;
    SInt32 YawRateFil_si32; // yaw rate offset, scaling: 0.01�/s
    // SInt32 YawRateOffsetStillSt_si32;     // yaw rate offset in stillstand,
    // scaling: 0.0001�/s

    UInt8 MinWICDelta_ui8;
    UInt8 MaxWICDelta_ui8;
    UInt8 WPErrCnt_ui8;
    UInt8 ContinuousWPErrCnt_ui8;
    UInt8 WPNoErrCnt_ui8; // counter of cycles with no error in bypass situation
    Boolean BypassWP_bl;
    UInt8 WPTimeCounterStart_ui8;
    UInt8 WPTimeCounterStart2_ui8; // second time counter to measure 2 seconds
    Boolean l_EEPROMWriteREQ_bl;
    UInt32 TimeLastEEPROMaccess_ui32;
    gType_VHOParaAutocalib_st AutocalibStateEEPROM_st;

} gType_VHOAutocalib_st;

#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

typedef struct
{
    // structure for 4 wheel steering
    // storage of virtual wheel base
    // Scaling: 1 LSB = 1 mm
    UInt16 WheelBaseVirtual_ui16;
    SInt16 DeltaWheelBaseVirtual_si16;
    // if steering angle rear has the same direction as steering angle front and
    // if | steering angle rear | > | steering angle front | than
    // the forbidden steering angle rear zone is achieved
    Boolean ForbiddenRearAngleZone_bl;
} gType_VHOWheelBaseVirtual_st;

#endif // COMPONENT_VHO

#endif // VHO_BAS_H
