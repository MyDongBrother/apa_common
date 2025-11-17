/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_api.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_API_H
#define VHO_API_H

#ifdef COMPONENT_VHO // Date of template
#define APPL_VHO_H_REF 0x140325
#define APPL_VHO_C_REF 0x140325
#endif // COMPONENT_VHO

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include <data_io/dataInput_format.h>

#include "_appl_vho.h" // SWCPA Include Structure: new
#include "vho_deg.h"

#define gd_SENSOR1               1  // front left
#define gd_SENSOR2               2  // front left vice radar
#define gd_SENSOR5               5  // front right vice radar
#define gd_SENSOR6               6  // front right
#define gd_SENSOR8               8  // rear right
#define gd_SENSOR9               9  // rear left vice radar
#define gd_SENSOR12              12 // rear right vice radar
#define gd_SENSOR13              13 // rear left
#define gd_VHOYAngHistRMSize_ui8 ((UInt8)20)

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

// window size for buffering of yaw rate signal

// -> Since ring memory size has to be fix use smallest possible value for WIC
// cycletime to define ring memory size #define gd_RM_SIZE_YAW_ui8 ((UInt8)
// (gd_TIME_WINDOW_YR_FILT_MS_ui16/(UInt16)gd_VHO_WIC_CYCLETIME_MS_ui32))
#define gd_RM_SIZE_YAW_ui8 ((UInt8)(gd_TIME_WINDOW_YR_FILT_MS_ui16 / (UInt16)10))

#define VHO_USSPROC_EXTRAPOLATION_RESTRICTED SW_ON

#ifndef LS_VHO_ENABLEFAULTINIT
#define LS_VHO_ENABLEFAULTINIT SW_OFF
#endif

#define gd_VHO_YANGHIST_STEPSIZE_MM_ui16 ((UInt16)1000)

// #define md_VHODIAG_MIN_TIME_BETW_GEAR_CHANGES_MS_ui16 ((UInt16)   80)
// #define md_VHODIAG_MAX_CORRECTION_COUNT_ui8           ( (UInt8)    3)
// #define md_VHODIAG_MAX_SIGNAL_CHANGE_FACTOR_ui8       ( (UInt8)   15)
// #define md_VHODIAG_MAX_ELAPSED_CAN_SIG_TIME_ui8       ( (UInt8)  150)
#define md_VHODIAG_RESET_TIME_MS_ui16           ((UInt16)1500)
#define md_VHODIAG_MAX_BYPASS_WIC_TIME_MS_ui16  ((UInt16)500)
#define md_VHODIAG_MAX_TIME_WP_NO_ERROR_MS_ui16 ((UInt16)500)
#define md_VHODIAG_MAX_TIME_FAULT_PRESENT_ui16  ((UInt16)2000)

#define SEARCH_VHO_REINIT SW_OFF

// fault IDs
// numbering consecutive and starting with "0"
// last enum used to indicate max. number of faults
typedef enum
{
    VHOLog_InitFault_enm               = 0,
    VHOLog_InSigFailureWIC_enm         = 1,
    VHOLog_EEPROMWrite_enm             = 2,
    VHOLog_EEPROMRead_enm              = 3,
    VHOLog_Autocalib_enm               = 4,
    VHOLog_DirRecog_enm                = 5,
    VHOLog_VHOMain_enm                 = 6,
    VHOLog_USSProc_enm                 = 7,
    VHOLog_OdoCore_enm                 = 8,
    VHOLog_ExtraPolPosFault_enm        = 9,
    VHOLog_UnspecErr_enm               = 10,
    VHOLog_MaxUnknownDistExceeded_enm  = 11,
    VHOLog_MaxUnknownDistDiscarded_enm = 12,
    VHOLog_TotalNumber_enm             = 13
} gType_vhoLogFaultID_en;

// fault states
typedef enum
{
    g_vhoLogStateOn_enm    = 0, // fault checked - fault active
    g_vhoLogStateOff_enm   = 1, // fault checked - fault inactive
    g_vhoLogStateReset_enm = 2  // reset fault and FSM filter variables
} gType_vhoLogState_en;

// rolling direction of the vehicle
typedef enum
{
    WICRawAvg_enm  = 1,
    WICRawLR_enm   = 2,
    WICRawRR_enm   = 3,
    WICRawLF_enm   = 4,
    WICRawRF_enm   = 5,
    SWAng_enm      = 6,
    Gear_enm       = 7,
    YawRate_enm    = 8,
    LongitAcc_enm  = 9,
    WheelVelLR_enm = 10,
    WheelVelRR_enm = 11,
    WheelVelLF_enm = 12,
    WheelVelRF_enm = 13,
    WICState_enm   = 14,
    VehVel_enm     = 15
} gType_VHOCanInputSignals_en;

typedef enum
{
    VHOWicSourceTwoWICRear_enm  = 1,
    VHOWicSourceTwoWICFront_enm = 2,
    VHOWicSourceFourWIC_enm     = 3,
    VHOWicSourceAvgFront_enm    = 4,
    VHOWicSourceAvgRear_enm     = 5
} gType_VHOWicSource_en;

// yaw angle history
typedef struct
{
    SInt16 YAng_psi16[gd_VHOYAngHistRMSize_ui8];
    UInt8 IdMax_ui8;         // max index (length of array)
    UInt8 IdNewest_ui8;      // index of most recent entry
    SInt32 SNewestMMF8_si32; // driven distance at time of most recent entry
                             // (refering to IdNewest)
} gType_VHOYawAngleHist_st;

typedef enum
{
    VHOYAngCalcSingleTrack_enm               = 1,
    VHOYAngCalcDualTrackRearAxle_enm         = 2,
    VHOYAngCalcDualTrackFrontAxle_enm        = 3,
    VHOYAngCalcDualTrackFrontRear_enm        = 4,
    VHOYAngCalcSingleXDualTrackRear_enm      = 5,
    VHOYAngCalcSingleXDualTrackFrontRear_enm = 6,
    VHOYAngCalcSingleXDualTrFront_enm        = 7, // not to be used in customer projects
                                           // name not consistent with others to fit
                                           // ISO:C90 (identifier shall not match other
                                           // identifiers in first 31 characters)
    VHOYAngCalcYawRate_enm = 8
} gType_VHOYAngCalc_en;

typedef struct
{
    /* KINEMATIC QUANTITIES */
    SInt32 XPosition_si32;              // x-position [mm]
    SInt32 YPosition_si32;              // y-position [mm]
    SInt32 DrivenDistance_si32;         // driven distance [mm] - decreases when driving
                                        // backward
    UInt32 DrivenDistanceAbsolute_ui32; // absolute driven distance --> update
                                        // independent of known moving direction
    UInt32 YawAngle_ui32;               // orientation [1/2^22 rad]
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt32 YawAngleDualTrack_ui32;   // angle, calculated by dual track model,
                                     // between inertial system and vehicle
                                     // longitudinal velocity in rad
    UInt32 YawAngleSingleTrack_ui32; // angle, calculated by single track model,
                                     // between inertial system and vehicle
                                     // longitudinal velocity in rad
    UInt32 YawAngleYRSens_ui32;      // angle between inertial system and vehicle
                                     // longitudinal velocity in rad calculated bay using
                                     // YawRate; range [0 ... 2Pi]; ratio: 1.0 rad/2^22
    UInt32 YawAngleYRSensK1_ui32;    // old yaw angle from yaw rate (k-1)
    SInt32 YawAngleCorrection_si32;  // the correction of YawAngle is bounded to
                                     // maximum value

#if (GS_4WS_VEHICLE == SW_ON)
    UInt32 DrivenDistanceAbsoluteInstantCenter_ui32; // absolute driven distance
                                                     // of instantaneous center
                                                     // --> update independent of
                                                     // known moving direction
#endif
#endif

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt16 YawRate_si16; // vehicle yaw rate
#endif
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 VehicleVelocity_ui16; // calculated vehicle velocity [1/256 m/s] - in
                                 // case a standstill has been detected, velocity
                                 // will be set to zero
#endif

    // for other configurations vehicle velocity is replaced by this value
    UInt16 VehicleVelCanMPerS_ui16; // vehicle velocity from CAN in [1/256 m/s]

// UInt16 adiVehicleVelocity_ui16;                      // introduced as
// measurement variable for application
#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt16 VehicleAcceleration_si16; // longitudinal acceleration  [0.01m/s^2]
                                     // calculated from wic
#endif
#endif

    gType_VHORollRecogState_en
        RollingDirection_en; // indicates driving forward / backward

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 StandstillCycleCnt_ui16; // number of cycles that vehicle velocity has
                                    // remained zero
#endif
#endif

    /* CORNERING STATE */
    SInt16 KappaF14_si16; // curvature [1/2^14 m]

    SInt16 SAngRadF12_si16; // steering angle of the virtual front wheel [1/2^12
                            // rad]
#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
    SInt16 SAngRearRadF12_si16; // steering angle of the virtual rear wheel
                                // [1/2^12 rad]
#endif
#endif
#endif

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    /* TYRE RADII */
    UInt16 WICLengthMMF8_pui16[5]; // calculated wic lengths
#endif
#endif

} gType_VHOVehicleState_st;
// result structure for g_VHMUSSProcessingGetDistList_ui8
typedef struct
{
    SInt16 XPositionCM_si16;
    SInt16 YPositionCM_si16;
    SInt16 DrivenDistanceCM_si16;
    UInt16 YawAngleRAD_F10_ui16;
} gType_VHOEstimatedPosition_st;

typedef enum
{
    g_vhoStateInit_enm       = 0, // VHM init
    g_vhoStateUpdated_enm    = 1, // position updated
    g_vhoStateNotUpdated_enm = 2, // position not updated
    g_vhoStateReInit_enm     = 3  // position reinitialized
} gType_vhoUpdateState_en;

typedef struct
{
    SInt32 SPosMM_si32;   // driven distance of the center of rear axle in mm
    SInt32 XPosMM_si32;   // X-Position of the center of rear axle in mm
    SInt32 YPosMM_si32;   // Y-Position of the center of rear axle in mm
    UInt32 YawAngle_ui32; // angle between inertial system and vehicle
                          // longitudinal velocity in rad
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt32 YawAngleYRSens_ui32; // angle between inertial system and vehicle
                                // longitudinal velocity in rad calculated bay using
                                // YawRate; range [0 ... 2Pi]; ratio: 1.0 rad/2^22
#if (GS_4WS_VEHICLE == SW_ON)
    SInt32 SlipAngle_si32;               // angle between vehicle longitudinal axis and
                                         // longitudinal velocity in rad
    SInt32 SPosInstantCenterMM_si32;     // driven distance of the instantaneous
                                         // center of the vehicle in mm
    SInt32 XPosInstantCenterMM_si32;     // X-Position of the instantaneous center of
                                         // the vehicle in mm
    SInt32 YPosInstantCenterMM_si32;     // Y-Position of the instantaneous center of
                                         // the vehicle in mm
    SInt32 WheelBaseVirtualMM_si32;      // virtual wheel base as a result of front
                                         // and rear steering in mm
    SInt32 DeltaWheelBaseVirtualMM_si32; // DeltaWheelBaseVirtual =
                                         // WheelBaseVirtual - WheelBase in  mm
#endif
#endif
    SInt16 WheelAngleFront_si16; // wheel angle at front axle: unit = rad,
                                 // resolution = 1.0 / 2^12
} gType_VehPosOdo_st;

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

typedef struct
{
    gType_VHOWicSource_en WicSource_en;
    gType_VHOYAngCalc_en YawAngCalc_en;
    Bitfield WeightSingleTrackModel_bf3 : 3;
    Bitfield WeightDualTrackModel_bf3 : 3;
    Bitfield WeightDualTrackRearAxle_bf3 : 3;
    Bitfield WeightDualTrackFrontAxle_bf3 : 3;
    Bitfield WeightDistanceFront_bf3 : 3;
    Bitfield WeightDistanceRear_bf3 : 3;
    Boolean ExternalCtrl;
} gType_VHOOdoState_st;

#endif
#endif

// data structure for development messages
#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (LS_VHO_PROVIDE_DEVELOPMENT_DATA == SW_ON)
typedef struct
{
    gType_VHOCalibState_en SWAState_enm;
    gType_VHOCalibState_en YRState_enm;
    SInt16 SWAOff_si16;
    SInt16 YROff_si16;
} gType_VHODevelopmentData_st;
#endif
#endif
#endif

#ifdef COMPONENT_VHO

extern gType_VHOCanSig_st g_VHOCanSig_st;
extern gType_VHORollRecog_st g_VHORollRecog_st;
extern gType_VHOVehicleState_st g_VHOVehicleState_st;
extern gType_VHOVehicleState_st g_VHOVehicleODO_st;
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
extern gType_VHOAutocalib_st g_VHOAutocalib_st;
extern gType_VHOCorrectedSignals_st g_VHOCorrectedSignals_st;
extern gType_VHOParameters_st g_VHOParameters_st;
extern gType_VHOOdoState_st g_VHOOdoState_st;
#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
extern gType_VHOSensorDataUSS_st g_VHOSensorDataUSS_pst[2];
#else
extern gType_VHOSensorDataUSS_st g_VHOSensorDataUSS_pst[4];
#endif

#endif

#endif

extern void g_vhoPreInit_vd(void);
extern void g_vhoInit_vd(void);
extern void g_vhoReInit_vd(SInt32 f_XPosition_si32, /* initial x-Position   [mm]    F0 */
                           SInt32 f_YPosition_si32, /* initial y-Position   [mm]    F0 */
                           UInt32 f_YawAngle_ui32, /* initial yaw angle    [rad]   F22  */
                           UInt8 f_ResetDrivenDistance_ui8);

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
extern void g_vhoResetDiscardedDist_vd(void);
#endif
#endif

extern void g_vhoProcessing_vd(void);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

extern UInt8 g_VHOUSSProcessingGetDistList_ui8(
    UInt8 f_SensorNumber_ui8, UInt16 **f_DistList_ppui16, UInt16 *f_StartTime_pui16,
    SInt16 *f_DrivenDistSinceLastEcho_si16,
    gType_VHOEstimatedPosition_st *f_EstimatedPosition_pst);

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

extern SInt16 g_VHOGetSWAFromKappa_si16(SInt16 f_KappaMF14_si16,
                                        Boolean f_RollingForward_bl);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
extern SInt16 g_VHOGetSARearFromSWA_si16(SInt16 f_SWAdeg25_si16,
                                         SInt16 f_SWAMaxdeg25_si16,
                                         SInt16 f_SARearMaxdeg50_si16);
#endif
#endif

extern SInt16 g_VHOGetKappaFromSWA_si16(SInt16 f_SWARadF11_si16,
                                        Boolean f_RollingForward_bl);

extern SInt16 g_VHOGetRadiusFromSWA_si16(SInt16 f_SWARadF11_si16,
                                         Boolean f_RollingForward_bl);

extern SInt16 g_VHOGetSWAFromRadius_si16(SInt16 f_RadiusCM_si16,
                                         Boolean f_RollingForward_bl);

extern gType_vhoUpdateState_en g_VHOGetUpdateState_en(void);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
extern gType_VHOYawAngleHist_st const *g_vhoGetYAngHist_pst(void);
extern UInt16 g_vhoGetStepsizeYAngHistMM_ui16(void);
#endif

extern SInt16 g_VHOConvertSWA2SA_si16(SInt16 f_SteeringWheelAngle_si16,
                                      Boolean f_RollingForward_bl);

extern SInt16 g_VHOConvertSA2SWA_si16(SInt16 f_SteeringAngle_si16,
                                      Boolean f_RollingForward_bl);

extern void g_VHOSignalPreprocInit_vd(void);

#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if ((LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON) || (LS_VHO_AUTOCALIB_TTC == SW_ON) || \
     (LS_VHO_AUTOCALIB_TYRE_RADII == SW_ON))
extern void g_InitAutocalibFromEEPROM_ui8(
    const gType_VHOParaAutocalib_st *fc_VHOAutocalibData_pst);
#endif
#endif
#endif

void g_vhoLogHdl_vd(gType_vhoLogFaultID_en f_fault_id_enm,
                    gType_vhoLogState_en f_state_enm);

Boolean g_adiGetVehPosOdo_bl(gType_VehPosOdo_st *f_VehPosOdo_pst);

Boolean g_adiGetVehKappaOdo_bl(SInt16 *f_Kappa_psi16);

Boolean g_adiVHODrivenDistanceAbsolute_bl(UInt32 *f_adiVHODrivenDistanceAbsolute_pui32);

/*
Boolean g_adiGetSWA_bl (gType_SWAandTime_st* f_SWA_pst);
Boolean g_adiGetLongAcc_bl (SInt16* f_LongAcc_psi16);
Boolean g_adiGetYawRate_bl (SInt16* f_YawRate_psi16);
*/
Boolean g_adiGetVehicleAcceleration_bl(SInt16 *f_VehAcc_psi16);
void g_adiSetVehPosOdo_bl(gType_FVInputVehOdoFrame_st *f_VehPosOdo_pst);
#ifdef COMPONENT_VHO
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (LS_VHO_PROVIDE_DEVELOPMENT_DATA == SW_ON)
extern Boolean g_adiGetVhoDevelopmentData_bl(
    gType_VHODevelopmentData_st *f_VhoDevelopmentData_pst);
#endif
#endif // belonging to nearest above #if (GS_VHO_RUN_MODE ==
       // GS_VHO_CONSIDER_USS_DATA)
#endif

#endif // #ifndef VHO_API_H
