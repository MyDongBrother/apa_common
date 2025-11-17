/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_H
#define VHO_H

#include "vho_bas.h"
#include "vho_deg.h"
#ifdef COMPONENT_VHO

// maximum size of position buffer ring memory
#define gd_VHOMaxWICHistUSSRMSize_ui8 ((UInt8)10)

// a position history of 100ms shall be stored
// required for determination of size of ring memory
#define gd_VHOWICHistUSSRMSizeMS_ui8 ((UInt8)100)

// If every WIC does not change since MinTimeBeforeSTOPMS
// the possibility is very high that vehicle stands still
// a good value even in the worst case car B-Class is 350 ms.
#define gd_VHOMinTimeBeforeSTOPMS_ui16 ((UInt16)350)

typedef struct
{
    UInt32 YawAngleRaw_ui32;

    // for the variables XPosition_si32, YPosition_si32 and DrivenDistance_si32 in
    // g_VHOVehicleState_st, the following variables contain the corresponding
    // high resolution part for values smaller than 1mm (range: 0mm - 255/256mm),
    // e.g. x = 1.5mm => g_VHOVehicleState_st.XPosition_si32 = 1;
    //              g_VHOOdoBuf_st.XPosition_bufF8_si16 = 128;
    SInt16 XPosition_bufF8_si16;
    SInt16 YPosition_bufF8_si16;
    SInt16 DrivenDistance_bufF8_si16;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    // the following variables contain values OF THE LAST CALL CYCLE
    UInt16 WIC_CanTime_buf_ui16; // wic can time of the last call
    SInt16 SWAngDegRes_buf_si16; // steering wheel angle of the last call
#if (GS_4WS_VEHICLE == SW_ON)
    SInt16 SAngRearDegRes_buf_si16; // rear steering angle of the last call
#endif
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 WICBuf_pui16[5]; // wic values of the last call
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    UInt16 WICBuf_pui16[2]; // wic values of the last call
#endif

// the following variables are used DURING A VHO CALL CYCLE to store values that
// are required more than once in the cycle
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 WICDelta_pui16[5]; // wic increments between last and current cycle
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    UInt16 WICDelta_pui16[2]; // wic increments between last and current cycle
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    UInt16 WheelVelokmh_pui16[4];  // normed wheel velocities [0.01kmh]
    UInt16 WICCanTimeDeltaMS_ui16; // time difference between current and last
                                   // wic timestamp
    UInt8 SWARMSize_ui8;           // used elements of SWA ring memory (depending on
                                   // parametrization)
    SInt8 SWARMLostDistMM_si8;     // not considered distance in last cycle for
                                   // filling SWA ring memory
#endif

    SInt32 DrivenDistanceMMF8_si32; // driven distance of current cycle
} gType_VHOOdoBuf_st;

//
// STRUCTURE WITH VARIABLES THAT ARE NEEDED FOR CONTROL PURPOSES
// LIKE FLAGS, COUNTERS,...
//
typedef struct
{

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    gType_vhoDegModes_en DegMode_en;           // degredation mode that is currently set
    SInt32 DistPassedDegModeExtrapolMMF8_si32; // distance that vehicle passed
                                               // during deg mode extrapolation
    UInt16 StarttimeDegModeExtrapolMS_ui16;    // starttime for degredation mode
                                               // extrapolation
    Boolean DegModeExtrapolForceExtrapol_bl;   // in case of invalid WIC values or
                                               // timeout within mode extrapolation
                                               // VHO shall run with simulated
                                               // values based on the assumption of
                                               // uniform velocity

    UInt8 InitialiseWIC_ui8; // flag will be set to 1 by m_VHOInit_vd
                             // ( => init mode)
    Boolean WICChanged_bl;   // indicates whether wic has been updated in the
                             // current cycle
    UInt16 StandstillDetectionTimeMS_ui16;
    // contains the time that the vehicle remained in standstill
    Boolean CrdUpdCrv_bl; // indicates whether straight line (FALSE) or circle arc
                          // approximation (TRUE) has been used for coordinate
                          // update in the last cycle

#endif

    UInt32 VHOCallCycle_ui32;      // contains the total number of calls since
                                   // initialization
    UInt32 VHOTotalRuntimeMS_ui32; // total time since initialization

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    Boolean VHOBypass_bl;
#endif

} gType_VHOCtrlStat_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

typedef struct
{
    SInt32 XPositionMM_si32;
    SInt32 YPositionMM_si32;
    SInt32 DrivenDistanceMM_si32;
    UInt32 YawAngleRAD_F22_ui32;
    UInt16 WIC_CanTime_ui16;
} gType_VHOPosBuffer_st;

// structure representing the current auto calibration state
typedef struct
{
    Boolean StateChangedInCurrentCycle_bl;

    gType_VHOCalibState_en SWAState_enm;
    UInt8 NumStableSWACycles_ui8;

    gType_VHOCalibState_en TyreRadState_enm;
    UInt8 NumStableTyreRadCycles_ui8;

    gType_VHOCalibState_en TTCState_enm;
    UInt8 NumStableTTCCycles_ui8;

    gType_VHOCalibState_en YRState_enm;

    gType_VHOCalibState_en AxleBaseState_enm;
    UInt8 NumStableAxleBaseCycles_ui8;

    SInt16 SWOffDegRes_si16;
    SInt16 YROff_si16;
    UInt16 VeloScalFac_ui16;
    UInt16 AvgWICLengthRearAxle_ui16;
    UInt16 ScalFacRR_ui16;
    UInt16 ScalFacLF_ui16;
    UInt16 ScalFacRF_ui16;
    UInt16 AxleBase_ui16;
} gType_VHOAutocalibCtrl_st;

// position history for USS position mapping
typedef struct
{
    UInt8 IdMax_ui8;     // maximum number of currently stored values
    UInt8 IdNewest_ui8;  // ID of newest entry
    UInt8 IdOldest_ui8;  // ID of oldest entry
    UInt8 PosRMSize_ui8; // size of position buffer ring memory
    UInt16 WIC_CanTime_pui16[gd_VHOMaxWICHistUSSRMSize_ui8];      // in case of a new
                                                                  // WIC message
    SInt32 XPositionMM_psi32[gd_VHOMaxWICHistUSSRMSize_ui8];      // WIC_CanTime,
                                                                  // XPosition,
    SInt32 YPositionMM_psi32[gd_VHOMaxWICHistUSSRMSize_ui8];      // YPosition,
                                                                  // DrivenDistance and
    SInt32 DrivenDistanceMM_psi32[gd_VHOMaxWICHistUSSRMSize_ui8]; // YawAngle will
                                                                  // be updated in
    UInt32 YawAngleRad_F22_pui32[gd_VHOMaxWICHistUSSRMSize_ui8];  // position buffer
                                                                  // ring memory
} gType_VHOWICHistoryUSS_st;

#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*----------------------------------------------------------------*/
/*- external objects declarations                                -*/
/*----------------------------------------------------------------*/
extern gType_VHOOdoBuf_st g_VHOOdoBuf_st;
extern gType_VHOCtrlStat_st g_VHOCtrlStat_st;
extern gType_VHOWheelBaseVirtual_st g_VHOWheelBaseVirtual_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
extern gType_VHOAutocalibCtrl_st g_VHOAutocalibCtrl_st;
extern gType_VHOPosBuffer_st g_VHOPosBuffer_st;
extern gType_VHOWICHistoryUSS_st g_VHOWICHistoryUSS_st;
#endif

/*----------------------------------------------------------------*/
/*- prototypes of exported functions                             -*/
/*----------------------------------------------------------------*/
void g_VHOCalculateNewPosition_vd(void);

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
void g_VHOBufferSWAandSAR_vd(SInt16 f_CurrentSWA_si16, SInt16 f_CurrentSAR_si16);
void g_VHOCalcVirtualWheelBase_vd(void);
SInt16 g_VHOGetSWAFromBuf_si16(void);
SInt16 g_VHOGetSARFromBuf_si16(void);
#else
void g_VHOBufferSWA_vd(SInt16 f_CurrentSWA_si16);
SInt16 g_VHOGetSWAFromBuf_si16(void);
#endif
#endif

#endif // COMPONENT_VHO

#endif // VHO_H
