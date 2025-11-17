/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: odo_external_adi.h
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/
#ifndef ODO_EXTERNAL_API_H
#define ODO_EXTERNAL_API_H
/*--------------------------------------------------------------------------*/
/*- include files                                                           -*/
/*--------------------------------------------------------------------------*/
#include <pa_components/base_type/global_include.h>
/*--------------------------------------------------------------------------*/
/*- MACRO DECLARE                                                          -*/
/*--------------------------------------------------------------------------*/

#define gd_SENSOR1               1  // front left
#define gd_SENSOR6               6  // front right
#define gd_SENSOR8               8  // rear right
#define gd_SENSOR13              13 // rear left
#define gd_VHMYAngHistRMSize_ui8 ((UInt8)20)

/*--------------------------------------------------------------------------*/
/*- TYPE DECLARE                                                           -*/
/*--------------------------------------------------------------------------*/

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
#if (GS_VHM_RUN_MODE == GS_VHM_CONSIDER_USS_DATA)
    UInt32 YawAngleYRSens_ui32; // angle between inertial system and vehicle
                                // longitudinal velocity in rad calculated bay using
                                // YawRate; range [0 ... 2Pi]; ratio: 1.0 rad/2^22

#endif
    SInt16 WheelAngleFront_si16; // wheel angle at front axle: unit = rad,
                                 // resolution = 1.0 / 2^12
} gType_VehPosOdo_st;

extern gType_VehPosOdo_st m_VehPosOdo_external_st;
extern gType_vhoUpdateState_en m_vhmUpdateState_external_en;
extern gType_vhoUpdateState_en m_VHoGetUpdateState_external_en(void);
extern Boolean m_adiGetVehPosOdo_external_bl(gType_VehPosOdo_st *f_VehPosOdo_pst);
extern void m_odoExternal_processing_vd(void);
#endif