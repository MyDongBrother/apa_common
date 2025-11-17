/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: vho.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_VHO

#include "vho.h"
#include "vho_api.h"
#include "data_io/dataInput_adi.h"

#include <pa_components/base_type/global_include.h>
#include <pa_components/mathlib/mathlib_api.h>

#if (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
#include "vho_mini.h" // SMS Include Structure: ok
#endif

#include "vho_dir_recog.h" // SWCPA Include Structure: ok

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#include "vho_kin.h"      // SWCPA Include Structure: ok
#include "vho_loghdl.h"   // SWCPA Include Structure: ok
#include "vho_odo_core.h" // SWCPA Include Structure: ok
#include "vho_sig_proc.h" // SWCPA Include Structure: ok

#endif

#include <string.h> // SWCPA Include Structure: ok
// #include <mai_api.h>

#ifndef MATLAB
// #include <vsm_api.h>
#include <veh/vhs/vhs_api.h>
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
// #include <mp_api.h>
//  #include <uss/mp_if/mpif_api.h>
#endif
#endif // MATLAB

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#include "vho_uss_proc.h" // SWCPA Include Structure: ok
#endif

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3892
#endif

#ifndef VISIBILITY
#error ('compiler switch VISIBILITY is not defined')
#endif

#ifndef GS_4WS_VEHICLE
#error ('compiler switch GS_4WS_VEHICLE is not defined')
#endif

#ifndef GS_VHO_MINI_VHO
#error ('compiler switch GS_VHO_RUN_MODE is not defined')
#endif
#ifndef GS_VHO_CONSIDER_USS_DATA
#error ('compiler switch GS_VHO_CONSIDER_USS_DATA is not defined')
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#ifndef LS_VHO_EXTRAPOL_ENGINE_RESTART
#error ('compiler switch LS_VHO_EXTRAPOL_ENGINE_RESTART is not defined')
#endif
#endif

#if (GS_VHS_WICRL != SW_ON)
#error ('GS_VHS_WICRL has to be set to ON if VHO is compiled')
#endif

#if (GS_VHS_WICRR != SW_ON)
#error ('GS_VHS_WICRR has to be set to ON if VHO is compiled')
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#ifndef LS_VHO_USSMAP_REAR_SENSOR_ACTIVE
#error ('compiler switch LS_VHO_USSMAP_REAR_SENSOR_ACTIVE is not defined')
#endif
#endif

#define md_VHOWicInvalid_ui16 ((UInt16)0)

#define PSD_USS_MASK      ((UInt16)0x0021) // 0b0000_0000_0010_0001
#define PSD_USS1_MASK     ((UInt16)0x0001) // 0b0000_0000_0000_0001
#define PSD_USS6_MASK     ((UInt16)0x0020) // 0b0000_0000_0010_0000
#define PSD_USS8_MASK     ((UInt16)0x0080) // 0b0000_0000_1000_0000
#define PSD_USS13_MASK    ((UInt16)0x1000) // 0b0001_0000_0000_0000
#define PSD_USS25_DE_MASK ((UInt16)0x0012) // 0b0000_0000_0001_0010
#define PSD_USS25_CE_MASK ((UInt16)0x0022) // 0b0000_0000_0010_0010
#define PDC_PSX_MODE
#define md_RM_SIZE_SWA_BUFF   16
#define md_RM_ARRAY_SIZE_BUFF 2
#define md_RM_SWA_ARRAY_BUFF  0
#define md_RM_SAR_ARRAY_BUFF  1

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
static SInt16 m_RMSWAandSARBuff_psi16[md_RM_ARRAY_SIZE_BUFF][md_RM_SIZE_SWA_BUFF];
#else
static SInt16 m_RMSWABuff_psi16[md_RM_SIZE_SWA_BUFF];
#endif
static UInt8 m_RMSWABuffLastElem_ui8;
static UInt8 m_RMSWABuffOldestElem_ui8;
static gType_VHOYawAngleHist_st m_VHOYawAngleHist_st;
static Boolean m_VHOReInitState_bl;
#endif

static gType_vhoUpdateState_en m_VHOUpdateState_enm;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if ((GS_VHS_WHEEL_DIR_RL == SW_ON) || (GS_VHS_WHEEL_DIR_RR == SW_ON) || \
     (GS_VHS_WHEEL_DIR_FL == SW_ON) || (GS_VHS_WHEEL_DIR_FR == SW_ON))

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))

static UInt8 m_WICLengthMM_ui8;
static UInt32 m_DrivDistAbsLastStopMM_ui32;

#endif /*   #if ( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
                  (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
                  (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL) ) */

#endif /* #if ( (GS_VHS_WHEEL_DIR_RL == SW_ON) || (GS_VHS_WHEEL_DIR_RR == \
          SW_ON) ||                                                       \
                (GS_VHS_WHEEL_DIR_FL == SW_ON) || (GS_VHS_WHEEL_DIR_FR == \
          SW_ON) ) */

#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

#if ((GS_VHS_WHEEL_DIR_RL == SW_ON) || (GS_VHS_WHEEL_DIR_RR == SW_ON))

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))

static UInt8 m_WICLengthMM_ui8;
static UInt32 m_DrivDistAbsLastStopMM_ui32;

#endif /*   #if ( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
                  (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
                  (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL) )  */

#endif // #if ( (GS_VHS_WHEEL_DIR_RL == SW_ON) || (GS_VHS_WHEEL_DIR_RR == SW_ON)
       // )

#endif

/*----------------------------------------------------------------*/
/*- prototypes of local functions                                -*/
/*----------------------------------------------------------------*/

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY void m_VHOInitWIC_vd(void);

#if (GS_4WS_VEHICLE == SW_ON)
VISIBILITY void m_VHOInitBufferSWAandSAR_vd(SInt16 f_CurrentSWA_si16,
                                            SInt16 f_CurrentSAR_si16);
#else
VISIBILITY void m_VHOInitBufferSWA_vd(SInt16 f_CurrentSWA_si16);
#endif

#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
VISIBILITY void m_vhoDegModeHdl_vd(void);
#endif

VISIBILITY void m_VHOInputConversion_vd(void);
// VISIBILITY void m_SetUSSMaskAndStoreData_vd(void);
#endif // (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

VISIBILITY void m_VHOInit_vd(
    SInt32 f_XPosition_si32,         /* initial x-Position   [mm]    F0          */
    SInt32 f_YPosition_si32,         /* initial y-Position   [mm]    F0          */
    UInt32 f_YawAngle_ui32,          /* initial yaw angle    [rad]   F22         */
    UInt8 f_ResetDrivenDistance_ui8, /* 1 => distance will be reset to zero */
    UInt8 f_InitWIC_ui8              /* 1 => difference between current and buf- */
                                     /* fered wic will not be considered in the  */
                                     /* next cycle (buffer initialisation first) */
);

VISIBILITY void m_VHOGetSignals_vd(gType_VHOCanSig_st *f_VHOCanSig_pst);

#if ((GS_VHS_WHEEL_DIR_RR == SW_ON) || (GS_VHS_WHEEL_DIR_RL == SW_ON) || \
     (GS_VHS_WHEEL_DIR_FR == SW_ON) || (GS_VHS_WHEEL_DIR_FL == SW_ON))
#if (((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||      \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC)) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
VISIBILITY gType_VHORollRecogState_en
m_VHONormVHSWicMoveDir_en(gType_WheelDir_en f_VHSWicMoveDir_en);
#endif
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
VISIBILITY void m_VHOInitBufferYAngHist_vd(void);
VISIBILITY void m_VHOReInitBufferYAngHist_vd(UInt32 f_CurrentYAng_ui32,
                                             UInt32 f_OdoBufYawAngle_ui32);
VISIBILITY void m_VHOWriteYAngHist_vd(UInt32 f_CurrentYAng_ui32);

VISIBILITY void m_VHOInitBufferWICHistUSS_vd(void);
VISIBILITY void m_VHOWriteWICHistUSS_vd(void);

// Will be removed as soon as available in Mathlib
VISIBILITY SInt16 g_vho_mtl_AngleF22ToF12_si16(UInt32 f_AngleIn_ui32);
VISIBILITY UInt32 g_vho_mtl_AngleSub_ui32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32);
VISIBILITY UInt32 g_vho_mtl_AngleF12ToF22_ui32(SInt16 f_AngleIn_si16);
#endif

/*----------------------------------------------------------------*/
/*- global data objects                                          -*/
/*----------------------------------------------------------------*/
gType_VHOCanSig_st g_VHOCanSig_st;
gType_VHOVehicleState_st g_VHOVehicleState_st;
gType_VHOVehicleState_st g_VHOVehicleODO_st;
gType_VHOOdoBuf_st g_VHOOdoBuf_st;
gType_VHOCtrlStat_st g_VHOCtrlStat_st;
gType_VHORollRecog_st g_VHORollRecog_st;
gType_VHOWheelBaseVirtual_st g_VHOWheelBaseVirtual_st;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
gType_VHOCorrectedSignals_st g_VHOCorrectedSignals_st;
gType_VHOAutocalib_st g_VHOAutocalib_st;
gType_VHOAutocalibCtrl_st g_VHOAutocalibCtrl_st;
gType_VHOParameters_st g_VHOParameters_st;
gType_VHOOdoState_st g_VHOOdoState_st;
#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
gType_VHOSensorDataUSS_st g_VHOSensorDataUSS_pst[2];
#else
gType_VHOSensorDataUSS_st g_VHOSensorDataUSS_pst[4];
#endif
gType_VHOPosBuffer_st g_VHOPosBuffer_st;
gType_VHOWICHistoryUSS_st g_VHOWICHistoryUSS_st;
#endif

/*----------------------------------------------------------------*/
/*- definition of exported functions                             -*/
/*----------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- function implementation                                                -*/
/*--------------------------------------------------------------------------*/
/**
 * @brief
 * @details
 *
 * @param         f_Angle1_ui32
 * @param         f_Angle2_ui32
 * @return        UInt32
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

VISIBILITY UInt32 g_vho_mtl_AngleSub_ui32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32)
{
    UInt32 l_Returnangle_ui32;

    if (f_Angle1_ui32 >= f_Angle2_ui32)
    {
        l_Returnangle_ui32 = f_Angle1_ui32 - f_Angle2_ui32;
    }
    else
    {
        l_Returnangle_ui32 = gd_MTL_2_PI_ui32 - (f_Angle2_ui32 - f_Angle1_ui32);
    }

    if (l_Returnangle_ui32 == gd_MTL_2_PI_ui32)
    {
        l_Returnangle_ui32 = 0;
    }

    return l_Returnangle_ui32;
}

/**
 * @brief
 * @details
 *
 * @param         f_AngleIn_ui32
 * @return        SInt16
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY SInt16 g_vho_mtl_AngleF22ToF12_si16(UInt32 f_AngleIn_ui32)
{
    SInt16 l_ReturnAngle_si16;

    if (f_AngleIn_ui32 < gd_MTL_PI_ui32)
    {
        l_ReturnAngle_si16 = (SInt16)(f_AngleIn_ui32 >> 10);
    }
    else
    {
        f_AngleIn_ui32     = g_vho_mtl_AngleSub_ui32(gd_MTL_2_PI_ui32, f_AngleIn_ui32);
        l_ReturnAngle_si16 = -(SInt16)(f_AngleIn_ui32 >> 10);
    }

    return l_ReturnAngle_si16;
}

/**
 * @brief
 * @details
 *
 * @param         f_AngleIn_si16
 * @return        UInt32
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY UInt32 g_vho_mtl_AngleF12ToF22_ui32(SInt16 f_AngleIn_si16)
{
    UInt32 l_ReturnAngle_ui32;

    if (f_AngleIn_si16 > 0)
    {
        l_ReturnAngle_ui32 = (UInt32)f_AngleIn_si16 << 10;
    }
    else
    {
        l_ReturnAngle_ui32 = (UInt32)g_mtl_Abs_mac(f_AngleIn_si16) << 10;
        l_ReturnAngle_ui32 =
            g_vho_mtl_AngleSub_ui32(gd_MTL_2_PI_ui32, l_ReturnAngle_ui32);
    }

    return l_ReturnAngle_ui32;
}
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
******************************************************************************/
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
void g_vhoPreInit_vd(void) { g_vhoPreInitAppl_vd(); }

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         Initialisation function for the VHO odometry.
 *                MUST be called before normal position calculation starts
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoInit_vd(void)
{
    m_VHOInit_vd(0, /* initial x-Position   [mm]    F0          */
                 0, /* initial y-Position   [mm]    F0          */
                 0, /* initial yaw angle    [rad]   F22         */
                 1, /* 1 => distance will be reset to zero      */
                 1  /* 1 => difference between current and buf- */
                    /* fered wic will not be considered in the  */
                    /* next cycle (buffer initialisation first) */
    );

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    g_vhoSetDegMode_vd(DegMode_VHO_NoDegradation_enm);
#endif

    // call project specific initialization steps
    g_vhoInitAppl_vd();

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    // initialize virtual wheel base
    g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 =
        g_VHOParameters_st.VHO_Veh_pst->WheelBase_ui16;
    g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 = ((SInt16)0);
    g_VHOWheelBaseVirtual_st.ForbiddenRearAngleZone_bl  = FALSE;

#if (GS_4WS_VEHICLE == SW_ON)
    g_VHOCanSig_st.SAR_ValidFlag_bl = TRUE;
    g_VHOCanSig_st.SAR_si16         = ((SInt16)0);
#endif
#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         Reset function for the VHO odometry.
 *                MAY be called to set the VHO to defined coordinates during
 *                cyclic operation
 * @details
 *
 * @param         f_XPosition_si32
 * @param         f_YPosition_si32
 * @param         f_YawAngle_ui32
 * @param         f_ResetDrivenDistance_ui8
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoReInit_vd(SInt32 f_XPosition_si32, SInt32 f_YPosition_si32,
                    UInt32 f_YawAngle_ui32, UInt8 f_ResetDrivenDistance_ui8)
{
    m_VHOInit_vd(f_XPosition_si32,          /* initial x-Position   [mm]    F0          */
                 f_YPosition_si32,          /* initial y-Position   [mm]    F0          */
                 f_YawAngle_ui32,           /* initial yaw angle    [rad]   F22         */
                 f_ResetDrivenDistance_ui8, /* 1 => distance will be reset to zero      */
                 0                          /* 1 => difference between current and buf- */
                                            /* fered wic will not be considered in the  */
                                            /* next cycle (buffer initialisation first) */
    );

    // call project specific initialization steps
    g_vhoReInitAppl_vd();
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         Initialisation function for the VHO odometry.
 * @details       MUST be called before normal position calculation starts
 *                MAY be called to set the VHO to defined coordinates during
 *                cyclic operation
 *
 * @param         f_XPosition_si32              x position  (mm)
 * @param         f_YPosition_si32              y position  (mm)
 * @param         f_YawAngle_ui32               yaw angle   (1/2^22 rad)
 * @param         f_ResetDrivenDistance_ui8     if this flag is set to 1, the
 * variable g_VHOVehicleState_st.DrivenDistance_si32 will be reset to zero
 * @param         f_InitWIC_ui8                 this flag MUST be set to 1 at
 * system startup; the wheel impulse buffers will be set to an initial value; as
 * long as the wheel impulse buffers contain this initial value the VHO remains
 * in init mode and quits immediately after the call of
 * g_VHOCalculateNewPosition_vd(); furthermore internal and external variables
 * of all modules will be set to a defined state
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOInit_vd(SInt32 f_XPosition_si32, SInt32 f_YPosition_si32,
                             UInt32 f_YawAngle_ui32, UInt8 f_ResetDrivenDistance_ui8,
                             UInt8 f_InitWIC_ui8)
{

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    const UInt32 l_OdoBufYawAngle_ui32 = g_VHOVehicleState_st.YawAngle_ui32;
#endif

    //
    // SET VEHICLE POSITION ACCORDING TO THE REQUESTED STATE
    //
    g_VHOVehicleState_st.XPosition_si32 = f_XPosition_si32;
    g_VHOVehicleState_st.YPosition_si32 = f_YPosition_si32;
    g_VHOVehicleState_st.YawAngle_ui32  = f_YawAngle_ui32;
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    g_VHOVehicleState_st.YawAngleDualTrack_ui32   = f_YawAngle_ui32;
    g_VHOVehicleState_st.YawAngleSingleTrack_ui32 = f_YawAngle_ui32;
    g_VHOVehicleState_st.YawAngleYRSens_ui32      = f_YawAngle_ui32;
#endif
    g_VHOOdoBuf_st.YawAngleRaw_ui32     = f_YawAngle_ui32;
    g_VHOOdoBuf_st.XPosition_bufF8_si16 = (SInt16)0;
    g_VHOOdoBuf_st.YPosition_bufF8_si16 = (SInt16)0;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    // initialize position buffer
    g_VHOPosBuffer_st.XPositionMM_si32     = f_XPosition_si32;
    g_VHOPosBuffer_st.YPositionMM_si32     = f_YPosition_si32;
    g_VHOPosBuffer_st.YawAngleRAD_F22_ui32 = f_YawAngle_ui32;
#endif

    if (f_InitWIC_ui8 == (UInt8)1)
    {
        //
        //  SYSTEM STARTUP => ENTER INIT MODE
        //

        // odo state will be set automatically per default

        m_VHOUpdateState_enm = g_vhoStateInit_enm;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        m_VHOReInitState_bl           = FALSE;
        g_VHOOdoState_st.ExternalCtrl = FALSE;
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        g_vhoInitParamsAppl_vd(&g_VHOParameters_st);
#endif

        (void)memset(&g_VHOCanSig_st, 0, sizeof(gType_VHOCanSig_st));

        g_VHOCanSig_st.WICRaw_pui16[LeftRear_enm]  = md_VHOWicInvalid_ui16;
        g_VHOCanSig_st.WICRaw_pui16[RightRear_enm] = md_VHOWicInvalid_ui16;

        g_VHOCanSig_st.WICState_pen[LeftRear_enm]  = VHO_MOV_UNKNOWN_enm;
        g_VHOCanSig_st.WICState_pen[RightRear_enm] = VHO_MOV_UNKNOWN_enm;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

        g_VHOCanSig_st.WICRaw_pui16[AvgLeftRight_enm] = md_VHOWicInvalid_ui16;
        g_VHOCanSig_st.WICRaw_pui16[LeftFront_enm]    = md_VHOWicInvalid_ui16;
        g_VHOCanSig_st.WICRaw_pui16[RightFront_enm]   = md_VHOWicInvalid_ui16;

        g_VHOCanSig_st.WICState_pen[AvgLeftRight_enm] = VHO_MOV_UNKNOWN_enm;
        g_VHOCanSig_st.WICState_pen[LeftFront_enm]    = VHO_MOV_UNKNOWN_enm;
        g_VHOCanSig_st.WICState_pen[RightFront_enm]   = VHO_MOV_UNKNOWN_enm;
#endif

        // set vehicle state into a defined state
        (void)memset(&g_VHOVehicleState_st, 0, sizeof(gType_VHOVehicleState_st));
        g_VHOVehicleState_st.RollingDirection_en = VHO_STOP_enm;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

        g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftRear_enm] =
            g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

        g_VHOVehicleState_st.WICLengthMMF8_pui16[RightRear_enm] =
            g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

        g_VHOVehicleState_st.WICLengthMMF8_pui16[LeftFront_enm] =
            g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

        g_VHOVehicleState_st.WICLengthMMF8_pui16[RightFront_enm] =
            g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

        g_VHOVehicleState_st.WICLengthMMF8_pui16[AvgLeftRight_enm] =
            g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16;

        (void)memset(&g_VHOCtrlStat_st, 0, sizeof(gType_VHOCtrlStat_st));
        g_VHOCtrlStat_st.InitialiseWIC_ui8 = (UInt8)1;
        g_VHOCtrlStat_st.VHOBypass_bl      = FALSE;

        (void)memset(&g_VHOCorrectedSignals_st, 0, sizeof(gType_VHOCorrectedSignals_st));

#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

        (void)memset(&g_VHOOdoBuf_st, 0, sizeof(gType_VHOOdoBuf_st));

        g_VHOOdoBuf_st.WICBuf_pui16[LeftRear_enm]  = md_VHOWicInvalid_ui16;
        g_VHOOdoBuf_st.WICBuf_pui16[RightRear_enm] = md_VHOWicInvalid_ui16;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        g_VHOOdoBuf_st.WICBuf_pui16[LeftFront_enm]  = md_VHOWicInvalid_ui16;
        g_VHOOdoBuf_st.WICBuf_pui16[RightFront_enm] = md_VHOWicInvalid_ui16;
#endif

        // initialize sub modules
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        g_VHOUSSProcessingInit_vd();
#endif
        g_VHODirRecogInit_vd();

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        g_VHOSignalPreprocInit_vd();
        g_VHOOdoSetup_vd();
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))
        g_VHOKinInit_vd();
#endif
        g_vhoLogHdlInit_vd();

#if (GS_4WS_VEHICLE == SW_ON)
        m_VHOInitBufferSWAandSAR_vd(g_VHOCanSig_st.SWA_si16, g_VHOCanSig_st.SAR_si16);
#else
        m_VHOInitBufferSWA_vd(g_VHOCanSig_st.SWA_si16);
#endif
        m_VHOInitBufferWICHistUSS_vd();
        m_VHOInitBufferYAngHist_vd();
#endif
    }
    else
    {
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        m_VHOReInitBufferYAngHist_vd(f_YawAngle_ui32, l_OdoBufYawAngle_ui32);
        m_VHOInitBufferWICHistUSS_vd();
        m_VHOReInitState_bl = TRUE;
#endif
        m_VHOUpdateState_enm = g_vhoStateReInit_enm;
    }

    if (f_ResetDrivenDistance_ui8 == (UInt8)1)
    {
        // set driven distance to zero
        g_VHOVehicleState_st.DrivenDistance_si32 = (SInt32)0;
        g_VHOOdoBuf_st.DrivenDistance_bufF8_si16 = (SInt16)0;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
        g_VHOPosBuffer_st.DrivenDistanceMM_si32 = (SInt32)0;
#endif
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief
 * @details       external reset of the discarded distance
 *                discarded distance is the cumulative distance between stop
 *                state with unknown moving direction (STOP-UNKNOWN-STOP)
 *                --> should be called after parking is finished
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
void g_vhoResetDiscardedDist_vd(void)
{
    g_VHORollRecog_st.s_discarded_MMF8_ui32 = (UInt32)0;
}
#endif

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details       main function to invoke one vho call cycle; this function has
 *                to be called periodically and is performing three steps:
 *                 x get up-to-date signals
 *                 x update internal variables
 *                 x perform one calculation cycle
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoProcessing_vd(void)
{

// the following function request is necessary in order to meassure the vho run
// time
#if (GS_PP_RUNTIME_MEASUREMENT_FX4L == SW_ON)
    isystem_profile_func_entry(
        g_vhoProcessing_vd); // intrumentation for run time measurement
#endif

// perform project specific actions before the execution of the
// standard procedure begins
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    g_vhoProcessingPreAppl_vd();
#endif

    // get signals from rte
    m_VHOGetSignals_vd(&g_VHOCanSig_st);
    // get signals from rte (project specific signals)
    g_VHOGetSignalsAppl_vd(&g_VHOCanSig_st);

    g_VHOCalculateNewPosition_vd();

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    // m_SetUSSMaskAndStoreData_vd();
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    // fault handling for degredation mode extrapolation
    m_vhoDegModeHdl_vd();
#endif

    // call FSV error reporting
    g_vhoLogHdlReportToFSV_vd();

#endif

    // perform project specific actions after the execution of the
    // standard procedure
    g_vhoProcessingPostAppl_vd();

// exit vho run time meassurement, if it was set active
#if (GS_PP_RUNTIME_MEASUREMENT_FX4L == SW_ON)
    isystem_profile_func_exit(); // intrumentation for measurement
#endif
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         function that VHOLog_ExtraPolPosFault_enm
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
#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
VISIBILITY void m_vhoDegModeHdl_vd(void)
{
#if (GS_VHS_ENGINE_START == SW_ON)
    Boolean l_RetVal_bl;
    gType_EngineStart_en l_EngineStart_en;

    l_RetVal_bl = g_adiGetEngineStart_bl(&l_EngineStart_en);
    if ((l_RetVal_bl != FALSE) && (l_EngineStart_en == EngineNoStart_enm))
    {
        // reset FSV error if restart condition is not present any more
        g_vhoLogHdl_vd(VHOLog_ExtraPolPosFault_enm, g_vhoLogStateOff_enm);
    }

    // handling of degredation mode extrapolation
    if (g_VHOCtrlStat_st.DegMode_en == DegMode_VHO_Extrapolation_enm)
    {
        // error has not been issued yet. if it has been issued already no further
        // action has to be taken
        UInt8 l_DistCM_ui8;
        UInt16 l_T_ui16;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3212, 3417, 3426
#endif

        // get time since activation of degredation mode
        // g_BswOs1msTimeCounter_en(&l_T_ui16);
        l_T_ui16 = g_GetTimeMs_ui16();

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3212, 3417, 3426
#endif

        l_T_ui16 -= g_VHOCtrlStat_st.StarttimeDegModeExtrapolMS_ui16;

        // scale passed distance 1/256mm => cm
        l_DistCM_ui8 =
            (UInt8)((((UInt32)g_VHOCtrlStat_st.DistPassedDegModeExtrapolMMF8_si32) >> 8) /
                    (UInt32)10);

        if ((l_DistCM_ui8 >=
             g_VHOParameters_st.VHO_Odo_pst->EngineRestartExtrapolationDistance_ui8) ||
            (l_T_ui16 >=
             g_VHOParameters_st.VHO_Odo_pst->EngineRestartExtrapolationTimeout_ui16))
        {
            g_vhoLogHdl_vd(VHOLog_ExtraPolPosFault_enm, g_vhoLogStateOn_enm);
        }
    }
#endif
}
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         set vho degredation mode
 * @details
 *
 * @param         f_vhoDegMode_en  deg mode that shall be set
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_vhoSetDegMode_vd(gType_vhoDegModes_en f_vhoDegMode_en)
{
#if 0
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
  switch(f_vhoDegMode_en)
  {
  case DegMode_VHO_NoDegradation_enm:
      // reset structures for setting of FSV error FsvVhoExtraPolPosFault_enm
      g_VHOCtrlStat_st.DistPassedDegModeExtrapolMMF8_si32    = (SInt32)   0;
      g_VHOCtrlStat_st.StarttimeDegModeExtrapolMS_ui16       = (UInt16)   0;

      // set degredation mode according to trigger
      g_VHOCtrlStat_st.DegMode_en = f_vhoDegMode_en;
      break;

#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
  case DegMode_VHO_Extrapolation_enm:
      if (g_VHOCtrlStat_st.DegMode_en == DegMode_VHO_NoDegradation_enm)
      {
        UInt16 l_T_ui16;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3212, 3417, 3426
#endif

        // get current OS time
       // g_BswOs1msTimeCounter_en(&l_T_ui16);
        l_T_ui16 = g_GetTimeMs_ui16();

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3212, 3417, 3426
#endif

        // switch to deg mode extrapolation
        g_VHOCtrlStat_st.StarttimeDegModeExtrapolMS_ui16 = l_T_ui16;
      }
      g_VHOCtrlStat_st.DegMode_en = f_vhoDegMode_en;
      break;
#endif

  default:
      // set error due to invalid setting for the degradation mode
      // note: to be able to recognize the error, m_vhoLogFaultArray_un
      // has to be monitored
      g_vhoLogHdl_vd(VHOLog_VHOMain_enm,g_vhoLogStateOn_enm);
      g_VHOCtrlStat_st.DegMode_en = DegMode_VHO_NoDegradation_enm;
      break;
  }
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#endif
    g_VHOCtrlStat_st.DegMode_en = DegMode_VHO_NoDegradation_enm;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         set USS filter mask and store incoming data
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
#if 0
VISIBILITY void m_SetUSSMaskAndStoreData_vd(void) {
  // temporary instance of mp adi measurement object
  gType_adiMeasurement_st l_TmpMeas_pst;

  UInt8 l_MeasObjCount_ui8 = 0;
  UInt16 l_ActTime_ui16;

  // current mp time stamp
  UInt16 l_CurMpTimeStamp_ui16 = 0;

  // indicator for new echo data
  Boolean l_NewData_bl;

  gType_adiEchoDataSet_st l_EchoData_st;

#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
  // masks for Sensor 1 and Sensor 6
  const UInt16 l_psdUssMask_ui16[2] = {PSD_USS1_MASK, PSD_USS6_MASK};
  const UInt8 l_psdUssID_ui8[2] = {gd_SENSOR1, gd_SENSOR6};
#else
  // masks for Sensor 1, Sensor 6, Sensor 8 and Sensor 13
  const UInt16 l_psdUssMask_ui16[4] = {PSD_USS1_MASK, PSD_USS6_MASK,
                                       PSD_USS8_MASK, PSD_USS13_MASK};
  const UInt8 l_psdUssID_ui8[4] = {gd_SENSOR1, gd_SENSOR6, gd_SENSOR8,
                                   gd_SENSOR13};
#endif

  SInt16 l_loopCounter_si16;

  UInt16 l_mpDe1_bf16;
  UInt16 l_mpDe2_bf16;
  UInt16 l_mpCe_bf16;
  gType_mpWorkMode_en l_MeasMode_en;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3212, 3417, 3426
#endif

  // Get OS time
 // g_BswOs1msTimeCounter_en(&l_ActTime_ui16);
  l_ActTime_ui16 = g_GetTimeMs_ui16();
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3212, 3417, 3426
#endif

  // access measurement data buffer
  g_adiGetMeasurement_vd(&l_TmpMeas_pst);

  while (l_MeasObjCount_ui8 < 2) {
    // a measurement object with data is found
    if (l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8] != 255) {
      // get mp time stamp - time stamp is in ECU time!!
      l_CurMpTimeStamp_ui16 =
          l_TmpMeas_pst.adiEcuStartMeasurementTime_ui16[l_MeasObjCount_ui8];

      // initialize masks
      l_mpDe1_bf16 = 0;
      l_mpDe2_bf16 = 0;
      l_MeasMode_en = MpWorkMode_Def_enm;
      l_mpCe_bf16 = 0;
      // get tx and rx mask of current measurement
      g_adiGetSensorsOfMeasurement_vd(
          l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8], &l_mpDe1_bf16,
          &l_mpDe2_bf16, &l_mpCe_bf16, &l_MeasMode_en);
      /*  DEBUG_PRINTF("vho mesa id:%d,de:%d mpCe:%d,psdussmask,0:%d,1:%d\n",
                     l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8],
                     l_mpDe1_bf16, l_mpCe_bf16, l_psdUssMask_ui16[0],
                     l_psdUssMask_ui16[1]);*/
#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
      // get echoes for sensor 1 and 6, so run over this functionality twice
      for (l_loopCounter_si16 = 1; l_loopCounter_si16 >= 0;
           l_loopCounter_si16--)
#else
      // get echoes for sensor 1, 6, 8 and 13, so run over this functionality
      // four times
      for (l_loopCounter_si16 = 3; l_loopCounter_si16 >= 0;
           l_loopCounter_si16--)
#endif
      {
        if (l_MeasMode_en == MpWorkMode_StdMeas_enm) {
          l_NewData_bl = FALSE;

          // check for correct measurement group: sensor 1 is Tx
          if (((l_mpDe1_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0) ||
              ((l_mpDe2_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0)) {
            // access to echo data buffer
            l_EchoData_st = g_adiGetEchosOfSensor_st(
                (l_psdUssID_ui8[l_loopCounter_si16]),
                l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8]);
            /*  DEBUG_PRINTF("vho adiEchoDistance:%d,adiEchoClassifi:%d\n",
                           l_EchoData_st.adiEchoDistance_ui16[0],
                           l_EchoData_st.adiEchoClassification_en[0]);*/

            if (l_EchoData_st.adiEchoDistance_ui16[0] != 0) {
              // take over echo data
              l_NewData_bl = TRUE;
            }
            /* if there is a second shot and no data is measured may be the
             * measuredistance was reduced so it was not possible to see an
             * object again - result: do not take a none value distance if
             * second shot was done */
            else if ((l_EchoData_st.adiEchoClassification_en[0] ==
                      MP_OBJECT_DEFAULT_enm)
                     //&&
                     //(l_TmpMeas_pst.adiMeasurementState_en[l_MeasObjCount_ui8]
                     //!= MP_MEAS_SECOND_enm)
            ) {
              // no object measured / value is necessary for PSD
              l_EchoData_st.adiEchoDistance_ui16[0] =
                  0xffff; // endless distance
              l_NewData_bl = TRUE;
            } else {
              // no new Data (second shot and on measured distance )
            }

            // data usage allowed
            if (l_NewData_bl != 0) {
              // store data for PSD usage
              (void)g_VHOUSSProcessingStoreDistList_ui8(
                  l_psdUssID_ui8[l_loopCounter_si16], // sensor index
                  &l_EchoData_st,                     // uss distance list
                  l_CurMpTimeStamp_ui16, // start time of uss measurement [OSEK
                                         // ticks]
                  l_ActTime_ui16,        // current time (timebase MP = OSEK)
                  l_ActTime_ui16); // current time (timebase CAN ticks = OSEK)
            } else {
              // no echo found
            }
          }
        } else if (l_MeasMode_en == MpWorkMode_AdvMeas_enm) {

          UInt8 i, j = 0;
          l_NewData_bl = FALSE;
          gType_adiEchoDataSet_st l_VHOEcho_st;
          // check for correct measurement group: sensor 1 is Tx
          if (((l_mpDe1_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0) ||
              ((l_mpDe2_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0)) {
            // access to echo data buffer
            l_EchoData_st = g_adiGetEchosOfSensor_st(
                (l_psdUssID_ui8[l_loopCounter_si16]),
                l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8]);
            for (i = 0; i < gd_MP_NUMBER_OF_MULTI_ECHOES_ui8; i++) {
              if ((((l_mpDe1_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) !=
                    0) &&
                   (l_EchoData_st.adiEchoType_en[i] == ADV_path1_enm)) ||
                  (((l_mpDe2_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) !=
                    0) &&
                   (l_EchoData_st.adiEchoType_en[i] == ADV_path2_enm))) {

                l_VHOEcho_st.adiEchoDistance_ui16[j] =
                    l_EchoData_st.adiEchoDistance_ui16[i];
                l_VHOEcho_st.adiEchoDuration_ui16[j] =
                    l_EchoData_st.adiEchoDuration_ui16[i];
                l_VHOEcho_st.adiEchoType_en[j] =
                    l_EchoData_st.adiEchoType_en[i];
                l_VHOEcho_st.adiEchoClassification_en[j] =
                    l_EchoData_st.adiEchoClassification_en[i];

                j++;
              } else {
              }
            }
            for (; j < gd_MP_NUMBER_OF_MULTI_ECHOES_ui8; j++) {
              l_VHOEcho_st.adiEchoDistance_ui16[j] = 0xffff; // endless distance
              l_VHOEcho_st.adiEchoDuration_ui16[j] = 0;
            }

            if (l_VHOEcho_st.adiEchoDistance_ui16[0] != 0) {
              // take over echo data
              l_NewData_bl = TRUE;
            }
            /* if there is a second shot and no data is measured may be the
             * measuredistance was reduced so it was not possible to see an
             * object again - result: do not take a none value distance if
             * second shot was done */
            else if ((l_VHOEcho_st.adiEchoClassification_en[0] ==
                      MP_OBJECT_DEFAULT_enm)
                     //&&
                     //(l_TmpMeas_pst.adiMeasurementState_en[l_MeasObjCount_ui8]
                     //!= MP_MEAS_SECOND_enm)
            ) {
              // no object measured / value is necessary for PSD
              l_VHOEcho_st.adiEchoDistance_ui16[0] = 0xffff; // endless distance
              l_NewData_bl = TRUE;
            } else {
              // no new Data (second shot and on measured distance )
            }

            // data usage allowed
            if (l_NewData_bl != 0) {
              // store data for PSD usage
              (void)g_VHOUSSProcessingStoreDistList_ui8(
                  l_psdUssID_ui8[l_loopCounter_si16], // sensor index
                  &l_VHOEcho_st,                      // uss distance list
                  l_CurMpTimeStamp_ui16, // start time of uss measurement [OSEK
                                         // ticks]
                  l_ActTime_ui16,        // current time (timebase MP = OSEK)
                  l_ActTime_ui16); // current time (timebase CAN ticks = OSEK)
            } else {
              // no echo found
            }
          }

        } else {
          /**QAC**/
        }
      }
    } // end of: if(l_TmpMeas_pst->adiMeasurementID_ui8[l_MeasObjCount_ui8] !=
      // 255)
    l_MeasObjCount_ui8++;
  }
}
// #else
VISIBILITY void m_SetUSSMaskAndStoreData_vd(void)
{
    // temporary instance of mp adi measurement object
    gType_adiMeasurement_st l_TmpMeas_pst;

    UInt8 l_MeasObjCount_ui8 = 0;
    UInt16 l_ActTime_ui16;

    // current mp time stamp
    UInt16 l_CurMpTimeStamp_ui16 = 0;

    // indicator for new echo data
    Boolean l_NewData_bl;
    Boolean l_mpDoubleKeep_bl;
    gType_adiEchoDataSet_st l_EchoData_st;

#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
    // masks for Sensor 1 and Sensor 6
    const UInt16 l_psdUssMask_ui16[2] = {PSD_USS1_MASK, PSD_USS6_MASK};
    const UInt8 l_psdUssID_ui8[2] = {gd_SENSOR1, gd_SENSOR6};
#else
    // masks for Sensor 1, Sensor 6, Sensor 8 and Sensor 13
    const UInt16 l_psdUssMask_ui16[4] = {PSD_USS1_MASK, PSD_USS6_MASK, PSD_USS8_MASK,
                                         PSD_USS13_MASK};
    const UInt8 l_psdUssID_ui8[4] = {gd_SENSOR1, gd_SENSOR6, gd_SENSOR8, gd_SENSOR13};
#endif

    SInt16 l_loopCounter_si16;

    UInt16 l_mpDe1_bf16;
    UInt16 l_mpDe2_bf16;
    UInt16 l_mpDe3_bf16;
    UInt16 l_mpCe_bf16;
    UInt16 l_mpMeasRange_ui16[2][2] = {0};
    gType_mpWorkMode_en l_MeasMode_en;

    /*
          QAC message 3417,3426 (level 4) caused by BSW function
          (void)g_BswOs1msTimeCounter_en(&Time_ui16) shall be suppressed.
          3212: "This explicit cast is redundant and could be removed."
          3417: "The comma operator has been used outside a for-loop header."
          It relate to MISRA-C:2004 Rule 12.10 "The comma operator shall not be
       used." 3426: "Right hand side of comma expression has no side effect and
       its value is not used." It relates to MISRA-C:2004 Rule 14.2.: "All
       non-null statements shall either (i) have at least one side-effect however
       executed, or (ii) cause control flow to change." For efficiency reasons,
       this function-like macro is accepted by AUTOSAR [BSW00330]: "Function-like
       macros may have to use the comma operator. Function-like macros are
       required for efficiency reasons." Refer to AUTOSAR_SWS_RTE.pdf, section
       MISRA C Compliance.
    */
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3212, 3417, 3426
#endif

    // Get OS time
    // g_BswOs1msTimeCounter_en(&l_ActTime_ui16);
    l_ActTime_ui16 = g_GetTimeMs_ui16();

    /*
          Reactivate: QAC message 3417,3426 (level 4) caused by BSW function
          (void)g_BswOs1msTimeCounter_en(&Time_ui16) shall be suppressed.
          3212: "This explicit cast is redundant and could be removed."
          3417: "The comma operator has been used outside a for-loop header."
          It relate to MISRA-C:2004 Rule 12.10 "The comma operator shall not be
       used." 3426: "Right hand side of comma expression has no side effect and
       its value is not used." It relates to MISRA-C:2004 Rule 14.2.: "All
       non-null statements shall either (i) have at least one side-effect however
       executed, or (ii) cause control flow to change." For efficiency reasons,
       this function-like macro is accepted by AUTOSAR [BSW00330]: "Function-like
       macros may have to use the comma operator. Function-like macros are
       required for efficiency reasons." Refer to AUTOSAR_SWS_RTE.pdf, section
       MISRA C Compliance.
    */
#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3212, 3417, 3426
#endif

    // access measurement data buffer
    // g_adiGetMeasurement_vd(&l_TmpMeas_pst);
    g_adiGetMpIfModule_pst()->adiGetMeasurement_pf(&l_TmpMeas_pst);

    while (l_MeasObjCount_ui8 < 2)
    {
        // a measurement object with data is found
        if (l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8] != 255)
        {
            // get mp time stamp - time stamp is in ECU time!!
            l_CurMpTimeStamp_ui16 =
                l_TmpMeas_pst.adiEcuStartMeasurementTime_ui16[l_MeasObjCount_ui8];

            // initialize masks
            l_mpDe1_bf16 = 0;
            l_mpDe2_bf16 = 0;
            l_mpDe3_bf16 = 0;
            l_MeasMode_en = MpWorkMode_Def_enm;
            l_mpCe_bf16 = 0;
            // get tx and rx mask of current measurement
            // g_adiGetSensorsOfMeasurement_vd(l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8],
            //							  &l_mpDe1_bf16,&l_mpDe2_bf16,&l_mpCe_bf16,&l_MeasMode_en);
            g_adiGetMpIfModule_pst()->adiGetSensorsOfMeasurement_pf(
                l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8],
                l_MeasObjCount_ui8, &l_mpDe1_bf16, &l_mpDe2_bf16, &l_mpDe3_bf16,
                &l_mpCe_bf16, &l_MeasMode_en, &l_mpDoubleKeep_bl, &l_mpMeasRange_ui16[l_MeasObjCount_ui8][0]);

#if (LS_VHO_USSMAP_REAR_SENSOR_ACTIVE == SW_OFF)
            // get echoes for sensor 1 and 6, so run over this functionality twice
            for (l_loopCounter_si16 = 1; l_loopCounter_si16 >= 0; l_loopCounter_si16--)
#else
            // get echoes for sensor 1, 6, 8 and 13, so run over this functionality
            // four times
            for (l_loopCounter_si16 = 3; l_loopCounter_si16 >= 0; l_loopCounter_si16--)
#endif
            {
                if (l_MeasMode_en == MpWorkMode_StdMeas_enm)
                {
                    l_NewData_bl = FALSE;
                    // check for correct measurement group: sensor 1 is Tx
                    if (((l_mpDe1_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0) ||
                        ((l_mpDe2_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0) ||
                        ((l_mpDe3_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0))
                    {
                        // access to echo data buffer
                        // l_EchoData_st =
                        // g_adiGetEchosOfSensor_st((l_psdUssID_ui8[l_loopCounter_si16]),
                        //					   l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8]);

                        l_EchoData_st = g_adiGetMpIfModule_pst()->adiGetEchosOfSensor_pf(
                            (l_psdUssID_ui8[l_loopCounter_si16]),
                            l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8]);

                        if (l_EchoData_st.adiEchoDistance_ui16[0] != 0)
                        {
                            // take over echo data
                            l_NewData_bl = TRUE;
                        }
                        /* if there is a second shot and no data is measured may be the
                         * measuredistance was reduced so it was not possible to see an
                         * object again - result: do not take a none value distance if
                         * second shot was done */
                        else if (
                            (l_EchoData_st.adiEchoClassification_en[0] ==
                             MP_OBJECT_DEFAULT_enm)
                            //&&
                            //(l_TmpMeas_pst.adiMeasurementState_en[l_MeasObjCount_ui8]
                            //!= MP_MEAS_SECOND_enm)
                        )
                        {
                            // no object measured / value is necessary for PSD
                            l_EchoData_st.adiEchoDistance_ui16[0] =
                                0xffff; // endless distance
                            l_NewData_bl = TRUE;
                        }
                        else
                        {
                            // no new Data (second shot and on measured distance )
                        }

                        // data usage allowed
                        if (l_NewData_bl != 0)
                        {
                            // store data for PSD usage
                            (void)g_VHOUSSProcessingStoreDistList_ui8(
                                l_psdUssID_ui8[l_loopCounter_si16], // sensor index
                                &l_EchoData_st,                     // uss distance list
                                l_CurMpTimeStamp_ui16,              // start time of uss measurement
                                                                    // [OSEK ticks]
                                l_ActTime_ui16,                     // current time (timebase MP = OSEK)
                                l_ActTime_ui16);                    // current time (timebase CAN ticks =
                                                                    // OSEK)
                        }
                        else
                        {
                            // no echo found
                        }
                    }
                }
                else if (l_MeasMode_en == MpWorkMode_AdvMeas_enm)
                {

                    UInt8 i, j = 0;
                    l_NewData_bl = FALSE;
                    gType_adiEchoDataSet_st l_VHOEcho_st;
                    // check for correct measurement group: sensor 1 is Tx
                    if (((l_mpDe2_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0) ||
                        ((l_mpDe3_bf16 & l_psdUssMask_ui16[l_loopCounter_si16]) != 0))
                    {
                        // access to echo data buffer
                        // l_EchoData_st =
                        // g_adiGetEchosOfSensor_st((l_psdUssID_ui8[l_loopCounter_si16]),
                        //					   l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8]);
                        l_EchoData_st = g_adiGetMpIfModule_pst()->adiGetEchosOfSensor_pf(
                            (l_psdUssID_ui8[l_loopCounter_si16]),
                            l_TmpMeas_pst.adiMeasurementID_ui8[l_MeasObjCount_ui8]);

                        for (i = 0; i < gd_MP_NUMBER_OF_MULTI_ECHOES_ui8; i++)
                        {
                            if ((((l_mpDe2_bf16 &
                                   l_psdUssMask_ui16[l_loopCounter_si16]) != 0) &&
                                 (l_EchoData_st.adiEchoType_en[i] == ADV_path1_enm)) ||
                                (((l_mpDe3_bf16 &
                                   l_psdUssMask_ui16[l_loopCounter_si16]) != 0) &&
                                 (l_EchoData_st.adiEchoType_en[i] == ADV_path2_enm)))
                            {

                                l_VHOEcho_st.adiEchoDistance_ui16[j] =
                                    l_EchoData_st.adiEchoDistance_ui16[i];
                                l_VHOEcho_st.adiEchoDuration_ui16[j] =
                                    l_EchoData_st.adiEchoDuration_ui16[i];
                                l_VHOEcho_st.adiEchoType_en[j] =
                                    l_EchoData_st.adiEchoType_en[i];
                                l_VHOEcho_st.adiEchoClassification_en[j] =
                                    l_EchoData_st.adiEchoClassification_en[i];

                                j++;
                            }
                            else
                            {
                            }
                        }
                        for (; j < gd_MP_NUMBER_OF_MULTI_ECHOES_ui8; j++)
                        {
                            l_VHOEcho_st.adiEchoDistance_ui16[j] =
                                0xffff; // endless distance
                            l_VHOEcho_st.adiEchoDuration_ui16[j] = 0;
                        }

                        if (l_VHOEcho_st.adiEchoDistance_ui16[0] != 0)
                        {
                            // take over echo data
                            l_NewData_bl = TRUE;
                        }
                        /* if there is a second shot and no data is measured may be the
                         * measuredistance was reduced so it was not possible to see an
                         * object again - result: do not take a none value distance if
                         * second shot was done */
                        else if (
                            (l_VHOEcho_st.adiEchoClassification_en[0] ==
                             MP_OBJECT_DEFAULT_enm)
                            //&&
                            //(l_TmpMeas_pst.adiMeasurementState_en[l_MeasObjCount_ui8]
                            //!= MP_MEAS_SECOND_enm)
                        )
                        {
                            // no object measured / value is necessary for PSD
                            l_VHOEcho_st.adiEchoDistance_ui16[0] =
                                0xffff; // endless distance
                            l_NewData_bl = TRUE;
                        }
                        else
                        {
                            // no new Data (second shot and on measured distance )
                        }

                        // data usage allowed
                        if (l_NewData_bl != 0)
                        {
                            // store data for PSD usage
                            (void)g_VHOUSSProcessingStoreDistList_ui8(
                                l_psdUssID_ui8[l_loopCounter_si16], // sensor index
                                &l_VHOEcho_st,                      // uss distance list
                                l_CurMpTimeStamp_ui16,              // start time of uss measurement
                                                                    // [OSEK ticks]
                                l_ActTime_ui16,                     // current time (timebase MP = OSEK)
                                l_ActTime_ui16);                    // current time (timebase CAN ticks =
                                                                    // OSEK)
                        }
                        else
                        {
                            // no echo found
                        }
                    }
                }
                else
                {
                    /**QAC**/
                }
            }
        } // end of: if(l_TmpMeas_pst->adiMeasurementID_ui8[l_MeasObjCount_ui8] !=
          // 255)
        l_MeasObjCount_ui8++;
    }
}
#endif
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N     D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         read input signals from ADI
 * @details
 *
 * @param         f_VHOCanSig_pst
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOGetSignals_vd(gType_VHOCanSig_st *f_VHOCanSig_pst)
{
#if (GS_VHS_YAW_RATE == SW_ON) && (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt16 l_A_si16;
#endif
#if (GS_VHS_VEH_SPEED == SW_ON)
    UInt16 l_Vel_ui16;
#endif
#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) ||      \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&              \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR)) || \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&              \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
    gType_Gear_en l_Gear_pen;
#endif

#if (GS_VHS_SWA == SW_ON)
    gType_SWAandTime_st l_SWA_st;
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHS_WICRL == SW_ON)
    UInt16 l_WICTick_ui16;
#else
    UInt16 l_WICTick_ui16 = 0xFFFF;
#endif
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (GS_VHS_WICRR == SW_ON)
    gType_WICandTime_st l_WIC_RR_st;
#endif
#if (GS_VHS_WICRL == SW_ON)
    gType_WICandTime_st l_WIC_RL_st;
#endif
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHS_WICFR == SW_ON)
    gType_WICandTime_st l_WIC_FR_st;
#endif
#if (GS_VHS_WICFL == SW_ON)
    gType_WICandTime_st l_WIC_FL_st;
#endif
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
    gType_WheelDir_en l_WheelDirRR_pen;
#endif
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
    gType_WheelDir_en l_WheelDirRL_pen;
#endif
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHS_WHEEL_DIR_FR == SW_ON)
    gType_WheelDir_en l_WheelDirFR_pen;
#endif
#if (GS_VHS_WHEEL_DIR_FL == SW_ON)
    gType_WheelDir_en l_WheelDirFL_pen;
#endif
#endif //(GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    // set forced extrapolation in deg mode extrapolation to FALSE (VHO shall
    // work with simulated WIC values based on velocity if WICs are invalid or
    // unavailable in deg mode extrapolation)
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    g_VHOCtrlStat_st.DegModeExtrapolForceExtrapol_bl = FALSE;
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        // init
        f_VHOCanSig_pst->WIC_Invalid_ui8 = (UInt8)0;
    }
#endif

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA))

#if (GS_VHS_WICRL == SW_ON)
    if (g_adiGetWICRL_bl(&l_WIC_RL_st) == FALSE)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WIC_Invalid_ui8++;
        }
    }
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->WICRaw_pui16[LeftRear_enm] = l_WIC_RL_st.WIC_Signal_ui16;
    }
    l_WICTick_ui16 = l_WIC_RL_st.WIC_Tick_ui16;
#endif
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
    (void)g_adiGetWheelDirRL_bl(&l_WheelDirRL_pen);
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WICState_pen[LeftRear_enm] =
                m_VHONormVHSWicMoveDir_en(l_WheelDirRL_pen);
        }
    }
#endif
#endif

#if (GS_VHS_WICRR == SW_ON)
    if (g_adiGetWICRR_bl(&l_WIC_RR_st) == FALSE)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WIC_Invalid_ui8++;
        }
    }
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->WICRaw_pui16[RightRear_enm] = l_WIC_RR_st.WIC_Signal_ui16;
    }
#endif
#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
    (void)g_adiGetWheelDirRR_bl(&l_WheelDirRR_pen);
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WICState_pen[RightRear_enm] =
                m_VHONormVHSWicMoveDir_en(l_WheelDirRR_pen);
        }
    }
#endif
#endif

#if (GS_VHS_WICFR == SW_ON)
    if (g_adiGetWICFR_bl(&l_WIC_FR_st) == FALSE)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WIC_Invalid_ui8++;
        }
    }
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->WICRaw_pui16[RightFront_enm] = l_WIC_FR_st.WIC_Signal_ui16;
    }
#endif
#if (GS_VHS_WHEEL_DIR_FR == SW_ON)
    (void)g_adiGetWheelDirFR_bl(&l_WheelDirFR_pen);
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WICState_pen[RightFront_enm] =
                m_VHONormVHSWicMoveDir_en(l_WheelDirFR_pen);
        }
    }
#endif
#endif

#if (GS_VHS_WICFL == SW_ON)
    if (g_adiGetWICFL_bl(&l_WIC_FL_st) == FALSE)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WIC_Invalid_ui8++;
        }
    }
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->WICRaw_pui16[LeftFront_enm] = l_WIC_FL_st.WIC_Signal_ui16;
    }
#endif
#if (GS_VHS_WHEEL_DIR_FL == SW_ON)
    (void)g_adiGetWheelDirFL_bl(&l_WheelDirFL_pen);
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WICState_pen[LeftFront_enm] =
                m_VHONormVHSWicMoveDir_en(l_WheelDirFL_pen);
        }
    }
#endif
#endif
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

#if (GS_VHS_WICRL == SW_ON)
    (void)g_adiGetWICRL_bl(&l_WIC_RL_st);
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->WICRaw_pui16[LeftRear_enm] = l_WIC_RL_st.WIC_Signal_ui16;
    }
#endif
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
    (void)g_adiGetWheelDirRL_bl(&l_WheelDirRL_pen);
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WICState_pen[LeftRear_enm] =
                m_VHONormVHSWicMoveDir_en(l_WheelDirRL_pen);
        }
    }
#endif
#endif

#if (GS_VHS_WICRR == SW_ON)
    (void)g_adiGetWICRR_bl(&l_WIC_RR_st);
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->WICRaw_pui16[RightRear_enm] = l_WIC_RR_st.WIC_Signal_ui16;
    }
#endif
#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
    (void)g_adiGetWheelDirRR_bl(&l_WheelDirRR_pen);
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->WICState_pen[RightRear_enm] =
                m_VHONormVHSWicMoveDir_en(l_WheelDirRR_pen);
        }
    }
#endif
#endif
#endif

// Do specific preprocessing of wheel direction signals. This is necessary as
// after direction change during the first ticks the old direction is put out,
// indicated as "valid"
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
#if (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
    if (((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Wic_enm) ||
        ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
         VHO_Direction_Yaw_Wic_enm))
#endif
    {
        // calclute the next higher whole number of WICLength in mm for decision
        // whether specific preprocessing of wheel direction signals is required
        // by application data or not.
        m_WICLengthMM_ui8 =
            (UInt8)(((UInt16)(g_parGetParaVHO_Veh_WICLength_ui16) >> (UInt16)(8)) +
                    (UInt16)(1));

        // if application data is greater than at least one WICLength in mm
        // the specific preprocessing of wheel direction signals will be done
        if (g_parGetParaVHO_Odo_MinDistAftStdstlNoWICDir_ui8 > m_WICLengthMM_ui8)
        {
            // vehicle is standing still
            if (g_VHOVehicleState_st.RollingDirection_en == VHO_STOP_enm)
            {
                m_DrivDistAbsLastStopMM_ui32 =
                    g_VHOVehicleState_st.DrivenDistanceAbsolute_ui32;

                /**************************************/
                /* Avoid possible use of zero pointer */
                /**************************************/
                if (f_VHOCanSig_pst != NULL)
                {
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
                    f_VHOCanSig_pst->WICState_pen[LeftRear_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
                    f_VHOCanSig_pst->WICState_pen[RightRear_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_FL == SW_ON)
                    f_VHOCanSig_pst->WICState_pen[LeftFront_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_FR == SW_ON)
                    f_VHOCanSig_pst->WICState_pen[RightFront_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
                    f_VHOCanSig_pst->WICState_pen[LeftRear_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
                    f_VHOCanSig_pst->WICState_pen[RightRear_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#endif
                }
                else
                {
                    // do nothing
                }
            }
            else
            {
                // vehicle is moving
                if ((g_VHOVehicleState_st.DrivenDistanceAbsolute_ui32 -
                     m_DrivDistAbsLastStopMM_ui32) <=
                    ((UInt32)(g_parGetParaVHO_Odo_MinDistAftStdstlNoWICDir_ui8)))
                {
                    /**************************************/
                    /* Avoid possible use of zero pointer */
                    /**************************************/
                    if (f_VHOCanSig_pst != NULL)
                    {
                        // consideration of wrap around is explicitly omitted here
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
                        f_VHOCanSig_pst->WICState_pen[LeftRear_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
                        f_VHOCanSig_pst->WICState_pen[RightRear_enm] =
                            VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_FL == SW_ON)
                        f_VHOCanSig_pst->WICState_pen[LeftFront_enm] =
                            VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_FR == SW_ON)
                        f_VHOCanSig_pst->WICState_pen[RightFront_enm] =
                            VHO_MOV_UNKNOWN_enm;
#endif
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
#if (GS_VHS_WHEEL_DIR_RL == SW_ON)
                        f_VHOCanSig_pst->WICState_pen[LeftRear_enm] = VHO_MOV_UNKNOWN_enm;
#endif
#if (GS_VHS_WHEEL_DIR_RR == SW_ON)
                        f_VHOCanSig_pst->WICState_pen[RightRear_enm] =
                            VHO_MOV_UNKNOWN_enm;
#endif
#endif
                    }
                    else
                    {
                        // do nothing
                    }
                }
                else
                {
                    // do not modify input signals
                }
            }
        }
        else
        // if application data is not greater than at least one WICLength in mm
        // the specific preprocessing of wheel direction signals will not be done
        {
            // do nothing
        }
    }
#endif /* #if ( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
                (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
                (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL) ) */

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        if (f_VHOCanSig_pst->WIC_Invalid_ui8 == 0)
        {
            // wic is valid, read timestamp
            f_VHOCanSig_pst->WIC_CanTime_ui16 = l_WICTick_ui16;
        }
    }

#if (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)
    if (g_VHOCtrlStat_st.DegMode_en == DegMode_VHO_Extrapolation_enm)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            // in deg mode extrapolation the vho shall be called even in case of
            // timeout or invalid values
            if (f_VHOCanSig_pst->WIC_Invalid_ui8 > 0)
            {
                UInt16 l_T_ui16;

                // at least one wic is not not valid => force extrapolation
                g_VHOCtrlStat_st.DegModeExtrapolForceExtrapol_bl = TRUE;

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_OFF 3212, 3417, 3426
#endif

                // no new wic timestamp available, replace by current OS time
                //  g_BswOs1msTimeCounter_en(&l_T_ui16);
                l_T_ui16 = g_GetTimeMs_ui16();

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3212, 3417, 3426
#endif

                if ((l_T_ui16 - f_VHOCanSig_pst->WIC_CanTime_ui16) >=
                    (UInt16)gd_VHO_DEGMODEEXTRAPOL_VIRT_WIC_CYCLETIME_MS_ui32)
                {
                    f_VHOCanSig_pst->WIC_CanTime_ui16 = l_T_ui16;
                }
            }
        }
    }
#endif // (LS_VHO_EXTRAPOL_ENGINE_RESTART == SW_ON)

#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_VHS_SWA == SW_ON)
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        // get steering wheel angle
        f_VHOCanSig_pst->SWA_ValidFlag_bl = g_adiGetSWA_bl(&l_SWA_st);
        f_VHOCanSig_pst->SWA_si16         = l_SWA_st.SWA_Signal_si16;
        f_VHOCanSig_pst->SWA_Tick_ui16    = l_SWA_st.SWA_Tick_ui16;
    }
#endif

#if (GS_4WS_VEHICLE == SW_ON)
    if (g_parGetParaPAGLOBAL_SysFct_RearAxleSteering_bl != FALSE)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            SInt16 l_SAR_Can_si16;
            // get steering angle rear (SAR) from VHS
            // ======================================
            f_VHOCanSig_pst->SAR_ValidFlag_bl =
                g_adiGetRearAxleActualSA_bl(&l_SAR_Can_si16);
            // get value only if it is valid, else use old value
            if (f_VHOCanSig_pst->SAR_ValidFlag_bl != FALSE)
            {
                f_VHOCanSig_pst->SAR_si16 = l_SAR_Can_si16;
            }
        }
    }
    else
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->SAR_ValidFlag_bl = FALSE;
            f_VHOCanSig_pst->SAR_si16         = ((SInt16)0);
        }
    }
#endif

#if (GS_VHS_YAW_RATE == SW_ON)
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        // Get Yaw Rate:
        f_VHOCanSig_pst->YawRate_ValidFlag_bl = g_adiGetYawRate_bl(&l_A_si16);
        f_VHOCanSig_pst->YawRate_si16         = l_A_si16;
    }
#endif
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
#if (GS_VHS_SWA == SW_ON)
    // get steering wheel angle
    (void)g_adiGetSWA_bl(&l_SWA_st);
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->SWA_si16 = l_SWA_st.SWA_Signal_si16;
    }
#else
#error ('required input signals SWA not enabled')
#endif
#endif

#if ((GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA) ||      \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&              \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_GEAR)) || \
     ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) &&              \
      (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)))
#if ((GS_VHO_RUN_MODE == GS_VHO_MINI_VHO) && \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
    if ((gType_VHORollRecogMode_en)g_parGetParaVHO_Odo_DirDetectMode_ui8 ==
        VHO_Direction_Gear_enm)
#endif
    {
#if (GS_VHS_GEAR == SW_ON)
        // get gear information
        (void)g_adiGetGear_bl(&l_Gear_pen);

        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            if (l_Gear_pen == Reverse_enm)
            {
                f_VHOCanSig_pst->ReverseGearInserted_bl = TRUE;
            }
            else
            {
                f_VHOCanSig_pst->ReverseGearInserted_bl = FALSE;
            }
        }
#endif
    }
#endif

#if (GS_VHS_VEH_SPEED == SW_ON)
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    {
        /**************************************/
        /* Avoid possible use of zero pointer */
        /**************************************/
        if (f_VHOCanSig_pst != NULL)
        {
            f_VHOCanSig_pst->Vel_ValidFlag_bl = g_adiGetVehSpeed_bl(&l_Vel_ui16);
            // the possibility of vehicle stands still is really high after time
            // before stop. In module vho_recog_dir.c the rolling direction is set to
            // VHO_STOP when Vel_ui16 gets zero. So zero is forbidden before time
            // before stop has reached. Above 90 km/h
            // g_VHORollRecog_st.TimeLastWICIncrement_ui16 will not be updated. So it
            // is recommended to do not check "vehicle stand still" above 5 km/h, just
            // store VHS velocity instead.
            if (l_Vel_ui16 <= (UInt16)(50))
            {
                if ((g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 == 0) &&
                    (g_VHORollRecog_st.WICTimeDeltaMS_ui16 >
                     gd_VHOMinTimeBeforeSTOPMS_ui16))
                {
                    f_VHOCanSig_pst->Vel_ui16 = (UInt16)(0);
                }
                else if (l_Vel_ui16 <= (UInt16)(1))
                {
                    f_VHOCanSig_pst->Vel_ui16 = (UInt16)(1);
                }
                else
                {
                    f_VHOCanSig_pst->Vel_ui16 = l_Vel_ui16;
                }
            }
            else
            {
                f_VHOCanSig_pst->Vel_ui16 = l_Vel_ui16;
            }
        }
    }
#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)
    (void)g_adiGetVehSpeed_bl(&l_Vel_ui16);
    /**************************************/
    /* Avoid possible use of zero pointer */
    /**************************************/
    if (f_VHOCanSig_pst != NULL)
    {
        f_VHOCanSig_pst->Vel_ui16 = l_Vel_ui16;
    }
#endif
#endif

    return;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         prepare vho call cycle by normation of input signals,
 *                calculation of wic increments of the current cycle,
 *                correction of input signals with results of
 * autocalibration,...
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
VISIBILITY void m_VHOInputConversion_vd(void)
{
    UInt8 l_I_ui8;

    // calculate wheel circumference from wic length and number of teeth
    const UInt32 lc_WheelCircumferenceMM_ui32 =
        ((((UInt32)g_VHOParameters_st.VHO_Veh_pst->WICLength_ui16) *
          ((UInt32)g_VHOParameters_st.VHO_Veh_pst->WICTeethCount_ui16)) >>
         8);

    if ((lc_WheelCircumferenceMM_ui32 < md_MinWheelCircumferenceMM_ui32) ||
        (lc_WheelCircumferenceMM_ui32 > md_MaxWheelCircumferenceMM_ui32))
    {
        g_vhoLogHdl_vd(VHOLog_VHOMain_enm, g_vhoLogStateOn_enm);
    }

    // set wic signals of the first message as startup values
    g_VHOCtrlStat_st.WICChanged_bl = FALSE;

    if (g_VHOCanSig_st.WIC_CanTime_ui16 == g_VHOOdoBuf_st.WIC_CanTime_buf_ui16)
    {
        // wic timestamp did not change, set delta to zero
        g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 = (UInt16)0;
    }
    else // wic can timestamp is NOT identical
    {
        const UInt16 lc_MaxCanDeltaMS_ui16 = (UInt16)2000;

        g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 =
            g_VHOCanSig_st.WIC_CanTime_ui16 - g_VHOOdoBuf_st.WIC_CanTime_buf_ui16;

        if (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 > lc_MaxCanDeltaMS_ui16)
        {
            // implausible change of can time; set delta time to zero so that the
            // vho will not be called
            g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 = (UInt16)0;

            g_VHOOdoBuf_st.WIC_CanTime_buf_ui16 = g_VHOCanSig_st.WIC_CanTime_ui16;
        }
        else
        {
            //
            // calculate UNCORRECTED
            //            - steering angle
            //            - curvature
            //            - curve radius
            //

            // convert steering wheel angle to steering angle of the virtual front
            // wheel. the average steering wheel angle between the current value and
            // the one of the last cycle is taken as the input value
            if (g_VHOVehicleState_st.RollingDirection_en == VHO_MOV_BACKW_enm)
            {
                // get steering angle for reverse movement
                g_VHOVehicleState_st.SAngRadF12_si16 = (SInt16)g_mtl_ReducePrecision_si32(
                    g_VHOCanSig_st.SWA_si16 + (SInt32)g_VHOOdoBuf_st.SWAngDegRes_buf_si16,
                    1);
                g_VHOVehicleState_st.SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
                    g_VHOVehicleState_st.SAngRadF12_si16, FALSE); // rad F12
            }
            else
            {
                // calculate steering angle for forward movement
                g_VHOVehicleState_st.SAngRadF12_si16 = (SInt16)g_mtl_ReducePrecision_si32(
                    g_VHOCanSig_st.SWA_si16 + (SInt32)g_VHOOdoBuf_st.SWAngDegRes_buf_si16,
                    1);
                g_VHOVehicleState_st.SAngRadF12_si16 = g_VHOConvertSWA2SA_si16(
                    g_VHOVehicleState_st.SAngRadF12_si16, TRUE); // rad F12
            }
            // buffer the current steering wheel angle for next calculation cycle
            g_VHOOdoBuf_st.SWAngDegRes_buf_si16 = g_VHOCanSig_st.SWA_si16;

#if (GS_4WS_VEHICLE == SW_ON)
            {
                SInt16 l_SARdeg_si16;
                // transform steering angle rear from deg to rad
                // SAR_deg = SAR_phys * 50 :: CAN-Quatisation 1 LSB :=: 0.02�
                // SAR_radF12 = (SAR_deg_new + SAR_deg_old) * 2^11 * (Pi/180) / 50
                // SAR_radF12 = (SAR_deg_new + SAR_deg_old) * 2048 * Pi / (9000)
                // SAR_radF12 = (SAR_deg_new + SAR_deg_old) * 6434 / 9000

                l_SARdeg_si16 =
                    g_VHOCanSig_st.SAR_si16 + g_VHOOdoBuf_st.SAngRearDegRes_buf_si16;

                g_VHOVehicleState_st.SAngRearRadF12_si16 = g_mtl_s16_Mul_s16_Div_s16_si16(
                    l_SARdeg_si16, (SInt16)(6434), (SInt16)(9000));

                // buffer the current rear steering angle for next calculation cycle
                g_VHOOdoBuf_st.SAngRearDegRes_buf_si16 =
                    g_VHOCanSig_st.SAR_si16; // CAN in deg
            }
#endif
            // Transform the result steering angle to the curvature at the
            // instantaneous center The instantaneous center depends on steering angle
            // rear if global parameter GS_4WS_VEHICLE is set to SW_OFF (steering
            // angle rear is always zero)
            //    The instantaneous center is reperesented by the center of the rear
            //    axle
            g_VHOVehicleState_st.KappaF14_si16 =
#if (GS_4WS_VEHICLE == SW_ON)
                g_VHOGetCurvatureFrPSCctualSteeringAngle_si16(
                    g_VHOCorrectedSignals_st.Corrected_SAngRadF12_si16);
#else
                g_VHOGetCurvatureFromSteeringAngle_si16(
                    g_VHOCorrectedSignals_st.Corrected_SAngRadF12_si16);
#endif

            // CALCULATE WIC INCREMENTS OF THE CURRENT CYCLE
            //
            for (l_I_ui8 = 0; l_I_ui8 < 5; l_I_ui8++)
            {
                if (g_VHOCanSig_st.WICRaw_pui16[l_I_ui8] >
                    g_VHOOdoBuf_st.WICBuf_pui16[l_I_ui8])
                {
                    // wic has been incremented without wrap around
                    g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8] =
                        g_VHOCanSig_st.WICRaw_pui16[l_I_ui8] -
                        g_VHOOdoBuf_st.WICBuf_pui16[l_I_ui8];

                    g_VHOCtrlStat_st.WICChanged_bl = TRUE;
                }
                else if (g_VHOCanSig_st.WICRaw_pui16[l_I_ui8] <
                         g_VHOOdoBuf_st.WICBuf_pui16[l_I_ui8])
                {
                    // wic has been incremented, wrap around occured
                    g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8] =
                        (UInt16)1 + g_VHOCanSig_st.WICRaw_pui16[l_I_ui8] +
                        ((UInt16)((g_VHOParameters_st.VHO_Veh_pst->MaxWICValue_ui16) -
                                  g_VHOOdoBuf_st.WICBuf_pui16[l_I_ui8]));

                    g_VHOCtrlStat_st.WICChanged_bl = TRUE;
                }
                else
                {
                    // wic value did not change, set delta to zero
                    g_VHOOdoBuf_st.WICDelta_pui16[l_I_ui8] = (UInt16)0;
                }
                g_VHOOdoBuf_st.WICBuf_pui16[l_I_ui8] =
                    g_VHOCanSig_st.WICRaw_pui16[l_I_ui8];
            }

            //
            //  NORMATION OF WHEEL VELOCITIES
            //
#if ((LS_VHO_AUTOCALIB_TTC == SW_ON) || (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON))
            // conversion from 0.5 rpm to 0.01 kmh
            for (l_I_ui8 = 0; l_I_ui8 < 4; l_I_ui8++)
            {
                //   lc_WheelCircumferenceMM_ui32 * 3.6 * 100
                //   lc_WheelCircumferenceMM_ui32 * 3
                //   --------------------------------------- =
                //   --------------------------------------- 2 * 1000 * 60 1000 max
                //   enumerator:
                g_VHOOdoBuf_st.WheelVelokmh_pui16[l_I_ui8] =
                    (UInt16)(((UInt32)g_VHOCanSig_st.WheelRev_pui16[l_I_ui8] *
                              lc_WheelCircumferenceMM_ui32 * (UInt32)3) /
                             (UInt32)1000);
            }
#endif

            // convert velocity from 0.1km/h to 1/256m/s
            // vel_conv = (vel * 256) / (3.6 *10)
            g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 = g_mtl_u16_Mul_u16_Div_u16_ui16(
                g_VHOCanSig_st.Vel_ui16, (UInt16)(256), (UInt16)(36));
        }
    }
}
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         main function, performs one position calculation cycle,
 *                also invokes call of
 *                - input validation component
 *                - calibration component
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOCalculateNewPosition_vd(void)
{

    // increment call cycle counter
    ++g_VHOCtrlStat_st.VHOCallCycle_ui32;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

    if (g_VHOCtrlStat_st.InitialiseWIC_ui8 == (UInt8)1)
    {
        // check whether all the WIC values that are used in the current setting
        // have been updated after initialization, otherwise init mode will not be
        // exited
        m_VHOInitWIC_vd();
    }
    else // init mode has been finished, normal operation mode
    {
        // calculate difference between current and buffered wic values
        m_VHOInputConversion_vd();

        if (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 > 0)
        {
            // add delta time to total runtime
            g_VHOCtrlStat_st.VHOTotalRuntimeMS_ui32 +=
                (UInt32)g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16;

            if (g_VHOCanSig_st.Vel_ui16 < gd_VHOShutdownVelocitySigRes_ui16)
            {
                if (g_VHOCtrlStat_st.VHOBypass_bl == FALSE)
                {
                    // call signal preprocessing
                    g_VHOSignalPreproc_vd();

                    // buffer current position for subsequent position interpolation /
                    // extrapolation
                    g_VHOUSSProcessingBufferPosition_vd();

                    // perform position update
                    g_VHOCalculateModelDependentPosition_vd();

                    // write new position to WIC history buffer
                    m_VHOWriteWICHistUSS_vd();

                    // write yaw angle to ring memory
                    if ((g_VHOVehicleState_st.RollingDirection_en != VHO_MOV_BACKW_enm) &&
                        (g_VHOVehicleState_st.RollingDirection_en !=
                         VHO_MOV_TURNED_BACKW_enm))
                    {
                        m_VHOWriteYAngHist_vd(g_VHOVehicleState_st.YawAngle_ui32);
                    }
                    else
                    {
                        m_VHOInitBufferYAngHist_vd();
                    }

                    // New position available?

                    if (m_VHOReInitState_bl != FALSE)
                    {
                        m_VHOUpdateState_enm = g_vhoStateReInit_enm;
                        m_VHOReInitState_bl  = FALSE;
                    }
                    else
                    {
                        if (g_VHOPosBuffer_st.DrivenDistanceMM_si32 !=
                            g_VHOVehicleState_st.DrivenDistance_si32)
                        {
                            m_VHOUpdateState_enm = g_vhoStateUpdated_enm;
                        }
                        else
                        {
                            m_VHOUpdateState_enm = g_vhoStateNotUpdated_enm;
                        }
                    }
                }
                else
                {
                    g_VHOCtrlStat_st.VHOBypass_bl = FALSE;
                    m_VHOInit_vd(0, 0, 0, 1, 0);
                    // ReInit state only relevant for one cycle because called by VHO
                    m_VHOReInitState_bl = FALSE;

                    g_VHOOdoBuf_st.WIC_CanTime_buf_ui16 = g_VHOCanSig_st.WIC_CanTime_ui16;
                }
            }
            else
            {
                m_VHOUpdateState_enm = g_vhoStateNotUpdated_enm;

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))
                // velocity is above shutdown threshold; replace velocity with
                // velocity read from can, convert from 0.1km/h to 1/256m/s
                // vel_conv = (vel * 256) / (3.6 *10)
                g_VHOVehicleState_st.VehicleVelocity_ui16 =
                    g_VHOVehicleState_st.VehicleVelCanMPerS_ui16;
#endif
#endif

                // write velocity to velocity ring buffer to prevent discontinuous
                // decrease of the velocity after switching to VHO velocity again
                /*
                m_VHOKinRMAddVel_vd(
                  (UInt16) g_VHODivide_si32(
                    (SInt32) g_VHOVehicleState_st.VehicleVelocity_ui16 * (SInt32) 100,
                (SInt32) 256));
                */
                g_VHOOdoBuf_st.WIC_CanTime_buf_ui16 = g_VHOCanSig_st.WIC_CanTime_ui16;
                g_VHOCtrlStat_st.VHOBypass_bl       = TRUE;
            }
        }
        else
        {
            if (m_VHOReInitState_bl != FALSE)
            {
                m_VHOUpdateState_enm = g_vhoStateReInit_enm;
                m_VHOReInitState_bl  = FALSE;
            }
            else
            {
                m_VHOUpdateState_enm = g_vhoStateNotUpdated_enm;
            }
        }
    }

#elif (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

    // MINI-VHO
    // ================================================================================
    g_VHOMini_vd();

    if (((g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 != 0) ||
         (g_VHORollRecog_st.ds_buf_MMF8_ui32 != 0)) &&
        (g_VHOVehicleState_st.RollingDirection_en != VHO_MOV_UNKNOWN_enm))
    {
        m_VHOUpdateState_enm = g_vhoStateUpdated_enm;
    }
    else
    {
        m_VHOUpdateState_enm = g_vhoStateNotUpdated_enm;
    }
    // =========================================================================================

#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    if (g_VHOOdoBuf_st.WICCanTimeDeltaMS_ui16 != 0)
    {
#endif

        g_VHOVehicleState_st.DrivenDistanceAbsolute_ui32 +=
            (((UInt32)g_mtl_Abs_mac(g_VHOOdoBuf_st.DrivenDistanceMMF8_si32) +
              (UInt32)g_mtl_Abs_mac(g_VHOOdoBuf_st.DrivenDistance_bufF8_si16)) /
             256);
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
#if (GS_4WS_VEHICLE == SW_ON)
        {
            UInt32 l_temp_ui32;

            //==============================================================================
            // map absolute driven distance from center of rear axle to instantaneous
            // center
            //==============================================================================
            l_temp_ui32 = g_VHOVehicleState_st.DrivenDistanceAbsolute_ui32;
            l_temp_ui32 = g_mtl_ReducePrecision_ui32(l_temp_ui32, 2);
            l_temp_ui32 = l_temp_ui32 * (UInt32)g_mtl_Cos_si16(
                                            g_VHOVehicleState_st.SAngRearRadF12_si16);
            l_temp_ui32 = g_mtl_ReducePrecision_ui32(l_temp_ui32, 12); // F20 -> F8
            g_VHOVehicleState_st.DrivenDistanceAbsoluteInstantCenter_ui32 = l_temp_ui32;
        }
#endif
#endif

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    }
    else
    { /*IGNORE*/
    }
#endif

} // g_VHOCalculateNewPosition_ui8()

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
******************************************************************************/
/**
 * @brief         will test whether all the wic values that are needed in the
 *                current setting have been updated; in this case the wic
 *                buffers will be initialised and init mode will be exited
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
static void m_VHOInitWIC_vd(void)
{
    UInt8 l_I_ui8;
    Boolean l_WICUpdated_bl;

    m_VHOUpdateState_enm = g_vhoStateInit_enm;

    if (g_VHOCanSig_st.WIC_CanTime_ui16 == 0)
    {
        l_WICUpdated_bl = FALSE;
    }
    else
    {
        l_WICUpdated_bl = TRUE;
    }

    if (l_WICUpdated_bl != FALSE)
    {
        // all wic values, that are used in the chosen configuration have been
        // initialized =>exit init mode
        g_VHOCtrlStat_st.InitialiseWIC_ui8 = (UInt8)0;

        // set WIC-CAN-Time buffer to current value
        g_VHOOdoBuf_st.WIC_CanTime_buf_ui16 = g_VHOCanSig_st.WIC_CanTime_ui16;

        // write current wic info to wic buffers
        for (l_I_ui8 = 0; l_I_ui8 < 5; l_I_ui8++)
        {
            g_VHOOdoBuf_st.WICBuf_pui16[l_I_ui8] = g_VHOCanSig_st.WICRaw_pui16[l_I_ui8];
        }
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         2WS: buffer current steering wheel angle
 *                4WS: buffer current steering wheel angle and rear wheel angle
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_4WS_VEHICLE == SW_ON)
VISIBILITY void m_VHOInitBufferSWAandSAR_vd(SInt16 f_CurrentSWA_si16,
                                            SInt16 f_CurrentSAR_si16)
#else
VISIBILITY void m_VHOInitBufferSWA_vd(SInt16 f_CurrentSWA_si16)
#endif
{
    UInt8 l_I_ui8;

    // define SWA ring memory size that is used depending on
    // g_parGetParaVHO_Odo_SWADelayUnitCM Example:
    // g_parGetParaVHO_Odo_SWADelayUnitCM = 20cm
    //          SWA has to be stored for 20cm, 18cm, 16cm, ..., 2cm, 0cm
    //          Thus, size of array = (g_parGetParaVHO_Odo_SWADelayUnitCM/2) + 1

    if ((g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 % 2) != 0)
    {
        // if g_parGetParaVHO_Odo_SWADelayUnitCM is an odd number then round up to
        // next even number
        g_VHOOdoBuf_st.SWARMSize_ui8 =
            ((g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 + 1) / 2) + 1;
    }
    else
    {
        g_VHOOdoBuf_st.SWARMSize_ui8 = (g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 / 2) + 1;
    }

    if (g_VHOOdoBuf_st.SWARMSize_ui8 > 1)
    {
        for (l_I_ui8 = 0; l_I_ui8 < g_VHOOdoBuf_st.SWARMSize_ui8; l_I_ui8++)
        {
#if (GS_4WS_VEHICLE == SW_ON)
            m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF][l_I_ui8] = f_CurrentSWA_si16;
            m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF][l_I_ui8] = f_CurrentSAR_si16;
#else
            m_RMSWABuff_psi16[l_I_ui8] = f_CurrentSWA_si16;
#endif
        }
        m_RMSWABuffLastElem_ui8 = g_VHOOdoBuf_st.SWARMSize_ui8 - 1;
        m_RMSWABuffOldestElem_ui8 =
            (m_RMSWABuffLastElem_ui8 + 1) % g_VHOOdoBuf_st.SWARMSize_ui8;
    }
    else
    {
#if (GS_4WS_VEHICLE == SW_ON)
        m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF][0] = f_CurrentSWA_si16;
        m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF][0] = f_CurrentSAR_si16;
#else
        m_RMSWABuff_psi16[0] = f_CurrentSWA_si16;
#endif
        m_RMSWABuffLastElem_ui8   = 0;
        m_RMSWABuffOldestElem_ui8 = m_RMSWABuffLastElem_ui8;
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         2WS: buffer current steering wheel angle
 *                4WS: buffer current steering wheel angle and rear wheel angle
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_4WS_VEHICLE == SW_ON)
void g_VHOBufferSWAandSAR_vd(SInt16 f_CurrentSWA_si16, SInt16 f_CurrentSAR_si16)
#else
void g_VHOBufferSWA_vd(SInt16 f_CurrentSWA_si16)
#endif
{
    UInt8 l_I_ui8;
    UInt8 l_RMElementCounter_ui8;        // how many elements in the ring buffer have to
                                         // be filled
    UInt16 l_RoundedEvenDrivDistMM_ui16; // rounded even driven distance in cm in
                                         // current cycle
    UInt16 l_RMDeltaDistMM_ui16;         // driven distance in current cycle in mm

    // calculation of driven distance in current cycle in mm
    l_RMDeltaDistMM_ui16 = (UInt16)g_mtl_Abs_mac(
        g_mtl_ReducePrecision_si32(g_VHOOdoBuf_st.DrivenDistanceMMF8_si32, 8));

    // check if actual distance + "lost" distance of last cycle (might be negative
    // if it was rounded up in last cycle) is positive or if still negative
    if (((SInt16)l_RMDeltaDistMM_ui16 + (SInt16)g_VHOOdoBuf_st.SWARMLostDistMM_si8) > 0)
    {
        // consider lost distance from last cycle
        l_RMDeltaDistMM_ui16 = (UInt16)g_mtl_Abs_mac(
            (SInt16)l_RMDeltaDistMM_ui16 + (SInt16)g_VHOOdoBuf_st.SWARMLostDistMM_si8);

        // set l_RoundedEvenDrivDistMM_ui16 = l_RMDeltaDistMM_ui16 as intermediate
        // step
        l_RoundedEvenDrivDistMM_ui16 = l_RMDeltaDistMM_ui16;

        // one SWA element in ring memory corresponds to a driven distance of 2 cm
        // (=> % 20 calculation) for calculation of number of ring elements that
        // have to be filled:
        // - rounding down, if driven distance in current cycle % 20 <= 10
        if ((l_RoundedEvenDrivDistMM_ui16 % 20) <= 10)
        {
            l_RoundedEvenDrivDistMM_ui16 -= (l_RoundedEvenDrivDistMM_ui16 % 20);
        }
        // - rounding up, if driven distance in current cycle % 20 > 10
        else
        {
            l_RoundedEvenDrivDistMM_ui16 -= (l_RoundedEvenDrivDistMM_ui16 % 20);
            l_RoundedEvenDrivDistMM_ui16 += 20;
        }
        // store driven distance that was not considered
        g_VHOOdoBuf_st.SWARMLostDistMM_si8 =
            (SInt8)(l_RMDeltaDistMM_ui16 - l_RoundedEvenDrivDistMM_ui16);
        // calculation of numbers of ring memory elements that have to be filled
        // with current SWA
        l_RMElementCounter_ui8 = (UInt8)(l_RoundedEvenDrivDistMM_ui16 / 20);
    }
    else // i.e. actual distance + "lost" distance of last cycle still negative
         // => overwrite last RM element
    {
        g_VHOOdoBuf_st.SWARMLostDistMM_si8 =
            (SInt8)((SInt16)l_RMDeltaDistMM_ui16 +
                    (SInt16)g_VHOOdoBuf_st.SWARMLostDistMM_si8);
        l_RMElementCounter_ui8 = 0;
    }

    // write SWA to ring memory
    if (g_VHOOdoBuf_st.SWARMSize_ui8 > 1)
    {
        if ((g_VHOVehicleState_st.VehicleVelCanMPerS_ui16 == 0) &&
            (g_VHOOdoBuf_st.DrivenDistanceMMF8_si32 == 0))
        {
#if (GS_4WS_VEHICLE == SW_ON)
            m_VHOInitBufferSWAandSAR_vd(f_CurrentSWA_si16, f_CurrentSAR_si16);
#else
            m_VHOInitBufferSWA_vd(f_CurrentSWA_si16);
#endif
        }
        else if (l_RMElementCounter_ui8 == 0)
        {
#if (GS_4WS_VEHICLE == SW_ON)
            m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF][m_RMSWABuffLastElem_ui8] =
                f_CurrentSWA_si16;
            m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF][m_RMSWABuffLastElem_ui8] =
                f_CurrentSAR_si16;
#else
            m_RMSWABuff_psi16[m_RMSWABuffLastElem_ui8] = f_CurrentSWA_si16;
#endif
        }
        else if (l_RMElementCounter_ui8 == 1)
        {
#if (GS_4WS_VEHICLE == SW_ON)
            m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF][m_RMSWABuffOldestElem_ui8] =
                f_CurrentSWA_si16;
            m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF][m_RMSWABuffOldestElem_ui8] =
                f_CurrentSAR_si16;
#else
            m_RMSWABuff_psi16[m_RMSWABuffOldestElem_ui8] = f_CurrentSWA_si16;
#endif
            m_RMSWABuffLastElem_ui8 = m_RMSWABuffOldestElem_ui8;
            m_RMSWABuffOldestElem_ui8 =
                (m_RMSWABuffOldestElem_ui8 + 1) % g_VHOOdoBuf_st.SWARMSize_ui8;
        }
        else if (l_RMElementCounter_ui8 <= g_VHOOdoBuf_st.SWARMSize_ui8)
        {
            for (l_I_ui8 = 0; l_I_ui8 < l_RMElementCounter_ui8; l_I_ui8++)
            {
#if (GS_4WS_VEHICLE == SW_ON)
                m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF]
                                       [(m_RMSWABuffOldestElem_ui8 + l_I_ui8) %
                                        g_VHOOdoBuf_st.SWARMSize_ui8] = f_CurrentSWA_si16;
                m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF]
                                       [(m_RMSWABuffOldestElem_ui8 + l_I_ui8) %
                                        g_VHOOdoBuf_st.SWARMSize_ui8] = f_CurrentSAR_si16;
#else
                m_RMSWABuff_psi16[(m_RMSWABuffOldestElem_ui8 + l_I_ui8) %
                                  g_VHOOdoBuf_st.SWARMSize_ui8] = f_CurrentSWA_si16;
#endif
            }
            m_RMSWABuffLastElem_ui8 =
                (m_RMSWABuffOldestElem_ui8 + (l_RMElementCounter_ui8 - 1)) %
                g_VHOOdoBuf_st.SWARMSize_ui8;
            m_RMSWABuffOldestElem_ui8 =
                (m_RMSWABuffOldestElem_ui8 + l_RMElementCounter_ui8) %
                g_VHOOdoBuf_st.SWARMSize_ui8;
        }
        else
        {
#if (GS_4WS_VEHICLE == SW_ON)
            m_VHOInitBufferSWAandSAR_vd(f_CurrentSWA_si16, f_CurrentSAR_si16);
#else
            m_VHOInitBufferSWA_vd(f_CurrentSWA_si16);
#endif
        }
    }
    else
    {
#if (GS_4WS_VEHICLE == SW_ON)
        m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF][0] = f_CurrentSWA_si16;
        m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF][0] = f_CurrentSAR_si16;
#else
        m_RMSWABuff_psi16[0] = f_CurrentSWA_si16;
#endif
        m_RMSWABuffLastElem_ui8   = 0;
        m_RMSWABuffOldestElem_ui8 = m_RMSWABuffLastElem_ui8;
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief
 * @details       2WS: returns steering wheel angle that has been valid
 *                     g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 centimeters before
 *
 *                4WS: returns steering wheel angle that has been valid
 *                     g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 centimeters before
 *                     and
 *                     returns steering angle rear that has been valid
 *                     g_parGetParaVHO_Odo_SWADelayUnitCM_ui8 centimeters before
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_4WS_VEHICLE == SW_ON)
SInt16 g_VHOGetSWAFromBuf_si16(void)
{
    return m_RMSWAandSARBuff_psi16[md_RM_SWA_ARRAY_BUFF][m_RMSWABuffOldestElem_ui8];
}
SInt16 g_VHOGetSARFromBuf_si16(void)
{
    return m_RMSWAandSARBuff_psi16[md_RM_SAR_ARRAY_BUFF][m_RMSWABuffOldestElem_ui8];
}
#else
SInt16 g_VHOGetSWAFromBuf_si16(void)
{
    return m_RMSWABuff_psi16[m_RMSWABuffOldestElem_ui8];
}
#endif
#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         convert VHS rolling direction enum into vho rolling direction
 * enum
 * @details
 *
 * @param
 * @return        vho rolling direction enum (forw,backw,unkn,stop)
 *
 * @note
 * @see
 * @warning
 */
#if ((GS_VHS_WHEEL_DIR_RR == SW_ON) || (GS_VHS_WHEEL_DIR_RL == SW_ON) || \
     (GS_VHS_WHEEL_DIR_FR == SW_ON) || (GS_VHS_WHEEL_DIR_FL == SW_ON))
#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||     \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC) || \
     (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL))
VISIBILITY gType_VHORollRecogState_en
m_VHONormVHSWicMoveDir_en(gType_WheelDir_en f_VHSWicMoveDir_en)
{
    gType_VHORollRecogState_en l_VHOWicMoveDir_en;

    switch (f_VHSWicMoveDir_en)
    {
        case WheelDirForward_enm:
            l_VHOWicMoveDir_en = VHO_MOV_FORW_enm;
            break;
        case WheelDirBackward_enm:
            l_VHOWicMoveDir_en = VHO_MOV_BACKW_enm;
            break;
        case WheelDirUnknown_enm:
            l_VHOWicMoveDir_en = VHO_MOV_UNKNOWN_enm;
            break;
        case WheelDirStop_enm:
            l_VHOWicMoveDir_en = VHO_STOP_enm;
            break;
        default:
            l_VHOWicMoveDir_en = VHO_MOV_UNKNOWN_enm;
            break;
    }

    return l_VHOWicMoveDir_en;
}
#endif // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_WIC) ||
       // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_YAW_WIC)
       // (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ALL)
#endif // (GS_VHS_WHEEL_DIR_RR == SW_ON) || ...

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         returns if the position was updated or not or if the VHO
 *                was initialized
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
gType_vhoUpdateState_en g_VHOGetUpdateState_en(void) { return m_VHOUpdateState_enm; }

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         write yaw angle to ring memory
 * @details
 *
 * @param         f_CurrentYAng_ui32
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOWriteYAngHist_vd(UInt32 f_CurrentYAng_ui32)
{
    SInt32 l_DrivDistCheck_si32;        // driven distance in cm
    SInt32 l_DrivDistCurCycleMMF8_si32; // absolute value of driven distance in
                                        // current cycle

    l_DrivDistCurCycleMMF8_si32 = g_mtl_Abs_mac(g_VHOOdoBuf_st.DrivenDistanceMMF8_si32);

    // calculation of driven distance for current ring memory element
    m_VHOYawAngleHist_st.SNewestMMF8_si32 =
        m_VHOYawAngleHist_st.SNewestMMF8_si32 + l_DrivDistCurCycleMMF8_si32;

    // precision reduction for check
    l_DrivDistCheck_si32 =
        g_mtl_ReducePrecision_si32(m_VHOYawAngleHist_st.SNewestMMF8_si32, 8);

    // check if driven distance > gd_VHOYAngHistRMSize_ui8 -> fill up next ring
    // memory element
    if (l_DrivDistCheck_si32 > gd_VHO_YANGHIST_STEPSIZE_MM_ui16)
    {
        // increment array index
        m_VHOYawAngleHist_st.IdNewest_ui8 =
            (m_VHOYawAngleHist_st.IdNewest_ui8 + 1) % gd_VHOYAngHistRMSize_ui8;
        if (m_VHOYawAngleHist_st.IdMax_ui8 < m_VHOYawAngleHist_st.IdNewest_ui8)
        {
            m_VHOYawAngleHist_st.IdMax_ui8 = m_VHOYawAngleHist_st.IdNewest_ui8;
        }
        m_VHOYawAngleHist_st.SNewestMMF8_si32 = l_DrivDistCurCycleMMF8_si32;
        m_VHOYawAngleHist_st.YAng_psi16[m_VHOYawAngleHist_st.IdNewest_ui8] =
            g_vho_mtl_AngleF22ToF12_si16(f_CurrentYAng_ui32);
    }
    else
    {
        m_VHOYawAngleHist_st.YAng_psi16[m_VHOYawAngleHist_st.IdNewest_ui8] =
            g_vho_mtl_AngleF22ToF12_si16(f_CurrentYAng_ui32);
    }
}

/**
 * @brief         initialization of yaw angle history buffer
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOInitBufferYAngHist_vd(void)
{
    UInt8 l_I_ui8;

    m_VHOYawAngleHist_st.IdMax_ui8        = 0;
    m_VHOYawAngleHist_st.IdNewest_ui8     = 0;
    m_VHOYawAngleHist_st.SNewestMMF8_si32 = 0;

    for (l_I_ui8 = 0; l_I_ui8 < gd_VHOYAngHistRMSize_ui8; l_I_ui8++)
    {
        m_VHOYawAngleHist_st.YAng_psi16[l_I_ui8] = 0x7FFF;
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         reinitialization of yaw angle history buffer
 * @details
 *
 * @param         f_CurrentYAng_ui32
 * @param         f_OdoBufYawAngle_ui32
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOReInitBufferYAngHist_vd(UInt32 f_CurrentYAng_ui32,
                                             UInt32 f_OdoBufYawAngle_ui32)
{
    UInt8 l_I_ui8;
    UInt32 l_YawAngNew_ui32;
    UInt32 l_YawAngDiff_ui32;

    l_YawAngDiff_ui32 =
        g_vho_mtl_AngleSub_ui32(f_OdoBufYawAngle_ui32, f_CurrentYAng_ui32);

    for (l_I_ui8 = 0; l_I_ui8 < gd_VHOYAngHistRMSize_ui8; l_I_ui8++)
    {
        if (m_VHOYawAngleHist_st.YAng_psi16[l_I_ui8] != 0x7FFF)
        {
            l_YawAngNew_ui32 = g_vho_mtl_AngleSub_ui32(
                g_vho_mtl_AngleF12ToF22_ui32(m_VHOYawAngleHist_st.YAng_psi16[l_I_ui8]),
                l_YawAngDiff_ui32);
            m_VHOYawAngleHist_st.YAng_psi16[l_I_ui8] =
                g_vho_mtl_AngleF22ToF12_si16(l_YawAngNew_ui32);
        }
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         returns VHO yaw angle history
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */

gType_VHOYawAngleHist_st const *g_vhoGetYAngHist_pst(void)
{
    return &m_VHOYawAngleHist_st;
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         returns VHO yaw angle history stepsize
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */

UInt16 g_vhoGetStepsizeYAngHistMM_ui16(void) { return gd_VHO_YANGHIST_STEPSIZE_MM_ui16; }

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         write position data to ring memory for USS position mapping
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOWriteWICHistUSS_vd(void)
{
    Boolean l_WritePos_bl;
    UInt8 l_IdNewest_ui8;
    UInt8 l_I_ui8;

    l_WritePos_bl = TRUE;

    for (l_I_ui8 = 0; l_I_ui8 < g_VHOWICHistoryUSS_st.PosRMSize_ui8; l_I_ui8++)
    {
        // check if timestamp already available -> if not, write new value
        if (g_VHOCanSig_st.WIC_CanTime_ui16 ==
            g_VHOWICHistoryUSS_st.WIC_CanTime_pui16[l_I_ui8])
        {
            l_WritePos_bl = FALSE;
        }
    }

    if (l_WritePos_bl != FALSE)
    {
        // increment IdNewest for current cycle if first element was already filled
        // up
        if (g_VHOWICHistoryUSS_st.IdMax_ui8 != 0)
        {
            g_VHOWICHistoryUSS_st.IdNewest_ui8 =
                (g_VHOWICHistoryUSS_st.IdNewest_ui8 + 1) %
                g_VHOWICHistoryUSS_st.PosRMSize_ui8;
        }

        // change IdOldest for current cycle only if position buffer was completely
        // filled
        if ((g_VHOWICHistoryUSS_st.IdMax_ui8 ==
             (g_VHOWICHistoryUSS_st.PosRMSize_ui8 - 1)) &&
            (g_VHOWICHistoryUSS_st.IdNewest_ui8 == g_VHOWICHistoryUSS_st.IdOldest_ui8))
        {
            g_VHOWICHistoryUSS_st.IdOldest_ui8 =
                (g_VHOWICHistoryUSS_st.IdOldest_ui8 + 1) %
                g_VHOWICHistoryUSS_st.PosRMSize_ui8;
        }

        // IdMax is increased after check for incrementing IdOldest because IdOldest
        // should not be increased as long as position buffer is not completely
        // filled up.
        if ((g_VHOWICHistoryUSS_st.IdMax_ui8 < g_VHOWICHistoryUSS_st.IdNewest_ui8) ||
            (g_VHOWICHistoryUSS_st.IdMax_ui8 == 0))
        {
            g_VHOWICHistoryUSS_st.IdMax_ui8++;
        }

        l_IdNewest_ui8 = g_VHOWICHistoryUSS_st.IdNewest_ui8;

        // write required values for mapping of USS data to vehicle position to
        // position buffer
        g_VHOWICHistoryUSS_st.WIC_CanTime_pui16[l_IdNewest_ui8] =
            g_VHOCanSig_st.WIC_CanTime_ui16;
        g_VHOWICHistoryUSS_st.XPositionMM_psi32[l_IdNewest_ui8] =
            g_VHOVehicleState_st.XPosition_si32;
        g_VHOWICHistoryUSS_st.YPositionMM_psi32[l_IdNewest_ui8] =
            g_VHOVehicleState_st.YPosition_si32;
        g_VHOWICHistoryUSS_st.DrivenDistanceMM_psi32[l_IdNewest_ui8] =
            g_VHOVehicleState_st.DrivenDistance_si32;
        g_VHOWICHistoryUSS_st.YawAngleRad_F22_pui32[l_IdNewest_ui8] =
            g_VHOVehicleState_st.YawAngle_ui32;
    }
}

/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         initialization of position history buffer for USS position
 * mapping
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
VISIBILITY void m_VHOInitBufferWICHistUSS_vd(void)
{
    UInt8 l_RMSize_ui8;

    (void)memset(&g_VHOWICHistoryUSS_st, 0, sizeof(gType_VHOWICHistoryUSS_st));

    l_RMSize_ui8 =
        gd_VHOWICHistUSSRMSizeMS_ui8 / g_parGetParaVHO_Veh_WIC_CycleTime_MS_ui8;
    if (l_RMSize_ui8 <= gd_VHOMaxWICHistUSSRMSize_ui8)
    {
        // just in case WIC cycle time >= 100ms
        if (l_RMSize_ui8 <= 1)
        {
            g_VHOWICHistoryUSS_st.PosRMSize_ui8 = 2;
        }
        else
        {
            g_VHOWICHistoryUSS_st.PosRMSize_ui8 = l_RMSize_ui8;
        }
    }
    else
    {
        g_VHOWICHistoryUSS_st.PosRMSize_ui8 = gd_VHOMaxWICHistUSSRMSize_ui8;
    }
}

#if (GS_4WS_VEHICLE == SW_ON)
/*****************************************************************************
|-----------------------------------------------------------------------------
| F U N C T I O N    D E S C R I P T I O N
|-----------------------------------------------------------------------------
*****************************************************************************/
/**
 * @brief         calculation of virtual wheel base according to four wheel
 *                steering based on single track model.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_VHOCalcVirtualWheelBase_vd(void)
{
    // temporary variables
    SInt16 l_SteeringAngleFront_si16;
    SInt16 l_SteeringAngleRear_si16;
    UInt16 l_TanAbsSteeringAngleFront_ui16;
    UInt16 l_TanAbsSteeringAngleRear_ui16;
    UInt16 l_SumAbsTanSteeringAngle_ui16;
    UInt16 l_AbsDeltaAbsTanSteeringAngle_ui16;

    UInt16 l_WheelBase_ui16;
    // end of temporary variables

    // steering rear angle in the opossite direction as steering front angle
    // in this case virtual wheel base is less or equal than real wheel base
    //
    // SAF :=: delayed and offset corrected steering angle front
    // SAR :=: delayed steering angle rear
    //
    //                                                |  tan(SteeringAngleFront) |
    // WheelBaseVirtual = WheelBase *
    // ----------------------------------------------------------
    //                                |  tan(SteeringAngleFront) | + |
    //                                tan(SteeringAngleRear) |
    //
    //
    // steering rear angle in the same direction as steering front angle
    // in this case virtual wheel base is greater or equal than treal wheel base
    //
    //                                                |  tan(SteeringAngleFront) |
    // WheelBaseVirtual = WheelBase *
    // ----------------------------------------------------------
    //                                |  tan(SteeringAngleFront) | - |
    //                                tan(SteeringAngleRear) |
    //

    // fetch steering angle; Scaling: 1/4096rad; Range:
    // ]-1.099557rad(63�)..1.099557rad(63�)[
    l_SteeringAngleFront_si16 = g_VHOCorrectedSignals_st.DelayedCorr_SAngRadF12_si16;
    l_SteeringAngleRear_si16  = g_VHOCorrectedSignals_st.DelayedCorr_SAngRearRadF12_si16;
    // fetch real wheel base; Scaling: 1 LSB = 1 mm
    l_WheelBase_ui16 = g_VHOParameters_st.VHO_Veh_pst->WheelBase_ui16;

    // get tan(steering angle front); Scaling F13 = 1/2^13
    l_TanAbsSteeringAngleFront_ui16 =
        (UInt16)g_mtl_Tan_si16(g_mtl_Abs_mac(l_SteeringAngleFront_si16));

    // get tan(steering angle rear); Scaling F13 = 1/2^13
    l_TanAbsSteeringAngleRear_ui16 =
        (UInt16)g_mtl_Tan_si16(g_mtl_Abs_mac(l_SteeringAngleRear_si16));

    // absolute difference between l_TanAbsSteeringAngleFront_si16 and
    // l_TanAbsSteeringAngleRear_si16
    l_AbsDeltaAbsTanSteeringAngle_ui16 = (UInt16)g_mtl_Abs_mac(
        (SInt16)l_TanAbsSteeringAngleFront_ui16 - (SInt16)l_TanAbsSteeringAngleRear_ui16);

    // absolute sum of l_TanAbsSteeringAngleFront_si16 and
    // l_TanAbsSteeringAngleRear_si16
    l_SumAbsTanSteeringAngle_ui16 =
        l_TanAbsSteeringAngleFront_ui16 + l_TanAbsSteeringAngleRear_ui16;

    // calculate virtual wheel base
    if ((g_mtl_Abs_mac(l_SteeringAngleFront_si16) < ((SInt16)12)) ||
        (g_mtl_Abs_mac(l_SteeringAngleRear_si16) < ((SInt16)8)) ||
        (l_AbsDeltaAbsTanSteeringAngle_ui16 < ((UInt16)7)))
    {
        g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 = l_WheelBase_ui16;

        g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 = ((SInt16)0);

        // forbidden steering angle rear has not been achieved
        g_VHOWheelBaseVirtual_st.ForbiddenRearAngleZone_bl = FALSE;
    }
    else
    {
        if (((l_SteeringAngleFront_si16 <= ((SInt16)-12)) &&
             (l_SteeringAngleRear_si16 <= ((SInt16)-8))) ||
            ((l_SteeringAngleFront_si16 >= ((SInt16)12)) &&
             (l_SteeringAngleRear_si16 >= ((SInt16)8))))
        {
            // steering angle rear is in the same direction as steering angle front
            // virtual wheel base
            if (l_TanAbsSteeringAngleFront_ui16 > l_TanAbsSteeringAngleRear_ui16)
            {
                g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 =
                    g_mtl_u16_Mul_u16_Div_u16_ui16(l_WheelBase_ui16,
                                                   l_TanAbsSteeringAngleFront_ui16,
                                                   l_AbsDeltaAbsTanSteeringAngle_ui16);

                // check possible overflow before calculation of delta virtual wheel
                // base
                if (g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 >=
                    (SInt16)(l_WheelBase_ui16))
                {
                    // Definition: if steering in the same direction is given,
                    // DeltaWheelBaseVirtual_si16 is >= 0
                    g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 =
                        (SInt16)(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) -
                        (SInt16)(l_WheelBase_ui16);
                }
                else
                {
                    g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 = (SInt16)(0);
                }

                // forbidden steering angle rear zone has not been achieved
                g_VHOWheelBaseVirtual_st.ForbiddenRearAngleZone_bl = FALSE;
            }
            else
            {
                // Forbidden steering angle rear zone has been achieved.
                // Calculation of wheel base virtual does not make sense in this case.
                // Set wheel base virtual to wheel base and delta virtual wheel base to
                // zero.
                g_VHOWheelBaseVirtual_st.ForbiddenRearAngleZone_bl  = TRUE;
                g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16      = l_WheelBase_ui16;
                g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 = (SInt16)(0);
            }
        }
        else
        {
            // steering angle rear is in the opposite direction as steering angle
            // front virtual wheel base
            g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 =
                g_mtl_u16_Mul_u16_Div_u16_ui16(l_WheelBase_ui16,
                                               l_TanAbsSteeringAngleFront_ui16,
                                               l_SumAbsTanSteeringAngle_ui16);

            // check possible overflow before calculation of delta virtual wheel base
            if (g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16 <=
                (SInt16)(l_WheelBase_ui16))
            {
                // Definition: if opposite steering is given, DeltaWheelBaseVirtual_si16
                // is <= 0
                g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 =
                    (SInt16)(g_VHOWheelBaseVirtual_st.WheelBaseVirtual_ui16) -
                    (SInt16)(l_WheelBase_ui16);
            }
            else
            {
                g_VHOWheelBaseVirtual_st.DeltaWheelBaseVirtual_si16 = (SInt16)(0);
            }

            // forbidden steering angle rear zone has not been achieved
            g_VHOWheelBaseVirtual_st.ForbiddenRearAngleZone_bl = FALSE;
        }
    }
}
#endif

#endif

#ifdef QAC_MSG_OFF
#pragma PRQA_MESSAGES_ON 3892
#endif
