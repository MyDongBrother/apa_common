/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: signal_input.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef MODULE_LOG_NAME
#define MODULE_LOG_NAME "signal_input"
#endif

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

// #include <uss/mp_if/mpif_api.h>

#ifdef md_RUN_ON_POSIX
#include <pa_components/debugprintf/debug_log.h>
#include <sys/time.h>
#endif // md_RUN_ON_POSIX

#include <data_io/dataInput_adi.h>

#include "appl_canSignal.h"
#include "sigif_api.h"
#include "signal_input.h"

/*--------------------------------------------------------------------------*/
/*- FUNCTION DECLARE PUBLIC                                                -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- FUNCTION DECLARE PRIVAT                                                -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/* VARIABLE DEFINE                                                          */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- MACRO DECLARE                                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- module implementations                                                 -*/
/*--------------------------------------------------------------------------*/

/************************ signal mapping & process*
 * *****************************/

/**
 * @brief mapping can signal from ethernet to alg.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */

void g_signalMapping_vd(void)
{
    // from eth com
    gType_FVInputCanFrame_st l_canData_st;
    g_adiGetMsgCan_vd(&l_canData_st, sizeof(gType_FVInputCanFrame_st));

    g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeedRC = l_canData_st.ESC_FLWheelSpeedRC;
    g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeedRC = l_canData_st.ESC_FRWheelSpeedRC;
    g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeedRC = l_canData_st.ESC_RLWheelSpeedRC;
    g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeedRC = l_canData_st.ESC_RRWheelSpeedRC;

    g_ApplCanSig_st.RXSignals_st.FL_WicTick_ui16 = l_canData_st.FL_WicTick_ui16;
    g_ApplCanSig_st.RXSignals_st.FR_WicTick_ui16 = l_canData_st.FR_WicTick_ui16;
    g_ApplCanSig_st.RXSignals_st.RL_WicTick_ui16 = l_canData_st.RL_WicTick_ui16;
    g_ApplCanSig_st.RXSignals_st.RR_WicTick_ui16 = l_canData_st.RR_WicTick_ui16;

    g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeedRC_valid =
        l_canData_st.ESC_FLWheelSpeedRC_valid;
    g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeedRC_valid =
        l_canData_st.ESC_FRWheelSpeedRC_valid;
    g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeedRC_valid =
        l_canData_st.ESC_RLWheelSpeedRC_valid;
    g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeedRC_valid =
        l_canData_st.ESC_RRWheelSpeedRC_valid;

    g_ApplCanSig_st.RXSignals_st.ESC_FLWheelDirection = l_canData_st.ESC_FLWheelDirection;
    g_ApplCanSig_st.RXSignals_st.ESC_FRWheelDirection = l_canData_st.ESC_FRWheelDirection;
    g_ApplCanSig_st.RXSignals_st.ESC_RLWheelDirection = l_canData_st.ESC_RLWheelDirection;
    g_ApplCanSig_st.RXSignals_st.ESC_RRWheelDirection = l_canData_st.ESC_RRWheelDirection;

    g_ApplCanSig_st.RXSignals_st.Gear_ui8 = l_canData_st.Gear_ui8;

    g_ApplCanSig_st.RXSignals_st.SteerWheelAngle_si32 = l_canData_st.SteerWheelAngle_si32;

    g_ApplCanSig_st.RXSignals_st.SteerWheelAngle_valid =
        l_canData_st.SteerWheelAngle_valid;

    g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeedKPH_ui16 =
        l_canData_st.ESC_FLWheelSpeedKPH_ui16;
    g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeedKPH_ui16 =
        l_canData_st.ESC_FRWheelSpeedKPH_ui16;
    g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeedKPH_ui16 =
        l_canData_st.ESC_RLWheelSpeedKPH_ui16;
    g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeedKPH_ui16 =
        l_canData_st.ESC_RRWheelSpeedKPH_ui16;

    g_ApplCanSig_st.RXSignals_st.ESC_FLWheelSpeed_valid =
        l_canData_st.ESC_FLWheelSpeed_valid;
    g_ApplCanSig_st.RXSignals_st.ESC_FRWheelSpeed_valid =
        l_canData_st.ESC_FRWheelSpeed_valid;
    g_ApplCanSig_st.RXSignals_st.ESC_RLWheelSpeed_valid =
        l_canData_st.ESC_RLWheelSpeed_valid;
    g_ApplCanSig_st.RXSignals_st.ESC_RRWheelSpeed_valid =
        l_canData_st.ESC_RRWheelSpeed_valid;

    g_ApplCanSig_st.RXSignals_st.VehicleSpeed_ui16  = l_canData_st.VehicleSpeed_ui16;
    g_ApplCanSig_st.RXSignals_st.VehicleSpeed_valid = l_canData_st.VehicleSpeed_valid;

    g_ApplCanSig_st.RXSignals_st.LateralAcce_si16  = l_canData_st.LateralAcce_si16;
    g_ApplCanSig_st.RXSignals_st.LateralAcce_valid = l_canData_st.LateralAcce_valid;

    g_ApplCanSig_st.RXSignals_st.LongitAcce_si16  = l_canData_st.LongitAcce_si16;
    g_ApplCanSig_st.RXSignals_st.LongitAcce_valid = l_canData_st.LongitAcce_valid;

    g_ApplCanSig_st.RXSignals_st.YawRate_si16  = l_canData_st.YawRate_si16;
    g_ApplCanSig_st.RXSignals_st.YawRate_valid = l_canData_st.YawRate_valid;

    g_ApplCanSig_st.RXSignals_st.Temperature_si8    = l_canData_st.Temperature_si8;
    g_ApplCanSig_st.RXSignals_st.TemperatureInvalid = l_canData_st.TemperatureInvalid;

    g_ApplCanSig_st.RXSignals_st.TurnIndicator = l_canData_st.TurnIndicator;

    g_ApplCanSig_st.RXSignals_st.RainfallLevel = l_canData_st.RainfallLevel;
}

/**
 * @brief  mapping odo when using external odo data
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (ODO_EXTERNAL_CFG == SW_ON)

void g_odoMapping_vd(void)
{
#if (ODO_DATA_SOURCE_CFG == ODO_DATA_FROM_ETH)
    // from eth com
    gType_FVInputVehOdoFrame_st l_odoData_st;
    g_adiGetInputMsgOdo_vd(&l_odoData_st, sizeof(gType_FVInputVehOdoFrame_st));

    m_VehPosOdo_external_st.SPosMM_si32   = l_odoData_st.SPosMM_si32;
    m_VehPosOdo_external_st.XPosMM_si32   = l_odoData_st.XPosMM_si32;
    m_VehPosOdo_external_st.YPosMM_si32   = l_odoData_st.YPosMM_si32;
    m_VehPosOdo_external_st.YawAngle_ui32 = l_odoData_st.YawAngle_ui32;

#endif

#if (ODO_DATA_SOURCE_CFG == ODO_DATA_FROM_RTE)
// from rte
#endif
}

#endif

/************************ input process  *****************************/

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

void g_SigIfInputProcessing_vd(void)
{

    g_signalMapping_vd();

#if (ODO_EXTERNAL_CFG == SW_ON)

    g_odoMapping_vd();

#endif

    g_vhsInputProcessing_vd();
}
