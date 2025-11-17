/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_sig_proc.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_SIG_PROC_H
#define VHO_SIG_PROC_H

#ifdef COMPONENT_VHO

#define gd_VCALTTCMinCyclesUntilStable_ui8  ((UInt8)2)
#define gd_VCALTyreMinCyclesUntilStable_ui8 ((UInt8)2)
#define gd_VCALSWAMinCyclesUntilStable_ui8  ((UInt8)2)

void g_VHOSignalPreproc_vd(void);
UInt8 g_VHOCalibrate_ui8(void);
UInt8 g_ResetDiagCalibInterface_ui8(void);

// necessary for delayed SWA
void g_VHOSWAPreprocSetCorrectedSWAToRawSWA_vd(void);

#if (GS_4WS_VEHICLE == SW_ON)
// necessary for delayed steering angle rear
void g_VHOSARPreprocSetCorrectedSARToRawSAR_vd(void);
#endif

#if (LS_VHO_AUTOCALIB_SWA_OFFSET == SW_ON)
void g_VHOSWAPreprocCalcCorrWithSWAOff_vd(void);
#endif

#endif // COMPONENT_VHO
#endif // VHO_SIG_PROC_H
