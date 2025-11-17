/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_kin.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_KIN_H
#define VHO_KIN_H

#ifdef COMPONENT_VHO

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))

#define md_RM_ELEM_OLDEST_ui8 ((UInt8)255)

#define m_VHOKinRMGetAxSync_si16() (g_VHOKinRMGetAcc_si16(4))
#endif // #if( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) ||
       // (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON) )

SInt32 g_VHOPosCalcGetDrivenDistance_si32(void);

UInt8 g_VHOKin_ui8(SInt32 f_DrivenDistanceMMF8_si32);

#if ((LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) || \
     (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON))

void g_VHOKinInit_vd(void);

SInt16 g_VHOKinGetAxDelayed_si16(UInt8 f_ID_ui8);

Boolean g_VHOKinVehicleStandstill_bl(void);

SInt16 g_VHOKinRMGetAcc_si16(UInt8 f_I_ui8);

SInt16 g_VHOKinRMGetAxSyncAvg_si16(UInt8 l_WindowSize_ui8);

SInt16 g_VHOKinRMGetAxGrad_si16(UInt8 f_I_ui8);

void m_VHOKinRMAddVel_vd(UInt16 l_VeloMPerS_ui16);

#endif // #if( (LS_VHO_DIRDETECT_MODE == LS_VHO_DIRDETECT_ACCSENS) ||
       // (LS_VHO_CALC_VEHICLE_ACCELERATION == SW_ON) )

#endif // #if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

#endif // COMPONENT_VHO

#endif // VHO_KIN_H
