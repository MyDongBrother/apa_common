/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_uss_proc.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_USS_PROC_H
#define VHO_USS_PROC_H

#ifdef COMPONENT_VHO
// #include <uss/mp_if/mp_base_type.h>
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)

void g_VHOUSSProcessingInit_vd(void);

UInt8 g_VHOUSSProcessingGetEstimatedPosition_ui8(
    gType_VHOEstimatedPosition_st *f_EstimatedPosition_pst, UInt16 f_TargetCANTime_ui16);

void g_VHOUSSProcessingBufferPosition_vd(void);
#if 0
// This function needs an include of mp_api.h
extern UInt8 g_VHOUSSProcessingStoreDistList_ui8(
    UInt8 f_SensorNumber_ui8, const gType_adiEchoDataSet_st *fc_EchoData_pst,
    UInt16 f_USSPeakListTime_ui16, UInt16 f_CurrentUSSTime_ui16,
    UInt16 f_CurrentCANTime_ui16);
#endif
#endif //  COMPONENT_VHO

#endif // VHO_USS_PROC_H

#endif
