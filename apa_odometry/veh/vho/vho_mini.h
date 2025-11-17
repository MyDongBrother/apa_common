/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_mini.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_MINI_H
#define VHO_MINI_H

#ifdef COMPONENT_VHO

#if (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

extern UInt16 g_TimeStampOSEK_MiniVHO_tt;
extern UInt16 g_LastTime_MiniVHO_ui16;
extern UInt16 g_ElapsedTimeMS_ui16;

VISIBILITY void g_VHOGetKappa_MiniVHO_si16(void);
extern void g_VHOMini_vd(void);
VISIBILITY UInt32 g_VHOGetDrivenDist_MiniVHO_ui32(void);
VISIBILITY void g_VHOCalcModelDepentPosition_MiniVHO_vd(void);

#endif // (GS_VHO_RUN_MODE == GS_VHO_MINI_VHO)

#endif // COMPONENT_VHO

#endif // VHO_MINI_H
