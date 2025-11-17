/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_dir_recog.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_DIR_RECOG_H
#define VHO_DIR_RECOG_H

#ifdef COMPONENT_VHO

#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
gType_VHORollRecogState_en g_VHODirRecogGetCurrentDirection_en(UInt8 f_VirtCanMsgRcv_ui8,
                                                               UInt8 f_ForceOutput_ui8);
#endif

void g_VHODirRecogInit_vd(void);

UInt16 g_VHOg_VHOCalcWICTimeDeltaMS_ui16(void);

extern gType_VHORollRecogState_en m_VHODirRecogMainGear_en(void);

extern gType_VHORollRecogState_en m_VHODirRecogMainWIC_en(void);

#endif // COMPONENT_VHO
#endif