/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_odo_core.h
 * @brief
 * @details
 *
 *****************************************************************************/
#ifndef VHO_ODO_CORE_H
#define VHO_ODO_CORE_H

#ifdef COMPONENT_VHO

typedef struct
{
    SInt32 XF8_mm_si32;
    SInt32 YF8_mm_si32;
    SInt32 PsiF22_rad_si32;
#if (GS_VHO_RUN_MODE == GS_VHO_CONSIDER_USS_DATA)
    SInt32 PsiDualTrackF22_rad_si32;
    SInt32 PsiSingleTrackF22_rad_si32;
#endif
} mType_DeltaValues_st;

void m_VHOPosCalcGetPosChanges_vd(mType_DeltaValues_st *f_DeltaValues_pst,
                                  SInt32 f_DrivenDistF8_si32,
                                  UInt32 f_YawAngleF22_rad_ui32);
/****************************************************************************/
UInt32 m_VHONormAngleRadF22_ui32(SInt32 l_InputAngleRadF22_si32);
/****************************************************************************/
void m_VHOOdoCoreCoordTransform_vd(const gType_VHORollRecog_st *fc_VHORollRecogBuf_pst,
                                   mType_DeltaValues_st *f_DeltaValues_pst,
                                   SInt16 f_YawAngleRadF10_si16);
/****************************************************************************/
void m_VHOPosCalcUpdateVehicleState_vd(const mType_DeltaValues_st *fc_DeltaValues_pst,
                                       SInt32 f_DrivenDistance_si32);
/****************************************************************************/
void g_VHOCalculateModelDependentPosition_vd(void);
/****************************************************************************/
SInt16 g_VHOGetCurvatureFromSteeringAngle_si16(SInt16 f_SteeringAngleRadF12_si16);
#if (GS_4WS_VEHICLE == SW_ON)
SInt16 g_VHOGetCurvatureFrPSCctualSteeringAngle_si16(SInt16 f_SteeringAngleRadF12_si16);
#endif
/****************************************************************************/
void g_VHOOdoSetup_vd(void);
/****************************************************************************/
#endif // COMPONENT_VHO
#endif // VHO_ODO_CORE_H
