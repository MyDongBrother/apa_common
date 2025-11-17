/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      vho_deg.h
 * @brief
 * @details
 *
 *****************************************************************************/

#ifndef VHO_DEG_H
#define VHO_DEG_H

typedef enum
{
    DegMode_VHO_NoDegradation_enm,
    DegMode_VHO_Extrapolation_enm
} gType_vhoDegModes_en;

extern void g_vhoSetDegMode_vd(gType_vhoDegModes_en f_vhoDegMode_en);

#endif /* VHO_DEG_H */
/*========================= EoF (vho_deg.h) ================================*/
