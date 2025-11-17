/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.锛孡td
 * All rights reserved.
 *
 * @file      parm_api.h
 * @brief
 * @details
 *
 *****************************************************************************/

#ifndef PARM_API_H
#define PARM_API_H

#ifdef COMPONENT_PAR // Only defined within this component!

#define APPL_PAR_H_REF 0x090326
#define APPL_PAR_C_REF 0x090326

#endif // COMPONENT_PAR
#ifndef ENABLE_MATHLIB_ONLY
#include <pa_components/base_type/base_typedef.h>

#include "pa_components/parameter/appl_par.h"
#include "pa_components/parameter/gen_par_header.h"
#include "pa_components/parameter/par_bas.h"
#else
#include "../base_type/base_typedef.h">

#include "../parameter/appl_par.h"
#include "../parameter/gen_par_header.h"
#include "../parameter/par_bas.h"
#endif

/*****************************************************************************
 * Interface:
 * g_parGetParaReadAccess_pst (cln)
 *
 * Description:
 * parameter read access
 *
 * Parameters:
 * cln: name of the cluster
 *
 * Return Value:
 * pointer to the parameter cluster
 *
 * Special Considerations:
 * - macro implementation
 * - use g_parSetMinorAndSubVariant_bl() to select the variant
 *
 *****************************************************************************/
#define g_parGetParaReadAccess_pst(cln)            \
    ((gType_par##cln##_st const *)(g_parRAMData_st \
                                       .ClusterPointer_ppvd[gd_PAR_IDX_##cln##_uix]))

// only for combatibility issues - don't use anymore
#define g_parGetParaReadAccess_vd(cln, ptr)                                           \
    do                                                                                \
    {                                                                                 \
        (ptr) = (gType_par##cln##_st const                                            \
                     *)(g_parRAMData_st.ClusterPointer_ppvd[gd_PAR_IDX_##cln##_uix]); \
    } while (0)

/*****************************************************************************
 * Interface:
 * g_parGetParaWriteAccess_pst (cln, minv, subv)
 *
 * Description:
 * parameter write access
 *
 * Parameters:
 * cln: name of the cluster
 * minv: minor variant index
 * subv: subvabriant index
 *
 * Return Value:
 * pointer to the parameter cluster
 *
 * Special Considerations:
 * macro implementation
 *
 *****************************************************************************/
#define g_parGetParaWriteAccess_pst(cln, minv, subv) \
    g_parGetParaWriteAccessCore_pvd(gd_PAR_IDX_WRITE_##cln##_uix, (minv), (subv))

/*****************************************************************************
 * Interface:
 * g_parGetMaxMinorVariants_ui8(cln)
 *
 * Description:
 * get max. number of minor variants for the given cluster
 *
 * Parameters:
 * cln: name of the cluster
 *
 * Return Value:
 * max. number of minor variants
 *
 * Special Considerations:
 * macro implementation
 *
 *****************************************************************************/
#define g_parGetMaxMinorVariants_ui8(cln) \
    g_parGetMaxMinorVariantsCore_ui8(gd_PAR_IDX_##cln##_uix)

/*****************************************************************************
 * Interface:
 * g_parGetMaxSubVariants_ui8 (cln, mvi)
 *
 * Description:
 * get max. number of sub variants for the given cluster and minor variant
 *
 * Parameters:
 * cln: name of the cluster
 * minv: minor variant index
 *
 * Return Value:
 * max. number of sub variants
 *
 * Special Considerations:
 * macro implementation
 *
 *****************************************************************************/
#define g_parGetMaxSubVariants_ui8(cln, mvi) \
    g_parGetMaxSubVariantsCore_ui8(gd_PAR_IDX_##cln##_uix, (mvi))

/*****************************************************************************
 * Interface:
 * g_parSetMinorAndSubVariant_bl (cln, mvi, svi)
 *
 * Description:
 * selection of th minor and sub variant for the parameter read access
 *
 * Parameters:
 * cln: name of the cluster
 * minv: minor variant index
 * subv: subvabriant index
 *
 * Return Value:
 * TRUE if successful - else FALSE
 *
 * Special Considerations:
 * macro implementation
 *
 *****************************************************************************/
#define g_parSetMinorAndSubVariant_bl(cln, mvi, svi) \
    g_parSetMinorAndSubVariantCore_bl(gd_PAR_IDX_##cln##_uix, (mvi), (svi))

/*****************************************************************************
 * Interface:
 * g_parSetParaUpdate_bl (cln)
 *
 * Description:
 * set the parameter update flag/event from the application
 *
 * Parameters:
 * cln: name of the cluster
 *
 * Return Value:
 * -
 *
 * Special Considerations:
 * - call g_parSetParaUpdate_bl (0) to set the update event for all clusters
 * - macro implementation
 *
 *****************************************************************************/
#define g_parSetParaUpdate_bl(cln) g_parSetParaUpdateCore_bl()

/*****************************************************************************
 * Interface:
 * g_parIsParaUpdate_bl (cln)
 *
 * Description:
 * check whether the update event for the given cluster is set or not
 *
 * Parameters:
 * cln: name of the cluster
 *
 * Return Value:
 * TRUE: parameters updated - FALSE: no update available
 *
 * Special Considerations:
 * - call g_parIsParaUpdate_bl (0) to generally request the update event
 * - macro implementation
 *
 *****************************************************************************/
#define g_parIsParaUpdate_bl(cln) g_parIsParaUpdateCore_bl()

#ifdef COMPONENT_PAR
#endif // COMPONENT_PAR

// general functions
Boolean g_parIsInitialized_bl(void);
void g_parInit_vd(void);
void g_parProcessing_vd(void);
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
extern void g_parInitApplMemory_vd(void);
extern void g_parUpdateClusterPointers_vd(UInt8 f_appl_mode_ui8);
#endif

// major variant access
UInt8 g_parGetMajorVariant_ui8(void);
UInt8 g_parGetNumMajorVariants_ui8(void);

// core functions to be used from macros - do not use directly
Boolean g_parSetMinorAndSubVariantCore_bl(UInt8 f_cl_idx_ui8, UInt8 f_mv_idx_ui8,
                                          UInt8 f_sv_idx_ui8);

UInt8 g_parGetMaxMinorVariantsCore_ui8(UInt8 f_cl_idx_ui8);
UInt8 g_parGetMaxSubVariantsCore_ui8(UInt8 f_cl_idx_ui8, UInt8 f_mv_idx_ui8);

void *g_parGetParaWriteAccessCore_pvd(UInt8 f_cl_idx_ui8, UInt8 f_minor_var_idx_ui8,
                                      UInt8 sub_var_idx_ui8);

Boolean g_parSetParaUpdateCore_bl(void);
Boolean g_parIsParaUpdateCore_bl(void);
UInt16 g_getParameterStructSize_ui16(UInt8 f_parameterID_ui8);
UInt8 g_getParameterVariantCount_ui8(UInt8 f_parameterID_ui8);
Boolean g_setParameterValue_bl(UInt8 f_parameterID_ui8, UInt8 f_var_parameterID_ui8,
                               const UInt8 *f_value_pui8, UInt16 f_valueSize_ui16);
// gType_VehParaType_en g_FV_adiGetCurrentVehParaType_bl(void);
#endif /* PARM_API_H */
/*========================= EoF (parm_api.h) ====================*/
