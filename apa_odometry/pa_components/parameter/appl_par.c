/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: appl_parm.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_PAR

#define APPL_PAR_C_TEMPL_NUMBER 0x090326

#include <pa_components/base_type/global_include.h>
#include <pa_components/parameter/par_api.h>

#if (GS_PP_FUNC_VARIANTHANDLER == SW_ON)
// #include "variant_handler_api.h"
#endif //  GS_PP_FUNC_VARIANTHANDLER

#if (APPL_PAR_H_TEMPL_NUMBER != APPL_PAR_H_REF)
#error ("Wrong version of H TEMPLATE FILE - check whether the template has changed!")
#endif

#if (APPL_PAR_C_TEMPL_NUMBER != APPL_PAR_C_REF)
#error ("Wrong version of C TEMPLATE FILE - check whether the template has changed!")
#endif

#define mdc_parSrcData_st gc_parRAMData_st

typedef struct Type_parRefHashSection_st
{
    // ID-String to find the start of the section
    UInt8 Header_pui8[16];

    // CAL structure integrity hash - 160 bit SHA1 hash
    UInt8 CALStructureIntegrityHash_pui8[20];

    // NVM structure integrity hash - 160 bit SHA1 hash
    UInt8 NVMStructureIntegrityHash_pui8[20];
} mType_parRefHashSection_st;

/*
// calibration header information (Start of CAL-section)

// redirect all following const definitions into linker section .CAL_HdrLow
#pragma ghs section rodata = ".CAL_HdrLow"

// (segment must be availabe in the linker file --> see Ld_CAL_HdrSizeLow)
volatile const gType_parCalHeaderLow_st  gc_parCalHeaderLow_st =
{
  0x00
};

// redirect all following const definitions into linker section .rodata
#pragma ghs section rodata = default
*/

/*
// calibration header information (End of CAL-section)

// redirect all following const definitions into linker section .CAL_HdrHigh
#pragma ghs section rodata =  ".CAL_HdrHigh"

// (segment must be availabe in the linker file --> see Ld_CAL_HdrSizeHigh)
volatile const gType_parCalHeaderHigh_st gc_parCalHeaderHigh_st =
{
  0x00
};

// redirect all following const definitions into linker section .rodata
#pragma ghs section rodata = default
*/

// hardcoded reference hash
#if 0
static const mType_parRefHashSection_st m_parRefHashSection_st = {
    {'R', 'O', 'M', '-', 'R', 'e', 'f', 'H', 'a', 's', 'h', 'S', 't', 'a', 'r',
     't'},
    gd_PAR_CAL_StructureIntegrityHash_pui16,
    gd_PAR_NVM_StructureIntegrityHash_pui16};
#endif
/**
 * @brief         Checks the parameter integrity.
 * @details
 *
 * @param
 * @return        Boolean TRUE if ok/done; else FALSE
 *
 * @note
 * @see
 * @warning
 */
Boolean g_parApplCheckParaIntegrity_bl(void)
{
    Boolean l_ret_val_bl = TRUE;

    // structure integrity check - example
    {
// TODO
#if 0
    UInt8 l_index_ui8;

    // check whether reference and current hash are the same
    l_index_ui8 = 0;

    while ( (l_index_ui8 < 20) && (l_ret_val_bl != FALSE) )
    {

      if (    m_parRefHashSection_st.CALStructureIntegrityHash_pui8[l_index_ui8] !=
            mdc_parSrcData_st.InfoData_st.StructureIntegrityHash_pui8[l_index_ui8]
            )

      {
        // set parameter set inconsistency fault
        // ...

        l_ret_val_bl = FALSE;
      }

      l_index_ui8++;
    }
#endif
    }

    return l_ret_val_bl;
}

/**
 * @brief         Requests the major variant from the application.
 * @details
 *
 * @param
 * @return        UInt8
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_parApplGetMajorVariant_ui8(void)
{
#if (GS_PP_FUNC_VARIANTHANDLER == SW_ON)
    return 0; //((UInt8)g_VaHaGetMajorVariantForParServer_en());
#else
    return (0);
#endif
}

/**
 * @brief         Implementation of project specific initialization.
 * @details
 *
 * @param
 * @param
 * @return
 *
 * @note          Called at the end of g_parProjectInit_vd().
 * @see
 * @warning
 */
void g_parApplInit_vd(void)
{
    // implementation of project specific initialization ...
}

/**
 * @brief         Implementation of project specific cyclic processing.
 * @details
 *
 * @param
 * @param
 * @return
 *
 * @note          Called cyclically in the context of g_parProcessing_vd()
 * before the components cyclic actions.
 * @see
 * @warning
 */
void g_parApplProcessing_vd(void)
{
    // implementation of project specific cyclic processing ...
}

/**
 * @brief         Provides the posibility to projectspecifically initialize or
 * overwrite
 * @details
 *
 * @param
 * @param
 * @return
 *
 * @note          The function will be called cyclically until it returns TRUE.
 *                To implement a initialization which requires more than one
 * cycle delay the return of TRUE until it has finished. The initialization will
 * be timeout monitored by SMS and canceled after a maximum of 100ms.
 * @see
 * @warning
 */
Boolean g_parApplInitPara_bl(void)
{
#if 0
// Set MP Minor Variant for Consideration of Sensor Configuration:
// Targets PDC_4CH & PDC_8CH:
#if ((GS_PP_FUNC_PSC == SW_OFF) && (GS_PP_FUNC_CPSC == SW_OFF) && \
     (GS_PP_FUNC_PS == SW_OFF) && (GS_PP_FUNC_BSD == SW_OFF))
// #if(GS_PP_FUNC_PDC_3CH == SW_ON)
//   // Minor Variant with 3 Rear Sensors:
//   (void)g_parSetMinorAndSubVariant_bl(MP_PdcMeasGroupParId,
//   gd_PAR_MIV_MP_PdcMeasGroupParId_F0_R3_uix, 0);
// #endif
// #if(GS_PP_FUNC_PDC_4CH == SW_ON)
//   // Minor Variant with 4 Rear Sensors:
//   (void)g_parSetMinorAndSubVariant_bl(MP_PdcMeasGroupParId,
//   gd_PAR_MIV_MP_PdcMeasGroupParId_F0_R4_uix, 0);
// #endif
#if (GS_PP_FUNC_PDC_8CH == SW_ON)
  // Minor Variant with 4 Front & 4 Rear Sensors:
  (void)g_parSetMinorAndSubVariant_bl(
      MP_PdcMeasGroupParId, gd_PAR_MIV_MP_PdcMeasGroupParId_F4_R4_uix, 0);
#endif
#if (GS_PP_FUNC_PDC_10CH == SW_ON)
  // Minor Variant with 6 Front & 4 Rear Sensors:
  (void)g_parSetMinorAndSubVariant_bl(
      MP_PdcMeasGroupParId, gd_PAR_MIV_MP_PdcMeasGroupParId_F6_R4_uix, 0);
#endif
#else // All other Targets:
#if (GS_PP_FUNC_PDC_10CH == SW_ON)
  // Minor Variant with 6 Front & 4 Rear Sensors:
  (void)g_parSetMinorAndSubVariant_bl(
      MP_PdcMeasGroupParId, gd_PAR_MIV_MP_PdcMeasGroupParId_F6_R4_uix, 0);
#else
  // Minor Variants with 6 Front & 6 Rear Sensors:
  (void)g_parSetMinorAndSubVariant_bl(
      MP_PdcMeasGroupParId, gd_PAR_MIV_MP_PdcMeasGroupParId_F6_R6_uix, 0);
#endif

// Set AP_CTL Minor Variant for Consideration of BSD Only / PSC//BSD Operation:
// Target BSD Only:
#if ((GS_PP_FUNC_BSD == SW_ON) && (GS_PP_FUNC_PSC == SW_OFF))
  (void)g_parSetMinorAndSubVariant_bl(APCTL_CFG, gd_PAR_MIV_APCTL_CFG_BSD_uix,
                                      0);
#else // All other Targets (especially PDC_BSD_PSC_POC):
  (void)g_parSetMinorAndSubVariant_bl(APCTL_CFG,
                                      gd_PAR_MIV_APCTL_CFG_PSC_BSD_uix, 0);
#endif
#endif
#else
// over write.
#endif
    return TRUE;
}

/*========================= EoF (appl_parm.c) ===================*/
