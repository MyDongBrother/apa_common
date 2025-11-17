/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: parm.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#define COMPONENT_PAR

#include <pa_components/base_type/global_include.h>

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON) || (GS_PAR_CFG_INIT_NVM_MIRROR_MEM == SW_ON)
#include <string.h>
#endif
#include "pa_components/parameter/par.h"
#include <pa_components/parameter/par_api.h>
#include "data_io/dataInput_adi.h"

#ifdef _PAR_ADAPTER_
#define mdc_parMetaData_st(n) &gc_parROMMetaDataCpy_st
#else
#define mdc_parMetaData_st(n) (&gc_parROMMetaData_st)

#endif

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
static UInt8 ms_parClusterMirrorIndex_ui8;
#endif

// component state
gType_parState_st g_parState_st;

// bit pattern look-up pattern
const UInt8 gc_parBitPatternLookUp_pui8[8] = {0x01, 0x02, 0x04, 0x08,
                                              0x10, 0x20, 0x40, 0x80};

// major variant
UInt8 g_parMajorVariant_ui8;
UInt8 g_Curparatype_en = 0;
// update flags
UInt8 g_parUpdateFlags_ui8;
UInt8 g_parUpdateFlags_Shadow_ui8;

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

// application mode flag
// current state
UInt8 g_parApplicationModeFlag_ui8;
// state during previous cycle
UInt8 g_parApplicationModeFlag_Prev_ui8;

#endif

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
/*gType_VehParaType_en g_FV_adiGetCurrentVehParaType_bl(void) {
  return g_Curparatype_en;
}
*/
/**
 * @brief         Get the currently active major variant.
 * @details
 *
 * @param
 * @return        UInt8
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_parGetMajorVariant_ui8(void) { return g_parMajorVariant_ui8; }

/**
 * @brief         Sets the update flag for all parameter clusters. The
 * application will be informed about the update with the next component cycle.
 * @details
 *
 * @param
 * @return        Boolean
 *
 * @note          The update flags can be set by an internal trigger source
 * (e.g. diagnostics) or an external trigger source (e.g. CANape).
 * @see
 * @warning
 */
Boolean g_parSetParaUpdateCore_bl(void)
{
    // set flag
    g_parUpdateFlags_Shadow_ui8 = 1;

    return TRUE;
}

/**
 * @brief         Request from the application whether a parameter cluster has
 * been updated.
 * @details
 *
 * @param
 * @return        Boolean      TRUE: update; else FALSE
 *
 * @note
 * @see
 * @warning
 */
Boolean g_parIsParaUpdateCore_bl(void)
{
    Boolean l_ret_val_bl;

    // return current state (normal mem)
    if (g_parUpdateFlags_ui8 != 0)
    {
        l_ret_val_bl = TRUE;
    }
    else
    {
        l_ret_val_bl = FALSE;
    }

    return l_ret_val_bl;
}

/**
 * @brief         Returns whether the component has been successfully
 * initialized or not.
 * @details
 *
 * @param
 * @return       Boolean       TRUE: successfully initialized; else FALSE
 *
 * @note
 * @see
 * @warning
 */
Boolean g_parIsInitialized_bl(void)
{
    Boolean l_ret_val_bl;

    if (g_parState_st.ProcessingState_en >= PAR_PROCESSING_STATE_NORMAL_enm)
    {
        l_ret_val_bl = TRUE;
    }
    else
    {
        l_ret_val_bl = FALSE;
    }

    return l_ret_val_bl;
}

/**
 * @brief         Initializes the component.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
void g_parInit_vd(void)
{
    Boolean l_bin_par_table_bl;

    // init state
    g_parState_st.ProcessingState_en = PAR_PROCESSING_STATE_CHECK_enm;

    // initialize major variant
    g_parMajorVariant_ui8 = g_parApplGetMajorVariant_ui8();

    // check major variant and set 0 as failsave default
    if (g_parMajorVariant_ui8 >= gd_PAR_NUM_MAJOR_VARIANTS_uix)
    {
        g_parMajorVariant_ui8 = 0;
    }

    // initialize update flags
    g_parUpdateFlags_ui8        = 0;
    g_parUpdateFlags_Shadow_ui8 = 0;

// initialize cluster mirror index
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
    ms_parClusterMirrorIndex_ui8 = 0;
#endif

    // initialize RAM-instancse of the application parameters using their ROM
    // defaults
    // ... done by compiler during start-up phase of the software

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

    // initialize application memory
    (void)memset((void *)&g_parApplicationMemory_st, 0,
                 (size_t)sizeof(g_parApplicationMemory_st));

    g_parApplicationModeFlag_ui8      = 0;
    g_parApplicationModeFlag_Prev_ui8 = 0;

#endif

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

    // initialize variant memory
    (void)memset((void *)&(g_parRAMData_st.VariantMemory_ppui8[0][0]), 0,
                 (size_t)sizeof(g_parRAMData_st.VariantMemory_ppui8));

#endif

// RAM-default initialization
#if (GS_PAR_CFG_INIT_NVM_MIRROR_MEM == SW_ON)

    {
        gType_parDefaultCopyData_st const *lc_my_pst =
            &((mdc_parMetaData_st(g_Curparatype_en))->DefaultCopyData_pst[0]);

        while (lc_my_pst->Source_pvd != NULL)
        {

            (void)memcpy(lc_my_pst->Target_pvd,
                         (void *)((UInt8 *)lc_my_pst->Source_pvd +
                                  (lc_my_pst->Size_six *
                                   lc_my_pst->SubVarOffset_pui8[g_parMajorVariant_ui8])),
                         (lc_my_pst->Size_six *
                          lc_my_pst->NumSubVars_pui8[g_parMajorVariant_ui8]));

            // next element
            lc_my_pst++;
        }
    }

#endif

    // Retrieve the parameter table from the binary data stored in memory.
    // The function `g_adiGetParameterTable_bl` returns TRUE if successful, FALSE
    // otherwise.
    l_bin_par_table_bl = g_adiGetParameterTable_bl(&g_parMemoryData_st,
                                                   (size_t)sizeof(g_parMemoryData_st));
    if (l_bin_par_table_bl == FALSE)
    {
        // If retrieving the parameter table fails, copy the default ROM data
        // into the parameter memory structure `g_parMemoryData_st`.
        (void)memcpy(&g_parMemoryData_st, &gc_parROMData_st,
                     (size_t)sizeof(g_parMemoryData_st));
    }

    // initialize cluster pointers
    {
        UInt32 l_cluster_idx_ui32;

        for (l_cluster_idx_ui32 = 0; l_cluster_idx_ui32 < gd_PAR_NUM_CLUSTERS_uix;
             l_cluster_idx_ui32++)
        {

            g_parRAMData_st.ClusterPointer_ppvd[l_cluster_idx_ui32] =
                (void *)(&((UInt8 *)((mdc_parMetaData_st(g_Curparatype_en))
                                         ->IndData_ppvd[l_cluster_idx_ui32]))
                             [(SInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                  ->ClusterSize_pui16[l_cluster_idx_ui32] *
                              (SInt32)(g_parGetDataOffset_ui8((UInt8)l_cluster_idx_ui32,
                                                              0, 0))]);
        }
    }

    // project specific initialization
    g_parApplInit_vd();
}

/**
 * @brief         Cyclic processing function of the component.
 *
 * @details
 *
 * @param
 * @return
 *
 * @note          Has to be called before all other cyclic processing functions.
 * @see
 * @warning
 */
void g_parProcessing_vd(void)
{

#if (GS_PP_RUNTIME_MEASUREMENT_FX4L == SW_ON)       // just with active switch
    isystem_profile_func_entry(g_parProcessing_vd); // intrumentation for measurement
#endif

    //
    // handle application mode
    //

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

    if (g_parApplicationModeFlag_Prev_ui8 != g_parApplicationModeFlag_ui8)
    {
        g_parUpdateClusterPointers_vd(g_parApplicationModeFlag_ui8);
    }

    // backup application mode flag
    g_parApplicationModeFlag_Prev_ui8 = g_parApplicationModeFlag_ui8;

#endif

    //
    // handle update flags
    //

    // copy update flag shadow mem into normal update flag mem
    g_parUpdateFlags_ui8        = g_parUpdateFlags_Shadow_ui8;
    g_parUpdateFlags_Shadow_ui8 = 0;

    // state machine
    // "fall through" implementation to have more than one state change per
    // component cycle

    if (g_parState_st.ProcessingState_en == PAR_PROCESSING_STATE_CHECK_enm)
    {
        // TODO: for debug
        g_parState_st.ProcessingState_en = PAR_PROCESSING_STATE_PARA_INIT_enm;
        // check parameter integrity and switch to next state if ok
        if (g_parApplCheckParaIntegrity_bl() != FALSE)
        {
            g_parState_st.ProcessingState_en = PAR_PROCESSING_STATE_PARA_INIT_enm;
        }
    }

    if (g_parState_st.ProcessingState_en == PAR_PROCESSING_STATE_PARA_INIT_enm)
    {
        // customer project specific parameter initialization - switch to next state
        // if done
        if (g_parApplInitPara_bl() != FALSE)
        {
            g_parState_st.ProcessingState_en = PAR_PROCESSING_STATE_NORMAL_enm;
        }
    }

    if (g_parState_st.ProcessingState_en == PAR_PROCESSING_STATE_NORMAL_enm)
    {
        // project specific cyclic actions
        g_parApplProcessing_vd();

#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
        {
            // check if clusters are in application mode
            if (g_parApplicationModeFlag_ui8 == 0)
            {
                // mirror all cluster which are set to application in the application
                // memory
                UInt8 l_max_update_counter_ui8;
                UInt8 l_update_counter_ui8;

                // limit max loop counter
                l_max_update_counter_ui8 =
                    (gd_PAR_NUM_CLUSTERS_uix < 2) ? gd_PAR_NUM_CLUSTERS_uix : 2;

                // due to computation time reason do NOT update more than X clusters per
                // cycle
                for (l_update_counter_ui8 = 0;
                     l_update_counter_ui8 < l_max_update_counter_ui8;
                     l_update_counter_ui8++)
                {
                    // check whether variant application is active for this cluster or not
                    switch ((mdc_parMetaData_st(g_Curparatype_en))
                                ->ApplSubVariantMode_pui8[ms_parClusterMirrorIndex_ui8])
                    {
                        case (UInt8)0: // SINGLE
                        {
                            // copy the currently selected element to the appl mem (one
                            // element only)
                            (void)memcpy(
                                (mdc_parMetaData_st(g_Curparatype_en))
                                    ->ApplMemClusterPointer_ppvd
                                        [ms_parClusterMirrorIndex_ui8],
                                g_parRAMData_st
                                    .ClusterPointer_ppvd[ms_parClusterMirrorIndex_ui8],
                                (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                        ->ClusterSize_pui16
                                            [ms_parClusterMirrorIndex_ui8] *
                                    (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                        ->MaxApplSubVariants_pui8
                                            [ms_parClusterMirrorIndex_ui8]);
                        }
                        break;

                        case (UInt8)1: // MINOR_ALL
                        {

                            // copy MaxApplSubVariants elements to the appl mem
                            // (num of elements = max. num of sub-variants of all
                            // minor-variants) as source pointer the current cluster
                            // pointer is used - but we have to go back (index of
                            // currently selected sub-variant) elements to get the start
                            // pointer of the currently selected minor variant
                            (void)memcpy(
                                (mdc_parMetaData_st(g_Curparatype_en))
                                    ->ApplMemClusterPointer_ppvd
                                        [ms_parClusterMirrorIndex_ui8],
                                (UInt8 *)g_parRAMData_st.ClusterPointer_ppvd
                                        [ms_parClusterMirrorIndex_ui8] -
                                    (g_parRAMData_st.VariantMemory_ppui8
                                         [ms_parClusterMirrorIndex_ui8][1] *
                                     (mdc_parMetaData_st(g_Curparatype_en))
                                         ->ClusterSize_pui16
                                             [ms_parClusterMirrorIndex_ui8]),
                                (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                        ->ClusterSize_pui16
                                            [ms_parClusterMirrorIndex_ui8] *
                                    (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                        ->MaxApplSubVariants_pui8
                                            [ms_parClusterMirrorIndex_ui8]);
                        }
                        break;

                        case (UInt8)2: // MAJOR_ALL
                        {
                            // array offset of the cluster dependent on the current
                            // major-variant (first sub-variant)
                            const UInt8 l_offset_ui8 = g_parGetDataOffset_ui8(
                                ms_parClusterMirrorIndex_ui8, 0, 0);

                            // copy all sub-variants of the currently selected
                            // major-variant to the appl mem source pointer is the pointer
                            // to the very first sub-variant of the current major variant
                            (void)memcpy(
                                (mdc_parMetaData_st(g_Curparatype_en))
                                    ->ApplMemClusterPointer_ppvd
                                        [ms_parClusterMirrorIndex_ui8],
                                (void *)(&(
                                    (UInt8 *)((mdc_parMetaData_st(g_Curparatype_en))
                                                  ->IndData_ppvd
                                                      [ms_parClusterMirrorIndex_ui8]))
                                             [(SInt32)(mdc_parMetaData_st(
                                                           g_Curparatype_en))
                                                  ->ClusterSize_pui16
                                                      [ms_parClusterMirrorIndex_ui8] *
                                              (SInt32)(l_offset_ui8)]),
                                (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                        ->ClusterSize_pui16
                                            [ms_parClusterMirrorIndex_ui8] *
                                    (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                        ->MaxApplSubVariants_pui8
                                            [ms_parClusterMirrorIndex_ui8]);
                        }
                        break;

                        default:
                            // do nothing
                            break;
                    }

                    // next update
                    ms_parClusterMirrorIndex_ui8++;

                    // handle overflow ...
                    if (ms_parClusterMirrorIndex_ui8 >= gd_PAR_NUM_CLUSTERS_uix)
                    {
                        ms_parClusterMirrorIndex_ui8 = 0;
                    }
                }
            }
        }
#endif
    }

#if (GS_PP_RUNTIME_MEASUREMENT_FX4L == SW_ON) // just with active switch
    isystem_profile_func_exit();              // intrumentation for measurement
#endif
}

/**
 * @brief         Initialize the application memory.
 * @details
 *
 * @param
 * @return
 *
 * @note          Currently only used for module tests.
 * @see
 * @warning
 */
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
void g_parInitApplMemory_vd(void)
{
    // check if cluster in application mode
    if (g_parApplicationModeFlag_ui8 != 0)
    {
        // mirror all cluster which are set to application in the application memory
        UInt8 l_cl_idx_ui8;

        // due to computation time reason do NOT update more than X clusters per
        // cycle
        for (l_cl_idx_ui8 = 0; l_cl_idx_ui8 < gd_PAR_NUM_CLUSTERS_uix; l_cl_idx_ui8++)
        {
            // check whether variant application is active for this cluster or not
            switch ((mdc_parMetaData_st(g_Curparatype_en))
                        ->ApplSubVariantMode_pui8[l_cl_idx_ui8])
            {
                case (UInt8)0: // SINGLE
                {
                    (void)memcpy((mdc_parMetaData_st(g_Curparatype_en))
                                     ->ApplMemClusterPointer_ppvd[l_cl_idx_ui8],
                                 g_parRAMData_st.ClusterPointer_ppvd[l_cl_idx_ui8],
                                 (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                         ->ClusterSize_pui16[l_cl_idx_ui8] *
                                     (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                         ->MaxApplSubVariants_pui8[l_cl_idx_ui8]);
                }
                break;

                case (UInt8)1: // MINOR_ALL
                {

                    (void)memcpy(
                        (mdc_parMetaData_st(g_Curparatype_en))
                            ->ApplMemClusterPointer_ppvd[l_cl_idx_ui8],
                        (UInt8 *)g_parRAMData_st.ClusterPointer_ppvd[l_cl_idx_ui8] -
                            (g_parRAMData_st.VariantMemory_ppui8[l_cl_idx_ui8][1] *
                             (mdc_parMetaData_st(g_Curparatype_en))
                                 ->ClusterSize_pui16[l_cl_idx_ui8]),
                        (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                ->ClusterSize_pui16[l_cl_idx_ui8] *
                            (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                ->MaxApplSubVariants_pui8[l_cl_idx_ui8]);
                }
                break;

                case (UInt8)2: // MAJOR_ALL
                {
                    const UInt8 l_offset_ui8 =
                        g_parGetDataOffset_ui8(ms_parClusterMirrorIndex_ui8, 0, 0);

                    (void)memcpy(
                        (mdc_parMetaData_st(g_Curparatype_en))
                            ->ApplMemClusterPointer_ppvd[l_cl_idx_ui8],
                        (void *)(&((UInt8 *)((mdc_parMetaData_st(g_Curparatype_en))
                                                 ->IndData_ppvd[l_cl_idx_ui8]))
                                     [(SInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                          ->ClusterSize_pui16[l_cl_idx_ui8] *
                                      (SInt32)(l_offset_ui8)]),
                        (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                ->ClusterSize_pui16[l_cl_idx_ui8] *
                            (UInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                ->MaxApplSubVariants_pui8[l_cl_idx_ui8]);
                }
                break;

                default:
                    // do nothing
                    break;
            }
        }
    }
}
#endif

/**
 * @brief         Get the total number of supported major variants.
 * @details
 *
 * @param
 * @return        UInt8    number of major variants
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_parGetNumMajorVariants_ui8(void)
{
    // return total number of major variants
    return (UInt8)(gd_PAR_NUM_MAJOR_VARIANTS_uix);
}

/**
 * @brief         Get the number of supported minor variants for the given
 * cluster.
 * @details
 *
 * @param         f_cl_idx_ui8       index of cluster
 * @return        UInt8              number of minor variants
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_parGetMaxMinorVariantsCore_ui8(UInt8 f_cl_idx_ui8)
{
    // indicate invalid variant by returning 255
    UInt8 l_ret_val_ui8 = 255;

    // check whether requested variant exists and therefore is valid
    if (f_cl_idx_ui8 < gd_PAR_NUM_CLUSTERS_uix)
    {
        // pointer to major variant meta data
        gType_parMajorVarMetaData_st const *const lc_vvmd_pst =
            &(mdc_parMetaData_st(g_Curparatype_en))->MajorVarMetaData_pst[f_cl_idx_ui8];

        // pointer to minor variant meta data
        gType_parMinorVarMetaData_st const *const lc_mvmd_pst =
            &((mdc_parMetaData_st(g_Curparatype_en))
                  ->MinorVarMetaData_pst
                      [lc_vvmd_pst->MinorVarMetaDataIdx_pui16[g_parMajorVariant_ui8]]);

        // return number of minor variants
        l_ret_val_ui8 = lc_mvmd_pst->NumVariants_ui8;
    }

    return l_ret_val_ui8;
}

/**
 * @brief         Get the number of supported sub variants for the given cluster
 * and minor variant.
 * @details
 *
 * @param         f_cl_idx_ui8     index of cluster
 * @param         f_mv_idx_ui8     index of minor variant
 * @return        UInt8
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_parGetMaxSubVariantsCore_ui8(UInt8 f_cl_idx_ui8, UInt8 f_mv_idx_ui8)
{
    // indicate invalid variant by returning 255
    UInt8 l_ret_val_ui8 = 255;

    // check whether requested variant exists and therefore is valid
    if (f_cl_idx_ui8 < gd_PAR_NUM_CLUSTERS_uix)
    {
        // pointer to major variant meta data
        gType_parMajorVarMetaData_st const *const lc_vvmd_pst =
            &(mdc_parMetaData_st(g_Curparatype_en))->MajorVarMetaData_pst[f_cl_idx_ui8];

        // pointer to minor variant meta data
        gType_parMinorVarMetaData_st const *const lc_mvmd_pst =
            &((mdc_parMetaData_st(g_Curparatype_en))
                  ->MinorVarMetaData_pst
                      [lc_vvmd_pst->MinorVarMetaDataIdx_pui16[g_parMajorVariant_ui8]]);

        // check minor variant index
        if (f_mv_idx_ui8 < lc_mvmd_pst->NumVariants_ui8)
        {
            // pointer to sub variant meta data
            gType_parSubVarMetaData_st const *const lc_svmd_pst =
                &(mdc_parMetaData_st(g_Curparatype_en))
                     ->SubVarMetaData_pst[lc_mvmd_pst->SubVarMetaDataIdx_ui16 +
                                          (UInt16)f_mv_idx_ui8];

            // return number of sub variants
            l_ret_val_ui8 = lc_svmd_pst->NumVariants_ui8;
        }
    }

    return l_ret_val_ui8;
}

/**
 * @brief         Initializes the component's timer generator control module.
 * @details
 *
 * @param         f_cl_idx_ui8         index of cluster
 * @param         f_minor_var_idx_ui8  minor variant index
 * @param         sub_var_idx_ui8      sub variant index
 * @return        UInt8
 *
 * @note
 * @see
 * @warning
 */
UInt8 g_parGetDataOffset_ui8(UInt8 f_cl_idx_ui8, UInt8 f_minor_var_idx_ui8,
                             UInt8 sub_var_idx_ui8)
{
    // indicate invalid variant by returning 255
    UInt8 l_ret_val_ui8 = 255;

    // check whether requested variant exists and therefore is valid
    if (f_cl_idx_ui8 < gd_PAR_NUM_CLUSTERS_uix)
    {
        // pointer to minor variant meta data
        gType_parMinorVarMetaData_st const *lc_mvmd_pst;

        // pointer to major variant meta data
        gType_parMajorVarMetaData_st const *const lc_vvmd_pst =
            &(mdc_parMetaData_st(g_Curparatype_en))->MajorVarMetaData_pst[f_cl_idx_ui8];

// set pointer to minor variant meta data
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

        if (g_parApplicationModeFlag_ui8 == 0)
        {
            // application mode off
            lc_mvmd_pst = &(
                (mdc_parMetaData_st(g_Curparatype_en))
                    ->MinorVarMetaData_pst
                        [lc_vvmd_pst->MinorVarMetaDataIdx_pui16[g_parMajorVariant_ui8]]);
        }
        else
        {
            // application mode on
            lc_mvmd_pst =
                &((mdc_parMetaData_st(g_Curparatype_en))
                      ->MinorVarMetaData_pst[lc_vvmd_pst->MinorVarMetaDataIdx_pui16[0]]);
        }

#else

        // application mode off
        lc_mvmd_pst =
            &((mdc_parMetaData_st(g_Curparatype_en))
                  ->MinorVarMetaData_pst
                      [lc_vvmd_pst->MinorVarMetaDataIdx_pui16[g_parMajorVariant_ui8]]);

#endif

        // the following check mays be obsolete in case the variant structure only
        // contains valid data that already has been checked by
        // g_parSetMinorAndSubVariant_bl()

        // check minor variant index
        if (f_minor_var_idx_ui8 < lc_mvmd_pst->NumVariants_ui8)
        {
            // pointer to sub variant meta data
            gType_parSubVarMetaData_st const *const lc_svmd_pst =
                &(mdc_parMetaData_st(g_Curparatype_en))
                     ->SubVarMetaData_pst[lc_mvmd_pst->SubVarMetaDataIdx_ui16 +
                                          (UInt16)f_minor_var_idx_ui8];

            // check sub variant index
            if (sub_var_idx_ui8 < lc_svmd_pst->NumVariants_ui8)
            {
                // return data offset for selected data
                l_ret_val_ui8 = (UInt8)(lc_svmd_pst->DataOffset_ui8 + sub_var_idx_ui8);
            }
        }
    }

    return l_ret_val_ui8;
}

/**
 * @brief         Sets the minor und sub variant for a given parameter cluster.
 * @details
 *
 * @param         f_cl_idx_ui8  index of cluster
 * @param         f_mv_idx_ui8  index of minor variant
 * @param         f_sv_idx_ui8  index of sub variant
 * @return        Boolean
 *
 * @note
 * @see
 * @warning
 */
Boolean g_parSetMinorAndSubVariantCore_bl(UInt8 f_cl_idx_ui8, UInt8 f_mv_idx_ui8,
                                          UInt8 f_sv_idx_ui8)
{
    Boolean l_ret_val_bl = FALSE;

    // check whether requested variant exists and therefore is valid
    if (f_cl_idx_ui8 < gd_PAR_NUM_CLUSTERS_uix)
    {
        // pointer to major variant meta data
        gType_parMajorVarMetaData_st const *const lc_vvmd_pst =
            &(mdc_parMetaData_st(g_Curparatype_en))->MajorVarMetaData_pst[f_cl_idx_ui8];

        // pointer to minor variant meta data
        gType_parMinorVarMetaData_st const *const lc_mvmd_pst =
            &((mdc_parMetaData_st(g_Curparatype_en))
                  ->MinorVarMetaData_pst
                      [lc_vvmd_pst->MinorVarMetaDataIdx_pui16[g_parMajorVariant_ui8]]);

        // check minor variant index
        if (f_mv_idx_ui8 < lc_mvmd_pst->NumVariants_ui8)
        {
            // pointer to sub variant meta data
            gType_parSubVarMetaData_st const *const lc_svmd_pst =
                &(mdc_parMetaData_st(g_Curparatype_en))
                     ->SubVarMetaData_pst[lc_mvmd_pst->SubVarMetaDataIdx_ui16 +
                                          (UInt16)f_mv_idx_ui8];

            // check sub variant index
            if (f_sv_idx_ui8 < lc_svmd_pst->NumVariants_ui8)
            {
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
                // set variant in variant memory
                g_parRAMData_st.VariantMemory_ppui8[f_cl_idx_ui8][0] = f_mv_idx_ui8;
                g_parRAMData_st.VariantMemory_ppui8[f_cl_idx_ui8][1] = f_sv_idx_ui8;
#endif

// only update cluster pointer with disabled application mode
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
                if (g_parApplicationModeFlag_ui8 == 0)
#endif
                {
                    const UInt8 l_offset_ui8 =
                        g_parGetDataOffset_ui8(f_cl_idx_ui8, f_mv_idx_ui8, f_sv_idx_ui8);

                    // minor and sub variants are valid here -> no check of the return
                    // value
                    g_parRAMData_st.ClusterPointer_ppvd[f_cl_idx_ui8] =
                        (void *)(&((UInt8 *)((mdc_parMetaData_st(g_Curparatype_en))
                                                 ->IndData_ppvd[f_cl_idx_ui8]))
                                     [(SInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                          ->ClusterSize_pui16[f_cl_idx_ui8] *
                                      (SInt32)l_offset_ui8]);
                }
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)
                else
                {
                    // update ClusterPointer according to its new position in the
                    // application RAM memory
                    switch ((mdc_parMetaData_st(g_Curparatype_en))
                                ->ApplSubVariantMode_pui8[f_cl_idx_ui8])
                    {
                        case (UInt8)0: // SINGLE
                            // nothing to do
                            break;

                        case (UInt8)1: // MINOR_ALL
                        {
                            g_parRAMData_st.ClusterPointer_ppvd[f_cl_idx_ui8] =
                                (void *)(&((UInt8 *)(mdc_parMetaData_st(g_Curparatype_en))
                                               ->ApplMemClusterPointer_ppvd[f_cl_idx_ui8])
                                             [(SInt32)(mdc_parMetaData_st(
                                                           g_Curparatype_en))
                                                  ->ClusterSize_pui16[f_cl_idx_ui8] *
                                              (SInt32)f_sv_idx_ui8]);
                        }
                        break;

                        case (UInt8)2: // MAJOR_ALL
                        {
                            const UInt8 l_offset_ui8 = g_parGetDataOffset_ui8(
                                f_cl_idx_ui8, f_mv_idx_ui8, f_sv_idx_ui8);

                            g_parRAMData_st.ClusterPointer_ppvd[f_cl_idx_ui8] =
                                (void *)(&((UInt8 *)(mdc_parMetaData_st(g_Curparatype_en))
                                               ->ApplMemClusterPointer_ppvd[f_cl_idx_ui8])
                                             [(SInt32)(mdc_parMetaData_st(
                                                           g_Curparatype_en))
                                                  ->ClusterSize_pui16[f_cl_idx_ui8] *
                                              (SInt32)l_offset_ui8]);
                        }
                        break;

                        default:
                            // do nothing
                            break;
                    }
                }
#endif

                l_ret_val_bl = TRUE;
            }
        }
    }

    return l_ret_val_bl;
}

/**
 * @brief         Allows write access to the specified cluster.
 * @details
 *
 * @param         f_cl_idx_ui8          index of cluster
 * @param         f_minor_var_idx_ui8   minor variant index
 * @param         sub_var_idx_ui8       sub variant index
 * @return
 *
 * @note
 * @see
 * @warning
 */
void *g_parGetParaWriteAccessCore_pvd(UInt8 f_cl_idx_ui8, UInt8 f_minor_var_idx_ui8,
                                      UInt8 sub_var_idx_ui8)
{
    void *l_void_ptr_pvd;

    const UInt8 l_offset_ui8 =
        g_parGetDataOffset_ui8(f_cl_idx_ui8, f_minor_var_idx_ui8, sub_var_idx_ui8);

    if (l_offset_ui8 != 255)
    {

        l_void_ptr_pvd = (void *)(&(
            (UInt8 *)((mdc_parMetaData_st(g_Curparatype_en))->IndData_ppvd[f_cl_idx_ui8]))
                                      [(SInt32)(mdc_parMetaData_st(g_Curparatype_en))
                                           ->ClusterSize_pui16[f_cl_idx_ui8] *
                                       (SInt32)l_offset_ui8]);
    }
    else
    {
        l_void_ptr_pvd = NULL;
    }

    return l_void_ptr_pvd;
}

/**
 * @brief         Sets the cluster pointers either to application or normal
 * mode.
 * @details
 *
 * @param         f_appl_mode_ui8 appl mode (1); normal mode (0)
 * @return
 *
 * @note
 * @see
 * @warning
 */
#if (GS_PAR_CFG_APPLICATION_MODE == SW_ON)

void g_parUpdateClusterPointers_vd(UInt8 f_appl_mode_ui8)
{
    if (f_appl_mode_ui8 != 0)
    {

        // switched "on" application mode
        (void)memcpy(
            (void *)&(g_parRAMData_st.ClusterPointer_ppvd[0]),
            (void *)&(
                (mdc_parMetaData_st(g_Curparatype_en))->ApplMemClusterPointer_ppvd[0]),
            (size_t)sizeof(
                (mdc_parMetaData_st(g_Curparatype_en))->ApplMemClusterPointer_ppvd));
    }
    else
    {
        // switched "off" application mode

        UInt8 l_cl_idx_ui8;

        // restore the cluster pointer by setting them according to the stored
        // minor- and sub-variant settings
        for (l_cl_idx_ui8 = 0; l_cl_idx_ui8 < gd_PAR_NUM_CLUSTERS_uix; l_cl_idx_ui8++)
        {
            (void)g_parSetMinorAndSubVariantCore_bl(
                l_cl_idx_ui8, g_parRAMData_st.VariantMemory_ppui8[l_cl_idx_ui8][0],
                g_parRAMData_st.VariantMemory_ppui8[l_cl_idx_ui8][1]);
        }
    }
}

#endif
/**
 * @brief Get the structure size of a parameter in the parameter table.
 *
 * @param f_parameterID Parameter ID.
 * @return UInt16 Returns the size of the parameter structure. If the parameter ID is
 * invalid, returns 0.
 *
 * @note This function assumes the parameter ID is within a valid range. If the parameter
 * ID is invalid, it will return 0.
 *
 * @see Other related functions, such as getParameterVariantCount, setParameterValue
 */
UInt16 g_getParameterStructSize_ui16(UInt8 f_parameterID_ui8)
{
    UInt16 l_ret_cluster_size_ui16 = 0;
    if (f_parameterID_ui8 < gd_PAR_NUM_CLUSTERS_uix)
    {
        l_ret_cluster_size_ui16 =
            mdc_parMetaData_st(g_Curparatype_en)->ClusterSize_pui16[f_parameterID_ui8];
    }
    else
    {
        l_ret_cluster_size_ui16 = 0;
    }
    return l_ret_cluster_size_ui16;
}

/**
 * @brief Get the number of variants for a parameter in the parameter table.
 *
 * @param f_parameterID Parameter ID.
 * @return UInt8 Returns the number of variants for the parameter. If the parameter ID is
 * invalid, returns 0.
 *
 * @note This function assumes the parameter ID is within a valid range. If the parameter
 * ID is invalid, it will return 0.
 *
 * @see Other related functions, such as getParameterStructSize, setParameterValue
 */
UInt8 g_getParameterVariantCount_ui8(UInt8 f_parameterID_ui8)
{
    UInt8 l_ret_number_subID_ui8 = 0;
    if (f_parameterID_ui8 < gd_PAR_NUM_CLUSTERS_uix)
    {
        l_ret_number_subID_ui8 = mdc_parMetaData_st(g_Curparatype_en)
                                     ->MinorVarMetaData_pst[f_parameterID_ui8]
                                     .NumVariants_ui8;
    }
    else
    {
        l_ret_number_subID_ui8 = 0;
    }
    return l_ret_number_subID_ui8;
}

/**
 * @brief Set the value of a parameter in the parameter table.
 *
 * @param parameterID Parameter ID.
 * @param var_parameterID Parameter variant ID.
 * @param value Pointer to the value to be set.
 * @param valueSize Size of the value.
 * @return Boolean Returns TRUE if the value is set successfully, FALSE otherwise.
 *
 * @note Ensure the value is not NULL and the valueSize matches the parameter structure
 * size.
 *
 * @see Other related functions, such as getParameterStructSize, getParameterVariantCount
 */
Boolean g_setParameterValue_bl(UInt8 f_parameterID_ui8, UInt8 f_var_parameterID_ui8,
                               const UInt8 *f_value_pui8, UInt16 f_valueSize_ui16)
{
    Boolean l_ret_valid_bl      = 0;
    UInt16 l_parameterSize_ui16 = g_getParameterStructSize_ui16(f_parameterID_ui8);
    UInt8 l_subID_ui8           = g_getParameterVariantCount_ui8(f_parameterID_ui8);
    if ((f_parameterID_ui8 < gd_PAR_NUM_CLUSTERS_uix) && (f_value_pui8 != NULL) &&
        (f_valueSize_ui16 == l_parameterSize_ui16) &&
        (f_var_parameterID_ui8 < l_subID_ui8))
    {
        void *Target_addr = (void *)((UInt8 *)(mdc_parMetaData_st(g_Curparatype_en)
                                                   ->IndData_ppvd[f_parameterID_ui8]) +
                                     (l_parameterSize_ui16 * f_var_parameterID_ui8));
        memcpy(Target_addr, f_value_pui8, l_parameterSize_ui16);
        l_ret_valid_bl = TRUE;
    }
    else
    {
        l_ret_valid_bl = FALSE;
    }
    return l_ret_valid_bl;
}
