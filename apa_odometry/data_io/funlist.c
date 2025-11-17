#include "veh/sig_if/sigif_api.h"
#include "funlist.h"
/**
 * @brief  Module Init Function
 */
void g_ModuleInit_vd(void)
{
    g_parInit_vd();
    g_parIsInitialized_bl();
    g_SigIfInit_vd();
}

void g_Function_10msTask_vd(void)
{
    g_SigIfInputProcessing_vd();
    g_SigIf_odoProcessing_vd();
}