/**
 * @file dataOutput_adi.h
 * @brief  //TODO
 * @author xin.shi (xin.shi@bm-intelligent.com)
 * @version 1.0
 * @date 2022-08-30
 *
 * @copyright Copyright (c) 2022  bm-intelligent
 *
 * @par 修改日志:
 *
 * Date        Version  Author   Description
 * 2022-08-30 1.0     shixin      First version
 *
 */
#ifndef _DATAOUTPUT_OUTPUT_ADI_H_
#define _DATAOUTPUT_OUTPUT_ADI_H_
#ifdef __cplusplus
extern "C"
{
#endif

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include <data_io/dataOutput_format.h>

    extern void g_adiGetMsgOdo_vd(gType_VehOdoFrame_st *f_VehOdoFrame_pst);
    extern UInt16 g_adiGetParameterStructSize_vd(UInt8 f_parameterID_ui8);
    extern UInt8 g_adiGetParameterVariantCount_ui8(UInt8 f_parameterID_ui8);
    extern UInt16 g_adiGetParameterNumberOfClusters_ui16(void);
#ifdef __cplusplus
}
#endif

#endif