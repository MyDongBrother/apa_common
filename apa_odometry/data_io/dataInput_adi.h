/**
 * @file dataInput_adi.h
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
#ifndef _DATAINPUT_ADI_H_
#define _DATAINPUT_ADI_H_
#ifdef __cplusplus
extern "C"
{
#endif

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include <data_io/dataOutput_format.h>
#include <data_io/dataInput_format.h>

    extern Boolean g_SetMsgOdo_vd(const UInt8 *f_Data_pui8, UInt16 f_DataLength_ui16);
    extern Boolean g_SetMsgCan_vd(const UInt8 *f_Data_pui8, UInt16 f_DataLength_ui16);
    extern Boolean g_adiGetMsgCan_vd(void *f_Data_pvd, const UInt16 StructLen_ui16);
    extern Boolean g_adiSetParameterValue_bl(UInt8 f_parameterID_ui8,
                                             UInt8 f_var_parameterID_ui8,
                                             const UInt8 *f_value_pui8,
                                             UInt16 f_valueSize_ui16);
    extern Boolean g_adiGetParameterData_bl(void *f_Data_pvd,
                                            const UInt16 StructLen_ui16);
    extern Boolean g_SetMsgParameterTable_vd(const UInt8 *f_Data_pui8,
                                             UInt16 f_DataLength_ui16);
    extern Boolean g_adiGetParameterTable_bl(void *f_Data_pvd,
                                             const UInt16 StructLen_ui16);
    extern UInt16 g_GetTimeMs_ui16(void);
    extern void g_SetTimeMs_ui16(UInt16 f_SetTimeMs_ui16);
    extern UInt16 g_CalcDeltaT_ui16(UInt16 f_Time_ui16);
#ifdef __cplusplus
}
#endif

#endif