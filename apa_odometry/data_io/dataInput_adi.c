/**
 * @file dataInput_adi.c
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

#ifndef MODULE_LOG_NAME
#define MODULE_LOG_NAME "dataInput_adi"
#endif

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include <pa_components/parameter/par_api.h>
#include <string.h>
#include "dataInput_adi.h"

/*--------------------------------------------------------------------------*/
/*- FUNCTION DECLARE PUBLIC                                                -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- FUNCTION DECLARE PRIVAT                                                -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- MACRO DECLARE                                                          -*/
/*--------------------------------------------------------------------------*/
volatile static UInt8 m_sigDataFlag[8];
/*--------------------------------------------------------------------------*/
/*- FUNCTION  PUBLIC                                                       -*/
/*--------------------------------------------------------------------------*/

gType_FVInputCanFrame_st m_RxMsgCan_st      = {0};
gType_FVInputVehOdoFrame_st m_RxMsgOdo_st   = {0};
gType_ParameterInfo_st m_RxMsgParameInfo_st = {0};
gType_parROMData_st *m_RxMsgParameTable_pst = NULL;
volatile UInt16 TimeMs_ui16;
/**
 * @brief   set odo msg to global var.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */

Boolean g_SetMsgOdo_vd(const UInt8 *f_Data_pui8, UInt16 f_DataLength_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pui8 != NULL) && (f_DataLength_ui16 == sizeof(m_RxMsgOdo_st)))
    {
        memcpy(&m_RxMsgOdo_st, f_Data_pui8, sizeof(gType_FVInputVehOdoFrame_st));
        l_ret_bl = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief get odo data from global var after set it.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetInputMsgOdo_vd(void *f_Data_pvd, const UInt16 StructLen_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pvd != NULL) && (StructLen_ui16 == sizeof(m_RxMsgOdo_st)))
    {
        memcpy(f_Data_pvd, &m_RxMsgOdo_st, sizeof(gType_FVInputVehOdoFrame_st));
        l_ret_bl = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief get can msg from global data var after set it.
 * @details
 *
 * @param
 * @return
 *
 * @note
 * @see
 * @warning
 */
Boolean g_adiGetMsgCan_vd(void *f_Data_pvd, const UInt16 StructLen_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pvd != NULL) && (StructLen_ui16 == sizeof(m_RxMsgCan_st)))
    {
        memcpy(f_Data_pvd, &m_RxMsgCan_st, sizeof(gType_FVInputCanFrame_st));
        l_ret_bl = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief  set can data global var m_RxMsgCan_st
 * @details
 *
 * @param f_Data_pui8 , data come from ethernet or can;
 * @param  f_DataLength_ui16 ,data length (byte)
 * @return
 *
 * @note
 * @see
 * @warning
 */

Boolean g_SetMsgCan_vd(const UInt8 *f_Data_pui8, UInt16 f_DataLength_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pui8 != NULL) && (f_DataLength_ui16 == sizeof(gType_FVInputCanFrame_st)))
    {
        memcpy(&m_RxMsgCan_st, f_Data_pui8, f_DataLength_ui16);
        UInt16 l_tick_ui16            = g_GetTimeMs_ui16();
        m_RxMsgCan_st.FL_WicTick_ui16 = l_tick_ui16;
        m_RxMsgCan_st.FR_WicTick_ui16 = l_tick_ui16;
        m_RxMsgCan_st.RL_WicTick_ui16 = l_tick_ui16;
        m_RxMsgCan_st.RR_WicTick_ui16 = l_tick_ui16;
        l_ret_bl                      = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief Sets the value of a specific parameter.
 *
 * This function sets the value of a parameter identified by its ID and variant ID.
 *
 * @note
 * Ensure f_value_pui8 is not NULL and f_valueSize_ui16 matches the size of the parameter
 * structure.
 *
 * @see setParameterValue
 */
Boolean g_adiSetParameterValue_bl(UInt8 f_parameterID_ui8, UInt8 f_var_parameterID_ui8,
                                  const UInt8 *f_value_pui8, UInt16 f_valueSize_ui16)
{
    Boolean l_ret_valid_bl = 0;
    l_ret_valid_bl = g_setParameterValue_bl(f_parameterID_ui8, f_var_parameterID_ui8,
                                            f_value_pui8, f_valueSize_ui16);
    return l_ret_valid_bl;
}

/**
 * @brief Set the global debug parameters m_RxMsgParameInfo_st.
 *
 * This function copies data from the provided buffer into the global variable
 * m_RxMsgParameInfo_st to update the debug parameters.
 *
 * @param f_Data_pui8 Pointer to the buffer containing the data to copy.
 * @param f_DataLength_ui16 Length of the data, used to verify data correctness.
 *
 * @note
 * - Ensure f_Data_pui8 is not NULL.
 * - Ensure f_DataLength_ui16 matches the size of the gType_ParameterInfo_st structure.
 *
 * @see m_RxMsgParameInfo_st, gType_ParameterInfo_st
 */
Boolean g_SetMsgParameterData_vd(const UInt8 *f_Data_pui8, UInt16 f_DataLength_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pui8 != NULL) && (f_DataLength_ui16 == sizeof(gType_ParameterInfo_st)))
    {
        memcpy(&m_RxMsgParameInfo_st, f_Data_pui8, f_DataLength_ui16);
        l_ret_bl = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief Get the global debug parameters m_RxMsgParameInfo_st.
 *
 * This function copies the debug parameters from the global variable m_RxMsgParameInfo_st
 * into the provided buffer.
 *
 * @param f_Data_pvd Pointer to the buffer receiving the data.
 * @param StructLen_ui16 Length of the structure, used to verify data correctness.
 * @return Boolean Returns TRUE on success, FALSE on failure.
 *
 * @note
 * - Ensure f_Data_pvd is not NULL.
 * - Ensure StructLen_ui16 matches the size of the gType_ParameterInfo_st structure.
 *
 * @see m_RxMsgParameInfo_st, gType_ParameterInfo_st
 */
Boolean g_adiGetParameterData_bl(void *f_Data_pvd, const UInt16 StructLen_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pvd != NULL) && (StructLen_ui16 == sizeof(gType_ParameterInfo_st)))
    {
        memcpy(f_Data_pvd, &m_RxMsgParameInfo_st, sizeof(gType_ParameterInfo_st));
        l_ret_bl = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief Sets the message parameter table by assigning the received data pointer.
 *
 * @param f_Data_pui8 Pointer to the received data.
 * @param f_DataLength_ui16 Length of the received data.
 * @return Boolean TRUE if the parameter table was successfully set, FALSE otherwise.
 *
 * @note This function only sets the table if the data pointer is not NULL and the data
 *       length matches the expected size of `gType_parROMData_st`.
 *
 * @see gType_parROMData_st
 */
Boolean g_SetMsgParameterTable_vd(const UInt8 *f_Data_pui8, UInt16 f_DataLength_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pui8 != NULL) && (f_DataLength_ui16 == sizeof(gType_parROMData_st)))
    {
        // Assign the received data pointer to the message parameter table pointer
        m_RxMsgParameTable_pst = (gType_parROMData_st *)f_Data_pui8;
        l_ret_bl               = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief Retrieves the parameter table into the provided data structure.
 *
 * @param f_Data_pvd Pointer to the destination structure where the parameter table will
 * be copied.
 * @param StructLen_ui16 Length of the destination structure.
 * @return Boolean TRUE if the parameter table was successfully retrieved, FALSE
 * otherwise.
 *
 * @note This function copies the parameter table into the destination structure only if
 *       the data pointer is not NULL, the structure length matches the size of
 * `gType_parROMData_st`, and the message parameter table pointer is valid.
 *
 * @see gType_parROMData_st
 */
Boolean g_adiGetParameterTable_bl(void *f_Data_pvd, const UInt16 StructLen_ui16)
{
    Boolean l_ret_bl = FALSE;
    if ((f_Data_pvd != NULL) && (StructLen_ui16 == sizeof(gType_parROMData_st)) &&
        (m_RxMsgParameTable_pst != NULL))
    {
        // Copy the parameter table to the provided data structure
        memcpy(f_Data_pvd, m_RxMsgParameTable_pst, sizeof(gType_parROMData_st));
        l_ret_bl = TRUE;
    }
    return l_ret_bl;
}

/**
 * @brief  Set System Time Tick ms
 * @param  f_SetTimeMs_ui16 set value
 */
void g_SetTimeMs_ui16(UInt16 f_SetTimeMs_ui16) { TimeMs_ui16 = f_SetTimeMs_ui16; }

/**
 * @brief  Get System Time Tick ms
 * @return UInt16 return time tick
 */
UInt16 g_GetTimeMs_ui16(void) { return TimeMs_ui16; }

UInt16 g_CalcDeltaT_ui16(UInt16 f_Time_ui16)
{
    UInt16 l_current_timer_ui16;
    UInt16 l_DeltaT_ui16;

    l_current_timer_ui16 = TimeMs_ui16;

    if (l_current_timer_ui16 < f_Time_ui16)
    {
        l_DeltaT_ui16 = (GD_MAX_UI16 - f_Time_ui16) + l_current_timer_ui16 + 1;
    }
    else
    {
        l_DeltaT_ui16 = l_current_timer_ui16 - f_Time_ui16;
    }
    return l_DeltaT_ui16;
}