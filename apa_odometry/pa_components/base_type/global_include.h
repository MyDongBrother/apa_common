#ifndef _GLOBAL_INCLUDE_H
#define _GLOBAL_INCLUDE_H

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#ifndef ENABLE_MATHLIB_ONLY
#include <pa_components/base_type/base_typedef.h>
#include <pa_components/mathlib/mathlib_typdef.h>
#include <pa_components/parameter/gen_par_switches.h>
#else
#include "base_typedef.h"
#include "../mathlib/mathlib_typdef.h"
#include "../parameter/gen_par_switches.h"

#endif
#if ((GS_PP_FUNC_PSI == SW_ON) || (GS_PP_FUNC_PSC == SW_ON))
#include "psx_typdef.h"
#endif

#if (GS_PP_FUNC_PDC == SW_ON)
#include "ovm_typdef.h"
#endif

/*--------------------------------------------------------------------------*/
/*- macro define                                                           -*/
/*--------------------------------------------------------------------------*/

// ! Run ON SOC Switch
// #define md_RUN_ON_POSIX
// #define PDC_SOC_SWITCH
// #define USS_GRID_MAP

#ifndef md_RUN_ON_POSIX

#define _OPEN_ 1
#define _CLOSE_ 0
#define _DEBUG_ _OPEN_

#if (_DEBUG_ == _OPEN_)
#define DBG (printf)
#define WAR (printf)
#define ERR (printf)
#else
#define DBG
#define WAR
#define ERR
#endif

#endif

#define COMPONENT_APT // For APT TASK

#define LS_COMPONENT_PSD_PSCONTOUR SW_ON // PS Contour

#endif /* _GLOBAL_INCLUDE_H */
/* END OF FILE */
