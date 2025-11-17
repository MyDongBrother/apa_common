/*****************************************************************************
 C O P Y R I G H T
------------------------------------------------------------------------------
 Copyright (c) 2010 by Frovision.          All rights reserved.

 This file is property of Frovision. Any unauthorized copy, use or
 distribution is an offensive act against international law and may be
 prosecuted under federal law. Its content is company confidential.
------------------------------------------------------------------------------
 D E S C R I P T I O N
 $ProjectName:
d:/MKS_Data/Projects/PPJ/PP_GEN5/PP_SIP5_Fx4/SW/SRC/PA/PF/MATHLIB/pc_pp_mathlib_pf.pj $
      $Source: mathlib_typdef.h $
    $Revision: 1.5 $
        $Date: 2013/03/08 20:45:51CST $
       $State: in_work $
------------------------------------------------------------------------------
 N O T E S                                                         *//*! \file
        \brief  "COMPONENTNAME" API header (interface to other components)  \n
      \details  _detailed_description_                                    \n\n
  Target system: NEC V850 Fx3/Fx4                                           \n
       Compiler: GHS Multi for V800 (>=V4.2.3 Patch01)
*//*--------------------------------------------------------------------------
 A U T H O R   I D E N T I T Y                                     *//*! \file
 \author   $name
*****************************************************************************/
/*! */
// the following line specifies a documentation section for MATHLIB
/*!\addtogroup COMPONENT_MATHLIB MATHLIB documentation */

#ifndef MATHLIB_TYPDEF_H
#define MATHLIB_TYPDEF_H

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilized in this file) are defined         -*/
/*--------------------------------------------------------------------------*/

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_M03           exported macros
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_M04           exported data types
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/**
Point or Vector in a Euclidean 2D space.
*/
typedef struct gT_mtlPoint_st
{
    SInt16 X_si16; //!< x-coordinate or x-component of vector
    SInt16 Y_si16; //!< y-coordinate or y-component of vector
} gType_mtlPoint_st;

typedef struct gT_mtlPointPos_st
{
    SInt32 X_si32; //!< x-coordinate or x-component of vector
    SInt32 Y_si32; //!< y-coordinate or y-component of vector
} gType_mtlPointPos_st;

/** Angle with sin/cos data. Use the g_mtlAngle*_vd functions to work with Angle values.
  This struct was introduced because many trignonometric functions work with sin/cos, but
sometimes the angle value is still needed sometimes (angle comparisons, mostly), so it's
kept until somebody complains about RAM usage too much.

*Note:* The smallest representable angle in F12 rad is 0.014°.
   The smalles representable angle with F14 sin/cos pairs is of course 4 times smaller,
i.e. 0.0035°. However, the cosine of such small angles will be inaccurate, since cos(0.4掳)
* 2^14 = 16384 already. But the beauty of the trigonometric angle add/sub function is that
the cosine error will be compensated by the sine value even if small angles are repeatedly
added/subtracted. The only problem that could occur with very small angles is that the
angle [rad] and the sin/cos angle become slightly inconsistent due to repeated rounding
errors.
 */
typedef struct gT_mtlAngle_st
{
    SInt16 Angle_si16; /**< orientation angle F12 [2^-12 rad] (-pi, pi] */
    SInt16 Sin_si16;   /**< sine of angle F14 [2^-14] */
    SInt16 Cos_si16;   /**< cosine of angle F14 [2^-14] */
} gType_mtlAngle_st;

/**
A position with orientation (or a line described by support point and orientation)
Points can have arbitrary scaling (cm, mm, 2^-10 m)
*/
typedef struct gT_mtlLine_st
{
    gType_mtlPoint_st P_st; /**< Position (coordinates) */
    SInt16 Phi_si16;        /**< orientation angle F12 [2^-12 rad] */
} gType_mtlLine_st;

/**
 A vector described by a point and an orientation given with sine/cosine pair.
 Points can have arbitrary scaling (cm, mm, 2^-10 m)
 */
typedef struct gT_mtlLineSinCos_st
{
    gType_mtlPoint_st P_st;   /**!< Position (coordinates) */
    gType_mtlAngle_st Phi_st; /**!< orientation angle */
} gType_mtlLineSinCos_st;

/**
A line segment described by two points. Points can have arbitrary scaling (cm, mm, 2^-10
m)
*/
typedef struct gT_mtlLineSegment_st
{
    gType_mtlPoint_st P1_st; /**< Position (coordinates) */
    gType_mtlPoint_st P2_st; /**< Position (coordinates) */
} gType_mtlLineSegment_st;

/** A circle with arbitrary scaling. The radius can be negative to indicate
  left/right direction in some functions
*/
typedef struct gT_mtlCircle_st
{
    gType_mtlPoint_st P_st; //!< Position (coordinates of center)
    SInt16 R_si16;          //!< Radius. Negative sign = right turn, positive = left turn
} gType_mtlCircle_st;

/** A circle with arbitrary scaling. The radius is only positive (UInt16)
 */
typedef struct gT_mtlCircleU16_st
{
    gType_mtlPoint_st P_st; //!< Position (coordinates of center)
    UInt16 R_ui16;          //!< Radius (positive)
} gType_mtlCircleU16_st;

/**
A Range in 2D space. Values can have arbitrary scaling (cm, mm, 2^-10 m)
*/
typedef struct gT_mtlRange2D_st
{
    SInt16 XMin_si16; //!< minimum x-boundary of the range
    SInt16 XMax_si16; //!< maximum x-boundary of the range
    SInt16 YMin_si16; //!< minimum y-boundary of the range
    SInt16 YMax_si16; //!< maximum y-boundary of the range
} gType_mtlRange2D_st;

/*!@} */ /*------------------------------------------------------------------*/
/*-                           external object declarations                 -*/
/*--------------------------------------------------------------------------*/

#endif /* MATHLIB_TYPDEF_H */

/*========================= EoF (MATHLIB_TYPDEF.h) ====================*/
