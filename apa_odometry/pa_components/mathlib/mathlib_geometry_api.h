/******************************************************************************
 * Copyright (C), 2020-2025, China bm-intelligent Group Co.，Ltd
 * All rights reserved.
 * @file: ap_psd.c
 * @brief：
 * @description:
 * @version： 1.0
 * @author： bm-intelligent
 *****************************************************************************/

#ifndef MATHLIB_GEOMETRY_API_H
#define MATHLIB_GEOMETRY_API_H

/*--------------------------------------------------------------------------*/
/*- include files                                                          -*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*- check if compiler switches (utilized in this file) are defined         -*/
/*--------------------------------------------------------------------------*/

#ifndef SW_ON
#error ('global switch SW_ON is not defined')
#endif

#ifndef SW_OFF
#error ('global switch SW_OFF is not defined')
#endif

#ifndef GS_MTL_GEOLIB
#error ('switch GS_MTL_GEOLIB is not defined')
#endif

/*--------------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_M02           exported symbolic constants
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

#if (GS_MTL_GEOLIB == SW_ON)

/// constant to indicate straight driving (curvature zero)
#define gd_RadiusStraight_si16 GD_MAX_SI16

/// constant to indicate that radius is not infinite (curvature nonzero), but not
/// representable in SInt16 range
#define gd_MaxRadius_si16 (GD_MAX_SI16 - 1)

/// constant to indicate that no valid radius is available
#define gd_InvalidRadius_si16 ((SInt16)0)

#endif /*(GS_MTL_GEOLIB == SW_ON)*/

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_M03           exported macros
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

#if (GS_MTL_GEOLIB == SW_ON)

// silently limit a result to SInt16 range
#define g_mtl_Limit_s32_si16(f_value_si32) \
    ((SInt16)g_mtl_Min_s32_Max_s32_si32((f_value_si32), GD_MIN_SI16, GD_MAX_SI16))
// silently limit a value to UInt16 range
#define g_mtl_Limit_u32_ui16(f_value_ui32)                                           \
    ((UInt16)g_mtl_Min_s32_Max_s32_si32((SInt32)(f_value_ui32), (SInt32)GD_MIN_UI16, \
                                        (SInt32)GD_MAX_UI16))

/** Multiplies a value with a cos/sin value and the result has the same scaling as the
 input value basically: Fxx * F14 / 2^14 = Fxx
 */
#define g_mtl_SinCosMul_si32(f_val_si32, f_SinCos_si16) \
    (g_mtl_ReducePrecision_si32((f_val_si32) * (f_SinCos_si16), 14))

/** Legacy name of the exterior product 2D function */
#define g_mtl_OuterProduct_si32 g_mtl_ExteriorProduct_si32

/// largest allowed value bit is 14, because bit 15 and above are critical (SInt16: Bit15
/// is sign bit) call like this: g_mtl_ScaleDown2_si32_toFit_si16_vd(&X, &Y, &S)
#define g_mtl_ScaleDown2_si32_toFit_si16_vd(X, Y, S) \
    g_mtl_ScaleDown2_si32_vd(14, (X), (Y), (S))

#endif /*(GS_MTL_GEOLIB == SW_ON)*/

/*!@} */ /*------------------------------------------------------------------*/
/*!\addtogroup MATHLIB_M04           exported data types
 * \ingroup COMPONENT_MATHLIB
 @{ *//*--------------------------------------------------------------------*/

/*!@} */ /*------------------------------------------------------------------*/
/*-                           external object declarations                 -*/
/*--------------------------------------------------------------------------*/

/** Statically initialized representation of angle pi/2 */
extern gType_mtlAngle_st const g_mtlAngle_PI_2_st;
/** Statically initialized representation of angle pi */
extern gType_mtlAngle_st const g_mtlAngle_PI_st;

/*--------------------------------------------------------------------------*/
/*-                         prototypes of exported functions               -*/
/*--------------------------------------------------------------------------*/

#if (GS_MTL_GEOLIB == SW_ON)

/************************************************************************/
/* MATHLIB CORE CANDIDATES                                              */
/************************************************************************/

extern UInt8 g_mtl_SolveQuadEquationWithSqrt_ui8(SInt32 *f_Solution1F16_si32,
                                                 SInt32 *f_Solution2F16_si32,
                                                 SInt16 f_a_si16, SInt16 f_beta_si16,
                                                 SInt16 f_c_si16);

extern UInt8 g_mtl_SolveQuadEquation_ui8(SInt32 *f_Solution1F16_si32,
                                         SInt32 *f_Solution2F16_si32, SInt16 f_a_si16,
                                         SInt16 f_beta_si16, SInt16 f_c_si16);

// angle math (F22 angles) TODO: add to mathlib
extern UInt32 g_mtl_AngleAdd_ui32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32);
extern UInt32 g_mtl_AngleSub_ui32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32);
extern SInt32 g_mtl_AngleSub_si32(UInt32 f_Angle1_ui32, UInt32 f_Angle2_ui32);
extern SInt16 g_mtl_AngleSub_si16(SInt16 f_Angle1_si16, SInt16 f_Angle2_si16);
extern SInt16 g_mtl_AngleAdd_si16(SInt16 f_Angle1_si16, SInt16 f_Angle2_si16);

extern UInt32 g_mtl_AngleF12ToF22_ui32(SInt16 f_Angle_si16);
extern SInt16 g_mtl_AngleF22ToF12_si16(UInt32 f_Angle_ui32);

// angle math (F12/SinCos) TODO: add to mathlib
extern SInt16 g_mtl_SinCosMul_si16(const SInt16 f_Multiplier_si16,
                                   const SInt16 f_SinCos_si16);
extern SInt32 g_mtl_AngleMul_si32(SInt16 f_Angle_si16, SInt16 f_Length_si16);
extern SInt16 g_mtl_NormalizeAngle_si16(SInt16 f_Angle_si16);

// Pseudo floating point value calculation functions
// TODO: Review and rework, then someday add to mathlib
extern UInt32 g_mtl_ScaledSqrt_ui32(UInt32 f_Value_ui32, UInt8 *f_Scaling_pui8);

extern SInt32 g_mtl_ScaledAdd_si32(SInt32 f_Summand1_si32, SInt32 f_Summand2_si32,
                                   SInt8 *f_Scaling_psi8);

extern UInt32 g_mtl_ShiftScaledValueToTargetResolution_ui32(UInt32 f_Value_ui32,
                                                            UInt8 f_Scaling_ui8,
                                                            UInt8 f_TargetResolution_ui8);

extern SInt32 g_mtl_ShiftScaledValueToTargetResolution_si32(SInt32 f_Value_si32,
                                                            SInt8 f_Scaling_si8,
                                                            UInt8 f_TargetResolution_ui8);

extern SInt32 g_mtl_ScaleUp_si32(SInt32 f_Value_si32, SInt8 *f_Scaling_psi8);

extern UInt32 g_mtl_ScaleUp_ui32(UInt32 f_Value_ui32, UInt8 *f_Scaling_pui8);

extern void g_mtl_ScaleDown_si32_toFit_si16_vd(SInt32 *f_X_si32, SInt8 *f_X_scale_si8);

extern void g_mtl_ScaleDown2_si32_vd(UInt8 f_largestAllowedValueBit_ui8, SInt32 *f_X_si32,
                                     SInt32 *f_Y_si32, SInt8 *f_scale_si8);

extern void g_mtl_AdjustScaling_vd(SInt32 *f_Value1_psi32, SInt8 *f_Scaling1_psi8,
                                   SInt32 *f_Value2_psi32, SInt8 *f_Scaling2_psi8);

/************************************************************************/
/* Angle functions                                                      */
/************************************************************************/

extern void g_mtlAngle_Add_vd( // result = a + b
    gType_mtlAngle_st *f_Result_pst, gType_mtlAngle_st const *f_A_pst,
    gType_mtlAngle_st const *f_B_pst);

extern void g_mtlAngle_Sub_vd( // result = a - b
    gType_mtlAngle_st *f_Result_pst, gType_mtlAngle_st const *f_A_pst,
    gType_mtlAngle_st const *f_B_pst);

extern void g_mtlAngle_Neg_vd(gType_mtlAngle_st *f_Angle_pst); // angle = -angle

extern void g_mtlAngle_Set_vd(gType_mtlAngle_st *f_Angle_pst,
                              SInt16 f_Alpha_si16); // angle := alpha

extern void g_mtl_NormalizeVector_ui16(gType_mtlPoint_st *f_Vector_pst,
                                       const UInt16 f_Norm_ui16);

extern SInt16 g_mtl_ArcTan_Approx_si16(SInt16 f_dY_si16, SInt16 f_dX_si16,
                                       UInt8 f_Order_ui8);
/************************************************************************/
/* GEOLIB FUNCTIONS                                                     */
/************************************************************************/

extern void g_mtl_Vector_Set_vd(const gType_mtlPoint_st *f_P1_pst,
                                const gType_mtlPoint_st *f_P2_pst,
                                gType_mtlPoint_st *f_out_pst);

extern UInt16 g_mtl_VectorNorm_ui16(gType_mtlPoint_st const *f_Vector_pst);

extern UInt32 g_mtl_VectorNormSqr_ui32(gType_mtlPoint_st const *f_Vector_pst);

extern SInt32 g_mtl_InnerProduct_si32(gType_mtlPoint_st const *f_Vector1_pst,
                                      gType_mtlPoint_st const *f_Vector2_pst);

extern SInt16 g_mtl_VectorProjection_si16(gType_mtlPoint_st const *f_Vector1_pst,
                                          gType_mtlPoint_st const *f_Vector2_pst);

extern SInt32 g_mtl_ExteriorProduct_si32(gType_mtlPoint_st const *f_Vector1_pst,
                                         gType_mtlPoint_st const *f_Vector2_pst);

extern SInt32 g_mtl_ExteriorProductExtend_si32(gType_mtlPointPos_st const *f_Vector1_pst,
                                               gType_mtlPointPos_st const *f_Vector2_pst);

extern SInt32 g_mtl_PointCCW_si32(gType_mtlPoint_st const *f_A_pst,
                                  gType_mtlPoint_st const *f_B_pst,
                                  gType_mtlPoint_st const *f_C_pst);

extern SInt32 g_mtl_PointCCWExtend_si32(gType_mtlPoint_st const *f_A_pst,
                                        gType_mtlPoint_st const *f_B_pst,
                                        gType_mtlPoint_st const *f_C_pst);

extern Boolean g_mtl_PointInsideConvexPolygon_bl(gType_mtlPoint_st const *f_P_pst,
                                                 gType_mtlPoint_st const *f_Vertices_pst,
                                                 UInt8 f_numVertices_ui8);

extern Boolean g_mtl_PointInsideConvexPolygonExtend_bl(
    gType_mtlPoint_st const *f_P_pst, gType_mtlPoint_st const *f_Vertices_pst,
    UInt8 f_numVertices_ui8);

extern SInt16 g_mtl_GetAngleBetweenThreePoints_si16(gType_mtlPoint_st const *f_p1_pst,
                                                    gType_mtlPoint_st const *f_p2_pst,
                                                    gType_mtlPoint_st const *f_p3_pst);

extern void g_mtl_Rotate_vd(gType_mtlPoint_st const *f_P_pst,
                            gType_mtlAngle_st const *f_Angle_pst,
                            gType_mtlPoint_st *f_P_rot_pst);

extern void g_mtl_RotateAroundPoint_vd(gType_mtlPoint_st const *f_P_pst,
                                       gType_mtlPoint_st const *f_RotationCenter_pst,
                                       gType_mtlAngle_st const *f_Angle_pst,
                                       gType_mtlPoint_st *f_P_rot_pst);

extern void g_mtl_TransformToLocal_vd(gType_mtlPoint_st const *f_P_pst,
                                      gType_mtlLineSinCos_st const *f_LocalCoordSys_pst,
                                      gType_mtlPoint_st *f_P_trans_pst);

extern void g_mtl_TransformToGlobal_vd(gType_mtlPoint_st const *f_P_pst,
                                       gType_mtlLineSinCos_st const *f_LocalCoordSys_pst,
                                       gType_mtlPoint_st *f_PTrans_pst);

extern Boolean g_mtl_GetPointOnCircle_bl(gType_mtlCircleU16_st const *f_Circle_pst,
                                         gType_mtlAngle_st const *f_Phi_st,
                                         gType_mtlPoint_st *f_PointOnCircle_pst);

extern Boolean g_mtl_LineIntersection_bl(gType_mtlLineSinCos_st const *f_Line1_pst,
                                         gType_mtlLineSinCos_st const *f_Line2_pst,
                                         SInt16 *f_Length1_si16, SInt16 *f_Length2_si16,
                                         gType_mtlPoint_st *f_P_pst);

extern SInt16 g_mtl_DistPointVectorSegment_si16(
    gType_mtlPoint_st const *f_P_pst, gType_mtlLineSegment_st const *f_LineSegment_pst);

extern SInt32 g_mtl_DistPointVectorAngle_si32(gType_mtlPoint_st const *f_Point_pst,
                                              gType_mtlLineSinCos_st const *f_Line_pst);

extern UInt32 g_mtl_DistPointPoint_ui32(gType_mtlPoint_st const *f_P1_pst,
                                        gType_mtlPoint_st const *f_P2_pst);

extern UInt32 g_mtl_DistPointLineSegment_ui32(
    gType_mtlPoint_st const *f_P_pst, gType_mtlLineSegment_st const *f_LineSeg_pst);

extern void g_mtl_FootPointLineSegment_vd(gType_mtlLineSegment_st const *f_Line_pst,
                                          gType_mtlPoint_st const *f_P_pst,
                                          gType_mtlPoint_st *f_FootPoint_pst);

extern UInt16 g_mtl_CalcCathetus_ui16(UInt16 f_Hypothenuse_ui16, UInt16 f_Cathetus_ui16);

extern UInt8 g_mtl_IntersectionCircleCircle_ui8(
    gType_mtlPoint_st *f_P1_pst, gType_mtlPoint_st *f_P2_pst,
    gType_mtlCircleU16_st const *f_Circle1_pst,
    gType_mtlCircleU16_st const *f_Circle2_pst);

extern Boolean g_mtl_IsValueInRangeS16_bl(const SInt16 f_value_si16,
                                          const SInt16 f_min_si16,
                                          const SInt16 f_max_si16);

extern Boolean g_mtl_IsValueInRangeU16_bl(const UInt16 f_value_ui16,
                                          const UInt16 f_min_ui16,
                                          const UInt16 f_max_ui16);

extern Boolean g_mtl_IsValueInOIRangeS16_bl(const SInt16 f_value_si16,
                                            const SInt16 f_min_si16,
                                            const SInt16 f_max_si16);

extern void g_mtl_Range2D_vd(gType_mtlPoint_st const *f_P1_pst,
                             gType_mtlPoint_st const *f_P2_pst,
                             gType_mtlRange2D_st *f_Range2D_pst);

extern UInt16 g_mtl_SideLengthOfRightTriangle_ui16(const UInt16 f_hypotenuse_ui16,
                                                   const UInt16 f_otherside_ui16);

extern SInt8 g_mtl_IntersectionCircleLine_si8(gType_mtlCircleU16_st const *f_Circle_pst,
                                              gType_mtlLineSinCos_st const *f_line_pst,
                                              gType_mtlPoint_st *f_P1_pst,
                                              gType_mtlPoint_st *f_P2_pst);

extern SInt32 g_mtl_GetRadiusFromTwoPoints_si32(const gType_mtlPoint_st *f_point1_pst,
                                                const gType_mtlPoint_st *f_point2_pst);

/************************************************************************/
/* Functions working with vehicle contour (APG/OOC relevant)            */
/************************************************************************/

extern SInt16 g_mtl_CalcRadiusFromPointToCorner_si16(
    gType_mtlPoint_st const *f_point_pst, gType_mtlCircleU16_st const *f_circle_pst,
    UInt16 f_halfWidth_ui16);

#endif /*(GS_MTL_GEOLIB == SW_ON)*/

#endif /* MATHLIB_GEOMETRY_API_H */

/*========================= EoF (mathlib_geometry_api.h) ====================*/
