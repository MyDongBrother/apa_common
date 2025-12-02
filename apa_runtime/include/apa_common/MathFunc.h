/**
 * @file MathFunc.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-14
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-14 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#ifndef _MATH_FUNC_H
#define _MATH_FUNC_H

#include "math.h"
#include "MathLib_Type.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef math_max
#define math_max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef math_min
#define math_min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef sign
#define sign(a) ((fabs(a) < ZERO_FLOAT) ? 0 : ((a) > 0 ? 1 : -1))
#endif

    float Round_PI(float theta);
    void Convert(const Pos2d_T *pO, const Pos2d_T *pin, Pos2d_T *pout);
    void ConvertVec(const Pos2d_T *pO, const Vec2d_T *pin, Vec2d_T *pout);

#ifdef __cplusplus
}
#endif

#endif
