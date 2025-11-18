/**
 * @file MathFunc.c
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
#include "Std_Types.h"
#include "MathFunc.h"

/**
 * @brief 将角度归一化到 [-π, π] 区间
 *
 * @param theta 输入角度（弧度）
 * @return float 归一化后的角度，范围为 [-π, π]
 */
float Round_PI(float theta)
{
    float value = fmodf(theta, 2.0f * PI);
    if (value > PI)
        value -= 2.0f * PI;
    else if (value < -PI)
        value += 2.0f * PI;
    return value;
}

/**
 * @brief 将局部坐标系下的点转换到全局坐标系
 *
 * @param pO    局部坐标系原点在全局坐标系的位置及朝向 (x, y, phi)
 * @param pin   局部坐标系下的点坐标  (x, y, phi)
 * @param pout  输出的全局坐标系下的点坐标  (x, y, phi)
 *
 * @note 使用二维刚体变换：先旋转，再平移
 */
void Convert(const Pos2d_T *pO, const Pos2d_T *pin, Pos2d_T *pout)
{
    pout->x   = pin->x * cosf(pO->phi) - pin->y * sinf(pO->phi) + pO->x;
    pout->y   = pin->x * sinf(pO->phi) + pin->y * cosf(pO->phi) + pO->y;
    pout->phi = Round_PI(pin->phi + pO->phi);
}

/**
 * @brief 将局部坐标系下的点转换到全局坐标系
 *
 * @param pO    局部坐标系原点在全局坐标系的位置及朝向 (x, y, phi)
 * @param pin   局部坐标系下的点坐标 (x, y)
 * @param pout  输出的全局坐标系下的点坐标 (x, y)
 *
 * @note 使用二维刚体变换：先旋转，再平移
 */
void ConvertVec(const Pos2d_T *pO, const Vec2d_T *pin, Vec2d_T *pout)
{
    pout->x = pin->x * cosf(pO->phi) - pin->y * sinf(pO->phi) + pO->x;
    pout->y = pin->x * sinf(pO->phi) + pin->y * cosf(pO->phi) + pO->y;
}
