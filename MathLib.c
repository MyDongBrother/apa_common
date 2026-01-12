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
 * @brief 将全局坐标系下的点转换到局部坐标系
 *
 * @param pO    局部坐标系原点在全局坐标系的位置及朝向 (x, y, phi)
 * @param pin   全局坐标系下的点坐标  (x, y, phi)
 * @param pout  输出的局部坐标系下的点坐标  (x, y, phi)
 *
 * @note 使用二维刚体逆变换：先平移，再旋转
 */
void RevConvert(const Pos2d_T *pO, const Pos2d_T *pin, Pos2d_T *pout)
{
    // 平移到局部原点
    float dx = pin->x - pO->x;
    float dy = pin->y - pO->y;

    // 旋转到局部坐标系
    float cos_phi = cosf(pO->phi);
    float sin_phi = sinf(pO->phi);

    pout->x = dx * cos_phi + dy * sin_phi;
    pout->y = -dx * sin_phi + dy * cos_phi;

    // phi
    pout->phi = Round_PI(pin->phi - pO->phi);
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

/**
 * @brief 将局部坐标系下的点转换到全局坐标系
 *
 * @param pO    局部坐标系原点在全局坐标系的位置及朝向 (x, y, phi)
 * @param pin   局部坐标系下的点坐标 (x, y)
 * @param pout  输出的全局坐标系下的点坐标 (x, y)
 *
 * @note 使用二维刚体变换：先旋转，再平移
 */
void ConvertArray(const float *pO, const float *pin, float *pout)
{
    pout[0] = pin[0] * cosf(pO[2]) - pin[1] * sinf(pO[2]) + pO[0];
    pout[1] = pin[0] * sinf(pO[2]) + pin[1] * cosf(pO[2]) + pO[1];
}

void RevConvertArray(const float *pO, const float *pin, float *pout)
{
    float dx = pin[0] - pO[0];
    float dy = pin[1] - pO[1];

    float cos_phi = cosf(pO[2]);
    float sin_phi = sinf(pO[2]);

    pout[0] = dx * cos_phi + dy * sin_phi;
    pout[1] = -dx * sin_phi + dy * cos_phi;
}