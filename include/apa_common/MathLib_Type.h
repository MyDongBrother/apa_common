/**
 * @file MathLib_Type.h
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

#ifndef __MATH_LIB_H__
#define __MATH_LIB_H__

#include "Global_Types.h"

#ifndef PI
#define PI 3.1415926535898f // 圆周率
#endif
#define PI2    6.2831853071796f  // 2倍圆周率
#define PI_2   1.5707963267949f  // 1/2圆周率
#define PI_RAD 0.0174532925199f  // 角度转换为弧度参数
#define PI_DEG 57.2957795130823f // 弧度转换为角度参数

#define ZERO_FLOAT 1e-6

/**
 * @brief 二维坐标点结构体
 */
typedef struct _Vec2d_T
{
    float x; //< x坐标
    float y; //< y坐标
} Vec2d_T;

/**
 * @brief 二维位姿
 */
typedef struct _Pos2d_T
{
    float x;   //< x坐标
    float y;   //< y坐标
    float phi; //< 航向角（单位：弧度）
} Pos2d_T;

/**
 * @brief 二维线段
 */
typedef struct _Line2d_T
{
    Vec2d_T start; //< 起点
    Vec2d_T end;   //< 终点
} Line2d_T;

/**
 * @brief 二维线段结构体
 */
typedef struct _LineSegment2d_T
{
    Vec2d_T start;          //< 起点
    Vec2d_T end;            //< 终点
    Vec2d_T unit_direction; //< 单位方向向量
    double heading = 0.0;   //< 线段朝向（单位：弧度）
    double length  = 0.0;   //< 线段长度
} LineSegment2d_T;

#endif
