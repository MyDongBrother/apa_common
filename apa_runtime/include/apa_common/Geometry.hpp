/**
 * @file Geometry.hpp
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-10-30
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-10-30 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#pragma once

#include "MathLib_Type.h"
#include <cmath>

/**
 * @brief 二维线段类
 */
struct GeomLine2d_T
{
  public:
    Vec2d_T start;
    Vec2d_T end;
    Vec2d_T unit_direction;
    double heading = 0.0;
    double length  = 0.0;

  public:
    GeomLine2d_T(const Vec2d_T &s, const Vec2d_T &e) : start(s), end(e)
    {
        double dx = e.x - s.x;
        double dy = e.y - s.y;
        length    = std::sqrt(dx * dx + dy * dy);

        if (length > ZERO_FLOAT)
        {
            unit_direction.x = dx / length;
            unit_direction.y = dy / length;
        }
        else
        {
            unit_direction = {0.0, 0.0};
            length         = 0.0;
        }

        heading = std::atan2(dy, dx);
    }

    /**
     * @brief 与另一条线段方向向量点积
     */
    double DotDirection(const GeomLine2d_T &other) const
    {
        return unit_direction.x * other.unit_direction.x +
               unit_direction.y * other.unit_direction.y;
    }
};
