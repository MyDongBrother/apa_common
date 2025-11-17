#pragma once
#include "stdio.h"
#include <sys/time.h>
#include "MathPara.h"

#define count_coeff 0.01137f
#define _W          1.64f

class Wheel_optimal
{
    // #define count_coeff 0.01066925f
    // #define count_coeff 0.01079925f
    // #define count_coeff 0.0115f
    // #define count_coeff 0.0110f
    // #define count_coeff 0.011f
    // #define count_coeff 0.0112f
    // #define count_coeff 0.0113f
    // #define count_coeff 0.01135f

  public:
    Wheel_optimal(const float &last_angle = 0, const bool &direction = 0)
        : m_last_angle(last_angle), m_rotate_direction(direction){};
    void update_last_angle(const float &last_angle, const float &last_angle_vehicle)
    {
        m_last_angle         = last_angle;
        m_last_angle_vehicle = last_angle_vehicle;
    };
    void update_cur_angle(const float &cur_angle, const float &cur_angle_vehicle)
    {
        if (cur_angle > PI)
        {
            m_cur_angle = cur_angle - 2 * PI;
        }
        else if (cur_angle < -PI)
        {
            m_cur_angle = cur_angle + 2 * PI;
        }
        else
        {
            m_cur_angle = cur_angle;
        }
        m_cur_angle_vehicle = cur_angle_vehicle;
    }
    void reset() { mark_init = true; }
    int Locating_WheelCounter(int count_last, int count_cur)
    {
        int dcount;
        if (count_cur - count_last >= 0)
            dcount = count_cur - count_last;
        else
            dcount = count_cur + 1022 - count_last;

        if (abs(dcount) > 100 || (count_cur == 0 && count_last < 610))
            dcount = 0;
        return dcount;
    }
    void update_cnt(const int &straight_direciton, const float &cur_RR_cnt,
                    const float &cur_RL_cnt)
    {
        m_acc_RR_cnt = m_acc_RR_cnt + straight_direciton * Locating_WheelCounter(
                                                               m_last_RR_cnt, cur_RR_cnt);
        m_acc_RL_cnt = m_acc_RL_cnt + straight_direciton * Locating_WheelCounter(
                                                               m_last_RL_cnt, cur_RL_cnt);
        m_last_RR_cnt = cur_RR_cnt;
        m_last_RL_cnt = cur_RL_cnt;
    }
    void is_valid(const int &rotate_direction, const float &cur_RR_cnt,
                  const float &cur_RL_cnt)
    {
        if (rotate_direction != m_rotate_direction || mark_init)
        {
            m_rotate_direction = rotate_direction;
            valid              = false;
            time_cnt           = 0;
            update_last_angle(m_cur_angle, m_cur_angle_vehicle);
            m_acc_RR_cnt  = 0;
            m_acc_RL_cnt  = 0;
            m_last_RR_cnt = cur_RR_cnt;
            m_last_RL_cnt = cur_RL_cnt;
            mark_init     = false;
        }
        else
        {
            time_cnt++;
            if (time_cnt >= 10)
            {
                valid = true;
            };
        }
    }
    bool omptial(const int &rotate_direction, const int &straight_direciton,
                 const float &cur_angle, const float &cur_RR_cnt, const float &cur_RL_cnt,
                 const float &vehicle_angle, const float &vehicle_angle_rate, FILE *fp,
                 float &optimal_angle)
    {
        if (fp != nullptr)
        {
            fprintf(fp, "%d,%d,%f,%f,%f,%f,%f,", rotate_direction, straight_direciton,
                    cur_angle, cur_RR_cnt, cur_RL_cnt, vehicle_angle, vehicle_angle_rate);
        }
        update_cur_angle(cur_angle, vehicle_angle);
        is_valid(rotate_direction, cur_RR_cnt, cur_RL_cnt);
        update_cnt(straight_direciton, cur_RR_cnt, cur_RL_cnt);
        if (valid)
        {
            diff_angle         = m_cur_angle - m_last_angle;
            diff_angle_vehicle = m_cur_angle_vehicle - m_last_angle_vehicle;
            angle_by_wheel =
                static_cast<float>(m_acc_RR_cnt - m_acc_RL_cnt) * count_coeff * 2 / _W;
            if (fabs(angle_by_wheel) > PI / 3) // 调用一次滤波
            {
                optimal_angle = angle_by_wheel;
                mark_init     = true;
                return true;
            }
        }
        return false;
    }
    void record(FILE *fp)
    {
        if (fp != nullptr)
        {
            if (valid)
                fprintf(fp, "%f,%f,%f,%f\n", diff_angle, angle_by_wheel,
                        diff_angle - angle_by_wheel, diff_angle_vehicle - diff_angle);
            else
                fprintf(fp, "%f,%f,%f,%f\n", 0.0, 0.0, 0.0, 0.0);
        }
        else
        {
            printf("Wheel_optimal.record() open file failed !\n");
        }
    }

  public:
    // imu来计算
    float m_last_angle{0};
    float m_cur_angle{0};
    float diff_angle{0};

    // 车载来计算
    float m_last_angle_vehicle{0};
    float m_cur_angle_vehicle{0};
    float diff_angle_vehicle{0};

    int m_rotate_direction{-1};  // -1为逆时针，1为顺时针
    int m_straight_direciton{1}; // 1为前进，-1为后退
    bool valid{false};           // 满足优化条件
    bool mark_init{true};
    uint32_t time_cnt{0}; // 每个周期约20ms

    int16_t m_last_RR_cnt;
    int16_t m_last_RL_cnt;
    int16_t m_acc_RR_cnt;
    int16_t m_acc_RL_cnt;
    float angle_by_wheel;
};