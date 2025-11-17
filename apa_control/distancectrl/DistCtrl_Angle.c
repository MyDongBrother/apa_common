#include "DistCtrl.h"

/**
 * @brief 得到要下发给车身的方向盘角度
 *
 *
 *
 * @param[in] targetAngle 目标方向盘角度
 * @param[in] dt 下发周期0.02s
 * @param[in] EspSPD esp速度
 * @param[in] distCfg 下发距离
 * @param[in] curSpd ESC速度
 * @param[in] isInit 初始化选项
 * @param[out] paras
 * @param[in,out]
 * @return 下发方向盘角度
 *
 * @note
 */
float AngleControl(const float targetAngle, const float dt, const float EspSPD,
                   const float distCfg, const float curSpd, float paras[3],
                   const int isInit)
{
    if (PK_VehConfigType() == CAR_TYPE_HAN)
    {
        return AngleControl_han(targetAngle, dt, EspSPD, distCfg, curSpd, paras, isInit);
    }
    else if (PK_VehConfigType() == CAR_TYPE_YUAN)
    {
        return AngleControl_yuan(targetAngle, dt, EspSPD, distCfg, curSpd, paras, isInit);
    }
    else
    {
        return 0;
    }
}
