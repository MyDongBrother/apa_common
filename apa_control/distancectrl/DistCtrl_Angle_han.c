/**
 * @file DistCtrl_Angle_han.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-08-28
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-08-28 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "DistCtrl.h"
#include "Rte_ComIF.h"

/**
 * @brief 得到要下发给车身的方向盘角度
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
float AngleControl_han(const float targetAngle, const float dt, const float EspSPD,
                       const float distCfg, const float curSpd, float paras[3],
                       const int isInit)
{
    static float lastAngleReq  = 2 * PK_EPS_ANGLE_MAX;
    static uint8_t lastEpsStat = 0;
    static float EPS_AngleReq  = 0.0f;
    static uint8_t epsflag     = 0;
    static float AngleCmd[2]   = {0.0f};
    static int startPark       = 0;

    float omgn      = 9.0f; // Frequency of track filter, in rad/s
    float Track_Err = 0.0f;
    float steerSpd  = EspSPD;

    float SAS_SteeringAngle = RTE_BSW_Get_EPS_Angle();
    float Ang_Error         = targetAngle - EPS_AngleReq;
    uint8_t epsStat         = RTE_BSW_Get_EpsCtrlStat();

    int clearFlag               = isInit;
    PK_ModuleStateType modState = RTE_PK_DistCtrl_Get_ModuleState_DistCtrl();
    if (modState == MSTAT_NORM)
    {
        startPark++;
        if (startPark < 100) // 2.0s
        {
            omgn     = 5.0;
            steerSpd = rte_min(EspSPD, 300);
            if (startPark == 1)
            {
                clearFlag = 1;
            }
        }
        else
        {
            startPark = 100;
        }
    }
    else
    {
        startPark = 0;
    }

    float deltaAng   = steerSpd * dt;
    auto InitVarFunc = [&epsStat]() {
        epsflag     = 0;
        lastEpsStat = epsStat;
    };

    if (clearFlag)
    {
        lastAngleReq = 2 * PK_EPS_ANGLE_MAX;
        lastEpsStat  = 0;
        EPS_AngleReq = 0.0f;
        epsflag      = 0;
        AngleCmd[0]  = 0.0f;
        AngleCmd[1]  = 0.0f;
    }

    paras[0] = EPS_AngleReq;
    paras[1] = AngleCmd[0];
    paras[2] = AngleCmd[1];

    if (lastAngleReq > PK_EPS_ANGLE_MAX)
    {
        lastAngleReq = SAS_SteeringAngle;
    }

    if (epsStat == 2 && lastEpsStat != epsStat)
    {
        printf("last epsStat != 2\n");
        InitVarFunc();
        lastAngleReq = SAS_SteeringAngle;
        return lastAngleReq;
    }
    else if (epsStat != 2)
    {
        // printf("current epsStat != 2\n");
        InitVarFunc();
        lastAngleReq = SAS_SteeringAngle;
        return lastAngleReq;
    }

    if (distCfg < 1.5 * ignoredist && curSpd < 0.08)
    {
        InitVarFunc();
        return lastAngleReq;
    }

    if (fabs(targetAngle - SAS_SteeringAngle) <
        5) // 20240420测试效果：导致最大角度可能超过450，方向盘转动不正常，
           // 尝试：加入最大角度限制
    {
        InitVarFunc();
        lastAngleReq = rte_min((fabs(lastAngleReq - targetAngle) < 5
                                    ? lastAngleReq
                                    : (targetAngle + SAS_SteeringAngle) * 0.50f),
                               450.0f);
        return lastAngleReq;
    }

    if (epsflag == 0)
    {
        EPS_AngleReq = SAS_SteeringAngle;
        AngleCmd[0]  = EPS_AngleReq;
        AngleCmd[1]  = 0.0f;
        epsflag      = 1;
    }
    else
    {
        if (fabsf(Ang_Error) < deltaAng)
        {
            EPS_AngleReq = targetAngle;
        }
        else
        {
            if (Ang_Error > ZERO_FLOAT)
            {
                EPS_AngleReq += deltaAng;
            }
            else
            {
                EPS_AngleReq -= deltaAng;
            }
        }
    }

    EPS_AngleReq = rte_min(PK_EPS_ANGLE_MAX, rte_max(-PK_EPS_ANGLE_MAX, EPS_AngleReq));
    Track_Err    = AngleCmd[0] - EPS_AngleReq;
    AngleCmd[0] += dt * AngleCmd[1];
    AngleCmd[1] += dt * (-omgn * omgn * Track_Err - 2.0f * omgn * AngleCmd[1]);
    AngleCmd[0] = rte_min(PK_EPS_ANGLE_MAX, rte_max(-PK_EPS_ANGLE_MAX, AngleCmd[0]));

    float max_esp_dlt = max_esp_spd * dt;
    float result      = AngleCmd[0];
    if (result > lastAngleReq + max_esp_dlt)
    {
        result = lastAngleReq + max_esp_dlt;
    }
    else if (result < lastAngleReq - max_esp_dlt)
    {
        result = lastAngleReq - max_esp_dlt;
    }

    lastAngleReq = rte_min(PK_EPS_ANGLE_MAX, rte_max(-PK_EPS_ANGLE_MAX, result));
    lastEpsStat  = epsStat;

    return lastAngleReq;
}
