/*
 * @Author: error: error: git config user.name & please set dead value or install git &&
 * error: git config user.email & please set dead value or install git & please set dead
 * value or install git
 * @Date: 2024-10-15 21:02:55
 * @LastEditors: error: error: git config user.name & please set dead value or install git
 * && error: git config user.email & please set dead value or install git & please set
 * dead value or install git
 * @LastEditTime: 2024-10-17 16:45:57
 * @FilePath: \apa-V2.0.9\apa_control\apa_distancectrl\DistCtrl.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PK_DISTCTRL_HEADER__
#define PK_DISTCTRL_HEADER__

#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "MathPara.h"
#include "Rte_BSW.h"
#include "Module_API.h"
#include "Rte_Func.h"

constexpr float ignoredist = 0.03;

float AngleControl(const float targetAngle, const float dt, const float EspSPD,
                   const float distCfg, const float curSpd, float paras[3],
                   const int isInit = 0);

float AngleControl_han(const float targetAngle, const float dt, const float EspSPD,
                       const float distCfg, const float curSpd, float paras[3],
                       const int isInit);

float AngleControl_yuan(const float targetAngle, const float dt, const float EspSPD,
                        const float distCfg, const float curSpd, float paras[3],
                        const int isInit);

void PK_DistanceCtrl_yuan();

void PK_DistanceCtrl_han();

void InitializeTrendQueue();
int AnalyzeSpeedTrend(float current_data);
int AnalyzeDistTrend(float current_data);

#endif
