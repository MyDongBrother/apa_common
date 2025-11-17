#ifndef _PK_PATHEXECUTCONFIG_H_
#define _PK_PATHEXECUTCONFIG_H_
#include "Rte.h"

enum
{
    PATHEXECUTE_SUCCESS,
    PATHEXECUTE_FAIL
};

const uint8_t B_DangeReplanWaitCnt_ccp =
    100; // 20181031: to adapt to objavoid need at least 2.4s to do the replan
const float front_swell         = 0.33;
const float CONFIG_SIDE_SWELL_2 = 0.30;
const float side_swell_0        = 0.35;
const float side_swell_1        = 0.45;
const float post_swell          = 0.15;

const float thetaGap_allow   = 0.02f; // 允许的角度误差（弧度）
const float dfradioGap_allow = 0.05f; // 允许的距离误差（米）
const float rxGap_allow      = 0.45f;

const uint8_t CONFIG_POSREPLAN_NUM = 28;
const float CONFIG_VALID_OBJ_LEN   = 0.35f;

#define PATH_ITEM_LEN (sizeof(float) * TRAJITEM_LEN)

#endif
