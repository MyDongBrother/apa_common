#include "DistCtrl.h"
#include "PK_Location.h"
#include "CanIf.h"
#include "Record_Buff.h"
#include "Rte_BSW.h"

const float timedt       = 0.02;
static float s_paras[6]  = {0.0, 0.0, 0.0, 0.0};
const float min_run_dist = 0.26;
static uint8_t execSt    = 0;

static float Interpolation(const float &x_down, const float &x_up, const float &y_down,
                           const float &y_up, const float &x)
{
    if (x < x_down)
    {
        return y_down;
    };
    if (x > x_up)
    {
        return y_up;
    };
    float k      = (y_up - y_down) / (x_up - x_down);
    float result = k * (x - x_down) + y_down;
    return result;
};

static int WheelCounter(int count_last, int count_cur)
{
    int dcount;
    if (count_cur - count_last >= 0)
    {
        dcount = count_cur - count_last;
    }
    else
    {
        dcount = count_cur + 1022 - count_last;
    }

    if (abs(dcount) > 100 || (count_cur == 0 && count_last < 610))
    {
        dcount = 0;
    }
    return dcount;
}

static void *InitDistLog()
{
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);

    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1, "%s/%d_%02d_%02d_%02d_%02d_%02d_dist.txt",
             pathName, absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday,
             absStamp.tm_hour, absStamp.tm_min, absStamp.tm_sec);

    auto distlogFid = fopen(fileName, "w");
    if (distlogFid == NULL)
    {
        return NULL;
    }

    static uint8_t logbuff[40 * 1024] = {0};
    void *distlog                     = Buff_Init(logbuff, sizeof(logbuff), distlogFid);
    if (distlog == NULL)
    {
        fclose(distlogFid);
    }

    return distlog;
}

static float WheelDistDt()
{
    static int lastRlCount = 0;
    static int lastRRCount = 0;
    int curRlCount         = RTE_BSW_Get_WheelCounter_RL();
    int curRRCount         = RTE_BSW_Get_WheelCounter_RR();

    int dCounterRL = WheelCounter(lastRlCount, curRlCount);
    int dCounterRR = WheelCounter(lastRRCount, curRRCount);
    float dsrw     = Rear_wheel_fac_ds * (dCounterRL + dCounterRR);
    lastRlCount    = curRlCount;
    lastRRCount    = curRRCount;
    return dsrw;
}

static void DistPidPara(const int init, const float curSpd, const float targSpd,
                        float &spdfact, float &spdsum)
{
    const int wind_size = 12;

    static float buff[wind_size] = {0};
    static int runCount          = 0;
    static float parai           = 0;

    if (init == 1)
    {
        runCount = 0;
        parai    = 0;
    }

    float factor = rte_min((curSpd - targSpd) / targSpd, 1.0);
    float parap  = factor - pow3(factor) / 6;
    if (runCount >= wind_size)
    {
        parai                      = parai + parap - buff[runCount % wind_size];
        buff[runCount % wind_size] = parap;
        spdsum                     = parai / wind_size;
    }
    else
    {
        buff[runCount] = parap;
        parai          = parai + buff[runCount];
        spdsum         = parai / (runCount + 1);
    }
    runCount = runCount + 1;
    spdfact  = parap;
}

static float calculate_acceleration(float curSpd, float velSpd, float accel_x)
{
    if (fabsf(accel_x) < 0.03)
    {
        return 0.0f;
    }
    // 判断是否加速或减速
    if (curSpd * accel_x > 0)
    {
        // 加速情况：确保返回正值
        return fabsf(accel_x); // 加速，返回正值
    }
    else if (curSpd * accel_x < 0)
    {
        // 减速情况：确保返回负值
        return -fabsf(accel_x); // 减速，返回负值
    }
    else
    {
        // curSpd为0的情况，返回真值
        return accel_x;
    }
}

static uint8_t BrakeToStop(const uint8_t stopstate, float *brakeDist, uint8_t *mebState,
                           const uint8_t brkSt, const uint8_t execSt,
                           const float brakeDistR, const float curSpd, const float velSpd)
{
    static uint8_t first_into   = 0;
    static float first_dist     = 0.0;
    static float last_brakeDist = 0.0;
    uint8_t brakState           = 0;

    if (stopstate == 0)
    {
        first_into     = 0;
        first_dist     = 0.0;
        last_brakeDist = *brakeDist;
        return brakState;
    }

    uint8_t temp_execSt = 0;
    uint8_t temp_brkSt  = 0;
    float distDiff      = 0;

    temp_execSt =
        brakeDistR > 0.03f && execSt < 2 && (fabs(curSpd) > 0.05f || velSpd > 0.05f)
            ? 0
            : execSt;
    temp_brkSt = brkSt > 1 && velSpd < 0.25f ? 1 : brkSt; //???????0???

    if ((execSt == 3) && fabsf(curSpd) < 0.20f)
    {
        brakState = rte_max(1, temp_brkSt);
    }
    else if (brkSt != 0 || ((execSt == 1 || execSt == 2) && fabsf(velSpd) < 0.12f))
    {
        temp_execSt = execSt > 2 ? 0 : execSt;
        brakState   = rte_max(temp_execSt, temp_brkSt);
    }

    *mebState = 3;

    if (brakState == 1)
    {
        if (first_into == 0)
        {
            first_into = 1;
            first_dist = brakeDistR;
            if (last_brakeDist > 0.30f)
            {
                last_brakeDist -= 0.20f;
            }
        }
        distDiff = fabs(first_dist - brakeDistR);

        if (brkSt == 1)
        {
            if (velSpd > 0.20f)
            {
                brakState  = 0;
                *brakeDist = last_brakeDist - velSpd * (5 * timedt);
            }
        }
        if (brkSt == 2)
        {
            if (distDiff < 0.05f && velSpd > 0.10f)
            {
                brakState  = 0;
                *brakeDist = last_brakeDist - velSpd * (5 * timedt);
            }
        }

        if (brakState == 0)
        {
            *brakeDist = rte_max(0.08, *brakeDist);
        }
    }
    else if (brakState == 2)
    {
        if (first_into == 0)
        {
            first_into = 1;
            *brakeDist = 0.5 * last_brakeDist;
            brakState  = 0;
        }
    }

    last_brakeDist = *brakeDist;

    return brakState;
}

static float SpeedConvert(const float brakeDist, const float brakeDistR,
                          const float curSpd, const float velSpd, const float targSpd,
                          const float targAngle, const float curAngle,
                          const int speed_trend, const int dist_trend,
                          const uint8_t curGear, const uint8_t obstacle_exist,
                          const uint8_t brkSt)
{
    static float last_speedlimit        = 0.0;
    static int repeat_count             = 0; // 用于计数重复次数
    static float last_output_speedlimit = 0.0;
    static float safeSpd                = 0.0f;
    static uint8_t last_brkSt           = 0;
    float speedLimit                    = 0.0;

    if (brakeDistR < 0.31f || brkSt == 2)
    {
        safeSpd = 0.0f;
    }
    else if (last_brkSt == 1 && brkSt == 0)
    {
        safeSpd = velSpd;
    }

    if (brakeDistR > 0.10f && brakeDist > 0.10f)
    {
        if (brakeDist < 0.8f || brakeDistR < 0.8f) //????
        {
            if (brakeDistR > 0.50f && brakeDist > 0.50f && velSpd < 0.35f)
            {
                speedLimit = 0.60f;
            }
            else
            {
                speedLimit = velSpd - 0.08f;
                speedLimit = rte_min(0.45f, rte_max(speedLimit, 0.35f));
            }
        }
        else if (brakeDist < 1.5f || brakeDistR < 1.5f)
        {
            if (velSpd > 0.30f)
            {
                speedLimit = velSpd - 0.08f;
                speedLimit = rte_min(0.60f, rte_max(speedLimit, 0.45f));
            }
            else
            {
                speedLimit = 0.60f;
            }
        }
        else if (brakeDist < 2.5f || brakeDistR < 2.5f)
        {
            if (velSpd > 0.30f)
            {
                speedLimit = velSpd - 0.08f;
                speedLimit = rte_min(0.75f, rte_max(speedLimit, 0.60f));
            }
            else
            {
                speedLimit = 0.70f;
            }
        }
        else if (fabs(targAngle) > 100 || fabs(targAngle - curAngle) > 50.0f ||
                 brakeDistR < 3.0f)
        {
            if (curGear == GEAR_D)
            {
                speedLimit = 0.75f;
            }
            else if (curGear == GEAR_R)
            {
                speedLimit = velSpd - 0.10f;
                speedLimit = rte_min(0.75f, rte_max(speedLimit, 0.60f));
            }
        }
        else if (fabs(targAngle) < 100)
        {
            if (curGear == GEAR_D)
            {
                speedLimit = 0.75f;
            }
            else if (curGear == GEAR_R)
            {
                speedLimit = 0.75f;
            }
        }
        else if (fabs(targSpd) > 0.001)
        {
            speedLimit = 0.75f;
        }
        else
        {
            speedLimit = 0.3f;
        }
    }
    // else if (speed_trend <= 0 && dist_trend < 0)
    // {
    //     speedLimit =  0.0f;
    // }
    else
    {
        speedLimit = 0.3f;
    }

    // if(fabs(targSpd) < 0.45f && brakeDist > 1.0f)
    // {
    //     speedLimit = rte_min(fabs(speedLimit), 0.50f);
    // }
    if (safeSpd > 0.0f && obstacle_exist > 0 && execSt == 4)
    {
        if (velSpd > safeSpd)
        {
            safeSpd = velSpd;
        }
        float safeLimit = floorf(safeSpd / 0.05f) * 0.05f;
        if (velSpd < 0.10f)
        {
            safeLimit = rte_max(safeLimit, 0.30f);
        }
        else
        {
            safeLimit = rte_max(safeLimit, 0.20f);
        }
        speedLimit = rte_min(speedLimit, safeLimit);
    }

    if (((last_speedlimit - 0.05) > speedLimit) && (speedLimit > 0.05) &&
        (last_speedlimit != 0))
    {
        speedLimit = last_speedlimit - 0.05;
    }

    last_brkSt = brkSt;

    if (last_output_speedlimit != speedLimit)
    {
        repeat_count           = 0;
        last_output_speedlimit = speedLimit;
    }
    else
    {
        repeat_count++;
    }
    if (repeat_count < 3)
    {
        return last_output_speedlimit;
    }
    else
    {
        repeat_count = 0;
    }

    last_speedlimit = speedLimit;

    return speedLimit;
}

static float DistConvert(const float brakeDistR, const float brakeDistB,
                         const float distdt, const float targetSpd, const float velSpd,
                         const float targetAngle, const int speed_trend,
                         const int dist_trend, uint8_t *stopState, const float accel_x,
                         uint8_t *obstacle_exist)
{
    static float codedist       = 0.0;
    static float spddist        = 0.0;
    static float lastcodedist   = 0.0;
    static float lastspddist    = 0.0;
    const float min_update_dist = 0.05;

    float tempDist = 0, brakeDist = 0;
    float minDist = rte_min(brakeDistB, brakeDistR);

    float calSpd = 0.0f;
    if (fabs(velSpd) > ZERO_FLOAT)
    {
        calSpd = velSpd + 0.3 * accel_x;
    }
    else
    {
        calSpd = 0.0f;
    }

    // int distb_trend = AnalyzeDistTrend(fabs(brakeDistB));

    // static int velflag = 0;

    // 1?????0.50m????????,2?????1.5?,3???????
    if ((brakeDistR - brakeDistB) > rte_min(0.2 * brakeDistR, 0.30f))
    {
        if (brakeDistB > 1.10f)
        {
            brakeDist = 0.5 * brakeDistR + 0.5 * brakeDistB;
        }
        else
        {
            brakeDist = brakeDistB;
            if (calSpd > 0.10)
            {
                *obstacle_exist = 2;
            }
            else
            {
                *obstacle_exist = 1;
            }
        }
    }
    else if (targetSpd > 0.01 && minDist > 0.50f) //??
    {
        tempDist =
            minDist > 0.7f ? minDist * 1.2f : (minDist + minDist * (minDist - 0.50f));
        brakeDist = tempDist;
    }
    else if ((brakeDistR - 0.10f) > brakeDistB && targetAngle < 30) //??,??
    {
        tempDist  = dist_trend >= 0 ? (brakeDistB * brakeDistR) * 0.50f
                                    : rte_min((2 * brakeDistB + brakeDistR) * 0.33f,
                                              brakeDistB * 1.5f);
        brakeDist = tempDist;
    }
    else
    {
        brakeDist = rte_min(brakeDistB, brakeDistR);
    }

    // brakeDist = brakeDist < 0.60f ? rte_min(brakeDistR, 0.60f) : brakeDist;
    // //???????????,???????????

    float rundist = brakeDist;

    if (rundist < min_update_dist || codedist < min_update_dist ||
        spddist < min_update_dist || velSpd < ZERO_FLOAT)
    {
        codedist     = rundist;
        lastcodedist = codedist;
        spddist      = rundist;
        lastspddist  = spddist;
    }
    else if (fabs(codedist - rundist) > min_update_dist ||
             fabs(spddist - rundist) > min_update_dist)
    {
        if ((codedist - rundist > 0.5) && rundist < 5.0f)
        {
            codedist = lastcodedist - 0.5f;
        }
        else if ((codedist - rundist > 0.10) && rundist < 5.0f && rundist > 0.80f)
        {
            codedist = lastcodedist - 0.08;
        }
        else if ((codedist - rundist > 0.25) && rundist < 0.80f)
        {
            codedist = lastcodedist - 0.05;
        }
        else if ((codedist - rundist > 0.05) && rundist < 0.80f)
        {
            codedist = codedist - distdt;
        }
        else
        {
            codedist = rundist;
        }
        lastcodedist = codedist;
        if ((spddist - rundist > 0.5) && rundist < 5.0f)
        {
            spddist = lastspddist - 0.5f;
        }
        else if ((spddist - rundist > 0.10f) && rundist < 5.0f && rundist > 0.8f)
        {
            spddist = lastspddist - 0.08f;
        }
        else if ((spddist - rundist > 0.20f) && rundist < 0.8f)
        {
            spddist = lastspddist - 0.05f;
        }
        else if ((spddist - rundist > 0.05f) && rundist < 0.8f)
        {
            spddist = spddist - timedt * velSpd;
        }
        else
        {
            spddist = rundist;
        }
        lastspddist = spddist;
    }
    else
    {
        codedist = codedist - distdt;
        spddist  = spddist - timedt * velSpd;

        float codedistdt = fabs(codedist - lastcodedist);
        float spddistdt  = fabs(spddist - lastspddist);
        if (fabs(codedistdt - spddistdt) > min_update_dist)
        {
            codedist     = rundist;
            lastcodedist = codedist;
            spddist      = rundist;
            lastspddist  = spddist;
        }
    }

    rundist = rte_min(spddist, 5.0f);

    if (((minDist < 0.20f && calSpd > 0.24f) || minDist < 0.15f) && calSpd != 0.0f)
    {
        *stopState = 1;
    }

    if (rundist < 0.80f && (rundist - minDist < 0.05f) && *stopState != 1) //??????
    {
        if (minDist > 0.22f && fabs(velSpd) < 0.30f)
        {
            if (fabs(velSpd) < 0.20f)
            {
                if (execSt == 4)
                {
                    rundist += 0.30f;
                }
                else
                {
                    rundist += 0.20f;
                }
            }
            else
            {
                if (execSt == 4)
                {
                    rundist += 0.20f;
                }
                else
                {
                    rundist += 0.15f;
                }
            }
        }
        else if (minDist < 0.22f && minDist > 0.10f && fabs(velSpd) < 0.18f)
        {
            if (execSt == 4)
            {
                rundist += 0.20f;
            }
            else
            {
                rundist += 0.15f;
            }
        }
    }
    else if (targetAngle < 30 && dist_trend >= 0 &&
             brakeDistB < rte_min(brakeDistR, 0.50f)) // ?????????????????
    {
        if (speed_trend >= 0 || velSpd < 0.40f)
        {
            rundist = spddist <= lastspddist ? (rte_min(lastspddist + 0.01f, brakeDistR))
                                             : rundist;
        }
        else if (velSpd > 0.400f)
        {
            rundist = spddist <= lastspddist ? (rte_min(lastspddist - 0.01f, brakeDistR))
                                             : rundist;
        }
    }

    codedist     = rundist;
    lastcodedist = codedist;
    spddist      = rundist;
    lastspddist  = spddist;

    s_paras[0] = codedist;
    s_paras[1] = lastcodedist;
    s_paras[2] = spddist;
    s_paras[3] = lastspddist;
    return spddist;
}

void PK_DistanceCtrlRun_yuan()
{
    const int gear_delay_counter = 6;
    const int delay_counter      = 30;
    const float esp_angle        = 5.0;

    static uint32_t counter = 0;
    static uint8_t lastGear = GEAR_P;
    static uint32_t count   = 0;
    RTE_PK_Set_ModuleActive_DistCtrl(count++);

    static PK_ModuleComType lastModState = MCOM_OFF;

    static void *distlog         = NULL;
    static char print_buff[2048] = {0};

    float targetSpd   = RTE_PK_ObjAvoid_Get_ObjAvoid_Velspd();
    float targetAngle = RTE_PK_ObjAvoid_Get_ObjAvoid_EPS_Angle();
    float brakeDist   = RTE_PK_ObjAvoid_Get_BrkDist(); //???

    struct timeval stamp;
    float temp[3];
    PK_ModuleComType modState  = RTE_PK_StateManage_Get_ModuleCom_DistCtrl();
    PK_ModuleStateType msState = RTE_PK_DistCtrl_Get_ModuleState_DistCtrl();

    int sign_angle                 = 0;
    static int stop_step           = 0;
    static float target_stop_angle = 0;
    static int pathexecDelay       = 0;
    static int testDelay           = 0;

    if (msState != MSTAT_COMPLETE)
    {
        stop_step         = 0;
        target_stop_angle = 0;
    }

    if (testDelay == 1 && distlog != NULL && is_save_log)
    {
        gettimeofday(&stamp, NULL);
        int buffLen =
            snprintf(print_buff, sizeof(print_buff), "%ld:%ld msState:%d,%d,%d\n",
                     stamp.tv_sec, stamp.tv_usec, msState, lastModState, modState);
        Buff_Put(distlog, print_buff, buffLen);
    }

    if (msState == MSTAT_COMPLETE)
    {
        // printf("\n ----------------------\n msState == MSTAT_COMPLETE
        // \n--------------------\n");
        float steeringAngle = RTE_BSW_Get_EPS_Angle();
        sign_angle          = steeringAngle > 0 ? -1 : 1;
        if (stop_step == 0)
        {
            stop_step = 1;

            // target_stop_angle = fabs(steeringAngle) < esp_angle ? 0 : sign_angle * 50;
            // //Radar_Vert_TargPos_off_dx;
            target_stop_angle = sign_angle * Radar_Vert_TargPos_off_dx;
        }

        if (stop_step == 1 && fabsf(steeringAngle - target_stop_angle) < esp_angle)
        {
            stop_step         = 2;
            target_stop_angle = 0; // sign_angle * Parkout_off_dx;
            // printf("now steeringAngle: %f\n", steeringAngle);
        }

        float angleCfg = AngleControl(target_stop_angle, timedt, EPS_SPD_CCP * 0.8f,
                                      2 * ignoredist, 1.0, temp);

        RTE_BSW_Set_OutStopCmdVal(APA_ACTIVE);

        RTE_BSW_Set_TargetDistanceVal(0);

        RTE_BSW_Set_TargetSpeedLimitVal(0);

        RTE_BSW_Set_FailureBrakeModeVal(0x1);

        lastGear     = GEAR_P;
        float velSpd = RTE_BSW_Get_ESC_VehSpd();

        // 车辆停止
        if (counter > delay_counter && fabs(velSpd) < ZERO_FLOAT && stop_step == 2)
        {
            RTE_BSW_Set_TargetGearVal(lastGear);
            if (fabs(steeringAngle) < esp_angle && counter < 4 * delay_counter)
            {
                counter  = 4 * delay_counter;
                angleCfg = 0.0;

                stop_step         = 0;
                target_stop_angle = 0;
            }
        }

        RTE_BSW_Set_TargeAngleVal(7800 + angleCfg * 10);
        if (counter++ > 4 * delay_counter + delay_counter / 2)
        {
            RTE_BSW_Set_OutStopCmdVal(APA_COMPLETE);
            RTE_BSW_Set_TargetGearVal(lastGear);
            if (pathexecDelay == 0)
            {
                AngleControl(0, timedt, EPS_SPD_CCP * 0.8f, 2 * ignoredist, 1.0, temp,
                             1); //????0
            }
            pathexecDelay++;
            RTE_PK_PathExecute_Set_ModuleState_PathExecute(MSTAT_COMPLETE);
            if (counter < 5 * delay_counter && modState != MCOM_OFF)
            {
                gettimeofday(&stamp, NULL);
                int buffLen =
                    snprintf(print_buff, sizeof(print_buff),
                             "%06ld:%06ld dist run end! motor:%06f gear:%d angle: %f "
                             "exec mode:%d self mod:%d\n",
                             stamp.tv_sec, stamp.tv_usec, RTE_BSW_Get_Motorspd(),
                             RTE_BSW_Get_TargetGearVal(), steeringAngle,
                             RTE_PK_PathExecute_Get_ModuleState_PathExecute(), modState);
                Buff_Put(distlog, print_buff, buffLen);
            }
            else
            {
                counter = 0;
                if (modState == MCOM_OFF)
                {
                    RTE_PK_PathExecute_Set_ModuleState_PathExecute(MSTAT_OFF);
                }
                RTE_PK_StateManage_Set_ModuleCom_DistCtrl(MCOM_OFF);
                RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_OFF);
                lastModState = modState = MCOM_OFF;

                stop_step         = 0;
                target_stop_angle = 0;
                gettimeofday(&stamp, NULL);
                int buffLen =
                    snprintf(print_buff, sizeof(print_buff),
                             "%06ld:%06ld apa run end! motor:%06f gear:%d angle: %f "
                             "execmode:%d distmod:%d\n",
                             stamp.tv_sec, stamp.tv_usec, RTE_BSW_Get_Motorspd(),
                             RTE_BSW_Get_TargetGearVal(), steeringAngle,
                             RTE_PK_PathExecute_Get_ModuleState_PathExecute(), modState);

                Buff_Put(distlog, print_buff, buffLen);
                Buff_UnInit(distlog);
                distlog = NULL;
            }
        }
    }
    else if ((lastModState != modState) &&
             (modState == MCOM_ON_PARKING || modState == MCOM_ON_PARKINGOUT))
    {
        pathexecDelay = 0;
        RTE_BSW_Set_OutStopCmdVal(APA_ACTIVE);
        RTE_BSW_Set_TargetDistanceVal(0);
        RTE_BSW_Set_TargetSpeedLimitVal(0);
        RTE_BSW_Set_FailureBrakeModeVal(0x1);
        float angleCfg = RTE_BSW_Get_EPS_Angle();
        RTE_BSW_Set_TargeAngleVal(7800 + angleCfg * 10);

        lastGear        = GEAR_P;
        uint8_t curGear = RTE_BSW_Get_CurrentGear();
        if (curGear == GEAR_REAL_D)
        {
            lastGear = GEAR_D;
            RTE_BSW_Set_TargetGearVal(GEAR_D);
        }
        else if (curGear == GEAR_REAL_R)
        {
            lastGear = GEAR_R;
            RTE_BSW_Set_TargetGearVal(GEAR_R);
        }
        else
        {
            if (curGear == GEAR_REAL_N)
            {
                lastGear = GEAR_N;
            }
            if (curGear == GEAR_REAL_P)
            {
                lastGear = GEAR_P;
            }
            PlanInfoType targetPlanInfo;
            RTE_PK_DataConvt_Get_Target_PlanInfo(&targetPlanInfo);
            if (targetPlanInfo.Path_num > 0)
            {
                if (targetPlanInfo.Act_traj[0][3] > 0)
                {
                    RTE_BSW_Set_TargetGearVal(1);
                }
                if (targetPlanInfo.Act_traj[0][3] < 0)
                {
                    RTE_BSW_Set_TargetGearVal(2);
                }
            }
            else
            {
                RTE_BSW_Set_TargetGearVal(0);
            }
        }

        InitializeTrendQueue();

        WheelDistDt();
        if (distlog == NULL && is_save_log)
        {
            gettimeofday(&stamp, NULL);
            distlog = InitDistLog();
            struct timeval stampEnd;
            gettimeofday(&stampEnd, NULL);
            int buffLen = snprintf(
                print_buff, sizeof(print_buff), "enter apa %d time:%ld:%ld %ld:%ld\n",
                counter, stamp.tv_sec, stamp.tv_usec, stampEnd.tv_sec, stampEnd.tv_usec);
            Buff_Put(distlog, print_buff, buffLen);
            testDelay = 1;
        }

        if (RTE_BSW_Get_EpsCtrlStat() == 2)
        {
            counter      = 0;
            lastModState = modState;
            RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_NORM);
            AngleControl(angleCfg, timedt, EPS_SPD_CCP * 0.8f, 2 * ignoredist, 1.0, temp,
                         1);

            Avm_Pot_T avmSlot;
            RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmSlot);
            uint8_t data[8];
            data[0] = 1;
            Uint32ToBuf(avmSlot.slot_index, &data[1]);
            Can_PushData(0x6200, data);
        }
    }
    else if ((lastModState == modState) &&
             (modState == MCOM_ON_PARKING || modState == MCOM_ON_PARKINGOUT))
    {
        testDelay         = 0;
        pathexecDelay     = 0;
        float spdCfg      = targetSpd;
        float curAngle    = RTE_BSW_Get_EPS_Angle();         //??????
        float curSpd      = RTE_PK_Location_Get_Vel_Spd();   //???????
        float velSpd      = RTE_BSW_Get_ESC_VehSpd() / 3.6f; // ESC??
        float motorSpd    = RTE_BSW_Get_Motorspd();          //???????
        float veh_accel_x = RTE_BSW_Get_ESC_AX() - RTE_BSW_Get_ESC_AXOffSet();

        veh_accel_x = calculate_acceleration(curSpd, velSpd, veh_accel_x);

        float wheelDt = WheelDistDt(); //?????

        float brakeDistR = RTE_PK_PathExecute_Get_rx_brake(); //???

        uint8_t mebState       = 0;
        uint8_t stopState      = 0;
        uint8_t obstacle_exist = 0;

        execSt = RTE_PK_PathExecute_Get_BrkSt();

        uint8_t brakState = 0;
        float speed       = 0.0;

        float imuspd[3];

        int speed_trend = AnalyzeSpeedTrend(fabs(velSpd));   //??????
        int dist_trend  = AnalyzeDistTrend(fabs(brakeDist)); //????????

        float distCfg =
            DistConvert(fabs(brakeDistR), fabs(brakeDist), wheelDt, targetSpd,
                        fabs(velSpd), fabs(targetAngle), speed_trend, dist_trend,
                        &stopState, veh_accel_x, &obstacle_exist); //??

        uint8_t gearCfg = lastGear;
        if (fabs(spdCfg) > ZERO_FLOAT)
        {
            if (spdCfg < -1.0 * ZERO_FLOAT)
            {
                gearCfg = GEAR_R;
            }
            else if (spdCfg > ZERO_FLOAT)
            {
                gearCfg = GEAR_D;
            }
        }
        else if (fabs(brakeDist) > ZERO_FLOAT)
        {
            if (brakeDist < -1.0 * ZERO_FLOAT)
            {
                gearCfg = GEAR_D;
            }
            else if (brakeDist > ZERO_FLOAT)
            {
                gearCfg = GEAR_R;
            }
        }

        uint8_t curGear = RTE_BSW_Get_CurrentGear();
        if (curGear == GEAR_REAL_D)
        {
            curGear = GEAR_D;
        }
        else if (curGear == GEAR_REAL_R)
        {
            curGear = GEAR_R;
        }
        else
        {
            curGear = GEAR_P;
        }

        if (lastGear != gearCfg)
        {
            counter++;
            if (fabs(motorSpd) >= 3 || fabs(velSpd) > ZERO_FLOAT)
            {
                counter   = 0;
                gearCfg   = lastGear;
                brakState = 1;
            }

            if (counter <= gear_delay_counter)
            {
                gearCfg   = lastGear;
                brakState = 1;
            }
            else
            {
                lastGear = gearCfg;
                counter  = 0;
            }
        }
        else
        {
            if (gearCfg != curGear)
            {
                counter = gear_delay_counter;
            }
            else if (counter > 0)
            {
                counter--;
                brakState = 1;
            }
        }

        // float brakeDistR = RTE_PK_PathExecute_Get_rx_brake();//???
        float espSpd = (velSpd < 0.1) ? EPS_SPD_CCP * 0.85f : EPS_SPD_CCP;
        float distTemp =
            rte_max(min_run_dist, distCfg); // control angle during spd or dist is zero...

        uint8_t brkSt = RTE_PK_ObjAvoid_Get_BrkSt();
        if (brkSt != 0 || execSt == 1 || execSt == 2 || execSt == 3)
        {
            stopState = 1;
        }

        uint8_t BrakeToStopState = 0;
        BrakeToStopState = BrakeToStop(stopState, &distCfg, &mebState, brkSt, execSt,
                                       fabs(brakeDistR), curSpd, velSpd);

        if (distCfg < 2 * ignoredist)
        {
            // distCfg = 0.0;
            spdCfg = 0.0;
            if (velSpd < 0.06)
            {
                brakState = 1;
            }
            else
            {
                brakState = 0;
            }
        }
        else
        {
            if (distCfg < 2 * ignoredist && fabs(curSpd) > 0.30)
            {
                brakState = 1;
            }
        }

        brakState = rte_max(BrakeToStopState, brakState);

        // uint8_t temp_brkSt = 0;

        // uint8_t temp_execSt = brakeDistR > 0.03f && execSt < 2 && (fabs(curSpd) > 0.05f
        // || velSpd > 0.05f)  ? 0 : execSt; temp_brkSt = brkSt > 1 && velSpd < 0.3f ? 1 :
        // brkSt; //???????0???

        // if ((execSt == 3) && fabsf(curSpd) < 0.20f)
        // {
        //     spdCfg = 0.10;
        //     brakState = rte_max(1, brkSt);
        // }
        // else if (brkSt != 0 || ((execSt == 1 || execSt == 2) && fabsf(velSpd) < 0.12f))
        // {
        //     spdCfg = 0.10;   // used for flag
        //     temp_execSt = execSt > 2 ? 0 : execSt;
        //     brakState = rte_max(temp_execSt, temp_brkSt);
        // }

        float torqr       = RTE_BSW_Get_IPU_MotorRTorq();
        float torq        = RTE_BSW_Get_IPU_MotorTorq();
        float epsSpd      = RTE_BSW_Get_EPS_AngleSpd();
        uint8_t vehMoveSt = RTE_PK_Location_Get_Vel_MoveMentSt();
        if (curGear != gearCfg || brakeDistR == 0)
        {
            brakState = 2;

            if (gearCfg == GEAR_P && (curGear == GEAR_D || curGear == GEAR_R))
            {
                gearCfg = curGear;
            }
        }

        float angleCfg = targetAngle;
        angleCfg = AngleControl(targetAngle, timedt, espSpd, distTemp, velSpd, imuspd);

        if (RTE_BSW_Get_BrakeSt())
        {
            vehMoveSt = vehMoveSt + 0x10;
        }

        RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_NORM);
        RTE_BSW_Set_OutStopCmdVal(APA_ACTIVE);
        RTE_BSW_Set_TargeAngleVal(7800 + angleCfg * 10);

        static int stopFlag = 0;
        distCfg             = rte_min(distCfg, 5.0f);
        if (distCfg < 0.15 && distCfg > 0.10)
        {
            distCfg = Interpolation(0.10, 0.15, 0.10, 0.3, distCfg);
        }
        else if (distCfg < 0.3)
        {
            distCfg = Interpolation(0.15, 0.3, 0.3, 0.5, distCfg);
        }
        else if (distCfg < 0.8)
        {
            distCfg = Interpolation(0.3, 0.8, 0.5, 0.8, distCfg);
        }
        else if (distCfg < 1.8)
        {
            distCfg = distCfg;
        }
        else if (distCfg < 3.5)
        {
            distCfg = Interpolation(1.5, 3.5, 1.5, 4.0, distCfg);
        }
        else
        {
            distCfg = Interpolation(3.5, 5.0, 4.0, 5.0, distCfg);
        }

        // if(curAngle < 30 && distCfg < 0.80f * brakeDistR && speed_trend >= 0)
        // {
        //     distCfg += 0.20f;
        // }

        if (curGear == GEAR_P)
        {
            brakState = 1;
            distCfg   = 0;
        }

        if ((sign(targetSpd) * sign(curSpd) <= 0 || fabs(velSpd) == 0) &&
            fabs(distCfg) < 0.1f)
        {
            stopFlag = 1;
        }
        else if (fabs(distCfg) > 0.1f)
        {
            stopFlag = 0;
        }

        if (stopFlag)
        {
            distCfg = distCfg < 0.1f ? 0 : distCfg;
        }

        brakState = brakState > 2 ? 0 : brakState;

        if (brakState != 0)
        {
            brakState = rte_max(1, brkSt);
            mebState  = 0x4;

            speed = brakState > 0 && curGear != GEAR_P ? 0 : 0xff;
            // distCfg = stopFlag == 1 ? 0 : distCfg;

            RTE_BSW_Set_TargetSpeedLimitVal(speed);
            RTE_BSW_Set_TargetDistanceVal(0);
        }
        else
        {
            RTE_BSW_Set_TargetDistanceVal(distCfg * 100);
            if (execSt == 3)
            {
                speed = 0.0;
            }
            else
            {
                speed = SpeedConvert(fabs(distCfg), fabs(brakeDistR), curSpd, velSpd,
                                     targetSpd, targetAngle, curAngle, speed_trend,
                                     dist_trend, curGear, obstacle_exist, brkSt);
            }
            RTE_BSW_Set_TargetSpeedLimitVal(fabs(speed) * 36);
        }

        if (distlog != NULL)
        {
            gettimeofday(&stamp, NULL);
            int buffLen = snprintf(
                print_buff, sizeof(print_buff),
                "%06ld:%06ld parking continue angle=%06f,%06f,%06f "
                "distance:%06f,%06f,%06f speed:%06f,%06f,%06f,%06f,%d,%d,%06f,%06f "
                "mebState:%d imu:%06f,%06f,%06f movest:%02x ",
                stamp.tv_sec, stamp.tv_usec, targetAngle, angleCfg, curAngle, brakeDist,
                brakeDistR, distCfg, targetSpd, spdCfg, curSpd, speed, brakState,
                (int)motorSpd, velSpd, veh_accel_x, mebState, imuspd[0], imuspd[1],
                imuspd[2], vehMoveSt);
            buffLen += snprintf(&print_buff[buffLen], sizeof(print_buff),
                                "gear:%d,%d,%d brake:%d replanStop:%d Torq:%04f,%04f "
                                "wdt:%04f,%06f adjust:%06f,%06f,%06f,%06f,%06f\n",
                                gearCfg, lastGear, curGear, brkSt, execSt, torqr, torq,
                                wheelDt, epsSpd, s_paras[0], s_paras[1], s_paras[2],
                                s_paras[3], s_paras[4]);
            Buff_Put(distlog, print_buff, buffLen);

            // printf("%06ld:%06ld angle: %06f distance: %06f speed: %06f, breakState: %d
            // \n", stamp.tv_sec, stamp.tv_usec, angleCfg, distCfg, curSpd, brakState);
        }
        RTE_BSW_Set_FailureBrakeModeVal(brakState);
        RTE_BSW_Set_TargetGearVal(gearCfg);
        RTE_BSW_Set_APA_MEB_States_S(mebState);
        // printf("distance: %06f speed: %06f, breakState: %d \n",distCfg, speed,
        // brakState);
    }
    else if ((lastModState != modState) &&
             (lastModState == MCOM_ON_PARKING || lastModState == MCOM_ON_PARKINGOUT))
    {
        pathexecDelay = 0;
        counter       = 0;
        RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_COMPLETE);
        if (distlog != NULL)
        {
            gettimeofday(&stamp, NULL);
            int buffLen =
                snprintf(print_buff, sizeof(print_buff), "%06ld:%06ld apa run end!\n",
                         stamp.tv_sec, stamp.tv_usec);
            Buff_Put(distlog, print_buff, buffLen);
        }
        printf("------------ parking end -------------------- %d %d\n", lastModState,
               modState);
    }
    else
    {
        counter       = 0;
        pathexecDelay = 0;
        lastModState  = modState;
        RTE_BSW_Set_OutStopCmdVal(APA_STANDBY);
        RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_OFF);
    }
}

void PK_DistanceCtrl_yuan(void)
{
    RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_OFF);
    // auto DistCtrlFunc = [](void *args) -> void * {
    //     while (1)
    //     {
    //         usleep(timedt * 1000 * 1000);
    //         PK_DistanceCtrlRun_yuan();
    //     }
    // };

    // ASyncRun(DistCtrlFunc, NULL, "apa_ctrl");
}
