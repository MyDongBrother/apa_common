#include "DistCtrl.h"
#include "PK_Location.h"
#include "CanIf.h"
#include "Record_Buff.h"
#include "Rte_BSW.h"
#include "float.h"

// #define MAX_SPEED_TEST
typedef struct
{
    float buffer[25];
    int size;
    int index;
    float max_val;
    float min_val;
    uint8_t collisioncounter;
} AccelBuffer;

const float timedt       = 0.02;
static float s_paras[6]  = {0.0, 0.0, 0.0, 0.0};
const float min_run_dist = 0.26;
static uint8_t execSt    = 0;
static int speed_level   = 2;

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

    // 过滤极低速情况下电机转速方向波动的情况
    if (fabs(curSpd) < 0.03f)
    {
        curSpd = 0.0f;
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
        // curSpd为0的情况，返回0
        return 0.0f;
    }
}

static void AccelBuffer_Init(AccelBuffer *accel_buf)
{
    memset(accel_buf->buffer, 0, sizeof(accel_buf->buffer));
    accel_buf->size             = 0;
    accel_buf->index            = 0;
    accel_buf->max_val          = -FLT_MAX;
    accel_buf->min_val          = FLT_MAX;
    accel_buf->collisioncounter = 0;
}

static void AccelBuffer_Update(AccelBuffer *accel_buf, float new_accel)
{
    // 更新环形缓冲区
    accel_buf->buffer[accel_buf->index] = new_accel;
    accel_buf->index                    = (accel_buf->index + 1) % 25;

    // 维护有效数据量
    if (accel_buf->size < 25)
    {
        accel_buf->size++;
    }

    // 重新计算极值（考虑数据覆盖情况）
    accel_buf->max_val = -FLT_MAX;
    accel_buf->min_val = FLT_MAX;
    for (int i = 0; i < accel_buf->size; i++)
    {
        if (accel_buf->buffer[i] > accel_buf->max_val)
        {
            accel_buf->max_val = accel_buf->buffer[i];
        }
        if (accel_buf->buffer[i] < accel_buf->min_val)
        {
            accel_buf->min_val = accel_buf->buffer[i];
        }
    }
}

static uint8_t CheckCollisionImpact(AccelBuffer *accel_buf)
{
    // 数据不足时不检测
    if (accel_buf->size < 25)
        return 0;

    uint8_t currentstate = 0;

    const float diff    = fabsf(accel_buf->max_val - accel_buf->min_val);
    const float abs_max = fabsf(accel_buf->max_val);
    const float abs_min = fabsf(accel_buf->min_val);

    // 三个碰撞条件同时满足
    currentstate = (diff > 0.7f) && (accel_buf->max_val * accel_buf->min_val < 0) &&
                   (abs_min > 0.25f) && (abs_max > 0.25f);

    if (currentstate)
    {
        accel_buf->collisioncounter = 10;
    }
    else if (accel_buf->collisioncounter > 0)
    {
        accel_buf->collisioncounter--;
    }

    return accel_buf->collisioncounter > 0 ? 1 : 0;
}

static uint8_t BrakeToStop(const uint8_t stopstate, float *brakeDist, uint8_t *mebState,
                           const uint8_t brkSt, const uint8_t execSt,
                           const uint8_t CollisionState, const float brakeDistR,
                           const float curSpd, const float velSpd, const float accel_x)
{
    static uint8_t first_into   = 0;
    static float first_dist     = 0.0;
    static float code_brakeDist = 0.0;
    static float brakeSpd       = 0.0f;
    uint8_t brakState           = 0;

    float calSpd = 0.0f;
    if (fabs(velSpd) > ZERO_FLOAT)
    {
        calSpd = velSpd + 0.4 * accel_x;
    }
    else
    {
        calSpd = 0.0f;
    }

    if (stopstate == 0)
    {
        first_into     = 0;
        first_dist     = 0.0;
        code_brakeDist = *brakeDist;
        return brakState;
    }

    uint8_t temp_execSt = 0;
    uint8_t temp_brkSt  = 0;
    uint8_t stopSt      = 0;
    float distDiff      = 0;

    temp_execSt =
        brakeDistR > 0.03f && execSt < 2 && (fabs(curSpd) > 0.05f || velSpd < 0.05f)
            ? 0
            : execSt;
    temp_brkSt = brkSt > 1 && velSpd < 0.3f ? 1 : brkSt; //???????0???
    if (brkSt == 1 && calSpd < 0.25f && velSpd < 0.40f)
    {
        temp_brkSt = 0;
    }
    stopSt = fabs(brakeDistR) < 0.15f && velSpd < 0.05f && execSt != 4 ? 1 : 0;

    if ((execSt == 3) && fabsf(curSpd) < 0.20f)
    {
        brakState = rte_max(1, temp_brkSt);
    }
    else if (brkSt != 0 || ((execSt == 1 || execSt == 2) && fabsf(curSpd) < 0.08f))
    {
        temp_execSt = execSt > 2 ? 0 : execSt;
        brakState   = rte_max(temp_execSt, temp_brkSt);
    }
    brakState = rte_max(brakState, stopSt);
    brakState = rte_max(brakState, CollisionState);

    *mebState = 3;
    if (brkSt == 0)
    {
        code_brakeDist = *brakeDist;
        if (*brakeDist < 0.60f && *brakeDist > 0.08f && brakeDistR < 0.25f)
        {
            *brakeDist = 0.08f;
        }
        else if (*brakeDist < 0.08f)
        {
            *brakeDist = 0.00f;
        }
        else if (*brakeDist > 0.35f)
        {
            *brakeDist = rte_max(0.5 * (*brakeDist), 0.08);
        }
        else
        {
            *brakeDist = rte_max(0.5 * (*brakeDist), 0.08);
        }
    }
    else if (brkSt == 1)
    {
        if (brakState == 1)
        {
            if (first_into == 0)
            {
                first_into = 1;
                first_dist = brakeDistR;
                brakeSpd   = velSpd;
            }
            distDiff = fabs(first_dist - brakeDistR);

            if (velSpd > 0.10f)
            {
                brakState = 0;
                if (calSpd > 0.30f)
                {
                    if (execSt == 4 && brakeSpd > 0.50f)
                    {
                        *brakeDist = 0.5 * code_brakeDist;
                    }
                    else
                    {
                        if (execSt == 4 && accel_x < -0.30f)
                        {
                            *brakeDist = 0.5 * code_brakeDist;
                        }
                        if (accel_x < -0.40f)
                        {
                            *brakeDist = 0.5 * code_brakeDist;
                        }
                        else
                        {
                            *brakeDist = 0.3 * code_brakeDist;
                        }
                    }
                }
                else
                {
                    *brakeDist = 0.5 * code_brakeDist;
                }
                *brakeDist = rte_max(*brakeDist, 0.08f);
            }
            else
            {
                brakState = 1;
            }
        }
        else if (brakState == 0)
        {
            if (code_brakeDist > 0.60f)
            {
                code_brakeDist = 0.60f;
            }
            *brakeDist = rte_max(code_brakeDist, 0.35f);
            if (velSpd < 0.10f)
            {
                brakState = 1;
            }
        }

        code_brakeDist = code_brakeDist - timedt * velSpd;
        // code_brakeDist =rte_max(code_brakeDist, 0.45f);
    }
    else if (brkSt == 2)
    {
        if (brakState == 1)
        {
            if (first_into == 0)
            {
                first_into = 1;
                first_dist = brakeDistR;
            }
            distDiff = fabs(first_dist - brakeDistR);

            if (distDiff < 0.05f && fabs(velSpd) > 0.10f)
            {
                brakState  = 0;
                *brakeDist = 0.08f;
            }
        }
    }

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

    float speed_config[3][4] = {
        {0.6, 0.6, 0.5, 0.4}, {0.8, 0.8, 0.7, 0.6}, {1.1, 0.9, 0.8, 0.65}};

    if (brakeDistR < 0.31f || brkSt == 2)
    {
        safeSpd = 0.0f;
    }
    else if (last_brkSt == 1 && brkSt == 0)
    {
        safeSpd = velSpd;
    }

    if (brakeDistR < 0.10f || brakeDist < 0.10f)
    {
        speedLimit = 0.20f;
    }
    else if (brakeDistR < 0.35f || brakeDist < 0.35f)
    {
        speedLimit = 0.30f;
        if (fabs(curSpd) < 0.05f)
        {
            speedLimit = 0.55f;
        }
        else if (fabs(curSpd) > 0.55f)
        {
            speedLimit = 0.20f;
        }
    }
    else if (brakeDistR < 0.50f || brakeDist < 0.50f)
    {
        speedLimit = 0.40f;
    }
    else if (brakeDistR < 1.50f || brakeDist < 1.50f)
    {
        speedLimit =
            rte_max(rte_min(velSpd - 0.10f, speed_config[speed_level - 1][3]), 0.50f);
        if (fabs(curSpd) < 0.40f)
        {
            speedLimit = speed_config[speed_level - 1][3];
        }
    }
    else if (brakeDist < 2.50f || brakeDistR < 2.50f)
    {
        speedLimit = rte_max(rte_min(velSpd - 0.10f, speed_config[speed_level - 1][2]),
                             speed_config[speed_level - 1][3]);
        if (fabs(curSpd) < 0.50f)
        {
            speedLimit = speed_config[speed_level - 1][2];
        }
    }
    else if (fabs(targAngle) > 100 || fabs(targAngle - curAngle) > 50.0f ||
             brakeDistR < 3.00f)
    {
        speedLimit = speed_config[speed_level - 1][2];
    }
    else if (fabs(targSpd) > 0.001)
    {
        if (brakeDist > 2.5 && brakeDistR > 2.5 && curSpd < 0.8)
        {
            speedLimit =
                speed_config[speed_level - 1][0]; // IPB 最大车速支持到 4.5kph, 因为超过
                                                  // 5kph，EPB 就会自行刹车
        }
        else
        {
            speedLimit = speed_config[speed_level - 1][1];
        }
        // speedLimit =  0.80f;
    }
    else
    {
        speedLimit = 0.20f;
    }

    if (safeSpd > 0.0f && obstacle_exist > 0 && execSt == 4)
    {
        if (velSpd > safeSpd)
        {
            safeSpd = velSpd;
        }
        float safeLimit = floorf(safeSpd / 0.05f) * 0.05f;
        safeLimit       = rte_max(safeLimit, 0.20f);
        speedLimit      = rte_min(speedLimit, safeLimit);
    }

    // if(fabs(targSpd) < 0.45f && brakeDistR > 1.0f && brakeDistR  < 3.0f)
    // {
    //     speedLimit = rte_min(fabs(speedLimit), fabs(targSpd));
    // }

    if (((last_speedlimit - 0.10f) > speedLimit) && (speedLimit > 0.05) &&
        (last_speedlimit != 0))
    {
        speedLimit = last_speedlimit - 0.10f;
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

static bool BrakeSt(const float minDist, const float calSpd, const float accel_x)
{
    if (calSpd == 0.0f)
    {
        return false;
    }

    if ((execSt != 4) &&
        ((minDist < 0.13f) || (minDist < 0.18f && fabs(calSpd) > 0.14f) ||
         (minDist < 0.20f && accel_x > 0.08f) ||
         (minDist < 0.25f && fabs(calSpd) > 0.22f) ||
         (minDist < 0.38f && fabs(calSpd) > 0.28f) ||
         (minDist < 0.40 && fabs(calSpd) > 0.34f) ||
         (minDist < 0.45f && fabs(calSpd) > 0.40f)))
    {
        if (minDist > 0.40f && accel_x < -0.13f)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else if ((execSt == 4) &&
             ((minDist < 0.10f) || (minDist < 0.18f && fabs(calSpd) > 0.14f) ||
              (minDist < 0.20f && accel_x > 0.08f) ||
              (minDist < 0.22f && fabs(calSpd) > 0.22f) ||
              (minDist < 0.30f && fabs(calSpd) > 0.28f) ||
              (minDist < 0.35 && fabs(calSpd) > 0.32f) ||
              (minDist < 0.40 && fabs(calSpd) > 0.38f) ||
              (minDist < 0.45f && fabs(calSpd) > 0.45f)))
    {
        if (minDist > 0.36f && accel_x < -0.18f)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}
static float DistConvert(const float brakeDistR, const float brakeDistB,
                         const float distdt, const float targetSpd, const float velSpd,
                         const float targetAngle, const int speed_trend,
                         const int dist_trend, uint8_t *stopstate, const float accel_x,
                         uint8_t *obstacle_exist)
{
    static float codedist       = 0.0;
    static float spddist        = 0.0;
    static float lastcodedist   = 0.0;
    static float lastspddist    = 0.0;
    const float min_update_dist = 0.05;

    float tempDist = 0, brakeDist = 0;
    float minDist = rte_min(brakeDistR, brakeDistB);
    float calSpd  = 0.0f;
    if (fabs(velSpd) > ZERO_FLOAT)
    {
        calSpd = velSpd + 0.5 * accel_x;
    }
    else
    {
        calSpd = 0.0f;
    }

    // static int velflag = 0;

    // 1?????0.50m????????,2?????1.5?,3???????
    if ((brakeDistR - brakeDistB) > rte_min(0.2 * brakeDistR, 0.30f))
    {
        if (brakeDistB > 1.10f && calSpd < 0.60f)
        {
            brakeDist = 0.6 * brakeDistB + 0.4 * rte_min(brakeDistR, 2.50f);
        }
        else if (brakeDistB > 1.10f && calSpd > 0.60f)
        {
            brakeDist = 0.8 * brakeDistB + 0.2 * rte_min(brakeDistR, 3.0f);
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
    else if (minDist > 0.8f) //??
    {
        tempDist =
            minDist > 1.1f ? minDist * 1.3f : (minDist + minDist * (minDist - 0.8f));
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

    float rundist = brakeDist;

    if (rundist < min_update_dist || codedist < min_update_dist ||
        spddist < min_update_dist || velSpd < ZERO_FLOAT)
    {
        codedist     = rundist;
        lastcodedist = codedist;
        spddist      = rundist;
        lastspddist  = spddist;
    }
    else if (fabs(codedist - rundist) >
                 rte_max(rte_min(0.10 * rundist, 0.10), min_update_dist) ||
             fabs(spddist - rundist) >
                 rte_max(rte_min(0.10 * rundist, 0.10), min_update_dist))
    {
        if (*obstacle_exist > 0 && (codedist - rundist > 1.5) && rundist < 5.0f)
        {
            codedist = lastcodedist - 1.5f;
        }
        else if ((codedist - rundist > 0.5) && rundist < 5.0f)
        {
            codedist = lastcodedist - 0.5f;
        }
        else if ((codedist - rundist > 0.10) && rundist < 5.0f && rundist > 0.80f)
        {
            codedist = lastcodedist - 0.05;
        }
        else if ((codedist - rundist > 0.25) && rundist < 0.80f)
        {
            codedist = lastcodedist - 0.04;
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
        if (*obstacle_exist > 0 && (spddist - rundist > 1.5) && rundist < 5.0f)
        {
            spddist = lastspddist - 1.5f;
        }
        else if ((spddist - rundist > 0.5) && rundist < 5.0f)
        {
            spddist = lastspddist - 0.5f;
        }
        else if ((spddist - rundist > 0.10f) && rundist < 5.0f && rundist > 0.8f)
        {
            spddist = lastspddist - 0.05f;
        }
        else if ((spddist - rundist > 0.25f) && rundist < 0.8f)
        {
            spddist = lastspddist - 0.04f;
        }
        else if ((spddist - rundist > 0.05f) && rundist < 0.8f)
        {
            spddist = spddist - 2 * timedt * velSpd;
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

    if (minDist < 0.06f)
    {
        rundist = 0.05f;
    }
    else if (BrakeSt(minDist, calSpd, accel_x))
    {
        *stopstate = 1;
    }

    if (rundist < 0.80f && (rundist - minDist < 0.10f) && *obstacle_exist < 2) //??????
    {
        if (rundist > 0.40f && fabs(calSpd) < 0.35f)
        {
            if (fabs(calSpd) < 0.25f)
            {
                if (execSt == 4 && *obstacle_exist == 0)
                {
                    rundist += 0.40f;
                }
                else
                {
                    rundist += 0.30f;
                }
            }
            else
            {
                if (execSt == 4 && *obstacle_exist == 0)
                {
                    rundist += 0.20f;
                }
                else
                {
                    rundist += 0.15f;
                }
            }
        }
        else if (rundist < 0.40f && rundist > 0.10f && fabs(calSpd) < 0.18f &&
                 accel_x > -0.20f)
        {
            if (execSt == 4 && *obstacle_exist == 0)
            {
                if (fabs(calSpd) < 0.10)
                {
                    rundist += 0.35f;
                }
                else
                {
                    rundist += 0.20f;
                }
            }
            else
            {
                if (fabs(calSpd) < 0.10)
                {
                    rundist += 0.25f;
                }
                else
                {
                    rundist += 0.15f;
                }
            }
        }
        else if (rundist < 0.20f && velSpd > 0.20f)
        {
            rundist = rte_min(rte_max(brakeDist * 0.8, 2 * ignoredist), rundist);
        }
        else if (rundist < 0.30f && rundist >= lastspddist)
        {
            rundist = rte_max(lastspddist - timedt * velSpd, ignoredist + 0.01);
        }
        else if (velSpd > 0.20f && rundist > ignoredist &&
                 rundist >= rte_max(lastspddist, ignoredist + 0.01))
        {
            rundist = rte_max(lastspddist - timedt * velSpd, ignoredist + 0.01);
        }
    }
    else if (*obstacle_exist == 1 && rundist > ignoredist &&
             rundist >= rte_max(lastspddist, ignoredist + 0.01))
    {
        rundist = rte_max(rundist - timedt * velSpd, ignoredist + 0.01);
    }

    if (minDist > 0.06f)
    {
        if (accel_x < -0.35f && *stopstate == 0)
        {
            rundist = 1.2 * rundist;
        }
        rundist = rte_max(0.08, rundist);
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

void PK_DistanceCtrlRun_han()
{
    const int gear_delay_counter = 6;
    const int delay_counter      = 30;
    const float esp_angle        = 5.0;

    static uint32_t counter = 0;
    static uint8_t lastGear = GEAR_P;

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

    static AccelBuffer accel_buffer;

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

            AccelBuffer_Init(&accel_buffer);

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
        AccelBuffer_Update(&accel_buffer, veh_accel_x);
        veh_accel_x = calculate_acceleration(curSpd, velSpd, veh_accel_x);

        float wheelDt = WheelDistDt(); //?????

        float brakeDistR = RTE_PK_PathExecute_Get_rx_brake(); //???

        uint8_t mebState       = 0;
        uint8_t stopState      = 0;
        uint8_t brakState      = 0;
        execSt                 = RTE_PK_PathExecute_Get_BrkSt();
        uint8_t obstacle_exist = 0;

        float speed = 0.0;

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

            if (counter < gear_delay_counter)
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
            else if (counter > 8)
            {
                counter--;
                brakState = 1;
            }
        }

        // float brakeDistR = RTE_PK_PathExecute_Get_rx_brake();//???
        float espSpd = (velSpd < 0.1) ? EPS_SPD_CCP * 0.85f : EPS_SPD_CCP;
        float distTemp =
            rte_max(min_run_dist, distCfg); // control angle during spd or dist is zero...

        uint8_t brkSt          = RTE_PK_ObjAvoid_Get_BrkSt();
        uint8_t CollisionState = 0;
        if ((curGear == GEAR_R) && (brakeDistR < 0.7))
        {
            CollisionState = CheckCollisionImpact(&accel_buffer);
            CollisionState = velSpd < 0.15 ? CollisionState : 0;
        }
        if (brkSt != 0 || execSt == 1 || execSt == 2 || execSt == 3 ||
            CollisionState != 0)
        {
            stopState = 1;
        }

        uint8_t BrakeToStopState = 0;
        BrakeToStopState =
            BrakeToStop(stopState, &distCfg, &mebState, brkSt, execSt, CollisionState,
                        fabs(brakeDistR), curSpd, velSpd, veh_accel_x);

        if (distCfg < 2 * ignoredist)
        {
            brakState = 1;
            distCfg   = 0.0;
            spdCfg    = 0.0;
        }
        else
        {
            if (distCfg < 2 * ignoredist && fabs(curSpd) > 0.30)
            {
                brakState = 1;
            }
        }

        brakState = rte_max(BrakeToStopState, brakState);

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

        if (brakState != 0)
        {
            RTE_BSW_Set_TargetSpeedLimitVal(0);
            RTE_BSW_Set_TargetDistanceVal(0);
            mebState = 0x4; // Actice
            if (fabs(velSpd) < 0.06f && brakState == 1)
            {
                brakState = 2;
            }
        }
        else
        {
            RTE_BSW_Set_TargetDistanceVal(distCfg * 100);
            speed = SpeedConvert(fabs(distCfg), fabs(brakeDistR), curSpd, velSpd,
                                 targetSpd, targetAngle, curAngle, speed_trend,
                                 dist_trend, curGear, obstacle_exist, brkSt);
            // speed = (brakeDistR- 0.01) > brakeDist && fabs(targetSpd) < 0.45f ?
            // rte_max(fabs(targetSpd),0.30) : speed;
            if (execSt == 3)
            {
                speed = 0.0;
            }
            RTE_BSW_Set_TargetSpeedLimitVal(fabs(speed) * 36);
        }

        if (distlog != NULL)
        {
            gettimeofday(&stamp, NULL);
            int buffLen = snprintf(
                print_buff, sizeof(print_buff),
                "%06ld:%06ld parking continue angle=%06f,%06f,%06f "
                "distance:%06f,%06f,%06f speed:%06f,%06f,%06f,%06f,%d,%d,%06f,%06f  "
                "mebState:%d imu:%06f,%06f,%06f movest:%02x ",
                stamp.tv_sec, stamp.tv_usec, targetAngle, angleCfg, curAngle, brakeDist,
                brakeDistR, distCfg, targetSpd, spdCfg, curSpd, speed, brakState,
                (int)motorSpd, velSpd, veh_accel_x, mebState, imuspd[0], imuspd[1],
                imuspd[2], vehMoveSt);
            buffLen +=
                snprintf(&print_buff[buffLen], sizeof(print_buff),
                         "gear:%d,%d,%d brake:%d replanStop:%d collision:%d "
                         "Torq:%04f,%04f wdt:%04f,%06f adjust:%06f,%06f,%06f,%06f,%06f\n",
                         gearCfg, lastGear, curGear, brkSt, execSt, CollisionState, torqr,
                         torq, wheelDt, epsSpd, s_paras[0], s_paras[1], s_paras[2],
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

void PK_DistanceCtrl_han(void)
{
    RTE_PK_DistCtrl_Set_ModuleState_DistCtrl(MSTAT_OFF);
    auto DistCtrlFunc = [](void *args) -> void * {
        uint32_t count  = 0;
        char config[64] = {0};
        if (PK_Calibration_GetDebugValue("apa_speed_level", config) > 0)
        {
            speed_level = atoi(config);
            if (speed_level > 3 || speed_level < 1)
            {
                speed_level = 2;
                printf("apa_speed_level error, set to default value 2\n");
            }
        }
        while (1)
        {
            usleep(timedt * 1000 * 1000);
            PK_DistanceCtrlRun_han();
            RTE_PK_Set_ModuleActive_DistCtrl(count++);
        }
    };

    ASyncRun(DistCtrlFunc, NULL, "apa_ctrl");
}
