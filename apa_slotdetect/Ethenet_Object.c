#include "Ethenet_Object.h"
#include "PK_Location.h"
#include "CanIf.h"
#include "Ethenet_AvmSlot.h"

using namespace std;

// #define Rear_Center_Length  (float)((VEHICLE_LEN/2) - REAR_SUSPENSION)

#define MAXFRAMESIZE 40
enum AvmObstacleType
{
    AvmObstacle_Type_Unknow,          // 未知0
    AvmObstacle_Type_StopFlag_Two,    // 分体式限位杆 1
    AvmObstacle_Type_Traffic_Cone,    // 锥桶 2
    AvmObstacle_Type_Ground_Lock_OFF, // 地锁(关闭状态) 3
    AvmObstacle_Type_Ground_Lock_ON,  // 地锁(打开状态) 4
    AvmObstacle_Type_StopFlag_One,    // 一体式限位杆（橡胶版）5
    AvmObstacle_Type_StopFlag_One2,   // 一体式限位杆（铁管版） 6
    AvmObstacle_Type_HANDICAP,        // 残疾人车位标志7
    AvmObstacle_Type_PVC,             // PVC管8
    AvmObstacle_Type_CARTON,          // 纸箱9
    AvmObstacle_Type_CHAIR,           // 凳子10
    AvmObstacle_Type_PEDSTRAIN,       // 行人
    AvmObstacle_Type_BUCKET,          // 水桶
    AvmObstacle_Type_E_BIKE,          // 电瓶车
    AvmObstacle_Type_STOP_SIGN,       // 禁停牌
    AvmObstacle_Type_CHARGING_STOP,   // 充电桩
    AvmObstacle_Type_DECEL_STRIP,     // 减速带
    AvmObstacle_Type_BUMPER,          // 防撞桶
    AvmObstacle_Type_FENCE,           // 护栏
    AvmObstacle_Type_WHEEL_CHAIR,     // 轮椅
    AvmObstacle_Type_FIRE_KIT,        // 消防箱
    AvmObstacle_Type_Max              // 最大障碍物数目
};

struct VisionObstacleBuff
{
    uint8_t report; // 上报状态0 - 未上报 1 - 可上报 2 - 已上报
    std::vector<LineSeg_T> beCache; // 障碍物接地2顶点缓存
    std::vector<Point_T> midCache;  // 障碍物接地2顶点线段中点缓存
    std::vector<float> widthCache;  // 障碍物接地2顶点线段长度缓存
    std::vector<float> thetaCache;  // 障碍物接地2顶点线段角度缓存
    Point_T be[4];                  // 障碍物接地2顶点坐标
    Point_T midPoint;               // 障碍物接地2顶点线段中点
    uint64_t timeStamp;             // 障碍物时间戳
    PK_ObstacleType obstacleType;   // 障碍物类型
    int8_t maxFrame;                // 最大缓存数量
    int16_t index;                  // 障碍物id
    float beLen;                    // 障碍物线段长度
    float obstacleTheta;            // 障碍物线段方向角
    float odm;                      // 障碍物与车辆距离

    VisionObstacleBuff()
    {
        index         = -1;
        report        = 0;
        odm           = 0.0;
        maxFrame      = min_frame_num;
        beLen         = 0.0;
        obstacleTheta = 0.0;
    }

    VisionObstacleBuff(const uint8_t value)
    {
        VisionObstacleBuff();
        maxFrame = value;
    }
    uint8_t ReportCount() const { return report; }
    uint8_t FrameCount() const { return beCache.size(); }
    uint8_t MaxFrame() const { return MAXFRAMESIZE; }
    int8_t ReportFrame() const { return maxFrame; }
    void ReportFrame(const int8_t value) { maxFrame = value; }
    void ReportCount(const uint8_t value) { report = value; }
    bool IsReported() const { return (report > 0); }
    float IsRemoveObstacle(const float curDist) const { return fabs(odm - curDist); }
    void ClearData()
    {
        report = 0;
        beCache.clear();
        midCache.clear();
        widthCache.clear();
        thetaCache.clear();
    }

    void loadData(std::vector<Point_T> &points) // 中点
    {
        for (size_t i = 0; i < beCache.size(); i++)
        {
            points.push_back(midCache.at(i));
        }
    }

    void loadLen(std::vector<Point_T> &points) // 宽度
    {
        for (size_t i = 0; i < beCache.size(); i++)
        {
            points.push_back({fabs(widthCache.at(i)), 0.0});
        }
    }

    void loadTheta(std::vector<Point_T> &points) // 方向角
    {
        for (size_t i = 0; i < beCache.size(); i++)
        {
            points.push_back({fabs(thetaCache.at(i)), 0.0});
        }
    }

    void GetObstacleLine()
    {
        float halfLength = beLen / 2.0f;
        // 计算水平分量和垂直分量
        float dx = halfLength * cos(obstacleTheta);
        float dy = halfLength * sin(obstacleTheta);
        // 计算起点和终点坐标
        be[0].x = midPoint.x - dx;
        be[0].y = midPoint.y - dy;
        be[1].x = midPoint.x + dx;
        be[1].y = midPoint.y + dy;
    }
};

static Point_T FindMidpointPt(Point_T A, Point_T B)
{
    Point_T midPoint;
    midPoint.x = (A.x + B.x) / 2;
    midPoint.y = (A.y + B.y) / 2;
    return midPoint;
}

static void FindMidpoint(float *pt1, float *pt2, float *midPoint)
{
    midPoint[0] = (pt1[0] + pt2[0]) / 2;
    midPoint[1] = (pt1[1] + pt2[1]) / 2;
}
static void FindCenterPoint(const float *midPoint, float lengthAB, float angleAB, float h,
                            float *centerPoint)
{
    // 计算线段AB的方向单位向量
    float vx = cos(angleAB);
    float vy = sin(angleAB);

    // 计算中垂线向量
    float vxC = -vy; // 沿着y轴方向取负值
    float vyC = vx;  // 沿着x轴方向取正值

    // 将向量缩放到距离h处
    centerPoint[0] = midPoint[0] + h * vxC;
    centerPoint[1] = midPoint[1] + h * vyC;
}

// // 计算新线段的端点
// void shortenSegment1(Point_T *p1, Point_T *p2, float angle, float length) {
//     // 计算中点
//     Point midPoint = {(p1->x + p2->x) / 2, (p1->y + p2->y) / 2};

//     // 计算新线段的端点坐标
//     p1->x = midPoint.x - length * cosf(angle);
//     p1->y = midPoint.y - length * sinf(angle);
//     p2->x = midPoint.x + length * cosf(angle);
//     p2->y = midPoint.y + length * sinf(angle);
// }

// 计算新线段的端点
static void shortenSegment(float *p1, float *p2, float length)
{

    float angle = atan2f(p2[1] - p1[1], p2[0] - p1[0]);
    // 计算中点
    float midPoint[2] = {(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2};

    // 计算新线段的端点坐标
    p1[0] = midPoint[0] - length * cosf(angle);
    p1[1] = midPoint[1] - length * sinf(angle);
    p2[0] = midPoint[0] + length * cosf(angle);
    p2[1] = midPoint[1] + length * sinf(angle);
}

/**
 * @brief 选择障碍物点
 *
 * 该函数从四个给定的障碍物点中选择两个Y值绝对值最小的点，并将它们分别放置在
 * `obstaclePoints[0]` 和 `obstaclePoints[1]` 位置。
 *
 * @param obstaclePoints 包含四个障碍物点的数组，每个点由两个浮点数表示。
 */
static void selectObstaclePoint(float obstaclePoints[4][2])
{
    float min1           = fabs(obstaclePoints[0][1]); // 初始化第一个最小Y值
    float min2           = fabs(obstaclePoints[1][1]); // 初始化第二个最小Y值
    int min_index1       = 0;                          // 第一个最小Y值的索引
    int min_index2       = 1;                          // 第二个最小Y值的索引
    float abs_numbers[4] = {0.0f};                     // 存储Y值的绝对值

    // 计算每个Y值的绝对值
    for (int i = 0; i < 4; ++i)
    {
        abs_numbers[i] = fabs(obstaclePoints[i][1]);
    }

    // 找到绝对值最小的两个Y值及其对应的索引
    for (int i = 2; i < 4; ++i)
    {
        if (abs_numbers[i] < min1)
        {
            min2       = min1;
            min_index2 = min_index1;
            min1       = abs_numbers[i];
            min_index1 = i;
        }
        else if (abs_numbers[i] < min2)
        {
            min2       = abs_numbers[i];
            min_index2 = i;
        }
    }

    // 将找到的两个点放置到 obstaclePoints 的前两个位置
    obstaclePoints[0][0] = obstaclePoints[min_index1][0];
    obstaclePoints[0][1] = obstaclePoints[min_index1][1];
    obstaclePoints[1][0] = obstaclePoints[min_index2][0];
    obstaclePoints[1][1] = obstaclePoints[min_index2][1];
}

ObsThreeDimen ObsTypeDime[AvmObstacle_Type_Max] = {0};
static void ObsTypeDimeData()
{
    ObsTypeDime[Obstacle_Type_Traffic_Cone].len    = 0.29;
    ObsTypeDime[Obstacle_Type_Traffic_Cone].wight  = 0.29;
    ObsTypeDime[Obstacle_Type_Traffic_Cone].height = 0;
}

/*
 *name:KeepTwoNovel
 *funtion:Keep two novels
 **/
static inline double KeepTwoNovel(double data)
{
    return data = (float)(((int)(data * 10000)) / 100) / 100;
}

static bool AddBeData(const VisionObstacleInfo &obstacleinfo,
                      VisionObstacleBuff *obstacle)
{
    uint8_t frameNum = obstacle[0].FrameCount();
    if (frameNum > obstacle[0].MaxFrame())
    {
        return true;
    }

    LineSeg_T line;
    line.pt1.x = obstacleinfo.pointOrd[0][0];
    line.pt1.y = obstacleinfo.pointOrd[0][1];
    line.pt2.x = obstacleinfo.pointOrd[1][0];
    line.pt2.y = obstacleinfo.pointOrd[1][1];

    // printf("273: %f %f %f %f \n",
    // obstacleinfo.pointOrd[0][0],obstacleinfo.pointOrd[0][1],obstacleinfo.pointOrd[1][0],obstacleinfo.pointOrd[1][1]);
    obstacle[0].beCache.push_back(line);

    Point_T point;
    point.x = obstacleinfo.midPoint[0];
    point.y = obstacleinfo.midPoint[1];

    obstacle[0].midCache.push_back(point);

    obstacle[0].widthCache.push_back(obstacleinfo.obstacleWidth);

    obstacle[0].thetaCache.push_back(obstacleinfo.obstacleTheta);

    // printf("report %d type %d count %d odm %f idx %d %d obstacleangle %d\n",
    //     obstacle[0].ReportCount(), obstacle[0].shape, obstacle[0].FrameCount(),
    //     obstacle[0].odm, obstacle[0].index, obstacleinfo.obstacleIndex,
    //     obstacleinfo.obstacleAngle);
    if (frameNum == 0)
    {
        obstacle[0].be[0].x = obstacle[0].beCache[0].pt1.x;
        obstacle[0].be[0].y = obstacle[0].beCache[0].pt1.y;
        obstacle[0].be[1].x = obstacle[0].beCache[0].pt2.x;
        obstacle[0].be[1].y = obstacle[0].beCache[0].pt2.y;

        obstacle[0].midPoint.x = obstacle[0].midCache[0].x;
        obstacle[0].midPoint.y = obstacle[0].midCache[0].y;

        obstacle[0].beLen = obstacle[0].widthCache[0];

        obstacle[0].obstacleTheta = obstacle[0].thetaCache[0];
    }
    else
    {
        obstacle[0].be[0].x =
            obstacle[0].be[0].x * 0.9 + obstacleinfo.pointOrd[0][0] * 0.1;
        obstacle[0].be[0].y =
            obstacle[0].be[0].y * 0.9 + obstacleinfo.pointOrd[0][1] * 0.1;
        obstacle[0].be[1].x =
            obstacle[0].be[1].x * 0.9 + obstacleinfo.pointOrd[1][0] * 0.1;
        obstacle[0].be[1].y =
            obstacle[0].be[1].y * 0.9 + obstacleinfo.pointOrd[1][1] * 0.1;

        obstacle[0].midPoint.x =
            obstacle[0].midPoint.x * 0.9 + obstacleinfo.midPoint[0] * 0.1;
        obstacle[0].midPoint.y =
            obstacle[0].midPoint.y * 0.9 + obstacleinfo.midPoint[1] * 0.1;

        obstacle[0].beLen = obstacle[0].beLen * 0.9 + obstacleinfo.obstacleWidth * 0.1;

        obstacle[0].obstacleTheta =
            obstacle[0].obstacleTheta * 0.9 + obstacleinfo.obstacleTheta * 0.1;
    }

    obstacle[0].obstacleType = (PK_ObstacleType)obstacleinfo.obstacleType;
    return (obstacle[0].FrameCount() >= obstacle[0].ReportFrame());
}

static int KMeans(std::vector<Point_T> &points, const int iters, const int groups,
                  std::vector<int> &label)
{
    auto getDistance = [](const Point_T &point, const Point_T &center) -> float {
        return sqrt(pow2(point.x - center.x) + pow2(point.y - center.y));
    };

    int pointCount = points.size();

    // init center points
    std::vector<Point_T> center;
    int step = (pointCount - 1) / groups;
    for (int i = 0; i < groups; ++i)
    {
        if (i > 0)
        {
            float dist = getDistance(points[i * step], center[i - 1]);
            if (dist < 0.015)
            {
                center.push_back(
                    {float(points[i * step].x + 0.01), float(points[i * step].y + 0.01)});
                continue;
            }
        }

        center.push_back(points[i * step]);
    }

    // loop
    for (int iter = 0; iter < iters; ++iter)
    {
        // merge point by its distance to center
        for (int pointNum = 0; pointNum < pointCount; ++pointNum)
        {
            float distance = 100000.0;
            for (int cluster = 0; cluster < groups; ++cluster)
            {
                float temp_distance = getDistance(points[pointNum], center[cluster]);
                if (temp_distance < distance)
                {
                    distance = temp_distance;

                    label[pointNum] = cluster;
                }
            }
        }

        // update center points
        for (int cluster = 0; cluster < groups; cluster++)
        {
            int count   = 0;
            float sum_x = 0;
            float sum_y = 0;
            for (int pointNum = 0; pointNum < pointCount; pointNum++)
            {
                if (label[pointNum] == cluster)
                {
                    count++;
                    sum_x += points[pointNum].x;
                    sum_y += points[pointNum].y;
                }
            }

            if (count > 0)
            {
                center[cluster].x = sum_x / count;
                center[cluster].y = sum_y / count;
            }
        }
    }

    std::vector<int> number(groups, 0);
    for (int i = 0; i < groups; i++)
    {
        // printf("\nlabel %d :\n", i);
        for (int j = 0; j < pointCount; j++)
        {
            if (label[j] == i)
            {
                number[i] = number[i] + 1;
                // printf("%03d ", j);
            }
        }
        // printf("\n");
    }

    int centerid = 0;
    for (int i = 1; i < groups; i++)
    {
        if (number[centerid] < number[i])
        {
            centerid = i;
        }
    }

    return centerid;
}

static int BuildBeData(VisionObstacleBuff *obstacle)
{
    std::vector<Point_T> points; // BE中点
    obstacle[0].loadData(points);
    int pointNum = points.size(); // 车位数量
    if (pointNum < min_frame_num)
    {
        return 1;
    }
    std::vector<int> label1(pointNum, -1);

    int loopNum = pointNum / 10;
    if (loopNum < 4)
    {
        loopNum = 4;
    }
    int type1 = KMeans(points, loopNum, 4, label1); // k-means聚类算法 x，y
    points.clear();

    // obstacle[0].loadTheta(points);
    // std::vector<int> label2(pointNum,-1);
    // int type2 = KMeans(points, loopNum, 4, label2);//k-means聚类算法 theta
    // points.clear();

    obstacle[0].loadLen(points);
    std::vector<int> label3(pointNum, -1);
    int type3 = KMeans(points, loopNum, 4, label3); // k-means聚类算法 len
    points.clear();

    int count = 0;
    float value[12];
    memset(value, 0.0, sizeof(value));

    for (int i = 0; i < pointNum; i++)
    {
        if (label1[i] == type1 /*&& label2[i] == type2*/ && label3[i] == type3)
        {
            count++;
            value[0] += obstacle[0].beCache[i].pt1.x;
            value[1] += obstacle[0].beCache[i].pt1.y;
            value[2] += obstacle[0].beCache[i].pt2.x;
            value[3] += obstacle[0].beCache[i].pt2.y;

            // printf("2: i = %03d b: %06f %06f e: %06f %06f\n",
            //     i, obstacle[0].beCache[i].pt1.x, obstacle[0].beCache[i].pt1.y,
            //     obstacle[0].beCache[i].pt2.x, obstacle[0].beCache[i].pt2.y);
        }
    }

    if (count == 0)
    {

        for (int i = 0; i < pointNum; i++)
        {
            if (label1[i] == type1)
            {
                count++;
                value[0] += obstacle[0].beCache[i].pt1.x;
                value[1] += obstacle[0].beCache[i].pt1.y;
                value[2] += obstacle[0].beCache[i].pt2.x;
                value[3] += obstacle[0].beCache[i].pt2.y;

                // printf("1: i = %03d b: %06f %06f e: %06f %06f\n",
                //     i, obstacle[0].beCache[i].pt1.x, obstacle[0].beCache[i].pt1.y,
                //     obstacle[0].beCache[i].pt2.x, obstacle[0].beCache[i].pt2.y);
            }
        }
    }

    if (count > 0)
    {
        obstacle[0].be[0].x = value[0] = value[0] / (float)count;
        obstacle[0].be[0].y = value[1] = value[1] / (float)count;
        obstacle[0].be[1].x = value[2] = value[2] / (float)count;
        obstacle[0].be[1].y = value[3] = value[3] / (float)count;

        obstacle[0].midPoint.x = (obstacle[0].be[0].x + obstacle[0].be[1].x) * 0.5f;
        obstacle[0].midPoint.y = (obstacle[0].be[0].y + obstacle[0].be[1].y) * 0.5f;

        obstacle[0].beLen = sqrt(pow(obstacle[0].be[1].x - obstacle[0].be[0].x, 2) +
                                 pow(obstacle[0].be[1].y - obstacle[0].be[0].y, 2));

        obstacle[0].obstacleTheta = atan2(obstacle[0].be[1].x - obstacle[0].be[0].x,
                                          obstacle[0].be[1].y - obstacle[0].be[0].y);
        printf("482: %f,%f,%f,%f \n", obstacle[0].be[0].x, obstacle[0].be[0].y,
               obstacle[0].be[1].x, obstacle[0].be[1].y);

        return AVM_SUCCESS;
    }
    else
    {
        log_warn("kmeans failed!\n");
        return 2;
    }
}

using CompFunc1 = int (*)(const VisionObstacleInfo &obstacle, const float midPt1[2],
                          const float wid1, const float angle1, const uint8_t type1);
static int FindBuf1(const VisionObstacleInfo &obstacle, const float midPt1[2],
                    const float wid1, const float angle1, const uint8_t type1)
{
    if (obstacle.obstacleType != type1)
    {
        return 0;
    }

    float dist = Cal_Dis_Pt2Pt(obstacle.midPoint, midPt1);
    return (dist < 1.5f /*&& widDiff < 1.0f && angleDiff < 1.57f*/) ? 1 : 0;
}

static VisionObstacleBuff *FindAvmObstacle(const VisionObstacleInfo &obstacle,
                                           CompFunc1 Func)
{
    const uint8_t obstaclebuff_size = SF_OBJ_NUM;
    static VisionObstacleBuff searchObstacle[obstaclebuff_size];
    int freeIdx     = -1;
    float curpos[4] = {0.0f};
    RTE_PK_Location_Get_CurPos(curpos);
    float distOdm  = 0;
    float distMax  = 0;
    int distMaxIdx = -1;

    if (RTE_PK_StateManage_Get_ModuleCom_Location() != MCOM_ON) // Init
    {
        AvmObstacle_T avmObstacle;
        memset(&avmObstacle, 0, sizeof(AvmObstacle_T));
        RTE_PK_SensorFusion_Set_Avm_Obstacle(&avmObstacle);
        for (int i = 0; i < obstaclebuff_size; i++)
        {
            searchObstacle[i].ClearData();
        }

        return NULL;
    }

    for (int i = 0; i < obstaclebuff_size; i++)
    {
        distOdm = searchObstacle[i].IsRemoveObstacle(curpos[3]);

        if (Func(obstacle, (const float *)&searchObstacle[i].midPoint,
                 searchObstacle[i].beLen, searchObstacle[i].obstacleTheta,
                 searchObstacle[i].obstacleType) ==
            1) // Pass the check, it is the same parking space
        {
            freeIdx            = i;
            static int intonum = 0;
            intonum++;
            // printf("%d ---- intonum: %d, i: %d\n", __LINE__, intonum, freeIdx);
            // printf("546--- distMacIdx: %d  freeIdx: %d\n", distMaxIdx , freeIdx);
            break;
        }
        else if (0 == searchObstacle[i].FrameCount() &&
                 searchObstacle[i].ReportCount() == 0 &&
                 searchObstacle[i].ReportCount() == 0)
        {
            searchObstacle[i].ClearData();
            searchObstacle[i].ReportFrame(min_frame_num);
            freeIdx = i;
            // frame_num ++;
            // printf("%d ---- frame_num: %d, i: %d\n", __LINE__, frame_num, freeIdx);
            break;
        }
        else if (distOdm > 18)
        {
            searchObstacle[i].ClearData();
            searchObstacle[i].ReportFrame(min_frame_num);
            freeIdx = i;
            // printf("558--- distMacIdx: %d  freeIdx: %d\n", distMaxIdx , freeIdx);
            break;
        }

        if (distOdm > distMax)
        {
            distMax    = distOdm;
            distMaxIdx = i;
            // printf("569--- distMacIdx: %d  freeIdx: %d\n", distMaxIdx , freeIdx);
        }
    }

    if (freeIdx < 0 && fabs(curpos[3]) > fabs(searchObstacle[distMaxIdx].odm))
    {
        searchObstacle[distMaxIdx].ClearData();
        searchObstacle[distMaxIdx].ReportFrame(min_frame_num);
        // log_warn("error input data old: %06f %06f %06f %06f", bexy[0][0], bexy[0][1],
        // bexy[1][0], bexy[1][1]);
        freeIdx    = distMaxIdx;
        distOdm    = -1;
        distMaxIdx = -1;
    }

    if (searchObstacle[freeIdx].ReportCount() == 2)
    {
        return NULL;
    }
    else if (freeIdx >= 0)
    {
        searchObstacle[freeIdx].obstacleType = (PK_ObstacleType)obstacle.obstacleType;
        searchObstacle[freeIdx].index        = freeIdx;
        searchObstacle[freeIdx].odm          = curpos[3];

        return &searchObstacle[freeIdx];
    }
    else
    {
        return NULL;
    }
}

static void GetCurposByTimestamp(const RD_PosStamp &recvStamp,
                                 VisionObstacleInfo &obstacle,
                                 float curPos[4]) // 相对坐标 -》 绝对坐标
{
    RD_PosStamp posStamp;
    uint32_t msStamp = obstacle.timeStamp;
    if (recvStamp.t < msStamp)
    {
        msStamp = recvStamp.t;
    }
    if (PK_PreLocation(msStamp, &posStamp) == 0)
    {
        memcpy(curPos, posStamp.pos, sizeof(float) * 4);
    }
    else
    {
        memcpy(curPos, recvStamp.pos, sizeof(float) * 4);
    }

    // obstacle.timeStamp = msStamp;
}

static int ConvertObstacle(VisionObstacleInfo &obstacle,
                           const float curPos[4]) // 相对坐标 -》 绝对坐标
{
    float NearRear[2]  = {obstacle.pointOrd[0][0], obstacle.pointOrd[0][1]};
    float NearFront[2] = {obstacle.pointOrd[1][0], obstacle.pointOrd[1][1]};
    Convert(curPos, NearFront, obstacle.pointOrd[1]);
    Convert(curPos, NearRear, obstacle.pointOrd[0]);

    obstacle.obstacleWidth =
        sqrt(pow(obstacle.pointOrd[1][0] - obstacle.pointOrd[0][0], 2) +
             pow(obstacle.pointOrd[1][1] - obstacle.pointOrd[0][1], 2));

    obstacle.obstacleTheta = atan2(obstacle.pointOrd[1][0] - obstacle.pointOrd[0][0],
                                   obstacle.pointOrd[1][1] - obstacle.pointOrd[0][1]);

    FindMidpoint(obstacle.pointOrd[0], obstacle.pointOrd[1], obstacle.midPoint);
    FindCenterPoint(obstacle.midPoint, obstacle.obstacleWidth, obstacle.obstacleTheta,
                    sign(obstacle.pointOrd[0][1]) *
                        ObsTypeDime[Obstacle_Type_Traffic_Cone].len / 2,
                    obstacle.centerPoint);

    return AVM_SUCCESS;
}

static int CheckObstacleType(int objtype)
{
    int obstacleType = Obstacle_Type_No;

    switch (objtype)
    {
        case AvmObstacle_Type_StopFlag_Two:
            obstacleType = Obstacle_Type_StopFlag;
            break;
        case AvmObstacle_Type_Traffic_Cone:
            obstacleType = Obstacle_Type_Traffic_Cone;
            break;
        case AvmObstacle_Type_Ground_Lock_OFF:
            obstacleType = Obstacle_Type_No;
            break;
        case AvmObstacle_Type_Ground_Lock_ON:
            obstacleType = Obstacle_Type_Ground_Lock;
            break;
        case AvmObstacle_Type_StopFlag_One:
            obstacleType = Obstacle_Type_StopFlag;
            break;
        case AvmObstacle_Type_StopFlag_One2:
            obstacleType = Obstacle_Type_StopFlag;
            break;
        case AvmObstacle_Type_HANDICAP:
            obstacleType = Obstacle_Type_HANDICAP;
            break;
        case AvmObstacle_Type_PVC:
            obstacleType = Obstacle_Type_PVC;
            break;
        case AvmObstacle_Type_CARTON:
            obstacleType = Obstacle_Type_CARTON;
            break;
        case AvmObstacle_Type_CHAIR:
            obstacleType = Obstacle_Type_CHAIR;
            break;
        case AvmObstacle_Type_PEDSTRAIN:
            obstacleType = Obstacle_Type_PEDSTRAIN;
            break;
        case AvmObstacle_Type_BUCKET:
            obstacleType = Obstacle_Type_BUCKET;
            break;
        case AvmObstacle_Type_E_BIKE:
            obstacleType = Obstacle_Type_E_BIKE;
            break;
        case AvmObstacle_Type_STOP_SIGN:
            obstacleType = Obstacle_Type_STOP_SIGN;
            break;
        case AvmObstacle_Type_CHARGING_STOP:
            obstacleType = Obstacle_Type_CHARGING_STOP;
            break;
        case AvmObstacle_Type_DECEL_STRIP:
            obstacleType = Obstacle_Type_DECEL_STRIP;
            break;
        case AvmObstacle_Type_BUMPER:
            obstacleType = Obstacle_Type_BUMPER;
            break;
        case AvmObstacle_Type_FENCE:
            obstacleType = Obstacle_Type_FENCE;
            break;
        case AvmObstacle_Type_WHEEL_CHAIR:
            obstacleType = Obstacle_Type_WHEEL_CHAIR;
            break;
        case AvmObstacle_Type_FIRE_KIT:
            obstacleType = Obstacle_Type_FIRE_KIT;
            break;
        default:
            obstacleType = Obstacle_Type_No;
            break;
    }

    return obstacleType;
}

static void SaveVisionObject(const AVM_ObjPointsType &Obstacle)
{
    int16_t value[8];
    value[0] = (int16_t)(Obstacle.mid[0] * 100.0);
    value[1] = (int16_t)(Obstacle.mid[1] * 100.0);
    value[2] = (int16_t)(Obstacle.obstacleTheta * 100.0);
    value[3] = (int16_t)(Obstacle.beLen * 100.0);
    value[4] = (int16_t)(Obstacle.pt1[0] * 100.0);
    value[5] = (int16_t)(Obstacle.pt1[1] * 100.0);
    value[6] = (int16_t)(Obstacle.pt2[0] * 100.0);
    value[7] = (int16_t)(Obstacle.pt2[1] * 100.0);

    int offset = 0;
    uint8_t candata[8];
    candata[offset++] = 0;
    candata[offset++] = (value[0] >> 8) & 0xff;
    candata[offset++] = value[0] & 0xff;
    candata[offset++] = (value[1] >> 8) & 0xff;
    candata[offset++] = value[1] & 0xff;
    candata[offset++] = (value[2] >> 8) & 0xff;
    candata[offset++] = value[2] & 0xff;
    candata[offset++] = Obstacle.index;
    Can_PushData(0x6002, candata);

    offset            = 0;
    candata[offset++] = 1;
    candata[offset++] = (value[3] >> 8) & 0xff;
    candata[offset++] = value[3] & 0xff;
    candata[offset++] = (value[4] >> 8) & 0xff;
    candata[offset++] = value[4] & 0xff;
    candata[offset++] = (value[5] >> 8) & 0xff;
    candata[offset++] = value[5] & 0xff;
    candata[offset++] = (uint8_t)Obstacle.obstacleType;
    Can_PushData(0x6002, candata);

    candata[offset++] = 2;
    candata[offset++] = (value[6] >> 8) & 0xff;
    candata[offset++] = value[6] & 0xff;
    candata[offset++] = (value[7] >> 8) & 0xff;
    candata[offset++] = value[7] & 0xff;
    candata[offset++] = 0;
    Can_PushData(0x6002, candata);
}

void PK_Set_Avm_Obstacle(AVM_ObjPointsType &avmObj)
{
    if (avmObj.obstacleType == Obstacle_Type_No)
    {
        return;
    }

    AvmObstacle_T avmObstacle;
    RTE_PK_SensorFusion_Get_Avm_Obstacle(&avmObstacle);

    int index                              = avmObj.index;
    avmObstacle.central_point_valid[index] = 1;
    avmObstacle.central_point[index].x     = avmObj.mid[0];
    avmObstacle.central_point[index].y     = avmObj.mid[1];
    avmObstacle.angle_valid[index]         = 1;
    avmObstacle.angle[index]               = avmObj.obstacleTheta;
    avmObstacle.length[index]              = avmObj.beLen;
    avmObstacle.width[index]               = 0;

    shortenSegment(avmObj.pt1, avmObj.pt2, 0.10f);
    avmObstacle.obj_valid[index] = 1;
    avmObstacle.obj[index].pt1.x = avmObj.pt1[0];
    avmObstacle.obj[index].pt1.y = avmObj.pt1[1];
    avmObstacle.obj[index].pt2.x = avmObj.pt2[0];
    avmObstacle.obj[index].pt2.y = avmObj.pt2[1];

    avmObstacle.num++;
    if (avmObstacle.num == 0)
    {
        avmObstacle.num = 1;
    }

    avmObstacle.type[index] = (int)(avmObj.obstacleType);

    RTE_PK_SensorFusion_Set_Avm_Obstacle(&avmObstacle);
    SaveVisionObject(avmObj);
}

static int ReportAvmObstacle(VisionObstacleBuff *pObstacle, FILE *avmflog = NULL)
{
    if (pObstacle == NULL)
    {
        return 0;
    }

    uint8_t reportNum = pObstacle[0].ReportCount();
    if (reportNum != 1)
    {
        pObstacle[0].ClearData();
        return 0;
    } // modify late!

    AVM_ObjPointsType avmObj;
    avmObj.index         = pObstacle[0].index;
    avmObj.obstacleTheta = pObstacle[0].obstacleTheta;
    avmObj.beLen         = pObstacle[0].beLen;
    avmObj.mid[0]        = pObstacle[0].midPoint.x;
    avmObj.mid[1]        = pObstacle[0].midPoint.y;
    avmObj.pt1[0]        = pObstacle[0].be[0].x;
    avmObj.pt1[1]        = pObstacle[0].be[0].y;
    avmObj.pt2[0]        = pObstacle[0].be[1].x;
    avmObj.pt2[1]        = pObstacle[0].be[1].y;
    avmObj.obstacleType  = pObstacle[0].obstacleType;

    PK_Set_Avm_Obstacle(avmObj);

    pObstacle[0].ClearData();
    pObstacle[0].ReportFrame(a_frame_num);
    pObstacle[0].ReportCount(2);
    return 1;
}

int CacheAvmObstacle(const VisionObstacleInfo &obstacle)
{
    VisionObstacleBuff *pObstacle = FindAvmObstacle(obstacle, FindBuf1);
    if (pObstacle == NULL)
    {
        return 0;
    }

    bool full = AddBeData(obstacle, pObstacle);
    if (!full)
    {
        return 0;
    }

    int result = BuildBeData(pObstacle); // 确定中心点，返回AVM_SUCCESS
    if (result != AVM_SUCCESS)
    {
        return 0;
    }

    pObstacle[0].timeStamp = RTE_BSW_Get_CurTime();
    pObstacle[0].ReportCount(1);

    ReportAvmObstacle(pObstacle);

    return 1;
}

int CheckSearchObstacle(VisionObstacleInfo &obstacle)
{
    int result = AVM_FAIL;

    switch (obstacle.obstacleType)
    {
        case Obstacle_Type_StopFlag:
            break;

        case Obstacle_Type_Traffic_Cone:
            result = AVM_SUCCESS;
            break;

        case Obstacle_Type_Ground_Lock:
            break;

        default:
            break;
    }
    return result;
}

#ifdef VISION_V1
static inline int JsonError(const char *inf, const char *key)
{
    printf("json: %s %s\n", inf, key);
    return -1;
}

static int RestoreObstacle(Json::Value jsonRoot, const char *msg,
                           struct VisionObstacleInfo &obstacle, FILE *avmflog)
{
    const char *key1 = "timeStamp";
    if (!jsonRoot[msg].isMember(key1))
    {
        return JsonError("invalid key/obstacletimeStamp string", key1);
    }
    obstacle.timeStamp = jsonRoot[msg][key1].asInt();

    const char *key2 = "objtype";
    if (!jsonRoot[msg].isMember(key2))
    {
        return JsonError("invalid key/objtype string", key2);
    }
    obstacle.obstacleType = CheckObstacleType(jsonRoot[msg][key2].asInt());

    if (obstacle.obstacleType == Obstacle_Type_No ||
        (obstacle.obstacleType != Obstacle_Type_Traffic_Cone &&
         obstacle.obstacleType != Obstacle_Type_Ground_Lock &&
         obstacle.obstacleType != Obstacle_Type_PEDSTRAIN))
    {
        return -1;
    }

    const char *key8 = "objPoints";
    float temp[4][2];
    if (jsonRoot[msg].isMember(key8) && jsonRoot[msg][key8].isArray() &&
        jsonRoot[msg][key8].size() == 8)
    {
        int offset = 0;
        temp[0][0] = jsonRoot[msg][key8][offset++].asDouble();
        temp[0][1] = jsonRoot[msg][key8][offset++].asDouble();
        temp[1][0] = jsonRoot[msg][key8][offset++].asDouble();
        temp[1][1] = jsonRoot[msg][key8][offset++].asDouble();
        temp[2][0] = jsonRoot[msg][key8][offset++].asDouble();
        temp[2][1] = jsonRoot[msg][key8][offset++].asDouble();
        temp[3][0] = jsonRoot[msg][key8][offset++].asDouble();
        temp[3][1] = jsonRoot[msg][key8][offset++].asDouble();

        selectObstaclePoint(temp);

        float midPoint[2] = {0};
        FindMidpoint(temp[0], temp[1], midPoint);

        if (midPoint[0] < (REAR_VIEW_MIRROR_X - 2.0f) ||
            midPoint[0] > (REAR_VIEW_MIRROR_X + 2.0f))
        {
            return -1;
        }

        if (avmflog != NULL)
        {
            fprintf(avmflog, "time, %u, type, %u, obj,%f,%f,%f,%f\n", obstacle.timeStamp,
                    obstacle.obstacleType, temp[0][0], temp[0][1], temp[1][0],
                    temp[1][1]);
        }

        if (temp[0][0] < temp[1][0])
        {
            obstacle.pointOrd[0][0] = temp[0][0];
            obstacle.pointOrd[0][1] = temp[0][1];
            obstacle.pointOrd[1][0] = temp[1][0];
            obstacle.pointOrd[1][1] = temp[1][1];
        }
        else
        {
            obstacle.pointOrd[1][0] = temp[0][0];
            obstacle.pointOrd[1][1] = temp[0][1];
            obstacle.pointOrd[0][0] = temp[1][0];
            obstacle.pointOrd[0][1] = temp[1][1];
        }

        obstacle.pointOrd[2][0] = temp[2][0];
        obstacle.pointOrd[2][1] = temp[2][1];
        obstacle.pointOrd[3][0] = temp[3][0];
        obstacle.pointOrd[3][1] = temp[3][1];
    }
    else
    {
        memset(obstacle.pointOrd, 0, sizeof(obstacle.pointOrd));
        return JsonError("invalid target data", key8);
    }

    sscanf(msg, "%d", &obstacle.obstacleIndex);

    return AVM_SUCCESS;
}

void ParseVisionObject(const char *str)
{
    static FILE *obstaclelog = NULL;
    struct VisionObstacleInfo obstacle;

    if (RTE_PK_StateManage_Get_ModuleCom_Location() != MCOM_ON)
    {
        FindAvmObstacle(obstacle, FindBuf1);

        AvmObstacle_T avmObstacle;
        RTE_PK_SensorFusion_Get_Avm_Obstacle(&avmObstacle);
        if (avmObstacle.num > 0)
        {
            memset(&avmObstacle, 0, sizeof(AvmObstacle_T));
            RTE_PK_SensorFusion_Set_Avm_Obstacle(&avmObstacle);
        }

        if (obstaclelog != NULL)
        {
            fclose(obstaclelog);
            obstaclelog = NULL;
        }

        return;
    }
    // else if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() != MCOM_ON_PARKING)
    // {
    //     if (RTE_BSW_Get_CurrentGear() == GEAR_REAL_R) {  return;  }
    // }

    if (obstaclelog == NULL && is_save_log)
    {
        obstaclelog = OpenLogFile("obstaclelog.txt");
    }

    ObsTypeDimeData();

    RD_PosStamp curPosStamp;
    RTE_PK_Location_Get_CurPos_With_Timestamp(&curPosStamp);

    Json::Reader jsonReader;
    Json::Value jsonRoot;

    if (!jsonReader.parse(
            str, jsonRoot)) // jsonRoot 是用来存储解析后的数据，是Json::Value的一种对象
    {
        /*parse创建一个 Json::Reader 对象 jsonReader，
        用于解析 JSON 字符串。然后使用 jsonReader 对象将传入的 str 解析为 jsonRoot，
        如果解析失败，输出错误信息并返回。
        */
        printf("invalid json string %s\n", str);
        return;
    }

    /*
    通过 jsonRoot 对象的 getMemberNames 函数获取 JSON 对象的字段名列表，
    然后对每个字段进行循环遍历。
    */
    // int count = 0;
    float curPos[4] = {0.0f};
    auto items      = jsonRoot.getMemberNames();
    for (auto item : items)
    {
        const char *key0 = "type";
        if (!jsonRoot[item.c_str()].isMember(key0) ||
            jsonRoot[item.c_str()][key0].asInt() != 1) // 视觉车位
        {
            continue;
        }
        memset(&obstacle, 0, sizeof(obstacle));

        int result = RestoreObstacle(jsonRoot, item.c_str(), obstacle, obstaclelog);
        if (result != AVM_SUCCESS)
        {
            continue;
        }

        // 1、校验过滤
        if (CheckSearchObstacle(obstacle) != AVM_SUCCESS)
        {
            continue;
        };

        // 2、时间戳对齐
        GetCurposByTimestamp(curPosStamp, obstacle, curPos);

        // 3、相对坐标——>绝对坐标
        result = ConvertObstacle(obstacle, curPos);
        if (result != AVM_SUCCESS)
        {
            continue;
        }

        // 4、跟踪
        result = CacheAvmObstacle(obstacle);
    }
}
#endif

#ifdef VISION_V2
#include "apa_msgs/frame.pb.h"
/**protobuf****************************************************/
static int RestoreObstaclePb(const ::raincom::apa::msgs::Object &obstacleData,
                             struct VisionObstacleInfo &obstacle, FILE *avmflog)
{
    obstacle.timeStamp = obstacleData.header().stamp();

    obstacle.obstacleIndex = obstacleData.header().id();

    obstacle.obstacleType = CheckObstacleType(obstacleData.clf());

    if (obstacle.obstacleType == Obstacle_Type_No)
    {
        return -1;
    }

    obstacle.status = obstacleData.conf() > 0.5f ? 1 : 0;

    if (obstacleData.center_valid())
    {
        obstacle.centerPoint[0] = obstacleData.center().x();
        obstacle.centerPoint[1] = obstacleData.center().y();
    }

    if (obstacleData.corners_size() == 4)
    {
        obstacle.pointOrd[0][0] = obstacleData.corners(0).x();
        obstacle.pointOrd[0][1] = obstacleData.corners(0).y();
        obstacle.pointOrd[1][0] = obstacleData.corners(1).x();
        obstacle.pointOrd[1][1] = obstacleData.corners(1).y();
        obstacle.pointOrd[2][0] = obstacleData.corners(2).x();
        obstacle.pointOrd[2][1] = obstacleData.corners(2).y();
        obstacle.pointOrd[3][0] = obstacleData.corners(3).x();
        obstacle.pointOrd[3][1] = obstacleData.corners(3).y();
        if (avmflog != NULL)
        {
            fprintf(avmflog, "time: %u, point:%f,%f,%f,%f type:%d\n", obstacle.timeStamp,
                    obstacle.pointOrd[0][0], obstacle.pointOrd[0][1],
                    obstacle.pointOrd[1][0], obstacle.pointOrd[1][1],
                    obstacle.obstacleType);
        }
    }
    else
    {
        memset(obstacle.pointOrd, 0, sizeof(obstacle.pointOrd));
        return -1;
    }

    return AVM_SUCCESS;
}

void ParseVisionObjectPb(
    const ::google::protobuf::RepeatedPtrField<::raincom::apa::msgs::Object> &obstacleMsg)
{
    static FILE *obstaclelog = NULL;
    struct VisionObstacleInfo obstacle;

    if (RTE_PK_StateManage_Get_ModuleCom_Location() != MCOM_ON)
    {
        FindAvmObstacle(obstacle, FindBuf1);

        AvmObstacle_T avmObstacle;
        RTE_PK_SensorFusion_Get_Avm_Obstacle(&avmObstacle);
        if (avmObstacle.num > 0)
        {
            memset(&avmObstacle, 0, sizeof(AvmObstacle_T));
            RTE_PK_SensorFusion_Set_Avm_Obstacle(&avmObstacle);
        }

        if (obstaclelog != NULL)
        {
            fclose(obstaclelog);
            obstaclelog = NULL;
        }

        return;
    }

    if (obstaclelog == NULL && is_save_log)
    {
        obstaclelog = OpenLogFile("obstaclelog.txt");
    }

    ObsTypeDimeData();

    RD_PosStamp curPosStamp;
    RTE_PK_Location_Get_CurPos_With_Timestamp(&curPosStamp);

    float curPos[4] = {0.0f};
    for (int i = 0; i < obstacleMsg.size(); i++)
    {
        const ::raincom::apa::msgs::Object &obstacleData = obstacleMsg.Get(i);

        memset(&obstacle, 0, sizeof(obstacle));
        // 解析
        int result = RestoreObstaclePb(obstacleData, obstacle, obstaclelog);
        if (result != AVM_SUCCESS)
        {
            continue;
        }

        // 1、校验过滤
        if (CheckSearchObstacle(obstacle) != AVM_SUCCESS)
        {
            continue;
        };

        // 2、时间戳对齐
        GetCurposByTimestamp(curPosStamp, obstacle, curPos);

        // 3、相对坐标——>绝对坐标
        result = ConvertObstacle(obstacle, curPos);
        if (result != AVM_SUCCESS)
        {
            continue;
        }

        // 4、跟踪上报
        result = CacheAvmObstacle(obstacle);
    }
}
/**************************************************************************/
#endif