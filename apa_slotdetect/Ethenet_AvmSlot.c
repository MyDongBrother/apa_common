#include "Ethenet_AvmSlot.h"
#include "Rte_BSW.h"
#include "Module_API.h"
#include "Rte_ComIF.h"
#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "SystemPara.h"
#include "MathFunc.h"
#include "Record_Log.h"
#include "PK_StateManage.h"
#include "PK_PathPlan.h"
#include <vector>

#define MAXFRAMESIZE 32

static int buffIndex = 0;
struct VisionSlotBuff
{
    uint8_t report;
    std::vector<LineSeg_T> beCache;   // be坐标缓存区
    std::vector<LineSeg_T> cdCache;   // cd坐标缓存区
    std::vector<LineSeg_T> stopCache; // 限位杆坐标缓存区
    std::vector<float> weights;       // 权重缓存区
    Point_T bcde[4];                  // 最终车位坐标
    Point_T stopPoint[2];             // 最终限位杆坐标
    PK_SlotShapeType shape;           // 车位类型
    int8_t hasStop;                   // 限位杆有效
    int8_t hasLock;                   // 地锁（占用标识）有效
    int8_t maxFrame;                  // 最大车位缓存个数
    uint64_t timestamp;               // 时间戳
    int16_t index;                    // 车位编号
    float odm;                        // 车位距车辆当前位置距离
    int8_t hasStopNum;                // 缓存车位中限位杆有效个数
    int8_t hasLockNum;                // 缓存车位中地锁有效个数
    int8_t hasReliableSlot;           // 缓存车位中可靠车位个数
    float last_sum_weight;            // 上一次上报车位的总权重

    VisionSlotBuff()
    {
        index           = buffIndex++;
        report          = 0;
        hasStop         = 0;
        hasLock         = 0;
        odm             = 0.0;
        maxFrame        = min_frame_num;
        hasReliableSlot = 0;
        last_sum_weight = 0;
    }

    VisionSlotBuff(const uint8_t value)
    {
        VisionSlotBuff();
        maxFrame = value;
    }
    uint8_t ReportCount() const { return report; }
    uint8_t FrameCount() const { return beCache.size(); }
    uint8_t MaxFrame() const { return MAXFRAMESIZE; }
    int8_t ReportFrame() const { return maxFrame; }
    void ReportFrame(const int8_t value) { maxFrame = value; }
    void ReportCount(const uint8_t value) { report = value; }
    bool IsReported() const { return (report > 0); }
    bool IsRemoveSlot(const float curDist) const { return (fabs(odm - curDist) > 25); }
    void ClearData()
    {
        odm             = 0.0;
        hasLock         = 0;
        hasStop         = 0;
        hasReliableSlot = 0;
        report          = 0;
        beCache.clear();
        cdCache.clear();
        stopCache.clear();
        weights.clear();
    }

    void loadData(std::vector<Point_T> &points)
    {
        for (size_t i = 0; i < beCache.size(); i++)
        {
            points.push_back({(float)(0.5 * (beCache[i].pt1.x + beCache[i].pt2.x)),
                              (float)((beCache[i].pt1.y + beCache[i].pt2.y) * 0.5)});
        }
    }

    void loadTheta(std::vector<Point_T> &points)
    {
        for (size_t i = 0; i < beCache.size(); i++)
        {
            points.push_back({fabs(atan2(beCache[i].pt2.x - beCache[i].pt1.x,
                                         beCache[i].pt2.y - beCache[i].pt1.y)),
                              0.0});
        }
    }

    float getSumWeight()
    {
        float Sum_weight = 0.0f;
        for (size_t i = 0; i < weights.size(); i++)
        {
            Sum_weight += weights[i];
        }
        return Sum_weight;
    }
};

static int kmeans(std::vector<Point_T> &points, const int iters, const int groups,
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

static int Kmeans_Weights(std::vector<Point_T> &points, const std::vector<float> &weights,
                          const int iters, const int groups, std::vector<int> &label)
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
                    distance        = temp_distance;
                    label[pointNum] = cluster;
                }
            }
        }

        // update center points with weights
        for (int cluster = 0; cluster < groups; cluster++)
        {
            float sum_weights    = 0;
            float weighted_sum_x = 0;
            float weighted_sum_y = 0;

            for (int pointNum = 0; pointNum < pointCount; pointNum++)
            {
                if (label[pointNum] == cluster)
                {
                    float weight = weights[pointNum];
                    sum_weights += weight;
                    weighted_sum_x += points[pointNum].x * weight;
                    weighted_sum_y += points[pointNum].y * weight;
                }
            }

            if (sum_weights > 0)
            {
                center[cluster].x = weighted_sum_x / sum_weights;
                center[cluster].y = weighted_sum_y / sum_weights;
            }
        }
    }

    std::vector<float> number(groups, 0.0f); // 使用浮点类型存储加权计数
    for (int i = 0; i < groups; i++)
    {
        for (int j = 0; j < pointCount; j++)
        {
            if (label[j] == i)
            {
                number[i] += weights[j]; // 将点的权重加入到该簇的加权计数
            }
        }
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

using CompFunc = int (*)(const float bexy1[2][2], const float bexy2[2][2]);
static int FindBuf1(const float bexy1[2][2], const float bexy2[2][2])
{
    float dist1 = Cal_Dis_Pt2Pt(bexy1[0], bexy2[0]);
    float dist2 = Cal_Dis_Pt2Pt(bexy1[1], bexy2[1]);

    Vec2_T vec1 = {bexy1[1][0] - bexy1[0][0], bexy1[1][1] - bexy1[0][1]};
    Vec2_T vec2 = {bexy2[1][0] - bexy2[0][0], bexy2[1][1] - bexy2[0][1]};

    float cos = Get_Dot_Vec2(vec1, vec2) / Get_Norm_Vec2(vec1) / Get_Norm_Vec2(vec2);

    return (dist1 < VEHICLE_WID / 3 && dist2 < VEHICLE_WID / 3 && cos > 0.9) ? 1 : 0;
}

static int FindBuf2(const float curpos[2][2], const float bexy2[2][2])
{
    float midxy[2];
    midxy[0] = (bexy2[1][0] + bexy2[0][0]) * 0.5;
    midxy[1] = (bexy2[1][1] + bexy2[0][1]) * 0.5;
    float rx = Project_PosTo1st_rx(&curpos[0][0], midxy);

    return (rx < -VEHICLE_WID * 0.7) ? 1 : 0;
    // return (rx < 0.0) ? 1 : 0;
    // return (fabs(rx) > VEHICLE_LEN * 0.4) ? 1 : 0;
}

/**
 * @brief 查找AVM缓存车位
 *
 * 该函数用于根据传入的车位坐标，查找缓存中的匹配车位。如果找到相同的车位，则返回对应的缓存槽位；
 * 如果未找到，则返回空闲的槽位。如果车位数据无效或不存在，则返回NULL。
 *
 * @param bexy 传入的车位顶点坐标，二维数组，表示车位的两个关键顶点（be两点）。
 * @param Func 用于比较车位顶点的函数指针，比较当前缓存中的车位和传入车位是否匹配。
 * @return VisionSlotBuff*
 * 返回指向匹配的缓存车位结构体的指针，如果没有找到或没有空闲槽位，返回NULL。
 */
static VisionSlotBuff *FindAvmSlot(const float bexy[2][2], CompFunc Func)
{
    // 定义缓存容量，存储最多30个车位信息
    const uint8_t slotbuff_size = 30;
    // 静态变量：缓存30个车位信息
    static VisionSlotBuff searchSlot[slotbuff_size];
    // 静态变量：泊车状态目标车位数据缓存
    static VisionSlotBuff parkSlot(b_frame_num);

    // 用于存储空闲槽位的索引
    int freeIdx = -1;

    // 当前车辆的位置
    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);

    // 如果位置模块通信未开启，清空所有缓存车位信息
    if (RTE_PK_StateManage_Get_ModuleCom_Location() != MCOM_ON)
    {
        for (int i = 0; i < slotbuff_size; i++)
        {
            searchSlot[i].ClearData();         // 清空车位数据
            searchSlot[i].last_sum_weight = 0; // 重置缓存权重
        }
        parkSlot.last_sum_weight = 0; // 重置缓存权重
        parkSlot.ClearData();         // 清空停车位缓存
        return NULL;                  // 返回NULL，表示无效数据
    }
    else
    {
        // 如果模块状态为“正在泊车”，清空所有缓存车位信息并返回parkSlot缓存
        if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() == MCOM_ON_PARKING)
        {
            for (int i = 0; i < slotbuff_size; i++)
            {
                searchSlot[i].ClearData();
                searchSlot[i].last_sum_weight = 0;
            }
            return &parkSlot;
        }
        else
        {
            // 非正在泊车状态时，清空停车位缓存
            parkSlot.ClearData(); // log_warn("clear VisionSlotBuff\n");
            parkSlot.last_sum_weight = 0;
        }
    }

    // 遍历缓存中的车位，查找符合条件的车位或空闲槽位
    for (int i = 0; i < slotbuff_size; i++)
    {
        // 判断当前车位是否空闲或需要移除
        if ((0 == searchSlot[i].FrameCount() && searchSlot[i].ReportCount() == 0) ||
            searchSlot[i].IsRemoveSlot(curpos[3]))
        {
            searchSlot[i].ClearData(); // log_warn("clear VisionSlotBuff\n"); //
                                       // 清除车位数据
            searchSlot[i].ReportFrame(min_frame_num); // 设置最小帧数
            searchSlot[i].last_sum_weight = 0;        // 重置缓存权重
            freeIdx                       = i;        // 标记空闲槽位
        }
        else
        {
            // 取出缓存中的两个关键顶点坐标
            float bexys[2][2];
            bexys[0][0] = searchSlot[i].bcde[0].x;
            bexys[0][1] = searchSlot[i].bcde[0].y;
            bexys[1][0] = searchSlot[i].bcde[3].x;
            bexys[1][1] = searchSlot[i].bcde[3].y;

            // 计算车位中心点
            float centerPoint[2];
            centerPoint[0] = (bexys[0][0] + bexys[1][0]) * 0.5;
            centerPoint[1] = (bexys[0][1] + bexys[1][1]) * 0.5;
            // 计算车位相对当前车辆的方向角
            float ry  = Project_PosTo1st_ry(curpos, centerPoint);
            float dir = (Slot_Dir(searchSlot[i].shape) == 1) ? 1 : -1;
            // 如果车位相对方向与车辆方向不一致，清除该车位数据
            if (ry * dir < 0)
            {
                searchSlot[i].ClearData();
                searchSlot[i].ReportFrame(min_frame_num);
                searchSlot[i].last_sum_weight = 0;
                freeIdx                       = i;
                continue;
            }

            // 如果通过了Func的检查，表示找到相同车位
            if (Func(bexy, bexys) == 1) // 判断是否为相同车位
            {
                freeIdx = i;
                break;
            }
        }
    }

    // 如果未找到空闲槽位或匹配车位，返回NULL
    if (freeIdx < 0)
    {
        // log_warn("error input data old: %06f %06f %06f %06f", bexy[0][0], bexy[0][1],
        // bexy[1][0], bexy[1][1]);
        return NULL;
    }
    else
    {
        // 找到空闲槽位或匹配车位，更新车位数据并返回
        searchSlot[freeIdx].odm = curpos[3]; // 更新车位的参考位置
        return &searchSlot[freeIdx];         // 返回匹配或空闲的缓存槽位指针
    }
}

/**
 * @brief 对缓存中的车位数据进行聚类分析，生成最终的车位角点和限位杆数据
 *
 * 该函数通过 K-Means
 * 聚类算法对车位的角点和方向角数据进行聚类分析，选出最可靠的数据并进行加权计算，
 * 最终生成车位的角点和限位杆位置。如果聚类失败或未能生成足够的有效数据，将返回错误码。
 *
 * @param slot 缓存中的车位槽位指针，包含了车位的角点、停止点和权重等数据。
 * @return int 返回 AVM_SUCCESS 表示成功，否则返回错误码。
 */
static int BuildBeData(VisionSlotBuff *slot)
{
    std::vector<Point_T> points;  // 存储 BE 中点数据（车位角点）
    slot[0].loadData(points);     // 加载缓存中的车位角点数据
    int pointNum = points.size(); // 获取车位数量

    if (pointNum < min_frame_num)
    {
        return 1;
    } // 如果点数不足，直接返回错误码

    std::vector<int> label1(pointNum, -1); // 存储第一轮 K-means 聚类的标签
    int groups = pointNum >= 12 ? 3 : (pointNum == 3 ? 1 : 2); // 根据点数决定聚类的组数

    int loopNum = pointNum / 10; // 根据点数决定聚类的迭代次数
    if (loopNum < 4)
    {
        loopNum = 4;
    }

    // 执行加权 K-means 聚类算法（基于 x, y 坐标）
    int type1 = Kmeans_Weights(points, slot[0].weights, loopNum, groups, label1);

    points.clear();
    slot[0].loadTheta(points);             // 加载车位的方向角数据
    std::vector<int> label2(pointNum, -1); // 存储第二轮 K-means 聚类的标签

    // 执行加权 K-means 聚类算法（基于方向角 theta）
    int type2 = Kmeans_Weights(points, slot[0].weights, loopNum, groups, label2);

    int count = 0;   // 累计权重的总和
    float value[12]; // 存储加权后的车位角点和限位杆的坐标
    int count2 = 0;  // 限位杆的权重总和
    memset(value, 0.0, sizeof(value));

    // 遍历所有点，选择与聚类结果相匹配的点，并加权计算
    for (int i = 0; i < pointNum; i++)
    {
        if (label1[i] == type1 && label2[i] == type2)
        {
            float weight = slot[0].weights[i]; // 获取该点的权重
            count += weight;

            // 加权累积车位的四个角点
            value[0] += slot[0].beCache[i].pt1.x * weight;
            value[1] += slot[0].beCache[i].pt1.y * weight;
            value[2] += slot[0].beCache[i].pt2.x * weight;
            value[3] += slot[0].beCache[i].pt2.y * weight;
            value[4] += slot[0].cdCache[i].pt1.x * weight;
            value[5] += slot[0].cdCache[i].pt1.y * weight;
            value[6] += slot[0].cdCache[i].pt2.x * weight;
            value[7] += slot[0].cdCache[i].pt2.y * weight;

            // 如果限位杆的两个点之间的距离大于 0.5，则进行加权累积
            if (Cal_Dis_Pt2Pt(slot[0].stopCache[i].pt1, slot[0].stopCache[i].pt2) > 0.5)
            {
                count2 += weight;
                value[8] += 0.5 *
                            (slot[0].stopCache[i].pt1.x + slot[0].stopCache[i].pt2.x) *
                            weight;
                value[9] += 0.5 *
                            (slot[0].stopCache[i].pt1.y + slot[0].stopCache[i].pt2.y) *
                            weight;
            }
        }
    }

    // 如果没有匹配到任何点，尝试仅根据第一轮聚类结果重新累积
    if (count == 0)
    {
        count2 = 0;
        for (int i = 0; i < pointNum; i++)
        {
            if (label1[i] == type1)
            {
                float weight = slot[0].weights[i];
                count += weight;

                value[0] += slot[0].beCache[i].pt1.x * weight;
                value[1] += slot[0].beCache[i].pt1.y * weight;
                value[2] += slot[0].beCache[i].pt2.x * weight;
                value[3] += slot[0].beCache[i].pt2.y * weight;
                value[4] += slot[0].cdCache[i].pt1.x * weight;
                value[5] += slot[0].cdCache[i].pt1.y * weight;
                value[6] += slot[0].cdCache[i].pt2.x * weight;
                value[7] += slot[0].cdCache[i].pt2.y * weight;

                if (Cal_Dis_Pt2Pt(slot[0].stopCache[i].pt1, slot[0].stopCache[i].pt2) >
                    0.5)
                {
                    count2 += weight;
                    value[8] +=
                        0.5 * (slot[0].stopCache[i].pt1.x + slot[0].stopCache[i].pt2.x) *
                        weight;
                    value[9] +=
                        0.5 * (slot[0].stopCache[i].pt1.y + slot[0].stopCache[i].pt2.y) *
                        weight;
                }
            }
        }
    }

    // 如果有足够的有效点，计算车位的最终角点和限位杆位置
    if (count > 0)
    {
        // 计算加权平均的角点坐标
        slot[0].bcde[0].x = value[0] = value[0] / (float)count;
        slot[0].bcde[0].y = value[1] = value[1] / (float)count;
        slot[0].bcde[3].x = value[2] = value[2] / (float)count;
        slot[0].bcde[3].y = value[3] = value[3] / (float)count;
        slot[0].bcde[1].x = value[4] = value[4] / (float)count;
        slot[0].bcde[1].y = value[5] = value[5] / (float)count;
        slot[0].bcde[2].x = value[6] = value[6] / (float)count;
        slot[0].bcde[2].y = value[7] = value[7] / (float)count;

        // 计算加权平均的限位杆坐标
        if (count2 > 0)
        {
            slot[0].stopPoint[0].x = value[8] = value[8] / (float)count2;
            slot[0].stopPoint[0].y = value[9] = value[9] / (float)count2;
            slot[0].stopPoint[1].x =
                slot[0].stopPoint[0].x; // 限位杆的第二个点与第一个点相同
            slot[0].stopPoint[1].y = slot[0].stopPoint[0].y;
        }
        else
        {
            // 如果没有有效的限位杆数据，重置限位杆坐标并标记为无限位杆
            slot[0].stopPoint[0].x = 0;
            slot[0].stopPoint[0].y = 0;
            slot[0].stopPoint[1].x = 0;
            slot[0].stopPoint[1].y = 0;
            slot[0].hasStop        = 0;
        }

        return AVM_SUCCESS; // 返回成功标志
    }
    else
    {
        // 如果聚类失败或无有效数据，返回错误码
        log_warn("kmeans failed!\n");
        return 2;
    }
}

// 计算两个点的叉积
float crossProduct(float ax, float ay, float bx, float by) { return ax * by - ay * bx; }

/**
 * @brief 判断点是否在四边形内部
 * @param vertices 矩形4个顶点数组
 * @param p 点
 * @return int 1 - 点在矩形内部 2 - 点在矩形外部
 */
static int isPointInQuadrilateral(const float vertices[4][2], const float p[2])
{
    float ax = vertices[0][0], ay = vertices[0][1];
    float bx = vertices[1][0], by = vertices[1][1];
    float cx = vertices[2][0], cy = vertices[2][1];
    float dx = vertices[3][0], dy = vertices[3][1];

    // 计算向量
    float abx = bx - ax, aby = by - ay;
    float bcx = cx - bx, bcy = cy - by;
    float cdx = dx - cx, cdy = dy - cy;
    float dax = ax - dx, day = ay - dy;

    float apx = p[0] - ax, apy = p[1] - ay;
    float bpx = p[0] - bx, bpy = p[1] - by;
    float cpx = p[0] - cx, cpy = p[1] - cy;
    float dpx = p[0] - dx, dpy = p[1] - dy;

    // 计算叉积
    float cross1 = crossProduct(abx, aby, apx, apy);
    float cross2 = crossProduct(bcx, bcy, bpx, bpy);
    float cross3 = crossProduct(cdx, cdy, cpx, cpy);
    float cross4 = crossProduct(dax, day, dpx, dpy);

    // 检查叉积符号是否一致
    if ((cross1 >= 0 && cross2 >= 0 && cross3 >= 0 && cross4 >= 0) ||
        (cross1 <= 0 && cross2 <= 0 && cross3 <= 0 && cross4 <= 0))
    {
        return 1; // 点在四边形内部
    }

    return 0; // 点在四边形外部
}

static void SetAvmSlotPoint(const VisionSlotBuff *pSlot, AVM_SlotPointsType &avmSlot)
{
    memcpy(avmSlot.NearRear, &pSlot[0].bcde[0], sizeof(float) * 2);  // B
    memcpy(avmSlot.FarRear, &pSlot[0].bcde[1], sizeof(float) * 2);   // C
    memcpy(avmSlot.FarFront, &pSlot[0].bcde[2], sizeof(float) * 2);  // D
    memcpy(avmSlot.NearFront, &pSlot[0].bcde[3], sizeof(float) * 2); // E

    avmSlot.hasStop   = pSlot[0].hasStop;
    avmSlot.hasLock   = pSlot[0].hasLock;
    avmSlot.shape     = pSlot[0].shape;
    avmSlot.timestamp = pSlot[0].timestamp;
    memcpy(avmSlot.stopPoint, &pSlot[0].stopPoint, sizeof(Point_T) * 2);

    // printf("---------------------report avm slot: %d report %d type %d  %06f %06f %06f
    // %06f %06f %06f %06f %06f %d\n",
    //      pSlot[0].hasReliableSlot,
    //      pSlot[0].ReportCount(), avmSlot.shape,
    //      avmSlot.NearRear[0],avmSlot.NearRear[1],avmSlot.FarRear[0],avmSlot.FarRear[1],
    //      avmSlot.FarFront[0],avmSlot.FarFront[1],avmSlot.NearFront[0],avmSlot.NearFront[1],avmSlot.hasLock);

    RTE_PD_Set_AVM_SlotPoints(&avmSlot);

#ifdef ANDROID_PLATFORM

#else
    // Avm_StopBar_T avmStopBar;
    // avmStopBar.stopbar.pt1.x = pSlot[0].stopPoint[0].x;
    // avmStopBar.stopbar.pt1.y = pSlot[0].stopPoint[0].y;
    // avmStopBar.stopbar.pt2.x = pSlot[0].stopPoint[1].x;
    // avmStopBar.stopbar.pt2.y = pSlot[0].stopPoint[1].y;
    // avmStopBar.slot_index = pSlot[0].index;
    // avmStopBar.slotshap = pSlot[0].shape;
    // PK_Set_Avm_StopBar(&avmStopBar);
#endif
}

static void SetAvmSlotPointB(const VisionSlotBuff *pSlot, FILE *avmflog)
{
    Avm_Pot_T avmPot;
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avmPot);

    // printf("\n\n------------------------\nSetAvmSlotPointB
    // %d\n---------------------------------\n\n", __LINE__);
    if (RTE_PK_DataConvt_Get_IsAVM_Slot() == 3)
    {
        RTE_PK_DataConvt_Set_IsAVM_Slot(1);
    }

    avmPot.near_rear  = pSlot[0].bcde[0]; // B
    avmPot.far_rear   = pSlot[0].bcde[1]; // C
    avmPot.far_front  = pSlot[0].bcde[2]; // D
    avmPot.near_front = pSlot[0].bcde[3]; // E

    avmPot.slot_index = avmPot.slot_index + 1;
    RTE_PD_Set_AVM_SlotInfo_B(&avmPot);

    Avm_Obj_T avmObj;
    memset(&avmObj, 0, sizeof(avmObj));
    if (pSlot[0].hasLock != 0)
    {
        avmObj.ls.pt1.x      = pSlot[0].stopPoint[0].x;
        avmObj.ls.pt1.y      = pSlot[0].stopPoint[0].y;
        avmObj.ls.pt2.x      = pSlot[0].stopPoint[1].x;
        avmObj.ls.pt2.y      = pSlot[0].stopPoint[1].y;
        avmObj.slot_index    = avmPot.slot_index;
        avmObj.SlotObsNum    = 1;
        avmObj.AlignIndex[0] = 1;
        RTE_PD_Set_AVM_SlotObs_B(&avmObj);
    }

    if (avmflog != NULL)
    {
        struct timeval stamp;
        gettimeofday(&stamp, NULL);
        fprintf(
            avmflog,
            "reportB "
            "%06ld:%06ld,%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f,%06f\n",
            stamp.tv_sec, stamp.tv_usec, avmObj.ls.pt1.x, avmObj.ls.pt1.y,
            avmObj.ls.pt2.x, avmObj.ls.pt2.y, avmPot.near_rear.x, avmPot.near_rear.y,
            avmPot.far_rear.x, avmPot.far_rear.y, avmPot.far_front.x, avmPot.far_front.y,
            avmPot.near_front.x, avmPot.near_front.y);
    }

#if 1
    float rsMid[2], finpos[3];
    RTE_PK_SlotDetect_Get_TargPos(finpos);

    Point_T pointBE, pointCD, pointTarg;

    pointBE.x = (avmPot.near_front.x + avmPot.near_rear.x) * 0.5;
    pointCD.x = (avmPot.far_front.x + avmPot.far_rear.x) * 0.5;
    pointBE.y = (avmPot.near_front.y + avmPot.near_rear.y) * 0.5;
    pointCD.y = (avmPot.far_front.y + avmPot.far_rear.y) * 0.5;

    memcpy((float *)&pointTarg, finpos, sizeof(pointTarg));
    float dist;
    Relation_T dir;
    Get_Dist_Dir_Pt2PointLine(pointCD, pointBE, pointTarg, &dist, &dir);
    if (dir == PK_ON_LEFT && dist > 0.01)
    {
        rsMid[0] = 0;
        rsMid[1] = dist;
    }
    else if (dir == PK_ON_RIGHT && dist > 0.01)
    {
        rsMid[0] = 0;
        rsMid[1] = -dist;
    }
    else
    {
        rsMid[0] = 0;
        rsMid[1] = 0;
    }

    float newTarget[3], curpos[4], rspos[3];
    Convert(finpos, rsMid, newTarget);

    newTarget[2] = atan2(pointBE.y - pointCD.y, pointBE.x - pointCD.x);

    RTE_PK_Location_Get_CurPos(curpos);
    CoordinadteTransfer(finpos, curpos, rspos);
    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    printf(
        "SetAvmSlotPointB:%06ld:%06ld cur %f %f %f old targ: %f %f %f new targ: %f %f "
        "%f\n",
        stamp.tv_sec, stamp.tv_usec, rspos[0], rspos[1], rspos[2], finpos[0], finpos[1],
        finpos[2], newTarget[0], newTarget[1], newTarget[2]);
    printf("SetAvmSlotPointB: old point: %f %f %f %f %f %f %f %f\n", avmPot.near_rear.x,
           avmPot.near_rear.y, avmPot.far_rear.x, avmPot.far_rear.y, avmPot.far_front.x,
           avmPot.far_front.y, avmPot.near_front.x, avmPot.near_front.y);
#endif
}

/**
 * @brief 向缓存的车位槽位中添加新的车位数据
 *
 * 该函数接收一个新的车位信息，并将其添加到指定的缓存槽位中，更新车位的相关数据。函数会根据车位的
 * 可靠性、上报次数以及限位杆和地锁信息，决定是否继续积累数据或结束上报。
 *
 * @param slotinfo 新的车位信息，包含了车位顶点、停止点、权重等数据。
 * @param slot 缓存中的车位槽位指针，用于存储该车位的累计数据。
 * @return bool 返回是否缓存槽位已积累足够的帧数据（达到最小帧数）。
 */
static bool AddBeData(const VisionSlotInfo &slotinfo, VisionSlotBuff *slot)
{
    // 获取当前缓存车位的帧数
    uint8_t frameNum = slot[0].FrameCount();
    // 如果帧数超过最大帧数，返回true，表示数据已充分积累
    if (frameNum > slot[0].MaxFrame())
    {
        return true;
    }

    // 如果车位已上报且新数据不可靠，同时之前有可靠车位数据，且当前帧数已满足最小帧数
    if (slot[0].ReportCount() == 2 && !slotinfo.isReliable && slot[0].hasReliableSlot > 0)
    {
        if (frameNum >= min_frame_num)
        {
            printf("%s %d: true\n", __FILE__, __LINE__);
            return true; // 车位已报告并且当前数据不可靠，返回true
        }
        else
        {
            return false; // 继续积累数据
        }
    }
    // 如果车位已上报，但这是第一次可靠数据，清空之前的车位数据重新累计
    else if (slot[0].ReportCount() == 2 && slotinfo.isReliable &&
             slot[0].hasReliableSlot == 0)
    {
        slot[0].ClearData();
        slot[0].ReportFrame(a_frame_num);
        slot[0].ReportCount(2);
    }

    // 将车位的顶点信息转换为线段并加入缓存
    LineSeg_T line;
    line.pt1.x = slotinfo.pointOrd[0][0];
    line.pt1.y = slotinfo.pointOrd[0][1];
    line.pt2.x = slotinfo.pointOrd[3][0];
    line.pt2.y = slotinfo.pointOrd[3][1];
    slot[0].beCache.push_back(line); // 添加车位边缘线段到缓存

    line.pt1.x = slotinfo.pointOrd[1][0];
    line.pt1.y = slotinfo.pointOrd[1][1];
    line.pt2.x = slotinfo.pointOrd[2][0];
    line.pt2.y = slotinfo.pointOrd[2][1];
    slot[0].cdCache.push_back(line); // 添加另一条车位边缘线段到缓存

    // 如果这是第一帧数据，直接存储车位顶点；否则进行数据平滑处理
    if (frameNum == 0)
    {
        slot[0].bcde[0] = slot[0].beCache[0].pt1;
        slot[0].bcde[3] = slot[0].beCache[0].pt2;
    }
    else
    {
        // 平滑处理车位顶点信息（加权平均）
        slot[0].bcde[0].x = slot[0].bcde[0].x * 0.9 + slotinfo.pointOrd[0][0] * 0.1;
        slot[0].bcde[0].y = slot[0].bcde[0].y * 0.9 + slotinfo.pointOrd[0][1] * 0.1;
        slot[0].bcde[3].x = slot[0].bcde[3].x * 0.9 + slotinfo.pointOrd[3][0] * 0.1;
        slot[0].bcde[3].y = slot[0].bcde[3].y * 0.9 + slotinfo.pointOrd[3][1] * 0.1;
    }

    // 添加停止点信息到缓存
    line.pt1.x = slotinfo.stopPoint[0][0];
    line.pt1.y = slotinfo.stopPoint[0][1];
    line.pt2.x = slotinfo.stopPoint[1][0];
    line.pt2.y = slotinfo.stopPoint[1][1];
    slot[0].stopCache.push_back(line);

    // 添加车位权重信息到缓存
    slot[0].weights.push_back(slotinfo.weight);

#if 0
    // 更新限位杆和地锁的标志
    if (slot[0].hasStop < slotinfo.stopFlag) { slot[0].hasStop = slotinfo.stopFlag; }
    if (slot[0].hasLock < slotinfo.lockFlag) { slot[0].hasLock = slotinfo.lockFlag; }
#else
    // 计算累计的权重
    float sum_weight = slot[0].getSumWeight();
    // 如果这是第一帧或第二帧数据，初始化限位杆和地锁的计数器
    if (frameNum <= 1)
    {
        slot[0].hasStopNum = 0;
        slot[0].hasLockNum = 0;
    }

    // 对于垂直车位或斜车位，判断是否出现限位杆
    if (slot[0].shape > 2)
    {
        if (slot[0].hasStop < slotinfo.stopFlag)
        {
            slot[0].hasStop = slotinfo.stopFlag;
        }
    }
    // 对于平行车位，限位杆出现比例大于60%时判定有限位杆
    else if (slotinfo.stopFlag > 0)
    {
        slot[0].hasStopNum += slotinfo.weight;
        slot[0].hasStop = (slot[0].hasStopNum / sum_weight > 0.60f) ? 1 : 0;
    }

    slot[0].hasLock = slotinfo.lockFlag;

#endif

    // 如果车位已上报并且此次数据可靠，增加可靠车位计数
    if (slot[0].ReportCount() == 2 && slotinfo.isReliable)
    {
        slot[0].hasReliableSlot++;
    }

    // 更新车位类型
    slot[0].shape = (PK_SlotShapeType)slotinfo.slotType;

    // 返回是否达到帧数要求
    return (slot[0].FrameCount() >= slot[0].ReportFrame());
}

static void CalcParaSlotData(VisionSlotBuff *slot)
{
    Vec2_T beVec = {slot[0].bcde[3].x - slot[0].bcde[0].x,
                    slot[0].bcde[3].y - slot[0].bcde[0].y};
    Vec2_T bcVec = {slot[0].bcde[1].x - slot[0].bcde[0].x,
                    slot[0].bcde[1].y - slot[0].bcde[0].y};
    Vec2_T edVec = {slot[0].bcde[2].x - slot[0].bcde[3].x,
                    slot[0].bcde[2].y - slot[0].bcde[3].y};

    Normalize_Vec2(&beVec);
    Normalize_Vec2(&bcVec);
    Normalize_Vec2(&edVec);

    float cos1 = Get_Dot_Vec2(beVec, bcVec);
    float cos2 = Get_Dot_Vec2(beVec, edVec);

    float distBC = Cal_Dis_Pt2Pt(slot[0].bcde[0], slot[0].bcde[1]);
    float distDE = Cal_Dis_Pt2Pt(slot[0].bcde[2], slot[0].bcde[3]);
    float distW  = (distBC + distDE) * 0.5;
    distW        = rte_min(rte_max(distW, VEHICLE_WID - 0.15), VEHICLE_WID + 1.0);

    if (fabs(cos1) + fabs(cos2) < 0.1)
    {
        float dir         = (Slot_Dir(slot[0].shape) == 1) ? 1.0 : -1.0;
        slot[0].bcde[1].x = -dir * beVec.vy * distW + slot[0].bcde[0].x;
        slot[0].bcde[1].y = dir * beVec.vx * distW + slot[0].bcde[0].y;
        slot[0].bcde[2].x = -dir * beVec.vy * distW + slot[0].bcde[3].x;
        slot[0].bcde[2].y = dir * beVec.vx * distW + slot[0].bcde[3].y;
    }
    else
    {
        bcVec.vx = (slot[0].bcde[1].x + slot[0].bcde[2].x - slot[0].bcde[0].x -
                    slot[0].bcde[3].x) *
                   0.5;
        bcVec.vy = (slot[0].bcde[1].y + slot[0].bcde[2].y - slot[0].bcde[0].y -
                    slot[0].bcde[3].y) *
                   0.5;
        Normalize_Vec2(&bcVec);

        slot[0].bcde[1].x = bcVec.vx * distW + slot[0].bcde[0].x;
        slot[0].bcde[1].y = bcVec.vy * distW + slot[0].bcde[0].y;
        slot[0].bcde[2].x = bcVec.vx * distW + slot[0].bcde[3].x;
        slot[0].bcde[2].y = bcVec.vy * distW + slot[0].bcde[3].y;
    }

    float distBE = Cal_Dis_Pt2Pt(slot[0].bcde[0], slot[0].bcde[3]);
    float distCD = Cal_Dis_Pt2Pt(slot[0].bcde[1], slot[0].bcde[2]);
    float distH  = (distBE + distCD) * 0.5;
    if (distH < VEHICLE_LEN + 0.1)
    {
        distH    = VEHICLE_LEN + 0.4;
        beVec.vx = slot[0].bcde[3].x - slot[0].bcde[0].x;
        beVec.vy = slot[0].bcde[3].y - slot[0].bcde[0].y;
        Normalize_Vec2(&beVec);

        if (slot[0].hasStop)
        {
            slot[0].bcde[3].x = beVec.vx * distH + slot[0].bcde[0].x;
            slot[0].bcde[3].y = beVec.vy * distH + slot[0].bcde[0].y;
            slot[0].bcde[2].x = beVec.vx * distH + slot[0].bcde[1].x;
            slot[0].bcde[2].y = beVec.vy * distH + slot[0].bcde[1].y;
        }
        else
        {
            slot[0].bcde[0].x = -beVec.vx * distH + slot[0].bcde[3].x;
            slot[0].bcde[0].y = -beVec.vy * distH + slot[0].bcde[3].y;
            slot[0].bcde[1].x = -beVec.vx * distH + slot[0].bcde[2].x;
            slot[0].bcde[1].y = -beVec.vy * distH + slot[0].bcde[2].y;
        }
    }

    // printf("b = %06f %06f, c = %06f %06f, d = %06f %06f e = %06f %06f\n",
    //     slot[0].bcde[0].x, slot[0].bcde[0].y, slot[0].bcde[1].x, slot[0].bcde[1].y,
    //     slot[0].bcde[2].x, slot[0].bcde[2].y, slot[0].bcde[3].x, slot[0].bcde[1].y);

    slot[0].timestamp = RTE_BSW_Get_CurTime();
    slot[0].ReportCount(1);
}

static void CalcVertSlotData1(VisionSlotBuff *slot)
{
    Vec2_T bcVec = {slot[0].bcde[1].x - slot[0].bcde[0].x,
                    slot[0].bcde[1].y - slot[0].bcde[0].y};
    Normalize_Vec2(&bcVec);

    float distBE = Cal_Dis_Pt2Pt(slot[0].bcde[0], slot[0].bcde[3]);
    float distBC = Cal_Dis_Pt2Pt(slot[0].bcde[0], slot[0].bcde[1]);
    float distDE = Cal_Dis_Pt2Pt(slot[0].bcde[2], slot[0].bcde[3]);
    float distD  = (distBC + distDE) * 0.5;
    distD        = rte_min(rte_max(distD, VEHICLE_LEN), VEHICLE_LEN + 0.1);

    float dir    = (Slot_Dir(slot[0].shape) == 1) ? 1.0 : -1.0;
    Vec2_T beVec = {dir * bcVec.vy, -dir * bcVec.vx};

    distBE            = rte_max(distBE, VEHICLE_WID + 0.1);
    slot[0].bcde[3].x = distBE * beVec.vx + slot[0].bcde[0].x;
    slot[0].bcde[3].y = distBE * beVec.vy + slot[0].bcde[0].y;

    slot[0].bcde[1].x = distD * bcVec.vx + slot[0].bcde[0].x;
    slot[0].bcde[1].y = distD * bcVec.vy + slot[0].bcde[0].y;
    slot[0].bcde[2].x = distD * bcVec.vx + slot[0].bcde[3].x;
    slot[0].bcde[2].y = distD * bcVec.vy + slot[0].bcde[3].y;

    // printf("b = %06f %06f, c = %06f %06f, d = %06f %06f e = %06f %06f\n",
    //     slot[0].bcde[0].x, slot[0].bcde[0].y, slot[0].bcde[1].x, slot[0].bcde[1].y,
    //     slot[0].bcde[2].x, slot[0].bcde[2].y, slot[0].bcde[3].x, slot[0].bcde[1].y);

    slot[0].timestamp = RTE_BSW_Get_CurTime();
    slot[0].ReportCount(1);
}

static void CalcVertSlotData2(VisionSlotBuff *slot)
{
    Vec2_T beVec = {slot[0].bcde[3].x - slot[0].bcde[0].x,
                    slot[0].bcde[3].y - slot[0].bcde[0].y};
    Vec2_T bcVec = {slot[0].bcde[1].x - slot[0].bcde[0].x,
                    slot[0].bcde[1].y - slot[0].bcde[0].y};

    Normalize_Vec2(&beVec);
    Normalize_Vec2(&bcVec);

    float distBE = Cal_Dis_Pt2Pt(slot[0].bcde[0], slot[0].bcde[3]);
    float distBC = Cal_Dis_Pt2Pt(slot[0].bcde[0], slot[0].bcde[1]);
    float distDE = Cal_Dis_Pt2Pt(slot[0].bcde[2], slot[0].bcde[3]);
    float distD  = (distBC + distDE) * 0.5;
    distD        = rte_min(rte_max(distD, VEHICLE_LEN), VEHICLE_LEN + 0.1);
    distBE       = rte_max(distBE, VEHICLE_WID + 0.1);

    Vec2_T be2 = {beVec.vx + bcVec.vx, beVec.vy + bcVec.vy};
    Normalize_Vec2(&be2);

    float dir = (Slot_Dir(slot[0].shape) == 1) ? 1.0 : -1.0;
    beVec.vx  = be2.vx * 0.707106781 + be2.vy * dir * 0.707106781;
    beVec.vy  = -be2.vx * dir * 0.707106781 + be2.vy * 0.707106781;
    bcVec.vx  = be2.vx * 0.707106781 - be2.vy * dir * 0.707106781;
    bcVec.vy  = be2.vx * dir * 0.707106781 + be2.vy * 0.707106781;

    slot[0].bcde[3].x = beVec.vx * distBE + slot[0].bcde[0].x;
    slot[0].bcde[3].y = beVec.vy * distBE + slot[0].bcde[0].y;
    slot[0].bcde[1].x = bcVec.vx * distD + slot[0].bcde[0].x;
    slot[0].bcde[1].y = bcVec.vy * distD + slot[0].bcde[0].y;
    slot[0].bcde[2].x = bcVec.vx * distD + slot[0].bcde[3].x;
    slot[0].bcde[2].y = bcVec.vy * distD + slot[0].bcde[3].y;

    // printf("b = %06f %06f, c = %06f %06f, d = %06f %06f e = %06f %06f\n",
    //     slot[0].bcde[0].x, slot[0].bcde[0].y, slot[0].bcde[1].x, slot[0].bcde[1].y,
    //     slot[0].bcde[2].x, slot[0].bcde[2].y, slot[0].bcde[3].x, slot[0].bcde[1].y);

    slot[0].timestamp = RTE_BSW_Get_CurTime();
    slot[0].ReportCount(1);
}

/**
 * @brief 报告 AVM 车位信息，进行泊车状态或车位搜索状态下的车位处理
 *
 * 根据当前泊车状态，对车位数据进行报告。如果车位处于泊车或车位搜索状态，并满足相应的条件，
 * 则会更新车位权重和位置，并清空车位缓存数据。
 *
 * @param slot 输入的 VisionSlotInfo 车位数据
 * @param avmSlot 输出的 AVM_SlotPointsType，存储车位的点信息
 * @param avmflog 文件指针，日志记录使用
 * @return int 返回 1 表示报告成功，返回 0 表示无有效车位或处理失败
 */
int ReportAvmSlot(VisionSlotInfo &slot, AVM_SlotPointsType &avmSlot, FILE *avmflog)
{
    // 提取车位的两个边界点坐标
    float bexys[2][2]     = {{slot.pointOrd[0][0], slot.pointOrd[0][1]},
                             {slot.pointOrd[3][0], slot.pointOrd[3][1]}};
    VisionSlotBuff *pSlot = FindAvmSlot(bexys, FindBuf1); // 查找车位缓存

    // 如果未找到车位缓存，返回 0
    if (pSlot == NULL)
    {
        return 0;
    }

    // 检查车位有效性
    uint8_t reportNum = pSlot[0].ReportCount();
    if (reportNum != 1)
    {
        pSlot[0].ClearData(); // 清除车位缓存数据
        return 0;
    }

    // 如果泊车模块当前状态为泊车执行中
    if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() == MCOM_ON_PARKING)
    {
        // 检查是否在车位侧方区域，且当前权重大于上次权重
        if (InSideErea() && pSlot[0].getSumWeight() > pSlot[0].last_sum_weight)
        {
            SetAvmSlotPointB(pSlot, avmflog); // 设置车位点信息

            pSlot[0].last_sum_weight = pSlot[0].getSumWeight(); // 更新上次权重
            pSlot[0].ClearData();                               // 清除车位缓存数据
            return 0;
        }
        // 如果在车位后区域，帧数达到报告帧数，直接设置车位点信息
        else if (InRearErea())
        {
            SetAvmSlotPointB(pSlot, avmflog); // 设置车位点信息
            pSlot[0].ClearData();             // 清除车位缓存数据
            return 0;
        }
    }
    // 如果车位搜索过程中，且当前权重大于上次权重
    else // if (pSlot[0].getSumWeight() > pSlot[0].last_sum_weight)
    {
        SetAvmSlotPoint(pSlot, avmSlot); // 设置车位点信息

        // 如果帧数达到报告帧数
        pSlot[0].last_sum_weight = pSlot[0].getSumWeight(); // 更新权重
        if (pSlot[0].hasReliableSlot == 0)
        {
            pSlot[0].ClearData(); // 清除车位缓存数据
        }
        pSlot[0].ReportFrame(32); // 重置报告帧数
        pSlot[0].ReportCount(2);  // 更新报告状态
        return 1;                 // 返回 1 表示报告成功
    }

    return 0; // 返回 0 表示无有效车位或处理失败
}

/**
 * @brief 缓存AVM（环视系统）车位信息
 *
 * 该函数的主要目的是将传入的视觉车位信息存储到缓存中，并根据车位形状类型（平行车位或垂直车位）计算其相关数据。
 *
 * @param slot 传入的视觉车位信息，包含车位的角点坐标和其他状态信息。
 * @return int 返回1表示成功缓存车位信息并完成处理，返回0表示未能成功缓存或处理。
 */
int CacheAvmSlot(const VisionSlotInfo &slot)
{
    // 提取车位的两个顶点（bexys数组），用于查找缓存中的对应车位。
    float bexys[2][2] = {{slot.pointOrd[0][0], slot.pointOrd[0][1]},
                         {slot.pointOrd[3][0], slot.pointOrd[3][1]}};

    // 在缓存中查找与传入车位坐标匹配的车位信息
    VisionSlotBuff *pSlot = FindAvmSlot(bexys, FindBuf1);
    // 如果未找到匹配的车位信息，则返回0，表示缓存失败
    if (pSlot == NULL)
    {
        return 0;
    }

    // 将新车位数据加入缓存，如果缓存未满，则返回0
    bool full = AddBeData(slot, pSlot);
    if (!full)
    {
        return 0;
    }

    // 计算或整理数据,获取最终车位数据,如果失败，则返回0
    int result = BuildBeData(pSlot);
    if (result != AVM_SUCCESS)
    {
        return 0;
    }

    // 根据车位的形状类型，调用相应的计算函数
    if (pSlot[0].shape == PK_SLOT_LEFT_PARA || pSlot[0].shape == PK_SLOT_RIGHT_PARA)
    {
        // 如果车位是平行车位（左侧或右侧）
        CalcParaSlotData(pSlot);
    }
    else
    {
        // 如果车位是垂直车位
        CalcVertSlotData1(pSlot);
    }

    // 返回1表示成功缓存车位信息并完成处理
    return 1;
}

int UpdateAvmSlot(AVM_SlotPointsType &avmSlot)
{
    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);

    float bexys[2][2] = {{curPos[0], curPos[1]}, {curPos[2], curPos[3]}};

    VisionSlotBuff *pSlot = FindAvmSlot(bexys, FindBuf2);
    if (pSlot == NULL)
    {
        return 0;
    }

    uint8_t frameNum = pSlot[0].FrameCount();

    if (frameNum == 0)
    {
        return 0;
    }

    if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() != MCOM_ON_PARKING)
    {
        if (frameNum < min_frame_num)
        {
            log_warn("discard detected slot! view number: %d b:%06f %06f e:%06f %06f\n",
                     frameNum, pSlot->bcde[0].x, pSlot->bcde[0].y, pSlot->bcde[3].x,
                     pSlot->bcde[3].y);
            pSlot[0].ClearData(); // log_warn("clear VisionSlotBuff\n");
            return 0;
        }
    }

    if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() == MCOM_ON_PARKING)
    {
        float target[3];
        RTE_PK_SlotDetect_Get_TargPos(target);
        if (Project_PosTo1st_rx(target, curPos) > 1.5 * REAR_SUSPENSION)
        {
            return 0;
        }

        if (frameNum < 3 * min_frame_num)
        {
            log_warn("discard detected slot! view number: %d b:%06f %06f e:%06f %06f\n",
                     frameNum, pSlot->bcde[0].x, pSlot->bcde[0].y, pSlot->bcde[3].x,
                     pSlot->bcde[3].y);
            pSlot[0].ClearData();
            return 0;
        }
    }

    int result = BuildBeData(pSlot);
    if (result == 0)
    {
        if (pSlot[0].shape == PK_SLOT_LEFT_PARA || pSlot[0].shape == PK_SLOT_RIGHT_PARA)
        {
            CalcParaSlotData(pSlot);
        }
        else
        {
            CalcVertSlotData1(pSlot);
        }

        // printf("UpdateAvmSlot avm slot b: %06f %06f, c: %06f %06f d: %06f %06f e: %06f
        // %06f count = %d type = %d\n",
        //     pSlot[0].bcde[0].x, pSlot[0].bcde[0].y, pSlot[0].bcde[1].x,
        //     pSlot[0].bcde[1].y, pSlot[0].bcde[2].x, pSlot[0].bcde[2].y,
        //     pSlot[0].bcde[3].x, pSlot[0].bcde[3].y, pSlot[0].FrameCount(),
        //     pSlot[0].shape);

        if (RTE_PK_StateManage_Get_ModuleCom_PathExecute() == MCOM_ON_PARKING)
        {
            SetAvmSlotPointB(pSlot, NULL);
        }
        else
        {
            SetAvmSlotPoint(pSlot, avmSlot);
        }

        pSlot[0].ClearData();
        return 1;
    }
    else
    {
        pSlot[0].ClearData();
        return 0;
    }
}

static void SwitchPoints(float pointOrd[4][2])
{
    float temp[2];
    memcpy(temp, pointOrd[0], sizeof(temp));
    memcpy(pointOrd[0], pointOrd[1], sizeof(temp));
    memcpy(pointOrd[1], pointOrd[2], sizeof(temp));
    memcpy(pointOrd[2], pointOrd[3], sizeof(temp));
    memcpy(pointOrd[3], temp, sizeof(temp));
}

Relation_T AvmSlotDir(const float pointOrd[4][2])
{
    if (pointOrd[0][0] < ZERO_FLOAT && pointOrd[1][0] < ZERO_FLOAT &&
        pointOrd[2][0] < ZERO_FLOAT && pointOrd[3][0] < ZERO_FLOAT)
    {
        return PK_ON_LEFT;
    }
    else if (pointOrd[0][0] > ZERO_FLOAT && pointOrd[1][0] > ZERO_FLOAT &&
             pointOrd[2][0] > ZERO_FLOAT && pointOrd[3][0] > ZERO_FLOAT)
    {
        return PK_ON_RIGHT;
    }
    else
    {
        return PK_INSIDE;
    }
}

static int CheckAngleSlot(VisionSlotInfo &slot)
{
    if (AvmSlotDir(slot.pointOrd) == PK_ON_LEFT)
    {
        slot.slotType = PK_SLOT_LEFT_VERT;
    }
    else if (AvmSlotDir(slot.pointOrd) == PK_ON_RIGHT)
    {
        slot.slotType = PK_SLOT_RIGHT_VERT;
    }
    else
    {
        return 1;
    }

    float avmpoint[4][2];
    memcpy(avmpoint, slot.pointOrd, sizeof(avmpoint));

    Vec2_T be = {avmpoint[3][0] - avmpoint[0][0], avmpoint[3][1] - avmpoint[0][1]};
    Vec2_T bc = {avmpoint[1][0] - avmpoint[0][0], avmpoint[1][1] - avmpoint[0][1]};
    Vec2_T de = {avmpoint[3][0] - avmpoint[2][0], avmpoint[3][1] - avmpoint[2][1]};
    Normalize_Vec2(&bc);
    Normalize_Vec2(&de);
    Normalize_Vec2(&be);

    Vec2_T vert = {de.vx - bc.vx, de.vy - bc.vy};
    Normalize_Vec2(&vert);

    float dir    = (Slot_Dir(slot.slotType) == 1) ? 1.0 : -1.0;
    Vec2_T beVec = {-dir * vert.vy, dir * vert.vx};

    float distBC = Cal_Dis_Pt2Pt(avmpoint[0], avmpoint[1]);
    float distDE = Cal_Dis_Pt2Pt(avmpoint[2], avmpoint[3]);
    float distD  = (distBC + distDE) * 0.5;
    distD        = rte_min(rte_max(distD, VEHICLE_LEN), VEHICLE_LEN + 0.1);

    float distEBC = PointToLineDist(avmpoint[0], avmpoint[1], avmpoint[3]);
    float distBDE = PointToLineDist(avmpoint[2], avmpoint[3], avmpoint[0]);
    float distW   = (distEBC + distBDE) * 0.5;

    float cross = Get_Dot_Vec2(be, bc);
    int beindex = 0;
    if (sign(cross) < 0)
    {
        avmpoint[3][0] = distW * beVec.vx + avmpoint[0][0];
        avmpoint[3][1] = distW * beVec.vy + avmpoint[0][1];
        beindex        = 3;
    }
    else
    {
        avmpoint[0][0] = -distW * beVec.vx + avmpoint[3][0];
        avmpoint[0][1] = -distW * beVec.vy + avmpoint[3][1];
        beindex        = 0;
    }

    avmpoint[1][0] = -distD * vert.vx + avmpoint[0][0];
    avmpoint[1][1] = -distD * vert.vy + avmpoint[0][1];
    avmpoint[2][0] = -distD * vert.vx + avmpoint[3][0];
    avmpoint[2][1] = -distD * vert.vy + avmpoint[3][1];

    float dist[3];
    LineSeg_T line;
    dist[0] = Cal_Dis_Pt2Pt(avmpoint[3], avmpoint[0]);
    memcpy(&line, &avmpoint[0][0], sizeof(line));
    Get_Dist_Dir_Pt2SegLine(line, {avmpoint[3][0], avmpoint[3][1]}, &dist[1], 0);

    memcpy(&line, &avmpoint[2][0], sizeof(line));
    Get_Dist_Dir_Pt2SegLine(line, {avmpoint[0][0], avmpoint[0][1]}, &dist[2], 0);

    float offset = distW * cosf(slot.slotAngle * PI_RAD);
    if (avmpoint[beindex][1] > AVM_VERT_START_START + offset + 1.5 ||
        avmpoint[0][1] < offset - 0.8)
    {
        return 12;
    }
    else if (avmpoint[3][1] < 0)
    {
        slot.status = 1;
    }
    else if (dist[0] > 2.8 || dist[0] < MIN_AVM_PARK_LEN_VERTICAL ||
             dist[1] < MIN_AVM_PARK_LEN_VERTICAL || dist[2] < MIN_AVM_PARK_LEN_VERTICAL)
    {
        log_warn("angle invalid wide:angle:%d wide:%03f,%03f(2.1),%03f(2.1)\n",
                 slot.slotAngle, dist[0], dist[1], dist[2]);
        return 13;
    }

    memcpy(slot.pointOrd, avmpoint, sizeof(avmpoint));
    return AVM_SUCCESS;
}

static int CheckVertSlot(VisionSlotInfo &slot)
{
    if (AvmSlotDir(slot.pointOrd) == PK_ON_LEFT)
    {
        slot.slotType = PK_SLOT_LEFT_VERT;
    }
    else if (AvmSlotDir(slot.pointOrd) == PK_ON_RIGHT)
    {
        slot.slotType = PK_SLOT_RIGHT_VERT;
    }
    else
    {
        return 1;
    }

    if (slot.pointOrd[0][1] > slot.pointOrd[3][1] ||
        slot.pointOrd[1][1] > slot.pointOrd[2][1])
    {
        return 2;
    }

    LineSeg_T line;
    float dist[3];
    dist[0] = Cal_Dis_Pt2Pt(slot.pointOrd[3], slot.pointOrd[0]);
    memcpy(&line, &slot.pointOrd[0][0], sizeof(line));
    Get_Dist_Dir_Pt2SegLine(line, {slot.pointOrd[3][0], slot.pointOrd[3][1]}, &dist[1],
                            0);

    memcpy(&line, &slot.pointOrd[2][0], sizeof(line));
    Get_Dist_Dir_Pt2SegLine(line, {slot.pointOrd[0][0], slot.pointOrd[0][1]}, &dist[2],
                            0);

    if (slot.pointOrd[0][1] > AVM_VERT_START_START)
    {
        return 11;
    }
    else if ((slot.pointOrd[0][1] + slot.pointOrd[3][1]) * 0.5 <
             (avm_vert_slot_min_dx + REAR_VIEW_MIRROR_X)) // AVM_VERT_START_END)
    {
        // #ifdef ANDROID_PLATFORM
        //         slot.status = 0;
        // #else
        slot.status = 1;
        // #endif
    }
    else if (dist[0] > 3.68 || dist[0] < MIN_AVM_PARK_LEN_VERTICAL ||
             dist[1] < MIN_AVM_PARK_LEN_VERTICAL || dist[2] < MIN_AVM_PARK_LEN_VERTICAL)
    {
        log_warn("vert invalid wide: %03f,%03f,%03f\n", dist[0], dist[1], dist[2]);
        return 12;
    }

    float avmpoint[4][2];
    memcpy(avmpoint, slot.pointOrd, sizeof(avmpoint));

    Vec2_T cb = {avmpoint[0][0] - avmpoint[1][0], avmpoint[0][1] - avmpoint[1][1]};
    Vec2_T de = {avmpoint[3][0] - avmpoint[2][0], avmpoint[3][1] - avmpoint[2][1]};
    Normalize_Vec2(&cb);
    Normalize_Vec2(&de);

    Vec2_T vert = {de.vx + cb.vx, de.vy + cb.vy};
    Normalize_Vec2(&vert);

    float dir    = (Slot_Dir(slot.slotType) == 1) ? 1.0 : -1.0;
    Vec2_T beVec = {-dir * vert.vy, dir * vert.vx};

    float distBC = Cal_Dis_Pt2Pt(avmpoint[0], avmpoint[1]);
    float distDE = Cal_Dis_Pt2Pt(avmpoint[2], avmpoint[3]);
    float distD  = (distBC + distDE) * 0.5;
    distD        = rte_min(rte_max(distD, VEHICLE_LEN), VEHICLE_LEN + 0.1);

    float distEBC = PointToLineDist(avmpoint[0], avmpoint[1], avmpoint[3]);
    float distBDE = PointToLineDist(avmpoint[2], avmpoint[3], avmpoint[0]);
    float distW   = (distEBC + distBDE) * 0.5;

    float temp[2];
    temp[0] = 0.5 * (avmpoint[0][0] + avmpoint[3][0]);
    temp[1] = 0.5 * (avmpoint[0][1] + avmpoint[3][1]);

    avmpoint[0][0] = temp[0] - 0.5 * distW * beVec.vx;
    avmpoint[0][1] = temp[1] - 0.5 * distW * beVec.vy;
    avmpoint[3][0] = temp[0] + 0.5 * distW * beVec.vx;
    avmpoint[3][1] = temp[1] + 0.5 * distW * beVec.vy;

    avmpoint[1][0] = -distD * vert.vx + avmpoint[0][0];
    avmpoint[1][1] = -distD * vert.vy + avmpoint[0][1];
    avmpoint[2][0] = -distD * vert.vx + avmpoint[3][0];
    avmpoint[2][1] = -distD * vert.vy + avmpoint[3][1];

    memcpy(slot.pointOrd, avmpoint, sizeof(avmpoint));
    return AVM_SUCCESS;
}

static int CheckParaSlot(VisionSlotInfo &slot)
{
    if (AvmSlotDir(slot.pointOrd) == PK_ON_LEFT)
    {
        slot.slotType = PK_SLOT_LEFT_PARA;
    }
    else if (AvmSlotDir(slot.pointOrd) == PK_ON_RIGHT)
    {
        slot.slotType = PK_SLOT_RIGHT_PARA;
    }
    else
    {
        return 1;
    }

    if (slot.pointOrd[0][1] > slot.pointOrd[3][1] ||
        slot.pointOrd[1][1] > slot.pointOrd[2][1])
    {
        return 2;
    }

    LineSeg_T line;

    float dist[3];
    dist[0] = Cal_Dis_Pt2Pt(slot.pointOrd[3], slot.pointOrd[0]);
    memcpy(&line, &slot.pointOrd[0][0], sizeof(line));
    Get_Dist_Dir_Pt2SegLine(line, {slot.pointOrd[3][0], slot.pointOrd[3][1]}, &dist[1],
                            0);

    memcpy(&line, &slot.pointOrd[2][0], sizeof(line));
    Get_Dist_Dir_Pt2SegLine(line, {slot.pointOrd[0][0], slot.pointOrd[0][1]}, &dist[2],
                            0);

    if (slot.pointOrd[0][1] > 1.0)
    {
        return 10;
    }
    else if (slot.pointOrd[0][1] < -3.0)
    {
        slot.status = PK_SLOT_NO;
    }
    else if (dist[0] < VEHICLE_LEN - 0.3 || dist[1] < VEHICLE_LEN - 0.5 ||
             dist[2] < VEHICLE_LEN - 0.5)
    {
        log_warn("para invalid wide: %03f,%03f,%03f\n", slot.slotIndex, dist[0], dist[1],
                 dist[2]);
        return 11;
    }

    return AVM_SUCCESS;
}

int CheckSearchSlot(VisionSlotInfo &slot)
{
    int result = AVM_SUCCESS;

    float avmpoint[4][2];
    memcpy(avmpoint, slot.pointOrd, sizeof(avmpoint));

    if (slot.stopFlag == 1 && CheckStopPoint(slot) == 0)
    {
        slot.stopFlag = 0;
        // log_warn("avm stop error %d! stamp:%d id:%d type:%d angle:%d
        // bcde:%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f\n",
        //     result, slot.timeStamp, slot.slotIndex, slot.slotType, slot.slotAngle,
        //     avmpoint[0][0], avmpoint[0][1], avmpoint[1][0], avmpoint[1][1],
        //     avmpoint[2][0], avmpoint[2][1], avmpoint[3][0], avmpoint[3][1],
        //     slot.stopPoint[0][0], slot.stopPoint[0][1], slot.stopPoint[1][0],
        //     slot.stopPoint[1][1]);
    }

    slot.status = 0;
    if ((slot.stopFlag == 1) && (result = CheckSlotEntry(slot)) != AVM_SUCCESS)
    {
        // log_warn("avm entry error %d! stamp:%d id:%d type:%d angle:%d
        // bcde:%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f\n",
        //     result, slot.timeStamp, slot.slotIndex, slot.slotType, slot.slotAngle,
        //     avmpoint[0][0], avmpoint[0][1], avmpoint[1][0], avmpoint[1][1],
        //   /  avmpoint[2][0], avmpoint[2][1], avmpoint[3][0], avmpoint[3][1],
        //     slot.stopPoint[0][0], slot.stopPoint[0][1], slot.stopPoint[1][0],
        //     slot.stopPoint[1][1]);
        return 1;
    }

    if (slot.slotAngle < 80)
    {
        result = CheckAngleSlot(slot);
        if (result != AVM_SUCCESS)
        {
            result = AVM_FAIL + 0x100;
        }
    }
    else
    {
        float distrnfn = Cal_Dis_Pt2Pt(slot.pointOrd[0], slot.pointOrd[3]);
        float distrnrf = Cal_Dis_Pt2Pt(slot.pointOrd[0], slot.pointOrd[1]);
        if (distrnfn < 3.5 && distrnrf > 4.1)
        {
#ifdef ANDROID_PLATFORM
            result = CheckVertSlot(slot);
            if (result != AVM_SUCCESS)
            {
                result = AVM_FAIL + 0x200;
            }
#else
            if (fabs(slot.pointOrd[0][0] - slot.pointOrd[1][0]) > 4.1) // y
            {
                result = CheckVertSlot(slot);
                if (result != AVM_SUCCESS)
                {
                    result = AVM_FAIL + 0x200;
                }
            }
            else
            {
                SwitchPoints(slot.pointOrd);
                result = CheckParaSlot(slot);
                if (result != AVM_SUCCESS)
                {
                    result += 0x300;
                }
            }
#endif
        }
        else if (distrnrf < 3.5 && distrnfn > 4.1)
        {
            result = CheckParaSlot(slot);
            if (result != AVM_SUCCESS)
            {
                result += 0x400;
            }
        }
        else
        {
            result = 0x500;
        }
    }

    if (result != AVM_SUCCESS)
    {
        // log_warn("avm points error 0x%04x! stamp:%d type:%d id:%d angle:%d
        // bcde:%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f\n",
        //     result, slot.timeStamp, slot.slotType, slot.slotIndex, slot.slotAngle,
        //     avmpoint[0][0], avmpoint[0][1], avmpoint[1][0], avmpoint[1][1],
        //     avmpoint[2][0], avmpoint[2][1], avmpoint[3][0], avmpoint[3][1],
        //     slot.stopPoint[0][0], slot.stopPoint[0][1], slot.stopPoint[1][0],
        //     slot.stopPoint[1][1]);
        return 3;
    }

    // BE point middle.
    if (fabs(avmpoint[0][0] + avmpoint[3][0]) / 2 > 5.2)
    {
        // printf("invalid slot too remote:stamp:%d id:%d type:%d angle:%d
        // bcde:%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f\n",
        //     slot.timeStamp, slot.slotIndex, slot.slotType, slot.slotAngle,
        //     slot.pointOrd[0][0], slot.pointOrd[0][1], slot.pointOrd[1][0],
        //     slot.pointOrd[1][1], slot.pointOrd[2][0], slot.pointOrd[2][1],
        //     slot.pointOrd[3][0], slot.pointOrd[3][1], slot.stopPoint[0][0],
        //     slot.stopPoint[0][1], slot.stopPoint[1][0], slot.stopPoint[1][1]);
        return 4;
    }
    else
    {
        // printf("valid slot:stamp:%d id %d type:%d angle:%d
        // bcde:%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f,%03f\n",
        //     slot.timeStamp, slot.slotIndex, slot.slotType, slot.slotAngle,
        //     slot.pointOrd[0][0], slot.pointOrd[0][1], slot.pointOrd[1][0],
        //     slot.pointOrd[1][1], slot.pointOrd[2][0], slot.pointOrd[2][1],
        //     slot.pointOrd[3][0], slot.pointOrd[3][1],
        // slot.stopPoint[0][0], slot.stopPoint[0][1], slot.stopPoint[1][0],
        // slot.stopPoint[1][1]);
    }

    return AVM_SUCCESS;
}

void CalcVertSlotData_UnitTest()
{
    float bcde[8]          = {2.454597, -2.333948, 2.335419, -7.364126,
                              4.718010, -7.444426, 4.776243, -2.391346};
    PK_SlotShapeType shape = PK_SLOT_RIGHT_VERT;
    // float bcde[8] =
    // {6.346544,5.147795,6.548069,10.024709,9.069712,10.040665,8.878008,5.160446};
    // PK_SlotShapeType shape = PK_SLOT_LEFT_VERT;

    VisionSlotBuff slot;
    memcpy(slot.bcde, bcde, sizeof(bcde));
    slot.shape = shape;
    CalcVertSlotData1(&slot);
    memcpy(slot.bcde, bcde, sizeof(bcde));
    CalcVertSlotData2(&slot);
}

/**
 * @brief 判断车辆正后方有效车位范围
 *
 * 用于泊车过程中，最后一段路径时校正车位，在有效范围内的车位数据会用于更新车位坐标
 *
 * @param
 * @return bool true - 在范围内 false - 不在范围内，舍弃
 */
bool InRearErea()
{
    float target[3], curpos[4], rspoint[3];
    RTE_PK_SlotDetect_Get_TargPos(target);
    RTE_PK_Location_Get_CurPos(curpos);
    CoordinadteTransfer(target, curpos, rspoint);

    if (fabs(rspoint[2]) > 15 * PI_RAD || fabs(rspoint[1]) > 0.6)
    {
        return false;
    }
    if (fabs(rspoint[0]) > 2.5 || fabs(rspoint[0]) < 0.5)
    {
        return false;
    }

    RTE_BSW_GearType gear = RTE_BSW_Get_CurrentGear();
    return (GEAR_REAL_R == gear && sign(rspoint[0]) == 1) ||
           (GEAR_REAL_D == gear && sign(rspoint[0]) == -1);
    return true;
}

/**
 * @brief 判断车辆侧方有效车位范围
 *
 * 用于提前释放车位，在有效范围内的车位数据会用于更新车位坐标
 *
 * @param
 * @return bool true - 在范围内 false - 不在范围内，舍弃
 */
bool InSideErea()
{
    float target[3], curpos[4], rstarget[3];
    RTE_PK_SlotDetect_Get_TargPos(target);
    RTE_PK_Location_Get_CurPos(curpos);
    PK_SlotShapeType shape = RTE_PK_SlotDetect_Get_SlotShape();
    CoordinadteTransfer(target, curpos, rstarget);
    if (fabs(rstarget[0]) < 7.5 && cosf(rstarget[2]) < cosf(80 * PI_RAD))
    {
        if (shape == PK_SLOT_LEFT_VERT && rstarget[1] > -1.9 && rstarget[1] < -0.5)
        {
            return true;
        }
        if (shape == PK_SLOT_RIGHT_VERT && rstarget[1] > 0.5 && rstarget[1] < 1.9)
        {
            return true;
        }
    }

    return false;
}