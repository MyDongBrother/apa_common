/**
 * @file MathFunc.c
 * @brief 常用数学运算函数实现（平方、立方、角度处理等）
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @note 所有函数都是地址无关代码（纯计算逻辑函数）
 * @date 2025-09-10
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-10 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "MathFunc.h"
#include "PK_Utility.h"
#include "SystemPara.h"
#include "Record_Log.h"

/***********************************************
平方运算函数
Pow2
INTPUT:     f - 输入
OUTPUT:     输出平方值
***********************************************/
float pow2(float f) { return f * f; }

/***********************************************
立方运算函数
Pow3
INTPUT:     f - 输入
OUTPUT:     输出立方值
***********************************************/
float pow3(float f) { return f * f * f; }
/***********************************************
对弧度角度取[-PI,PI]范围内值
Round_PI
INTPUT:     输入 任意角度
OUTPUT:     输出[-PI,PI]范围内值
***********************************************/
float Round_PI(float theta)
{
    if (fabs(theta) > 1e4)
    {
        log_err("Project_PosTo1st_rtheta Err!\n");
    }
    auto value = theta;
    if (fabs(value) <= PI)
    {
        return value;
    }
    while (value > PI)
    {
        value -= 2.0 * PI;
    }
    while (value < -PI)
    {
        value += 2.0 * PI;
    }
    return value;
}

float Get_Segment_Len(LineSeg_T seg)
{
    return sqrtf(pow2(seg.pt1.x - seg.pt2.x) + pow2(seg.pt1.y - seg.pt2.y));
}
/***********************************************
对两个弧度角求其平均值（向量劣弧的角平分线方向）
Round_PI
INTPUT:     输入
OUTPUT:     输出[-PI,PI]范围内值
***********************************************/
float GetAverAngle(float Theta1, float Theta2)
{
    float Angle;
    Theta1 = Round_PI(Theta1);
    Theta2 = Round_PI(Theta2);
    if (fabs(Theta1 - Theta2) > PI)
    {
        if (Theta1 < 0)
            Theta1 = 2 * PI + Theta1;
        else
            Theta2 = 2 * PI + Theta2;
    }
    Angle = Round_PI((Theta1 + Theta2) / 2.0f);
    return Angle;
}

float Get_AverAngle_W(float Theta1, float Theta2, float w1, float w2)
{
    float Angle;
    Theta1 = Round_PI(Theta1);
    Theta2 = Round_PI(Theta2);
    if (fabsf(Theta1 - Theta2) > PI)
    {
        if (Theta1 < 0)
            Theta1 = 2 * PI + Theta1;
        else
            Theta2 = 2 * PI + Theta2;
    }
    Angle = Theta1 * w1 + Theta2 * w2;
    return Round_PI(Angle);
}

/*
坐标变换函数
CoordinadteTransfer
INTPUT:     cd0 - 输入坐标 x0 y0 theta0
            cd1 - 输入坐标 x1 y1 theta1
OUTPUT:     cd - 输出坐标 x y theta : cd1 在 cd0 下的投影
***********************************************/
void CoordinadteTransfer(const float *cd0, const float *cd1, float *cd)
{
    float dx    = cd1[0] - cd0[0];
    float dy    = cd1[1] - cd0[1];
    float c     = cosf(cd0[2]);
    float s     = sinf(cd0[2]);
    float theta = cd1[2] - cd0[2];
    cd[0]       = dx * c + dy * s;
    cd[1]       = dy * c - dx * s;
    // cd[2] = cd1[2] - cd0[2];

    theta = Round_PI(theta);
    cd[2] = theta;
}

void PK_CopyPos(float *desti_tra, const float *source_tra) // copy posture
{
    memcpy(desti_tra, source_tra, 3 * sizeof(float));
}

void PK_CopyTra(float *desti_tra, const float *source_tra) // copy trajectory
{
    memcpy(desti_tra, source_tra, TRAJITEM_LEN * sizeof(float));
}

/********************************************
function : InterPolationOneDim 一维插值算法
Param :     float * x - X 轴坐标
                    float * y   - Y 轴坐标
                    float xx - 输入的 X 轴坐标
                    int n - 坐标数据个数
Return : float - 对应输入X坐标的Y值
********************************************/
float InterPolationOneDim(const float *x, const float *y, const float xx, int n)
{
    int i, sign_x, out_intp = 0;
    float yy = 0;
    int l, h;
    if (x[1] - x[0] >= 0)
    {
        sign_x = 1;
        if (xx >= x[n - 1])
        {
            out_intp = 1;
            yy       = y[n - 1];
        }
        else if (xx <= x[0])
        {
            out_intp = 1;
            yy       = y[0];
        }
    }
    else
    {
        sign_x = -1;
        if (xx >= x[0])
        {
            out_intp = 1;
            yy       = y[0];
        }
        else if (xx <= x[n - 1])
        {
            out_intp = 1;
            yy       = y[n - 1];
        }
    }

    if (out_intp == 0)
    {
        l = 0;
        h = n - 1;
        while (l < h)
        {
            i = (l + h) / 2;
            if (sign_x * xx >= sign_x * x[i] && sign_x * xx <= sign_x * x[i + 1])
            {
                yy = y[i] + (xx - x[i]) * (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
                break;
            }
            else if (sign_x * xx > sign_x * x[i + 1])
            {
                l = i + 1;
            }
            else
            {
                h = i;
            }
        }
    }
    return yy;
}

/********************************************
function : SearchOptimalRoute 最优路径搜索
Param :     int * trajnum - 路径段的个数数组
            float * trajsum - 路径长度总和
            int n - 坐标数据个数
Return : int - 对应最优路径的索引
********************************************/
int SearchOptimalRoute(const int *trajnum, const float *trajsum, int n)
{
    int mini = 0, i;

    if (n == 0)
        return 0;

    for (i = 1; i < n; i++)
    {
        if (trajnum[i] == 0)
            continue;
        if (trajnum[mini] == 0)
        {
            mini = i;
            continue;
        }
        // 查找单个搜索路径数量 最小的
        if (trajnum[mini] > trajnum[i])
        {
            mini = i;
        }
        // 如果数量相同，再查找路径长度总和最小的
        else if (trajnum[mini] == trajnum[i])
        {
            if (trajsum[mini] > trajsum[i])
                mini = i;
        }
    }
    return mini;
}

void Convert(const float *pO, const float *pin, float *pout)
{
    pout[0] = pin[0] * cosf(pO[2]) - pin[1] * sinf(pO[2]) + pO[0];
    pout[1] = pin[0] * sinf(pO[2]) + pin[1] * cosf(pO[2]) + pO[1];
}

void RevConvert(const float *pO, const float *pin, float *pout)
{
    pout[0] = (pin[0] - pO[0]) * cosf(pO[2]) + (pin[1] - pO[1]) * sinf(pO[2]);
    pout[1] = -(pin[0] - pO[0]) * sinf(pO[2]) + (pin[1] - pO[1]) * cosf(pO[2]);
}

float Project_PosTo1stPos_rx(VehPos_T cd0, VehPos_T cd1)
{
    float dx = cd1.x - cd0.x;
    float dy = cd1.y - cd0.y;
    float c  = cosf(cd0.theta);
    float s  = sinf(cd0.theta);

    return (dx * c + dy * s);
}

float Project_PosTo1stPos_ry(VehPos_T cd0, VehPos_T cd1)
{
    float dx = cd1.x - cd0.x;
    float dy = cd1.y - cd0.y;
    float c  = cosf(cd0.theta);
    float s  = sinf(cd0.theta);
    return (dy * c - dx * s);
}

float Project_PointTo1stPos_rx(VehPos_T cd0, Point_T cd1)
{
    float dx = cd1.x - cd0.x;
    float dy = cd1.y - cd0.y;
    float c  = cosf(cd0.theta);
    float s  = sinf(cd0.theta);
    return (dx * c + dy * s);
}
/**************************************************
Description     :point project position calculate
Owner           :LJM
Modified Date   :2018.05.14
Parameter[In]   :cd0 -  x0 y0 theta0
cd1 -  x1 y1
Parameter[Out]  :
Return          :ry -  cd1 project on cd0
***********************************************/

float Project_PointTo1stPos_ry(VehPos_T cd0, Point_T cd1)
{
    float dx = cd1.x - cd0.x;
    float dy = cd1.y - cd0.y;
    float c  = cosf(cd0.theta);
    float s  = sinf(cd0.theta);
    return dy * c - dx * s;
}

float Project_PosTo1st_rx(const float *cd0, const float *cd1)
{
    float dx = cd1[0] - cd0[0];
    float dy = cd1[1] - cd0[1];
    float c  = cosf(cd0[2]);
    float s  = sinf(cd0[2]);
    return dx * c + dy * s;
}

float Project_PosTo1st_ry(const float *cd0, const float *cd1)
{
    float dx = cd1[0] - cd0[0];
    float dy = cd1[1] - cd0[1];
    float c  = cosf(cd0[2]);
    float s  = sinf(cd0[2]);
    return dy * c - dx * s;
}

float Project_PosTo1st_rtheta(const float *cd0, const float *cd1)
{
    float theta = cd1[2] - cd0[2];
    if (fabs(theta) > 1e4)
    {
        log_err("Project_PosTo1st_rtheta Err!\n");
    }
    return Round_PI(theta);
}

float Cal_PowDist_Pos2Pos(VehPos_T pos1, VehPos_T pos2)
{
    return (pow2(pos1.x - pos2.x) + pow2(pos1.y - pos2.y));
}

float Get_Norm_Vec2(Vec2_T vec) { return sqrtf(pow2(vec.vx) + pow2(vec.vy)); }

void Normalize_Vec2(Vec2_T *vec)
{
    float VecLen = Get_Norm_Vec2(*vec);
    vec->vx /= VecLen;
    vec->vy /= VecLen;
}

float Cal_Dis_Pt2Pt(const VehPos_T &pt1, const VehPos_T &pt2)
{
    return sqrtf((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

float Cal_Dis_Pt2Pt(const Point_T &pt1, const Point_T &pt2)
{
    return sqrtf((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

float Cal_Dis_Pt2Pt(const float *cd0, const float *cd1)
{
    return sqrtf((cd1[0] - cd0[0]) * (cd1[0] - cd0[0]) +
                 (cd1[1] - cd0[1]) * (cd1[1] - cd0[1]));
}

float Get_Dot_Vec2(Vec2_T vec1, Vec2_T vec2)
{
    return (vec1.vx * vec2.vx + vec1.vy * vec2.vy);
}

float Get_Cross_Vec2(Vec2_T vec1, Vec2_T vec2)
{
    return (vec1.vx * vec2.vy - vec1.vy * vec2.vx);
}

/**************************************************************************
    Description     :calculate the cross angle of two 2D vectors
    Owner           :CSJ
    Modified Date   :2018.05.14
    Parameter[In]   :two 2D vector
    Parameter[Out]  :None
    Return          :the cross angle of two 2D vectors
    ***************************************************************************/
float Get_CrossAngle_Vec2(Vec2_T vec1, Vec2_T vec2)
{
    float dot   = Get_Dot_Vec2(vec1, vec2);
    float temp  = dot / (Get_Norm_Vec2(vec1) * Get_Norm_Vec2(vec2));
    float Anlge = acosf(temp);
    return Anlge;
}

// convert SlotInfo_T to float [5][4]
void SlotObj_Convert_float(const SlotObj_T &IN_Slot, float OUT_Slot[5][4])
{
    memcpy(&OUT_Slot[0][0], &IN_Slot.ptA, 2 * sizeof(float));
    memcpy(&OUT_Slot[0][2], &IN_Slot.ptB, 2 * sizeof(float));
    memcpy(&OUT_Slot[1][0], &IN_Slot.ptB, 2 * sizeof(float));
    memcpy(&OUT_Slot[1][2], &IN_Slot.ptC, 2 * sizeof(float));
    memcpy(&OUT_Slot[2][0], &IN_Slot.ptC, 2 * sizeof(float));
    memcpy(&OUT_Slot[2][2], &IN_Slot.ptD, 2 * sizeof(float));
    memcpy(&OUT_Slot[3][0], &IN_Slot.ptD, 2 * sizeof(float));
    memcpy(&OUT_Slot[3][2], &IN_Slot.ptE, 2 * sizeof(float));
    memcpy(&OUT_Slot[4][0], &IN_Slot.ptE, 2 * sizeof(float));
    memcpy(&OUT_Slot[4][2], &IN_Slot.ptF, 2 * sizeof(float));
}

void Get_Dist_Dir_Pt2PointLine(Point_T LinePt_A, Point_T LinePt_B, Point_T Pt_C,
                               float *dist, Relation_T *dir)
{
    float a, b, c; //  line param
    float vec_z;

    //   common line param
    a = LinePt_B.y - LinePt_A.y;
    b = LinePt_A.x - LinePt_B.x;
    c = -b * LinePt_A.y - a * LinePt_A.x;

    if (dist != NULL)
    {
        dist[0] = fabsf(a * Pt_C.x + b * Pt_C.y + c) /
                  sqrtf(a * a + b * b); //  点到直线距离的一般公式
    }

    //   判断点所在方位：向量叉乘，CA X CB 的Z轴方向
    if (dir != NULL)
    {
        vec_z = (LinePt_A.x - Pt_C.x) * (LinePt_B.y - Pt_C.y) -
                (LinePt_A.y - Pt_C.y) *
                    (LinePt_B.x - Pt_C.x); //(x1-x3)*(y2-y3)-(y1-y3)*(x2-x3);
        if (vec_z > 1e-3)
        {
            dir[0] = PK_ON_LEFT;
        }
        else if (vec_z < -1e-3)
        {
            dir[0] = PK_ON_RIGHT;
        }
        else
        {
            dir[0] = PK_ON_THE_LINE;
        }
    }
}

void Get_Dist_Dir_Pt2SegLine(LineSeg_T segline, Point_T Pt_C, float *dist,
                             Relation_T *dir)
{
    Get_Dist_Dir_Pt2PointLine(segline.pt1, segline.pt2, Pt_C, dist, dir);
}

int PK_RectLineCross(const float rect[4], float line[4]) // return_re=0,free =1 cross
{
    int return_re;
    float rxmax, rxmin, rymax, rymin;
    float Cx, Dx, Cy, Dy, k;
    float tx, ty;

    rxmax = rte_max(rect[0], rect[2]);
    rxmin = rte_min(rect[0], rect[2]);
    rymax = rte_max(rect[1], rect[3]);
    rymin = rte_min(rect[1], rect[3]);

    if (line[1] == line[3]) // line is parallel to x axis
    {
        if (line[1] <= rymax && line[1] >= rymin)
        {
            if (rte_min(line[0], line[2]) > rxmax || rte_max(line[0], line[2]) < rxmin)
            {
                return_re = 0;
            }
            else
            {
                return_re = 1;
            }
        }
        else
        {
            return_re = 0;
        }
    }
    // exchange A and B, make the Y of B is the bigger one
    else
    {
        if (line[1] > line[3])
        {
            tx      = line[0];
            ty      = line[1];
            line[0] = line[2];
            line[1] = line[3];
            line[2] = tx;
            line[3] = ty;
        }

        // to confirm C, D on the line of AB
        // two points to form a line: (x-x1)/(x2-x1)=(y-y1)/(y2-y1)
        k = (line[2] - line[0]) / (line[3] - line[1]);
        if (line[1] < rymin)
        {
            Cy = rymin;
            Cx = k * (Cy - line[1]) + line[0];
        }
        else
        {
            Cx = line[0];
            Cy = line[1];
        }

        //////////////////////////////////////////////////////
        if (line[3] > rymax)
        {
            Dy = rymax;
            Dx = k * (Dy - line[1]) + line[0];
        }
        else
        {
            Dx = line[2];
            Dy = line[3];
        }

        //////////////////////////////////////////////////////
        if (Dy >= Cy) // has intersection along y axis
        {
            // return_re= IntervalOverlap(rxmin, rxmax, Dx, Cx);
            if (rte_min(Dx, Cx) > rxmax || rte_max(Dx, Cx) < rxmin)
            {
                return_re = 0;
            }
            else
            {
                return_re = 1;
            }
        }
        else
        {
            return_re = 0;
        }
    }
    return return_re;
}

int Cal_SlotShap(float finpos[3], float slotobj[5][4], float *left_fac)
{
    float CD_staySameSide;
    LineSeg_T seg_CD;
    Point_T TargPt;
    Relation_T pt_dir;
    int slotshape;
    //  if C,D stay the same side of targpos then it is verticle slot else it is parallel
    //  slot
    CD_staySameSide = Project_PosTo1st_ry(finpos, &slotobj[2][0]) *
                      Project_PosTo1st_ry(finpos, &slotobj[2][2]);
    memcpy(&TargPt, finpos, sizeof(TargPt));
    memcpy(&seg_CD, slotobj[2], sizeof(seg_CD));
    Get_Dist_Dir_Pt2SegLine(seg_CD, TargPt, NULL, &pt_dir);
    if (CD_staySameSide > 0) // parallel slot
    {
        if (pt_dir == PK_ON_LEFT)
        {
            *left_fac = -1.0f; // right slot
            slotshape = PK_SLOT_RIGHT_PARA;
        }
        else
        {
            *left_fac = 1.0f; // left slot
            slotshape = PK_SLOT_LEFT_PARA;
        }
    }
    else //  verticle slot
    {
        if (pt_dir == PK_ON_LEFT)
        {
            *left_fac = -1.0f; // right slot
            slotshape = PK_SLOT_RIGHT_VERT;
        }
        else
        {
            *left_fac = 1.0f; // left slot
            slotshape = PK_SLOT_LEFT_VERT;
        }
    }
    return slotshape;
}

float PK_PosObjCrossMinDist(const float stpoint[3], const int obj_num,
                            const float obj[][4])
{
    int i = 0; // n side of obj n=5
    float len, width, h;
    float rect[4], line[4];
    float min_dist = 9999.0f, cur_dist = 0;
    len   = VEHICLE_LEN;
    width = VEHICLE_WID;
    h     = REAR_SUSPENSION;

    rect[0] = len - h;
    rect[1] = width / 2;
    rect[2] = -h;
    rect[3] = -width / 2;

    for (i = 0; i < obj_num; i++)
    {
        line[0]  = Project_PosTo1st_rx(stpoint, obj[i]);
        line[1]  = Project_PosTo1st_ry(stpoint, obj[i]);
        line[2]  = Project_PosTo1st_rx(stpoint, &obj[i][2]);
        line[3]  = Project_PosTo1st_ry(stpoint, &obj[i][2]);
        cur_dist = PK_RectLineCrossDist(rect, line);
        if (cur_dist < 0)
        {
            min_dist = -1.0f; // 0925: new add revise bug
            break;
        }
        else if (i == 0)
        {
            min_dist = cur_dist;
        }
        else if (cur_dist < min_dist)
        {
            min_dist = cur_dist;
        }
    }

    return min_dist;
}

int PK_PosObjCross(const float stpoint[3], const int obj_num, const float obj[][4],
                   const float ds, const float swell)
{
    int i = 0, Cross = 0; // n side of obj n=5
    float rect[4], line[4];
    float len   = VEHICLE_LEN + swell;
    float width = VEHICLE_WID + swell;
    float h     = REAR_SUSPENSION + swell / 2;

    if (ds < 0)
    {
        rect[0] = len - h;
        rect[1] = width / 2;
        rect[2] = -h + ds;
        rect[3] = -width / 2;
    }
    else
    {
        rect[0] = len - h + ds;
        rect[1] = width / 2;
        rect[2] = -h;
        rect[3] = -width / 2;
    }

    for (i = 0; i < obj_num; i++)
    {
        line[0] = Project_PosTo1st_rx(stpoint, obj[i]);
        line[1] = Project_PosTo1st_ry(stpoint, obj[i]);
        line[2] = Project_PosTo1st_rx(stpoint, &obj[i][2]);
        line[3] = Project_PosTo1st_ry(stpoint, &obj[i][2]);
        if (PK_RectLineCross(rect, line))
        {
            Cross = i + 1; // from 1
            break;
        }
        else
            Cross = 0;
    }

    return Cross;
}

void SlotObj_Convert_struct(SlotObj_T *OUT_Slot, const float IN_Slot[5][4])
{
    memcpy(&(OUT_Slot->ptA), &IN_Slot[0][0], 2 * sizeof(float));
    memcpy(&(OUT_Slot->ptB), &IN_Slot[1][0], 2 * sizeof(float));
    memcpy(&(OUT_Slot->ptC), &IN_Slot[2][0], 2 * sizeof(float));
    memcpy(&(OUT_Slot->ptD), &IN_Slot[3][0], 2 * sizeof(float));
    memcpy(&(OUT_Slot->ptE), &IN_Slot[4][0], 2 * sizeof(float));
    memcpy(&(OUT_Slot->ptF), &IN_Slot[4][2], 2 * sizeof(float));
}

void Get_Veh_CornerPt(VehPos_T CurPos, Rect_T *CornerPt, float swell)
{
    Vec2_T ForwdVec, LeftVec;
    ForwdVec.vx = cosf(CurPos.theta);
    ForwdVec.vy = sinf(CurPos.theta);
    LeftVec.vx  = -ForwdVec.vy;
    LeftVec.vy  = ForwdVec.vx;
    // calculate corner points
    CornerPt->pt_fl.x = CurPos.x + ForwdVec.vx * (VEHICLE_LEN - REAR_SUSPENSION + swell) +
                        LeftVec.vx * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_fl.y = CurPos.y + ForwdVec.vy * (VEHICLE_LEN - REAR_SUSPENSION + swell) +
                        LeftVec.vy * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_fr.x = CurPos.x + ForwdVec.vx * (VEHICLE_LEN - REAR_SUSPENSION + swell) -
                        LeftVec.vx * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_fr.y = CurPos.y + ForwdVec.vy * (VEHICLE_LEN - REAR_SUSPENSION + swell) -
                        LeftVec.vy * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_rr.x = CurPos.x - ForwdVec.vx * (REAR_SUSPENSION + swell) -
                        LeftVec.vx * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_rr.y = CurPos.y - ForwdVec.vy * (REAR_SUSPENSION + swell) -
                        LeftVec.vy * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_rl.x = CurPos.x - ForwdVec.vx * (REAR_SUSPENSION + swell) +
                        LeftVec.vx * (VEHICLE_WID / 2.0f + swell);
    CornerPt->pt_rl.y = CurPos.y - ForwdVec.vy * (REAR_SUSPENSION + swell) +
                        LeftVec.vy * (VEHICLE_WID / 2.0f + swell);
}

int Get_CrossPt_PointLine(Point_T Line1_A, Point_T Line1_B, Point_T Line2_A,
                          Point_T Line2_B, Point_T *CrossPt)
{
    float a1, b1, c1, a2, b2, c2;
    float temp;
    //  两点构成的直线一般方程
    a1 = Line1_B.y - Line1_A.y;
    b1 = Line1_A.x - Line1_B.x;
    c1 = -b1 * Line1_A.y - a1 * Line1_A.x;

    a2   = Line2_B.y - Line2_A.y;
    b2   = Line2_A.x - Line2_B.x;
    c2   = -b2 * Line2_A.y - a2 * Line2_A.x;
    temp = a1 * b2 - a2 * b1;

    //  判断两直线是否平行
    if (fabsf(temp) < 1e-4)
    {
        return 0;
    }
    //  计算交点
    CrossPt->x = (b1 * c2 - b2 * c1) / temp;
    CrossPt->y = (a2 * c1 - a1 * c2) / temp;
    return 1;
}

float Get_CrossProduct(Point_T pt1, Point_T pt2, Point_T pt3)
{
    float cross = (pt2.x - pt1.x) * (pt3.y - pt2.y) - (pt3.x - pt2.x) * (pt2.y - pt1.y);
    return cross;
}

/**************************************************************************
    Description     :check if p3 is in the rectangle whose diagonal is composed of p1 and
   p2 Owner           :CSJ Modified Date   :2018.05.16 Parameter[In]   :three points
    Parameter[Out]  :None
    Return          :1 -in the rectangle; 0 -not;
    ***************************************************************************/
int onSegment(Point_T pt1, Point_T pt2, Point_T pt3)
{
    if (pt3.x >= rte_min(pt1.x, pt2.x) && pt3.x <= rte_max(pt1.x, pt2.x) &&
        pt3.y >= rte_min(pt1.y, pt2.y) && pt3.y <= rte_max(pt1.y, pt2.y))
    {
        return 1;
    }
    return 0;
}

int Is_TwoSegment_Cross(Point_T pt1, Point_T pt2, Point_T pt3, Point_T pt4)
{
    float d1 = Get_CrossProduct(pt1, pt2, pt3);
    float d2 = Get_CrossProduct(pt1, pt2, pt4);
    float d3 = Get_CrossProduct(pt3, pt4, pt1);
    float d4 = Get_CrossProduct(pt3, pt4, pt2);
    if (d1 * d2 < 0 && d3 * d4 < 0)
    {
        return 1;
    }
    //  check if two points are superposition
    if (fabsf(d1) <= ZERO_FLOAT && onSegment(pt1, pt2, pt3))
        return 1;
    if (fabsf(d2) <= ZERO_FLOAT && onSegment(pt1, pt2, pt4))
        return 1;
    if (fabsf(d3) <= ZERO_FLOAT && onSegment(pt3, pt4, pt1))
        return 1;
    if (fabsf(d4) <= ZERO_FLOAT && onSegment(pt3, pt4, pt2))
        return 1;
    return 0;
}

int Is_Point_OnLine(Point_T pt1, Point_T pt2, Point_T pt)
{
    Vec2_T v1 = {pt1.x - pt.x, pt1.y - pt.y};
    Vec2_T v2 = {pt2.x - pt.x, pt2.y - pt.y};
    auto m1   = Get_Cross_Vec2(v1, v2);
    auto m2   = Get_Dot_Vec2(v1, v2);
    return sign(m1) == 0 && sign(m2) < 0;
}

int Normal_RectLineCross(float rect[4], float line[4]) //
{
    int return_re;
    float rxmax, rxmin, rymax, rymin;
    float Cx, Dx, Cy, Dy, k;
    float tx, ty;

    rxmax = rte_max(rect[0], rect[2]);
    rxmin = rte_min(rect[0], rect[2]);
    rymax = rte_max(rect[1], rect[3]);
    rymin = rte_min(rect[1], rect[3]);
    if (line[1] == line[3]) // ??????DDóúx?á
    {
        if (line[1] <= rymax && line[1] >= rymin)
        {
            if (rte_min(line[0], line[2]) > rxmax || rte_max(line[0], line[2]) < rxmin)
            {
                return_re = 0;
            }
            else
            {
                return_re = 1;
            }
        }
        else
        {
            return_re = 0;
        }
    }
    // ABá?μ?????￡?è?Bμ?μ?y×?±ê×?′ó ?±?? ??D? ?à?? ?D?¨ ?μ??
    else
    {
        if (line[1] > line[3])
        {
            tx      = line[0];
            ty      = line[1];
            line[0] = line[2];
            line[1] = line[3];
            line[2] = tx;
            line[3] = ty;
        }

        // ?ú????ABé?è·?¨μ?CoíD
        // á?μ?è·?¨ò?ì??±??: (x-x1)/(x2-x1)=(y-y1)/(y2-y1)
        k = (line[2] - line[0]) / (line[3] - line[1]);
        if (line[1] < rymin)
        {
            Cy = rymin;
            Cx = k * (Cy - line[1]) + line[0];
        }
        else
        {
            Cx = line[0];
            Cy = line[1];
        }

        //////////////////////////////////////////////////////
        if (line[3] > rymax)
        {
            Dy = rymax;
            Dx = k * (Dy - line[1]) + line[0];
        }
        else
        {
            Dx = line[2];
            Dy = line[3];
        }

        //////////////////////////////////////////////////////
        if (Dy >= Cy) // y??é?óD???ˉ
        {
            // return_re= IntervalOverlap(rxmin, rxmax, Dx, Cx);
            if (rte_min(Dx, Cx) > rxmax || rte_max(Dx, Cx) < rxmin)
            {
                return_re = 0;
            }
            else
            {
                return_re = 1;
            }
        }
        else
        {
            return_re = 0;
        }
    }

    return return_re;
}

int PK_PosLineSegCross_Check(VehPos_T stpoint, int obj_num, LineSeg_T obj[], float swell)
{
    int i = 0, Cross = 0; // n side of obj n=5
    // float xc[4],yc[4];//xc_stp[4],yc_stp[4],xc_obj[4],yc_obj[4];
    float len, width, h;
    float rect[4], line[4];
    len   = VEHICLE_LEN + swell;
    width = VEHICLE_WID + swell;
    h     = REAR_SUSPENSION + swell / 2;
    // x0=stpoint[0];y0=stpoint[1];theta0=stpoint[2];

    rect[0] = len - h;
    rect[1] = width / 2;
    rect[2] = -h;
    rect[3] = -width / 2;

    for (i = 0; i < obj_num; i++) //?e???ì2a??°-??
    {
        line[0] = Project_PointTo1stPos_rx(stpoint, obj[i].pt1);
        line[1] = Project_PointTo1stPos_ry(stpoint, obj[i].pt1);
        line[2] = Project_PointTo1stPos_rx(stpoint, obj[i].pt2);
        line[3] = Project_PointTo1stPos_ry(stpoint, obj[i].pt2);
        if (Normal_RectLineCross(rect, line))
        {
            Cross = Cross + 1; // from 1
        }
    }

    return Cross;
}

void PK_Get_Path_EndPos(const float Path[5], float EndPos[3])
{
    float x0 = Path[0], y0 = Path[1], theta0 = Path[2], ds = Path[3], Rr = Path[4];
    float ftheta = 0;
    if (fabsf(Rr) < 1)
    {
        EndPos[0] = cosf(theta0) * ds + x0;
        EndPos[1] = sinf(theta0) * ds + y0;
        EndPos[2] = theta0;
    }
    else
    {
        ftheta    = -sign(Rr) * PI / 2 + theta0;
        EndPos[0] = cosf(ds / Rr + ftheta) * fabsf(Rr) + x0 - fabsf(Rr) * cosf(ftheta);
        EndPos[1] = sinf(ds / Rr + ftheta) * fabsf(Rr) + y0 - fabsf(Rr) * sinf(ftheta);
        EndPos[2] = theta0 + ds / Rr;
    }
}

float PK_PointToLineDist(const float line[4], const float pt[2])
{
    float dist_min;
    float x1 = line[0];
    float y1 = line[1];
    float x2 = line[2];
    float y2 = line[3];
    float A  = y2 - y1;
    float B  = x1 - x2;
    float C  = x2 * y1 - x1 * y2;

    float d = fabsf(A * pt[0] + B * pt[1] + C) / sqrtf(A * A + B * B);
    float x = (B * B * pt[0] - A * B * pt[1] - A * C) / (A * A + B * B);
    float y = (-A * B * pt[0] + A * A * pt[1] - B * C) / (A * A + B * B);

    if ((x - x1) * (x2 - x) + (y - y1) * (y2 - y) > 0)
    {
        dist_min = d;
    }
    else
    {
        dist_min =
            rte_min(sqrtf((x1 - pt[0]) * (x1 - pt[0]) + (y1 - pt[1]) * (y1 - pt[1])),
                    sqrtf((x2 - pt[0]) * (x2 - pt[0]) + (y2 - pt[1]) * (y2 - pt[1])));
    }
    return dist_min;
}

float PK_RectLineCrossDist(
    const float rect[4],
    float line[4]) // return <0 Cross; return rte_min distance from rectangle;
{
    //  int return_re;
    float rxmax, rxmin, rymax, rymin;
    float Cx, Dx, Cy, Dy, k;
    float tx, ty;
    float near_dist = 0, near_dist_dx = 0, near_dist_dy = 0; //
    float ptx[4] = {0.0f}, pty[4] = {0.0f},
          line_detec[4][4] =
              {
                  {0.0f},
              },
          pt[2] = {0.0f};
    int i       = 0;
    // int run_con = 0;

    rxmax = rte_max(rect[0], rect[2]);
    rxmin = rte_min(rect[0], rect[2]);
    rymax = rte_max(rect[1], rect[3]);
    rymin = rte_min(rect[1], rect[3]);

    if (fabsf(line[1] - line[3]) < 0.001f) // the segment is parallel to x axis
    {
        if (line[1] <= rymax && line[1] >= rymin)
        {
            if (rte_min(line[0], line[2]) > rxmax || rte_max(line[0], line[2]) < rxmin)
            {
                if (rte_min(line[0], line[2]) > rxmax)
                {
                    near_dist = rte_min(line[0], line[2]) - rxmax;
                }
                else
                {
                    near_dist = rxmin - rte_max(line[0], line[2]);
                }
            }
            else
            {
                near_dist = -1.0;
            }
        }
        else
        {
            if (rte_min(line[0], line[2]) > rxmax || rte_max(line[0], line[2]) < rxmin)
            {
                if (rte_min(line[0], line[2]) > rxmax)
                {
                    near_dist_dx = rte_min(line[0], line[2]) - rxmax;
                }
                else
                {
                    near_dist_dx = rxmin - rte_max(line[0], line[2]);
                }
            }
            else
            {
                near_dist_dx = 0;
            }

            if (line[1] > rymax)
            {
                near_dist_dy = line[1] - rymax;
            }
            else
            {
                near_dist_dy = rymin - line[1];
            }
            near_dist = sqrtf(near_dist_dy * near_dist_dy + near_dist_dx * near_dist_dx);
        }
    }
    else
    {
        if (line[1] > line[3])
        {
            tx      = line[0];
            ty      = line[1];
            line[0] = line[2];
            line[1] = line[3];
            line[2] = tx;
            line[3] = ty;
        }

        k = (line[2] - line[0]) / (line[3] - line[1]);
        if (line[1] < rymin) // small compare with small
        {
            Cy = rymin;
            Cx = k * (Cy - line[1]) + line[0];
        }
        else
        {
            Cx = line[0];
            Cy = line[1];
        }

        //////////////////////////////////////////////////////
        if (line[3] > rymax)
        {
            Dy = rymax;
            Dx = k * (Dy - line[1]) + line[0];
        }
        else
        {
            Dx = line[2];
            Dy = line[3];
        }

        //////////////////////////////////////////////////////
        if (Dy >= Cy)
        {
            // return_re= IntervalOverlap(rxmin, rxmax, Dx, Cx);
            if (rte_min(Dx, Cx) > rxmax || rte_max(Dx, Cx) < rxmin)
            {
                if (rte_min(Dx, Cx) > rxmax) // 4 condition
                {
                    ptx[0]           = rxmax;
                    pty[0]           = rymax;
                    line_detec[0][0] = line[0];
                    line_detec[0][1] = line[1];
                    line_detec[0][2] = line[2];
                    line_detec[0][3] = line[3];
                    ptx[1]           = rxmax;
                    pty[1]           = rymin;
                    line_detec[1][0] = line[0];
                    line_detec[1][1] = line[1];
                    line_detec[1][2] = line[2];
                    line_detec[1][3] = line[3];

                    ptx[2]           = line[0];
                    pty[2]           = line[1];
                    line_detec[2][0] = rxmax;
                    line_detec[2][1] = rymax;
                    line_detec[2][2] = rxmax;
                    line_detec[2][3] = rymin;
                    ptx[3]           = line[2];
                    pty[3]           = line[3];
                    line_detec[3][0] = rxmax;
                    line_detec[3][1] = rymax;
                    line_detec[3][2] = rxmax;
                    line_detec[3][3] = rymin;
                }
                else
                {
                    ptx[0]           = rxmin;
                    pty[0]           = rymax;
                    line_detec[0][0] = line[0];
                    line_detec[0][1] = line[1];
                    line_detec[0][2] = line[2];
                    line_detec[0][3] = line[3];
                    ptx[1]           = rxmin;
                    pty[1]           = rymin;
                    line_detec[1][0] = line[0];
                    line_detec[1][1] = line[1];
                    line_detec[1][2] = line[2];
                    line_detec[1][3] = line[3];

                    ptx[2]           = line[0];
                    pty[2]           = line[1];
                    line_detec[2][0] = rxmin;
                    line_detec[2][1] = rymax;
                    line_detec[2][2] = rxmin;
                    line_detec[2][3] = rymin;
                    ptx[3]           = line[2];
                    pty[3]           = line[3];
                    line_detec[3][0] = rxmin;
                    line_detec[3][1] = rymax;
                    line_detec[3][2] = rxmin;
                    line_detec[3][3] = rymin;
                }
            }
            else
            {
                near_dist = -1.0;
            }
        }
        else
        {
            if (line[3] < rymin)
            {
                ptx[0]           = rxmax;
                pty[0]           = rymin;
                line_detec[0][0] = line[0];
                line_detec[0][1] = line[1];
                line_detec[0][2] = line[2];
                line_detec[0][3] = line[3];
                ptx[1]           = rxmin;
                pty[1]           = rymin;
                line_detec[1][0] = line[0];
                line_detec[1][1] = line[1];
                line_detec[1][2] = line[2];
                line_detec[1][3] = line[3];

                ptx[2]           = line[0];
                pty[2]           = line[1];
                line_detec[2][0] = rxmax;
                line_detec[2][1] = rymin;
                line_detec[2][2] = rxmin;
                line_detec[2][3] = rymin;

                ptx[3]           = line[2];
                pty[3]           = line[3];
                line_detec[3][0] = rxmax;
                line_detec[3][1] = rymin;
                line_detec[3][2] = rxmin;
                line_detec[3][3] = rymin;
            }
            else // if (line[1]>rymax)
            {
                ptx[0]           = rxmax;
                pty[0]           = rymax;
                line_detec[0][0] = line[0];
                line_detec[0][1] = line[1];
                line_detec[0][2] = line[2];
                line_detec[0][3] = line[3];
                ptx[1]           = rxmin;
                pty[1]           = rymax;
                line_detec[1][0] = line[0];
                line_detec[1][1] = line[1];
                line_detec[1][2] = line[2];
                line_detec[1][3] = line[3];

                ptx[2]           = line[0];
                pty[2]           = line[1];
                line_detec[2][0] = rxmax;
                line_detec[2][1] = rymax;
                line_detec[2][2] = rxmin;
                line_detec[2][3] = rymax;
                ptx[3]           = line[2];
                pty[3]           = line[3];
                line_detec[3][0] = rxmax;
                line_detec[3][1] = rymax;
                line_detec[3][2] = rxmin;
                line_detec[3][3] = rymax;
            }
        }

        for (i = 0; i < 4; i++)
        {
            pt[0] = ptx[i];
            pt[1] = pty[i];
            if (i == 0)
            {
                near_dist = PK_PointToLineDist(line_detec[i], pt);
            }
            else
            {
                near_dist = rte_min(near_dist, PK_PointToLineDist(line_detec[i], pt));
            }
        }
    }

    return near_dist;
}

typedef union {
    float fdata;
    uint32_t ldata;
} FloatLongType;

uint8_t Uint16ToBuf(const uint16_t value, uint8_t *buff)
{
    uint16_t temp = htons(value);
    buff[0]       = (temp >> 8);
    buff[1]       = (temp >> 0);
    return sizeof(uint16_t);
}

uint8_t Uint32ToBuf(const uint32_t value, uint8_t *buff)
{
    uint32_t temp = htonl(value);
    buff[0]       = (temp >> 24);
    buff[1]       = (temp >> 16);
    buff[2]       = (temp >> 8);
    buff[3]       = (temp >> 0);
    return sizeof(uint32_t);
}

uint8_t FloatToBuf(const float value, uint8_t *buff)
{
    FloatLongType ftemp;
    ftemp.fdata = value;
    return Uint32ToBuf(ftemp.ldata, buff);
}

uint8_t BufToUint16(const uint8_t *buff, uint16_t *value)
{
    uint16_t temp = (buff[0] << 8) | buff[1];
    *value        = ntohs(temp);
    return sizeof(uint16_t);
}

uint8_t BufToUint32(const uint8_t *buff, uint32_t *value)
{
    uint32_t temp = (buff[0] << 24) | (buff[1] << 16) | (buff[2] << 8) | (buff[3] << 0);
    *value        = ntohl(temp);
    return sizeof(uint32_t);
}

uint8_t BufToFloat(const uint8_t *buff, float *value)
{
    FloatLongType ftemp;
    BufToUint32(buff, &ftemp.ldata);
    *value = ftemp.fdata;
    return sizeof(uint32_t);
}

int GetLineLen(const char *start, const int len)
{
    int index = 0;
    while (index + 1 < len)
    {
        if (start[index] == '\r')
        {
            if (start[index + 1] == '\n')
            {
                return index + 2;
            }
            else
            {
                return index + 1;
            }
        }
        else if (start[index] == '\n')
        {
            return index + 1;
        }
        else
        {
            index++;
        }
    }
    return index + 2;
}

/**
 * @brief 比较两个车位索引是否指向同一物理车位
 *
 * 车位索引是五位整数编码，包含象限(type)、栅格坐标(m_x, m_y)和极坐标距离(dist_o)。
 * 函数通过解析索引，判断两个索引是否对应同一车位。允许小范围偏差和部分象限对称情况。
 *
 * @param index_apa APA系统生成的车位索引
 * @param index_hmi HMI界面传入的车位索引
 * @return true 两个索引表示同一车位
 * @return false 两个索引表示不同车位
 */
bool CompareSlotIndex(const int index_apa, const int index_hmi)
{
    if (index_apa == index_hmi)
    {
        return 1;
    }

    struct slot_data
    {
        int type;
        int m_x;
        int m_y;
        int dist_o;
    } slot_apa, slot_hmi;

    slot_apa.type   = index_apa / 10000;
    slot_apa.m_x    = (index_apa / 1000) % 10;
    slot_apa.m_y    = (index_apa / 100) % 10;
    slot_apa.dist_o = index_apa % 100;

    slot_hmi.type   = index_hmi / 10000;
    slot_hmi.m_x    = (index_hmi / 1000) % 10;
    slot_hmi.m_y    = (index_hmi / 100) % 10;
    slot_hmi.dist_o = index_hmi % 100;

    if ((slot_apa.m_x != slot_hmi.m_x && slot_apa.m_y != slot_hmi.m_y) ||
        fabs(slot_apa.m_x - slot_hmi.m_x) > 1 || fabs(slot_apa.m_y - slot_hmi.m_y) > 1 ||
        fabs(slot_apa.dist_o - slot_hmi.dist_o) > 1)
    {
        return 0;
    }

    if (slot_apa.dist_o != slot_hmi.dist_o &&
        (slot_apa.m_x != slot_hmi.m_x || slot_apa.m_y != slot_hmi.m_y))
    {
        return 0;
    } // 极坐标距离与x或y坐标不相同

    if (slot_apa.type == slot_hmi.type)
    {
        return 1;
    } // 目标点所在象限一致

    // 目标点所在象限不同，极坐标距离、x、y坐标相同
    if (slot_apa.m_y == slot_hmi.m_y && slot_apa.m_x < 1 && slot_hmi.m_x < 1 &&
        slot_apa.dist_o == slot_hmi.dist_o &&
        (slot_apa.type + slot_hmi.type == 3 ||
         slot_apa.type + slot_hmi.type == 7)) // 象限 1 + 2 = 3, 3 + 4 = 7
    {
        return 1;
    }

    if (slot_apa.m_x == slot_hmi.m_x && slot_apa.m_y < 1 && slot_hmi.m_y < 1 &&
        slot_apa.dist_o == slot_hmi.dist_o &&
        (slot_apa.type + slot_hmi.type == 5)) // 象限 2 + 3 = 5, 1 + 4 = 5
    {
        return 1;
    }

    return 0;
}