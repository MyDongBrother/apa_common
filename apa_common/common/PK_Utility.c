#include "PK_Utility.h"
#include "MathFunc.h"
#include "SystemPara.h"
void Convert_Line_to_Pos(LineSeg_T line, VehPos_T *pos) // line (x0,y0,x1,y1)
{
    pos->x     = line.pt1.x;
    pos->y     = line.pt1.y;
    pos->theta = Round_PI(atan2f((line.pt2.y - line.pt1.y), (line.pt2.x - line.pt1.x)));
}

Relation_T Is_Point_In_Quad(Quad_T quad, Point_T pt)
{
    Vec2_T pfl, pfr, prl, prr;
    float VecAngleSum;
    float angle1, angle2, angle3, angle4;
    pfl.vx      = quad.pt_fl.x - pt.x;
    pfl.vy      = quad.pt_fl.y - pt.y;
    pfr.vx      = quad.pt_fr.x - pt.x;
    pfr.vy      = quad.pt_fr.y - pt.y;
    prl.vx      = quad.pt_rl.x - pt.x;
    prl.vy      = quad.pt_rl.y - pt.y;
    prr.vx      = quad.pt_rr.x - pt.x;
    prr.vy      = quad.pt_rr.y - pt.y;
    angle1      = Get_CrossAngle_Vec2(pfl, pfr);
    angle2      = Get_CrossAngle_Vec2(pfl, prl);
    angle3      = Get_CrossAngle_Vec2(pfr, prr);
    angle4      = Get_CrossAngle_Vec2(prl, prr);
    VecAngleSum = angle1 + angle2 + angle3 + angle4;
    if (fabsf(VecAngleSum - 2 * PI) < 0.01f) //  夹角之和为360度
    {
        if (fabsf(angle1 - PI) < 0.01f || fabsf(angle2 - PI) < 0.01f ||
            fabsf(angle3 - PI) < 0.01f || fabsf(angle4 - PI) < 0.01f)
        {
            return PK_ON_THE_LINE;
        }
        else
        {
            return PK_INSIDE;
        }
    }
    return PK_OUTSIDE;
}
void GetFootPt_LineParam(float a, float b, float c, Point_T Pt, Point_T *FootPt)
{
    float x, y;
    if (fabsf(b) < 1e-4)
    {
        x = -c / a;
        y = Pt.y;
    }
    else
    {
        x = (b * b * Pt.x - a * c - a * b * Pt.y) / (a * a + b * b);
        y = -(a * x + c) / b;
    }
    FootPt->x = x;
    FootPt->y = y;
}

void GetFootPt_PointLine(Point_T line_pt1, Point_T line_pt2, Point_T Pt, Point_T *FootPt)
{
    float a, b, c;
    //  line common params
    a = line_pt2.y - line_pt1.y;
    b = line_pt1.x - line_pt2.x;
    c = -b * line_pt1.y - a * line_pt1.x;
    GetFootPt_LineParam(a, b, c, Pt, FootPt);
}

void GetFootPt_PointAndVecLine(Point_T line_pt1, Vec2_T line_vec, Point_T Pt,
                               Point_T *FootPt)
{
    Point_T line_pt2;
    line_pt2.x = line_pt1.x + line_vec.vx;
    line_pt2.y = line_pt1.y + line_vec.vy;
    GetFootPt_PointLine(line_pt1, line_pt2, Pt, FootPt);
}
void Get_Veh_LineSeg(VehPos_T CurPos, LineSeg_T vehseg[4], float swell)
{
    Rect_T CornerPt;
    Get_Veh_CornerPt(CurPos, &CornerPt, swell);
    vehseg[0].pt1 = CornerPt.pt_fl;
    vehseg[0].pt2 = CornerPt.pt_fr;
    vehseg[1].pt1 = CornerPt.pt_fr;
    vehseg[1].pt2 = CornerPt.pt_rr;
    vehseg[2].pt1 = CornerPt.pt_rr;
    vehseg[2].pt2 = CornerPt.pt_rl;
    vehseg[3].pt1 = CornerPt.pt_rl;
    vehseg[3].pt2 = CornerPt.pt_fl;
}
int SegClip(float p, float q, float *t0, float *t1)
{
    float r;
    int rst = 1; //  初始化
    if (p < 0 && q < 0)
    {
        r = q / p;
        if (r > *t1)
        {
            rst = 0;
        }
        else if (r > *t0)
        {
            *t0 = r; //   更新
        }
    }
    else if (p > 0 && q < p)
    {
        r = q / p;
        if (r < *t0)
        {
            rst = 0;
        }
        else if (r < *t1)
        {
            *t1 = r;
        }
    }
    else if (p == 0 && q < 0)
    {
        rst = 0;
    }
    return rst;
}
int RectClipSegment(Rect_T Rect, LineSeg_T Segm, LineSeg_T *Seg_Cross)
{
    int rt = 0; //   输出初始化
    Point_T RectCentPt;
    Vec2_T e0, e1, tempVec1, tempVec2;
    float halfLen_e0, halfLen_e1;
    float x0, y0, x1, y1, t0, t1, xmin, ymin, xmax, ymax, dx, dy, x, y;
    float p1, p2, p3, p4, q1, q2, q3, q4;
    int rt1, rt2, rt3, rt4;

    RectCentPt.x = (Rect.pt_fl.x + Rect.pt_rr.x) * 0.5f;
    RectCentPt.y = (Rect.pt_fl.y + Rect.pt_rr.y) * 0.5f;
    e0.vx        = Rect.pt_fr.x - Rect.pt_fl.x;
    e0.vy        = Rect.pt_fr.y - Rect.pt_fl.y;
    e1.vx        = Rect.pt_fr.x - Rect.pt_rr.x;
    e1.vy        = Rect.pt_fr.y - Rect.pt_rr.y;
    halfLen_e0   = Get_Norm_Vec2(e0) * 0.5f;
    halfLen_e1   = Get_Norm_Vec2(e1) * 0.5f;
    Normalize_Vec2(&e0);
    Normalize_Vec2(&e1);

    //  将线段的两个端点转换到矩形中心轴对称坐标系下

    tempVec1.vx = Segm.pt1.x - RectCentPt.x;
    tempVec1.vy = Segm.pt1.y - RectCentPt.y;
    tempVec2.vx = Segm.pt2.x - RectCentPt.x;
    tempVec2.vy = Segm.pt2.y - RectCentPt.y;

    x0 = Get_Dot_Vec2(tempVec1, e0); //   点积
    y0 = Get_Dot_Vec2(tempVec1, e1);
    x1 = Get_Dot_Vec2(tempVec2, e0);
    y1 = Get_Dot_Vec2(tempVec2, e1);
    // 计算矩形四条边对线段的裁剪
    t0   = 0;
    t1   = 1; //  线段端点参数初始化为无效值
    xmin = -halfLen_e0;
    xmax = halfLen_e0; //   矩形边界
    ymin = -halfLen_e1;
    ymax = halfLen_e1;
    dx   = x1 - x0;
    dy   = y1 - y0;
    p1   = -dx;
    q1   = x0 - xmin;
    p2   = dx;
    q2   = xmax - x0;
    p3   = -dy;
    q3   = y0 - ymin;
    p4   = dy;
    q4   = ymax - y0;

    rt1 = SegClip(p1, q1, &t0, &t1);
    rt2 = SegClip(p3, q3, &t0, &t1);
    if (rt1 == 1 && rt2 == 1)
    {
        rt3 = SegClip(p2, q2, &t0, &t1);
        rt4 = SegClip(p4, q4, &t0, &t1);
        if (rt3 == 1 && rt4 == 1)
        {
            rt = 1;
            if (Seg_Cross != NULL)
            {
                if (t0 >= 0)
                {
                    x                = x0 + t0 * dx;
                    y                = y0 + t0 * dy;
                    Seg_Cross->pt1.x = x * e0.vx + y * e1.vx +
                                       RectCentPt.x; //   全局坐标系下裁剪后的线段端点
                    Seg_Cross->pt1.y = x * e0.vy + y * e1.vy + RectCentPt.y;
                }
                if (t1 <= 1)
                {
                    x                = x0 + t1 * dx;
                    y                = y0 + t1 * dy;
                    Seg_Cross->pt2.x = x * e0.vx + y * e1.vx +
                                       RectCentPt.x; //   全局坐标系下裁剪后的线段端点
                    Seg_Cross->pt2.y = x * e0.vy + y * e1.vy + RectCentPt.y;
                }
            }
        }
    }
    return rt;
}

float LenTwoSegmentsOverlap(LineSeg_T Line1, LineSeg_T Line2)
{
    float RxPtA, RxPtB, RxPtF;
    VehPos_T pos;
    float localmin, localmax;
    // Make Line_1 as a psuedo Vehicle position
    Convert_Line_to_Pos(Line1, &pos);

    // Project Points onto psuedo position
    RxPtA = Project_PointTo1stPos_rx(pos, Line2.pt1);
    RxPtB = Project_PointTo1stPos_rx(pos, Line2.pt2);
    RxPtF = Project_PointTo1stPos_rx(pos, Line1.pt2);

    localmin = rte_min(RxPtA, RxPtB);
    localmax = rte_max(RxPtA, RxPtB);
    RxPtA    = localmin;
    RxPtB    = localmax;

    if (RxPtB <= 0 || RxPtA >= RxPtF)
    {
        return 0;
    }
    else if (RxPtA < 0 && RxPtB > 0 && RxPtB < RxPtF)
    {
        return RxPtB;
    }
    else if (RxPtA > 0 && RxPtA < RxPtF && RxPtB > RxPtF)
    {
        return RxPtF - RxPtA;
    }
    else if (RxPtA > 0 && RxPtA < RxPtF && RxPtB > 0 && RxPtB < RxPtF)
    {
        return RxPtB - RxPtA;
    }
    else if (RxPtA < 0 && RxPtB > RxPtF)
    {
        return RxPtF;
    }

    return 0;
}

int Slot_Dir(int shapeType)
{
    if (shapeType == PK_SLOT_LEFT_PARA || shapeType == PK_SLOT_LEFT_VERT ||
        shapeType == PK_SLOT_LEFT_ANG_FORWARD || shapeType == PK_SLOT_LEFT_ANG_REVERSE)
    {
        return 1;
    }

    if (shapeType == PK_SLOT_RIGHT_PARA || shapeType == PK_SLOT_RIGHT_VERT ||
        shapeType == PK_SLOT_RIGHT_ANG_FORWARD || shapeType == PK_SLOT_RIGHT_ANG_REVERSE)
    {
        return 2;
    }

    return 0;
}

// 计算两条线段之间的最小距离
float Calc_Dist_Line2Line(float a1[2], float a2[2], float b1[2], float b2[2])
{
    Point_T p1, p2, p3, p4;
    memcpy(&p1, a1, sizeof(Point_T));
    memcpy(&p2, a2, sizeof(Point_T));
    memcpy(&p3, b1, sizeof(Point_T));
    memcpy(&p4, b2, sizeof(Point_T));

    if (Is_TwoSegment_Cross(p1, p2, p3, p4) > 0)
    {
        return 0.0f;
    }

    float line[4];
    memcpy(&line[0], a1, sizeof(Point_T));
    memcpy(&line[2], a2, sizeof(Point_T));
    float dist1 = PK_PointToLineDist(line, b1);
    float dist2 = PK_PointToLineDist(line, b2);

    memcpy(&line[0], b1, sizeof(Point_T));
    memcpy(&line[2], b2, sizeof(Point_T));
    float dist3 = PK_PointToLineDist(line, a1);
    float dist4 = PK_PointToLineDist(line, a2);

    float dist = dist1;
    dist       = rte_min(dist, dist2);
    dist       = rte_min(dist, dist3);
    dist       = rte_min(dist, dist4);

    return dist;
}
