/**
 * @file PK_SF_Subfun.c
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-09-02
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-09-02 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#include "PK_SF_Subfun.h"
#include "PK_Calibration.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "PK_SF.h"
#include "PK_SF_A_FreeSpace.h"
#include "PK_SensorT2S.h"
#include "MathFunc.h"
#include "hobotlog/hobotlog.hpp"
#include "PK_StateManage.h"
#include "Record_Log.h"
/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/

/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */

#if ((PK_SF_SUBFUN_MAJOR_VERSION != (3u)) || (PK_SF_SUBFUN_MINOR_VERSION != (0u)) || \
     (PK_SF_SUBFUN_PATCH_VERSION != (14U)))
#error "Version numbers of PK_SF_Subfun.c and PK_SF_Subfun.h are inconsistent!"
#endif

#pragma section all "CPU1.Private"

// Getfilterdata
#define MEDIAN_FILTERLEN 15 // Define median filter radius

// CalOutline Dedicated macro definition
/*The maximum number of contour segments of obstacles, greater than this value, will
repeat the line segments calculated before the coverage, mainly for the lower computer
programming to define the array size.*/
#define CONTOUR_NUMMAX 20
// Distance between each sampling point
#define SAMPLE_DISTH 0.01f
// Fitting Linear Average Deviation
//  #define  STDOFFSET_TH   0.05f
//  The cutting point set is the maximum value of the numerical fluctuation of the
//  ultrasonic wave and the cutting point set is larger than the maximum value. #define
//  UDELTA_TH      300
// odometer range maximum, cut point set if greater than maximum
//  #define  PATHDELTA_TH    0.6f

// CalObs_part
#define ATTRI_VD_PATH_TH 2.5f // Obstacle attributes are computed beyond this threshold

// JudgeObsClass
#define UDATACLASS_TH1 \
    4800 // Distinguishing Threshold of Near Body, Far Body, Roadside and 5510
#define UDATACLASS_TH2 \
    2600 // Distinguishing Threshold of Near Body, Far Body, Roadside and 5510

// CalObsMainDir CalOutline Dedicated macro definition
#define DISMIN_MAINDIR_SQURE 0.09f
/*********************************************************************************************************
**Definition of exported variables
*********************************************************************************************************/
float cur_fsr          = 0;
float cur_fsr_2nd      = 0; // quadratic echo values 二次回波
float cur_fsr_peak     = 0;
float cur_fsr_peak_2nd = 0;
float cur_fsr_spd      = 0;
float cur_fsl          = 0;
float cur_fsl_2nd      = 0; // quadratic echo values
float cur_fsl_peak     = 0;
float cur_fsl_peak_2nd = 0;
float cur_fsl_spd      = 0;
float cur_rsr          = 0;
float cur_rsr_2nd      = 0; // quadratic echo values
float cur_rsr_peak     = 0;
float cur_rsr_peak_2nd = 0;
float cur_rsr_spd      = 0;
float cur_rsl          = 0;
float cur_rsl_2nd      = 0; // quadratic echo values
float cur_rsl_peak     = 0;
float cur_rsl_peak_2nd = 0;
float cur_rsl_spd      = 0;
float sf_ultrasMotorPos[USS_ECHO_INDEX_MAX][4];

/******************************************************************************************
**  Verify accuracy of IMU data, using wheel counter, add by xls@2022.7.12
*******************************************************************************************/
void Check_IMU_Data()
{
    const float wheel_distance = 1.64;
    // static uint16_t last_wheelcounter_FL= 0,last_wheelcounter_FR = 0;
    static uint16_t last_wheelcounter_RL = 0, last_wheelcounter_RR = 0;
    // uint16_t wheelcounter_FL = RTE_BSW_Get_WheelCounter_FL();
    // uint16_t wheelcounter_FR = RTE_BSW_Get_WheelCounter_FR();
    uint16_t wheelcounter_RL = RTE_BSW_Get_WheelCounter_RL();
    uint16_t wheelcounter_RR = RTE_BSW_Get_WheelCounter_RR();

    RTE_BSW_GearType current_gear = RTE_BSW_Get_CurrentGear();
    // int delta_FL = wheelcounter_FL - last_wheelcounter_FL;
    // int delta_FR = wheelcounter_FR - last_wheelcounter_FR;
    int delta_RL = wheelcounter_RL - last_wheelcounter_RL;
    int delta_RR = wheelcounter_RR - last_wheelcounter_RR;

    static float last_position[4] = {0};
    float current_position[4]     = {0}; //  x,y,theta,s
    RTE_PK_Location_Get_CurPos(current_position);
    static Apa_WorkStateType last_APA_Status = CarSta_Passive;
    Apa_WorkStateType APA_Status             = RTE_SM_Get_ApaWorkState();
    static uint32_t search_count             = 0;
    if (APA_Status == CarSta_Serching && last_APA_Status != CarSta_Serching)
    {
        search_count = 0;
    }
    if (search_count < 5)
    {
        memcpy(last_position, current_position, sizeof(last_position));
    }
    search_count++;

    last_APA_Status = APA_Status;

    float deltaX_imu     = current_position[0] - last_position[0];
    float deltaY_imu     = current_position[1] - last_position[1];
    float deltaTheta_imu = current_position[2] - last_position[2];

    float deltaX_wc, deltaY_wc, deltaTheta_wc;
    if (current_gear == GEAR_REAL_D)
    {
        deltaX_wc =
            rte_min(delta_RR, delta_RL) * Rear_wheel_fac_ds * cos(last_position[2]);
        deltaY_wc =
            rte_min(delta_RR, delta_RL) * Rear_wheel_fac_ds * sin(last_position[2]);
        deltaTheta_wc = (delta_RR - delta_RL) / wheel_distance;
    }
    else if (current_gear == GEAR_REAL_R)
    {
        deltaY_wc =
            -rte_min(delta_RR, delta_RL) * Rear_wheel_fac_ds * cos(last_position[2]);
        deltaY_wc =
            -rte_min(delta_RR, delta_RL) * Rear_wheel_fac_ds * sin(last_position[2]);
        deltaTheta_wc = -(delta_RR - delta_RL) / wheel_distance;
    }
    else
    {
        deltaX_wc     = deltaX_imu;
        deltaY_wc     = deltaY_imu;
        deltaTheta_wc = deltaTheta_imu;
    }
    LOGI << "===========current_gear:" << current_gear << "==================";
    LOGI << "deltaX_wc:" << deltaX_wc << " deltaX_imu:" << deltaX_imu;
    LOGI << "deltaY_wc:" << deltaY_wc << " deltaY_imu" << deltaY_imu;
    LOGI << "deltaTheta_wc:" << deltaTheta_wc << " deltaTheta_imu:" << deltaTheta_imu;

    if (fabs(last_position[2] - current_position[2]) > (PI / 12))
    {
        LOGW << "Theta change significantly, last_theta:" << last_position[2]
             << " current_theta:" << current_position[2];
    }
    // update last data
    // last_wheelcounter_FL = wheelcounter_FL;
    // last_wheelcounter_FL = wheelcounter_FL;
    last_wheelcounter_RL = wheelcounter_RL;
    last_wheelcounter_RR = wheelcounter_RR;
    memcpy(last_position, current_position, sizeof(current_position));
}

/*********************************************************************************************************
**  geometry functions
*********************************************************************************************************/
uint8 LineFitting_LimitedSquare2(Point_T Pt[], int num, Line_T *pline)
{
    float x_mean = 0, y_mean = 0, xy_mean = 0, x2_mean = 0;
    float XX = 0, X = 0, XY = 0, Y = 0 /*, YY=0*/;
    int i;
    float aa = 0, bb = 0;
    Point_T Pt_Nor0 = {0, 0}, Pt_Nor1 = {0, 0};
    Point_T Pt_Start = {0, 0}, Pt_End = {0, 0};

    if (num < 2) //  at least two sample point.
    {
        return FALSE;
    }

    Pt_Start.x = Pt[0].x;
    Pt_Start.y = Pt[0].y;
    Pt_End.x   = Pt[num - 1].x;
    Pt_End.y   = Pt[num - 1].y;

    for (i = 0; i < num; i++)
    {
        Pt[i].x = Pt[i].x - Pt_Start.x;
        Pt[i].y = Pt[i].y - Pt_Start.y;

        XX += Pt[i].x * Pt[i].x;
        X += Pt[i].x;
        XY += Pt[i].x * Pt[i].y;
        Y += Pt[i].y;
        // YY += Pt[i].y * Pt[i].y;
    }

    x_mean  = X / num;
    y_mean  = Y / num;
    xy_mean = XY / num;
    x2_mean = XX / num;

    // if (x2_mean - x_mean * x_mean != 0)  // 2018.07.07
    if (fabsf(x2_mean - x_mean * x_mean) >= 0.0000001f)
    {
        bb       = (xy_mean - x_mean * y_mean) / (x2_mean - x_mean * x_mean); // gradient
        aa       = y_mean + Pt_Start.y - bb * (x_mean + Pt_Start.x);          // intecept
        pline->a = bb;
        pline->b = -1;
        pline->c = aa;
    }
    else
    {
        /*Equal to zero means that all points are on the same line, and the line is X=?
        The line is the same as the X coordinate.*/
        pline->a = 1;
        pline->b = 0;
        pline->c = (float)(-Pt_Start.x);
    }

    // 线性方程是 ax -y + c = 0;
    for (i = 0; i < num; ++i)
    {
        Pt[i].x = Pt[i].x + Pt_Start.x;
        Pt[i].y = Pt[i].y + Pt_Start.y;
    }

    GetNormalPt2(*pline, Pt_Start, &Pt_Nor0);
    GetNormalPt2(*pline, Pt_End, &Pt_Nor1);

    pline->pt1 = Pt_Nor0;
    pline->pt2 = Pt_Nor1;

    return TRUE;
}

float DistToPt(float a, float b, float c, float xt, float yt)
{
    float dis = 0;

    if (a == 0 && b == 0 && c == 0)
    {
        return dis;
    }

    if (a == 0 && b == 0)
    {
        return dis; // a,b could not be equal 0 at the same time.
    }

    if (a * b != 0)
    {
        dis = (fabsf(a * xt + b * yt + c) / sqrtf(a * a + b * b));
    }
    else if (a == 0)
    {
        dis = (fabsf(yt - (-c / b)));
    }
    else
    {
        dis = (fabsf(xt - (-c / a)));
    }

    return dis;
}

float DistToPt2(Line_T line, Point_T pt)
{
    float dis = 0;

    if (line.a == 0 && line.b == 0 && line.c == 0)
    {
        return dis;
    }

    if (line.a * line.b != 0)
    {
        dis = (fabsf(line.a * pt.x + line.b * pt.y + line.c) /
               sqrtf(line.a * line.a + line.b * line.b));
    }
    else if (line.a == 0)
    {
        dis = (fabsf(pt.y - (-line.c / line.b)));
    }
    else
    {
        dis = (fabsf(pt.x - (-line.c / line.a)));
    }

    return dis;
}

uint8 GetNormalPt2(Line_T line, Point_T pt, Point_T *pt_normal)
{
    if ((line.a == 0) && (line.b == 0))
    {
        return FALSE;
    }

    pt_normal->x = (line.b * line.b * pt.x - line.a * line.b * pt.y - line.a * line.c) /
                   (line.a * line.a + line.b * line.b);
    pt_normal->y = (line.a * line.a * pt.y - line.a * line.b * pt.x - line.b * line.c) /
                   (line.a * line.a + line.b * line.b);

    return TRUE;
}

uint8 Getlineabc2(Point_T Pt0, Point_T Pt1, Line_T *pline)
{
    float de, dissqure;

    // de= 0.000001f;
    // dis = (sqrtf((Pt1.x - Pt0.x) * (Pt1.x - Pt0.x) + (Pt1.y - Pt0.y) * (Pt1.y -
    // Pt0.y)));
    de       = 0.00000001f;
    dissqure = (Pt1.x - Pt0.x) * (Pt1.x - Pt0.x) + (Pt1.y - Pt0.y) * (Pt1.y - Pt0.y);

    if (dissqure < de)
    {
        pline->a   = 0;
        pline->b   = 0;
        pline->c   = 0;
        pline->pt1 = Pt0;
        pline->pt2 = Pt1;
        return FALSE;
    }
    else if (fabsf(Pt0.x - Pt1.x) < de)
    {
        pline->a = 1;
        pline->b = 0;
        pline->c = -Pt0.x;
    }
    else if (fabsf(Pt0.y - Pt1.y) < de)
    {
        pline->a = 0;
        pline->b = 1;
        pline->c = -Pt0.y;
    }
    else
    {
        pline->a = (Pt1.y - Pt0.y) / (Pt1.x - Pt0.x);
        pline->b = -1;
        pline->c = Pt0.y - (pline->a) * Pt0.x;
    }

    pline->pt1 = Pt0;
    pline->pt2 = Pt1;

    return TRUE;
}

/*
    %% Judging whether the point is on the left or right side of the line and whether the
   point is on the line

    % The left and right directions are relative to the forward directions, so long as the
   forward directions are specified, we can know the left and right directions (for
   example, the starting point to the end point of a straight line). Judging whether the
   point is on the left or right side of a straight line is a basic algorithm in
   computational geometry. Vectors are used to judge it. % Definition: the area of three
   points P1 (x1, y1), P2 (x2, y2), P3 (x3, y3) on the plane:
    %
    % S(P1,P2,P3)=|y1 y2 y3|= (x1-x3)*(y2-y3)-(y1-y3)*(x2-x3)
    %
    % S is positive when P1P2P3 is counterclockwise and negative when P1P2P3 is clockwise
    %
    % Let the starting point of the vector be A, the ending point be B, and the judgment
   point be C. % If S (A, B, C) is positive, then C is on the left side of vector AB.; %
   If S (A, B, C) is negative, then C is on the right side of vector AB.; % If S (A, B, C)
   is zero, then C is on straight line AB.


    % Return to 1: left
    % Return to 0: straight line
    % Return to -1: right side.

    % X1, y1: The beginning of a straight line, the beginning of a vector
    % X2, y2: The end point of a straight line, the end point of a vector
    % X_P, y_P: the point to be judged
    %
*/
int GetPtLoc2(Point_T Pt1, Point_T Pt2, Point_T Pt_P)
{
    float s;
    int onside;

    s = (Pt1.x - Pt_P.x) * (Pt2.y - Pt_P.y) - (Pt1.y - Pt_P.y) * (Pt2.x - Pt_P.x);

    if (s > 0)
    {
        onside = 1;
    }
    else if (s == 0)
    {
        onside = 0;
    }
    else
    {
        onside = -1;
    }

    return onside;
}

/*
    % Note here: Since the intersection has two points,
    it is stipulated that the calculated point is on the left side of vector AB for
   judgment purposes.

    %     C(x2,y2)
    %        *
    %   b *   *   a
    %     *     *
    %   * * * * * *
    %  A(x0,y0)     B(x1,y1)

    % Computational Thinking:
    % First calculate the three-sided length abc, then calculate the vector AB direction,
   A angle, and make a difference between them to get the vector AC direction, the
   simultaneous vector AC length, and calculate the coordinates of C point.*/

/*
    %% This script calculates four endpoints after the point set is fitted into a straight
   line. %  March 22, 2017 11:31:36 %  Including: the endpoints of two segments, and the
   two endpoints farthest from the straight line %  The line segment is a directional line
   segment from pT0 - > pt1. %  Ptmax: the most remote point on the left side of line
   segment pT0 - > pt1 %  Ptmin: the most remote point on the right side of line segment
   pT0 - > pt1 %  Line_a, line_b: Line direction vector
    %
    On May 3, 2017, 16:14:51, according to the latest program of matlab, add num==1
   judgment item
 */

uint8 CalLineSegAttribute(Point_T Pt[], int num, Point_T *pPt_0, Point_T *pPt_1,
                          Point_T *pPt_max, Point_T *pPt_min, Line_T *pLine,
                          float *pMean_X, float *pStd_offset)
{
    float aerfa;
    Point_T Pt_trans;
    int i;
    int ptminIdx = 0, ptmaxIdx = 0, pt1Idx = 0, pt0Idx = 0;
    float xtransSum = 0, xtransMean = 0, xtranstemp = 0, stdSum = 0;
    float sintemp1 = 0, costemp1 = 0, sintemp2 = 0, costemp2 = 0;

    Line_T fitline = {0, 0, 0, {0, 0}, {0, 0}};

    if (num <= 1)
    {
        if (pPt_0 != NULL && pPt_1 != NULL && pPt_max != NULL && pPt_min != NULL)
        {
            pPt_0->x   = Pt[0].x;
            pPt_0->y   = Pt[0].y;
            pPt_1->x   = Pt[0].x;
            pPt_1->y   = Pt[0].y;
            pPt_max->x = Pt[0].x;
            pPt_max->y = Pt[0].y;
            pPt_min->x = Pt[0].x;
            pPt_min->y = Pt[0].y;
        }
        if (pLine != NULL)
        {
            memset(&fitline, 0, sizeof(Line_T));
        }
        if (pMean_X != NULL)
        {
            *pMean_X = 0;
        }
        if (pStd_offset != NULL)
        {
            *pStd_offset = 0;
        }
        return FALSE;
    }

    // 线性拟合 ax+by+c=0, start point/end point.
    LineFitting_LimitedSquare2(Pt, num, &fitline);

    // 直线的斜率
    if (fitline.b == 0)
        aerfa = PI_GEO / 2.0f;
    else
        aerfa = atanf(-fitline.a / fitline.b);

    xtransSum = 0;
    stdSum    = 0;

    // 与Y轴夹角
    sintemp1 = sinf(-PI_GEO / 2.0f - aerfa);
    costemp1 = cosf(-PI_GEO / 2.0f - aerfa);

    sintemp2 = sinf(PI_GEO / 2.0f - aerfa);
    costemp2 = cosf(PI_GEO / 2.0f - aerfa);

    for (i = 0; i < num; ++i)
    {
        // Transform the spindle to the Y-axis, and the points are distributed along the
        // y-axis.
        //  Slope k < 0

        if (aerfa >= -PI_GEO / 2.0f && aerfa <= 0)
        {
            Pt_trans.x = Pt[i].x * costemp1 - Pt[i].y * sintemp1;
            Pt_trans.y = Pt[i].x * sintemp1 + Pt[i].y * costemp1;
        }
        else
        {
            // Slope k > 0
            Pt_trans.x = Pt[i].x * costemp2 - Pt[i].y * sintemp2;
            Pt_trans.y = Pt[i].x * sintemp2 + Pt[i].y * costemp2;
        }

        if (i == 0)
        {
            if (pPt_0 != NULL && pPt_1 != NULL && pPt_max != NULL && pPt_min != NULL)
            {
                pPt_min->x = Pt_trans.x;
                pPt_min->y = Pt_trans.y;
                pPt_max->x = Pt_trans.x;
                pPt_max->y = Pt_trans.y;
                ptminIdx   = 0;
                ptmaxIdx   = 0;
                pt1Idx     = 0;
                pt0Idx     = 0;
            }
            xtransSum = xtransSum + Pt_trans.x;
        }
        else
        {
            if (pPt_0 != NULL && pPt_1 != NULL && pPt_max != NULL && pPt_min != NULL)
            {
                if (Pt_trans.x < pPt_min->x)
                {
                    pPt_min->x = Pt_trans.x;
                    ptminIdx   = i;
                }
                if (Pt_trans.x > pPt_max->x)
                {
                    pPt_max->x = Pt_trans.x;
                    ptmaxIdx   = i;
                }
                if (Pt_trans.y > pPt_max->y)
                {
                    pPt_max->y = Pt_trans.y;
                    pt1Idx     = i;
                }
                if (Pt_trans.y < pPt_min->y)
                {
                    pPt_min->y = Pt_trans.y;
                    pt0Idx     = i;
                }
            }
            xtransSum = xtransSum + Pt_trans.x;
        }
    }

    if (pPt_0 != NULL && pPt_1 != NULL && pPt_max != NULL && pPt_min != NULL)
    {
        pPt_0->x   = Pt[pt0Idx].x;
        pPt_0->y   = Pt[pt0Idx].y;
        pPt_1->x   = Pt[pt1Idx].x;
        pPt_1->y   = Pt[pt1Idx].y;
        pPt_max->x = Pt[ptmaxIdx].x;
        pPt_max->y = Pt[ptmaxIdx].y;
        pPt_min->x = Pt[ptminIdx].x;
        pPt_min->y = Pt[ptminIdx].y;
    }

    if (pLine != NULL)
    {
        memcpy(pLine, &fitline, sizeof(Line_T));
    }

    xtransMean = xtransSum / (float)num;
    if (pMean_X != NULL)
    {
        *pMean_X = xtransMean;
    }

    if (pStd_offset != NULL)
    {
        for (i = 0; i < num; ++i)
        {
            if (aerfa >= -PI_GEO / 2.0f && aerfa <= 0)
            {
                xtranstemp = Pt[i].x * costemp1 - Pt[i].y * sintemp1;
            }
            else
            {
                xtranstemp = Pt[i].x * costemp2 - Pt[i].y * sintemp2;
            }
            stdSum += (xtranstemp - xtransMean) * (xtranstemp - xtransMean);
        }

        *pStd_offset = sqrtf(stdSum / (float)(num - 1));
    }

    return TRUE;
}

/*********************************************************************************************************
**                                             Quick sort
*********************************************************************************************************/

void sort(float *a, int left, int right)
{
    int i = 0, j = 0;
    float key = 0;
    /* If the index on the left is greater than or equal to the index on the right,
    it means that a group has been sorted out */
    if (left >= right)
    {
        return;
    }
    i   = left;
    j   = right;
    key = a[left];

    while (i < j) /* Control the search within the group */
    {
        while (i < j && key <= a[j])
        /* And the conditions for finding the end are: 1. Find a number less than or
        greater than the key (greater or less depending on whether you want to ascend or
        descend the order).
        2. There is no qualified 1, and the size of I and j is not reversed */
        {
            j--; /*forward looking*/
        }

        a[i] = a[j];
        /*When you find a number like this, assign it to the value of the previous removed
        i (if the first loop and the key is a [left], then the key)*/

        while (i < j && key >= a[i])
        /*This is I looking forward in the group, the same as above, but notice that
        the size relationship with the key stops looping and the top is the opposite,
        because the sorting idea is to throw the number to both sides, so the size of
        the left and right sides is opposite to the size of the key.*/
        {
            i++;
        }

        a[j] = a[i];
    }

    a[i] = key; /*Return the middle key when you have searched it in the group.*/
    sort(a, left,
         i - 1); /*Finally, do the same thing for the left-hand group in the same way.*/
    sort(a, i + 1, right); /*In the same way, do the same for the right-hand group.*/
    /*Of course, eventually there may be a lot of points, until i = j for each group.*/
}

/*********************************************************************************************************
**                                             GetFilterData
*********************************************************************************************************/
void GetFilterData_V3(void)
{
    int i = 0;
    //////////////////////////////////////////////////////////////////////////
    // GetFilterData Special global variables
    static float cycarr_curx[4][MEDIAN_FILTERLEN] = {
        0}; //  Cyclic data, used for fetchingcurlocalx,sf_curlocaly,sf_curtheta;
    static float cycarr_cury[4][MEDIAN_FILTERLEN]     = {0};
    static float cycarr_curtheta[4][MEDIAN_FILTERLEN] = {0};
    static float cycarr_curpath[4][MEDIAN_FILTERLEN]  = {0};
    // 3. Right Probe FSR Cyclic Array
    static float cycarr_fsr[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // Circular array
    static float cycarr_fsr_2nd[MEDIAN_FILTERLEN] = {
        0, 0, 0, 0, 0, 0, 0}; // quadratic echo Circular array
    static uint8 cycarr_fsr_peak[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // peak value
    static uint8 cycarr_fsr_peak_2nd[MEDIAN_FILTERLEN] = {0, 0, 0, 0,
                                                          0, 0, 0}; // the 2nd peak value
    static float cycarr_fsr_spd[MEDIAN_FILTERLEN]      = {0, 0, 0, 0,
                                                          0, 0, 0}; // matched speed

    // 4.Left probe FSL loop array
    static float cycarr_fsl[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // Circular array
    static float cycarr_fsl_2nd[MEDIAN_FILTERLEN] = {
        0, 0, 0, 0, 0, 0, 0}; // Quadratic echo cyclic array
    static uint8 cycarr_fsl_peak[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // peak value
    static uint8 cycarr_fsl_peak_2nd[MEDIAN_FILTERLEN] = {0, 0, 0, 0,
                                                          0, 0, 0}; // the 2nd peak value
    static float cycarr_fsl_spd[MEDIAN_FILTERLEN]      = {0, 0, 0, 0,
                                                          0, 0, 0}; // matched speed

    // 5.RSR Cyclic Array of Right Rear Probe
    static float cycarr_rsr[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // Circular array
    static float cycarr_rsr_2nd[MEDIAN_FILTERLEN] = {
        0, 0, 0, 0, 0, 0, 0}; // Quadratic echo cyclic array
    static uint8 cycarr_rsr_peak[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // peak value
    static uint8 cycarr_rsr_peak_2nd[MEDIAN_FILTERLEN] = {0, 0, 0, 0,
                                                          0, 0, 0}; // the 2nd peak value
    static float cycarr_rsr_spd[MEDIAN_FILTERLEN]      = {0, 0, 0, 0,
                                                          0, 0, 0}; // matched speed

    // 6.Left rear probe RSL loop array
    static float cycarr_rsl[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // Circular array
    static float cycarr_rsl_2nd[MEDIAN_FILTERLEN] = {
        0, 0, 0, 0, 0, 0, 0}; // Quadratic echo cyclic array
    static uint8 cycarr_rsl_peak[MEDIAN_FILTERLEN] = {0, 0, 0, 0, 0, 0, 0}; // peak value
    static uint8 cycarr_rsl_peak_2nd[MEDIAN_FILTERLEN] = {0, 0, 0, 0,
                                                          0, 0, 0}; // the 2nd peak value
    static float cycarr_rsl_spd[MEDIAN_FILTERLEN]      = {0, 0, 0, 0,
                                                          0, 0, 0}; // matched speed

    int cycarr_trueIdx = 0;
    int cycarrIdx      = 0;

    // ultrasonic sensors' order for
    // FOL,FCL,FCR,FOR;ROL,RCL,RCR,ROR;8
    // FCL_FOL,FCL_FCR,FCR_FOR,FCR_FCL;4
    // RCL_ROL,RCL_RCR,RCR_ROR,RCR_RCL;4
    // FSL,FSR,RSL,RSR;4
    // FSL_2,FSR_2,RSL_2,RSR_2;4
    U_RadarDataType ultrasData[USS_ECHO_INDEX_MAX];
    PK_SensorT2S_CurrUltrasData(ultrasData);
    PK_SensorT2S_CurrUltrasMotorPos(sf_ultrasMotorPos);

    cycarrIdx             = TimeSeries_PK_SF % MEDIAN_FILTERLEN;
    cycarr_fsr[cycarrIdx] = ultrasData[FSR_IDX].Distance; // sf_udata.FSR;
    cycarr_fsl[cycarrIdx] = ultrasData[FSL_IDX].Distance; // sf_udata.FSL;
    cycarr_rsr[cycarrIdx] = ultrasData[RSR_IDX].Distance; // sf_udata.RSR;
    cycarr_rsl[cycarrIdx] = ultrasData[RSL_IDX].Distance; // sf_udata.RSL;

    cycarr_fsr_2nd[cycarrIdx] = ultrasData[FSR2_IDX].Distance; // sf_udata.FSR_2;
    cycarr_fsl_2nd[cycarrIdx] = ultrasData[FSL2_IDX].Distance; // sf_udata.FSL_2;
    cycarr_rsr_2nd[cycarrIdx] = ultrasData[RSR2_IDX].Distance; // sf_udata.RSR_2;
    cycarr_rsl_2nd[cycarrIdx] = ultrasData[RSL2_IDX].Distance; // sf_udata.RSL_2;

    cycarr_fsr_peak[cycarrIdx] = ultrasData[FSR_IDX].PeakLevel;
    cycarr_fsl_peak[cycarrIdx] = ultrasData[FSL_IDX].PeakLevel;
    cycarr_rsr_peak[cycarrIdx] = ultrasData[RSR_IDX].PeakLevel;
    cycarr_rsl_peak[cycarrIdx] = ultrasData[RSL_IDX].PeakLevel;

    cycarr_fsr_peak_2nd[cycarrIdx] = ultrasData[FSR2_IDX].PeakLevel;
    cycarr_fsl_peak_2nd[cycarrIdx] = ultrasData[FSL2_IDX].PeakLevel;
    cycarr_rsr_peak_2nd[cycarrIdx] = ultrasData[RSR2_IDX].PeakLevel;
    cycarr_rsl_peak_2nd[cycarrIdx] = ultrasData[RSL2_IDX].PeakLevel;

    cycarr_fsr_spd[cycarrIdx] = RTE_PK_Location_Get_Vel_Spd();
    cycarr_fsl_spd[cycarrIdx] = RTE_PK_Location_Get_Vel_Spd();
    cycarr_rsr_spd[cycarrIdx] = RTE_PK_Location_Get_Vel_Spd();
    cycarr_rsl_spd[cycarrIdx] = RTE_PK_Location_Get_Vel_Spd();

#ifdef _ULTRADEBUG
    cycarr_fsr_spd[cycarrIdx] = TestCurSpd[TimeSeries_PK_SF];
    cycarr_fsl_spd[cycarrIdx] = TestCurSpd[TimeSeries_PK_SF];
    cycarr_rsr_spd[cycarrIdx] = TestCurSpd[TimeSeries_PK_SF];
    cycarr_rsl_spd[cycarrIdx] = TestCurSpd[TimeSeries_PK_SF];
#endif

    // fsr
    cycarr_curx[0][cycarrIdx]     = sf_ultrasMotorPos[FSR_IDX][0];
    cycarr_cury[0][cycarrIdx]     = sf_ultrasMotorPos[FSR_IDX][1];
    cycarr_curtheta[0][cycarrIdx] = sf_ultrasMotorPos[FSR_IDX][2];
    cycarr_curpath[0][cycarrIdx]  = sf_ultrasMotorPos[FSR_IDX][3];
    // fsl
    cycarr_curx[1][cycarrIdx]     = sf_ultrasMotorPos[FSL_IDX][0];
    cycarr_cury[1][cycarrIdx]     = sf_ultrasMotorPos[FSL_IDX][1];
    cycarr_curtheta[1][cycarrIdx] = sf_ultrasMotorPos[FSL_IDX][2];
    cycarr_curpath[1][cycarrIdx]  = sf_ultrasMotorPos[FSL_IDX][3];
    // rsl
    cycarr_curx[2][cycarrIdx]     = sf_ultrasMotorPos[RSL_IDX][0];
    cycarr_cury[2][cycarrIdx]     = sf_ultrasMotorPos[RSL_IDX][1];
    cycarr_curtheta[2][cycarrIdx] = sf_ultrasMotorPos[RSL_IDX][2];
    cycarr_curpath[2][cycarrIdx]  = sf_ultrasMotorPos[RSL_IDX][3];
    // rsr
    cycarr_curx[3][cycarrIdx]     = sf_ultrasMotorPos[RSR_IDX][0];
    cycarr_cury[3][cycarrIdx]     = sf_ultrasMotorPos[RSR_IDX][1];
    cycarr_curtheta[3][cycarrIdx] = sf_ultrasMotorPos[RSR_IDX][2];
    cycarr_curpath[3][cycarrIdx]  = sf_ultrasMotorPos[RSR_IDX][3];

    //////////////////////////////////////////////////////////////////////////
    // Data after median filtering are stored in cur_fsr

    // Start data in the MEDIAN_FILTERLEN range
    if ((TimeSeries_PK_SF + 1) <
        MEDIAN_FILTERLEN) // Primary time series self-increasing in PK_SF_Main, 20180613
    {
        cur_fsr = ultrasData[FSR_IDX].Distance; // sf_udata.FSR;
        cur_fsl = ultrasData[FSL_IDX].Distance; // sf_udata.FSL;
        cur_rsr = ultrasData[RSR_IDX].Distance; // sf_udata.RSR;
        cur_rsl = ultrasData[RSL_IDX].Distance; // sf_udata.RSL;

        cur_fsr_2nd = cycarr_fsr_2nd[cycarrIdx];
        cur_fsl_2nd = cycarr_fsl_2nd[cycarrIdx];
        cur_rsr_2nd = cycarr_rsr_2nd[cycarrIdx];
        cur_rsl_2nd = cycarr_rsl_2nd[cycarrIdx];

        cur_fsr_peak = cycarr_fsr_peak[cycarrIdx];
        cur_fsl_peak = cycarr_fsl_peak[cycarrIdx];
        cur_rsr_peak = cycarr_rsr_peak[cycarrIdx];
        cur_rsl_peak = cycarr_rsl_peak[cycarrIdx];

        cur_fsr_peak_2nd = cycarr_fsr_peak_2nd[cycarrIdx];
        cur_fsl_peak_2nd = cycarr_fsl_peak_2nd[cycarrIdx];
        cur_rsr_peak_2nd = cycarr_rsr_peak_2nd[cycarrIdx];
        cur_rsl_peak_2nd = cycarr_rsl_peak_2nd[cycarrIdx];

        cur_fsr_spd = cycarr_fsr_spd[cycarrIdx];
        cur_fsl_spd = cycarr_fsl_spd[cycarrIdx];
        cur_rsr_spd = cycarr_rsr_spd[cycarrIdx];
        cur_rsl_spd = cycarr_rsl_spd[cycarrIdx];

        for (i = 0; i < 4; ++i)
        {
            sf_curlocalx[i] = cycarr_curx[i][cycarrIdx];
            sf_curlocaly[i] = cycarr_cury[i][cycarrIdx];
            sf_curtheta[i]  = cycarr_curtheta[i][cycarrIdx];
            sf_curpath[i]   = cycarr_curpath[i][cycarrIdx];
        }
    }
    else
    {
        // Modify the filter radius to 5 at 17:49:18 on March 15, 2018
        float sortarr[MEDIAN_FILTERLEN]; // Circular array
        cycarr_trueIdx = (int)((MEDIAN_FILTERLEN - 1) / 2);
        // Later contiguous data
        memcpy(sortarr, cycarr_fsr, MEDIAN_FILTERLEN * sizeof(float));
        sort(sortarr, 0, MEDIAN_FILTERLEN - 1);
        cur_fsr = (float)sortarr[cycarr_trueIdx];

        memcpy(sortarr, cycarr_fsl, MEDIAN_FILTERLEN * sizeof(float));
        sort(sortarr, 0, MEDIAN_FILTERLEN - 1);
        cur_fsl = (float)sortarr[cycarr_trueIdx];

        memcpy(sortarr, cycarr_rsr, MEDIAN_FILTERLEN * sizeof(float));
        sort(sortarr, 0, MEDIAN_FILTERLEN - 1);
        cur_rsr = (float)sortarr[cycarr_trueIdx];

        memcpy(sortarr, cycarr_rsl, MEDIAN_FILTERLEN * sizeof(float));
        sort(sortarr, 0, MEDIAN_FILTERLEN - 1);
        cur_rsl = (float)sortarr[cycarr_trueIdx];

        /* Current cur_x, cur_y, cur_theta, cur_path are the positions of the first four
           frames, which correspond to the ultrasonic data processed.*/
        if (cycarrIdx >= cycarr_trueIdx)
        {
            cycarr_trueIdx = cycarrIdx - (int)((MEDIAN_FILTERLEN - 1) / 2);
        }
        else
        {
            // Four data backwards, stored in cycarr size 9, is equivalent to five forward
            cycarr_trueIdx = cycarrIdx + (int)((MEDIAN_FILTERLEN + 1) / 2);
        }
        cur_fsr_2nd = cycarr_fsr_2nd[cycarr_trueIdx];
        cur_fsl_2nd = cycarr_fsl_2nd[cycarr_trueIdx];
        cur_rsl_2nd = cycarr_rsl_2nd[cycarr_trueIdx];
        cur_rsr_2nd = cycarr_rsr_2nd[cycarr_trueIdx];

        cur_fsr_peak = cycarr_fsr_peak[cycarr_trueIdx];
        cur_fsl_peak = cycarr_fsl_peak[cycarr_trueIdx];
        cur_rsr_peak = cycarr_rsr_peak[cycarr_trueIdx];
        cur_rsl_peak = cycarr_rsl_peak[cycarr_trueIdx];

        cur_fsr_peak_2nd = cycarr_fsr_peak_2nd[cycarr_trueIdx];
        cur_fsl_peak_2nd = cycarr_fsl_peak_2nd[cycarr_trueIdx];
        cur_rsr_peak_2nd = cycarr_rsr_peak_2nd[cycarr_trueIdx];
        cur_rsl_peak_2nd = cycarr_rsl_peak_2nd[cycarr_trueIdx];

        cur_fsr_spd = cycarr_fsr_spd[cycarr_trueIdx];
        cur_fsl_spd = cycarr_fsl_spd[cycarr_trueIdx];
        cur_rsr_spd = cycarr_rsr_spd[cycarr_trueIdx];
        cur_rsl_spd = cycarr_rsl_spd[cycarr_trueIdx];

        for (i = 0; i < 4; ++i)
        {
            sf_curlocalx[i] = cycarr_curx[i][cycarr_trueIdx];
            sf_curlocaly[i] = cycarr_cury[i][cycarr_trueIdx];
            sf_curtheta[i]  = cycarr_curtheta[i][cycarr_trueIdx];
            sf_curpath[i]   = cycarr_curpath[i][cycarr_trueIdx];
        }
    }

    // extract the sf_before_path[4] from the special time stamp by which pk_com was
    // changed from MCOM_OFF to MCOM_ON.
    if ((PK_SF_com_last != MCOM_ON && PK_SF_com == MCOM_ON) && TimeSeries_PK_SF > 0)
    {
        memcpy(sf_before_path, sf_curpath, sizeof(sf_before_path));
    }
}

int GetObsLocation2(float aerfa, float deltax, float deltay, float invalidTh, float Radio,
                    float localx, float localy, float theta, float *x_detected_out,
                    float *y_detected_out)
{
    float x_detected = 0, y_detected = 0;
    float sintemp1, costemp1, sintemp2, costemp2;

    sintemp1 = sinf(aerfa);
    costemp1 = cosf(aerfa);

    sintemp2 = sinf(theta);
    costemp2 = cosf(theta);

    // Vehicle coordinates of ultrasonic detection points
    x_detected = deltax + Radio / 1000.f * costemp1;
    y_detected = deltay + Radio / 1000.f * sintemp1;

    *x_detected_out = x_detected * costemp2 - y_detected * sintemp2 + localx;
    *y_detected_out = x_detected * sintemp2 + y_detected * costemp2 + localy;
    return 1;
}

void UDetectCal(Point_T Pt0, float theta0, float deltax, float deltay, float aerfa,
                float dis_detected, Point_T *pPt_detected_t)
{
    Point_T Pt_detected = {0, 0};
    // Vehicle coordinates of ultrasonic detection points
    Pt_detected.x = deltax + dis_detected / 1000.f * cosf(aerfa);
    Pt_detected.y = deltay + dis_detected / 1000.f * sinf(aerfa);
    // Odometry coordinates of ultrasonic detection points
    pPt_detected_t->x =
        Pt_detected.x * cosf(theta0) - Pt_detected.y * sinf(theta0) + Pt0.x;
    pPt_detected_t->y =
        Pt_detected.x * sinf(theta0) + Pt_detected.y * cosf(theta0) + Pt0.y;
}

void CalAllDetected_12(float udata_dir[12], float udata_indir[8],
                       Point_T *pcurudata_detected)
{
    // //B process calculation parameters
    // //前方四个探头的坐标
    // Point_T Pt_ultraXYArr[4] = { { 0,0 },{ 0,0 },{ 0,0 },{ 0,0 } };      //Location of
    // four probes in front
    // //前方四个探头的测量值
    // float udataArr[8] = { 0,0,0,0,0,0,0,0 };                 //Measurements of four
    // probes in front
    // //前方四个探头计算的障碍物世界坐标
    // Point_T Pt_xl_geo = { 0,0 }, Pt_xr_geo = { 0,0 }, Pt_xc_geo = { 0,0 };   //Geodetic
    // coordinates of obstacles calculated by four probes in front

    //////////////////////////////////////////////////////////////////////////
    // Calculating the data of four lateral probes in process A
    // 计算A过程中四个横向探头的数据
    //////////////////////////////////////////////////////////////////////////
    int i                = 0;
    float x_detected_out = 0, y_detected_out = 0;
    for (i = 0; i < 12; ++i)
    {
        pcurudata_detected[i].x = 0;
        pcurudata_detected[i].y = 0;
    }

    // Lateral probe侧边探头(长波超声雷达)
    GetObsLocation2(FSR_AERFA, FSR_DELTAX, FSR_DELTAY, SIDE_INVALID, udata_dir[0],
                    sf_curlocalx[0], sf_curlocaly[0], sf_curtheta[0], &x_detected_out,
                    &y_detected_out);
    pcurudata_detected[0].x = x_detected_out;
    pcurudata_detected[0].y = y_detected_out;

    GetObsLocation2(FSL_AERFA, FSL_DELTAX, FSL_DELTAY, SIDE_INVALID, udata_dir[1],
                    sf_curlocalx[1], sf_curlocaly[1], sf_curtheta[1], &x_detected_out,
                    &y_detected_out);
    pcurudata_detected[1].x = x_detected_out;
    pcurudata_detected[1].y = y_detected_out;

    GetObsLocation2(RSL_AERFA, RSL_DELTAX, RSL_DELTAY, SIDE_INVALID, udata_dir[2],
                    sf_curlocalx[2], sf_curlocaly[2], sf_curtheta[2], &x_detected_out,
                    &y_detected_out);
    pcurudata_detected[2].x = x_detected_out;
    pcurudata_detected[2].y = y_detected_out;

    GetObsLocation2(RSR_AERFA, RSR_DELTAX, RSR_DELTAY, SIDE_INVALID, udata_dir[3],
                    sf_curlocalx[3], sf_curlocaly[3], sf_curtheta[3], &x_detected_out,
                    &y_detected_out);
    pcurudata_detected[3].x = x_detected_out;
    pcurudata_detected[3].y = y_detected_out;

#if 0
    //////////////////////////////////////////////////////////////////////////
    // Calculating the geodetic coordinates of the four probe points ahead
    //////////////////////////////////////////////////////////////////////////
    Pt_ultraXYArr[0].x = FOL_DELTAX;
    Pt_ultraXYArr[0].y = FOL_DELTAY;
    Pt_ultraXYArr[1].x = FCL_DELTAX;
    Pt_ultraXYArr[1].y = FCL_DELTAY;
    Pt_ultraXYArr[2].x = FCR_DELTAX;
    Pt_ultraXYArr[2].y = FCR_DELTAY;
    Pt_ultraXYArr[3].x = FOR_DELTAX;
    Pt_ultraXYArr[3].y = FOR_DELTAY;
    udataArr[0] = udata_dir[4];
    udataArr[1] = udata_dir[5];
    udataArr[2] = udata_dir[6];
    udataArr[3] = udata_dir[7];
    udataArr[4] = udata_indir[0];
    udataArr[5] = udata_indir[1];
    udataArr[6] = udata_indir[2];
    udataArr[7] = udata_indir[3];

    // In this case, the corresponding odometer coordinates should be used instead of sf_curlocalx [1], 20180718.
    UltraGeoCal(Pt_ultraXYArr, udataArr, sf_curlocalx[1], sf_curlocaly[1], sf_curtheta[1], TRUE, &Pt_xl_geo, &Pt_xr_geo, &Pt_xc_geo);

    pcurudata_detected[4].x = Pt_xl_geo.x;
    pcurudata_detected[4].y = Pt_xl_geo.y;
    pcurudata_detected[6].x = Pt_xc_geo.x;
    pcurudata_detected[6].y = Pt_xc_geo.y;
    pcurudata_detected[7].x = Pt_xr_geo.x;
    pcurudata_detected[7].y = Pt_xr_geo.y;

    //////////////////////////////////////////////////////////////////////////
    // Calculating Geodetic Coordinates of Four Probe Points Behind
    //////////////////////////////////////////////////////////////////////////
    Pt_ultraXYArr[0].x = ROL_DELTAX;
    Pt_ultraXYArr[0].y = ROL_DELTAY;
    Pt_ultraXYArr[1].x = RCL_DELTAX;
    Pt_ultraXYArr[1].y = RCL_DELTAY;
    Pt_ultraXYArr[2].x = RCR_DELTAX;
    Pt_ultraXYArr[2].y = RCR_DELTAY;
    Pt_ultraXYArr[3].x = ROR_DELTAX;
    Pt_ultraXYArr[3].y = ROR_DELTAY;
    udataArr[0] = udata_dir[8];
    udataArr[1] = udata_dir[9];
    udataArr[2] = udata_dir[10];
    udataArr[3] = udata_dir[11];
    udataArr[4] = udata_indir[4];
    udataArr[5] = udata_indir[5];
    udataArr[6] = udata_indir[6];
    udataArr[7] = udata_indir[7];

    // 此处按理来讲应该要使用其对应的里程计坐标，而非sf_curlocalx[0]，20180718
    UltraGeoCal(Pt_ultraXYArr, udataArr, sf_curlocalx[0], sf_curlocaly[0], sf_curtheta[0], FALSE, &Pt_xl_geo, &Pt_xr_geo, &Pt_xc_geo);

    pcurudata_detected[8].x = Pt_xl_geo.x;
    pcurudata_detected[8].y = Pt_xl_geo.y;
    pcurudata_detected[10].x = Pt_xc_geo.x;
    pcurudata_detected[10].y = Pt_xc_geo.y;
    pcurudata_detected[11].x = Pt_xr_geo.x;
    pcurudata_detected[11].y = Pt_xr_geo.y;
#endif
}

void CalMeanStdValue(float arr[], int len, float *pMean, float *pStd)
{
    int i           = 0;
    float meanValue = 0;
    float stdValue  = 0;

    if (len <= 1)
    {
        if (pMean != NULL)
        {
            *pMean = arr[0];
        }
        if (pStd != NULL)
        {
            *pStd = 0;
        }

        return;
    }

    for (i = 0; i < len; ++i)
    {
        meanValue = meanValue + arr[i];
    }

    meanValue = meanValue / len;

    if (pMean != NULL)
    {
        *pMean = meanValue;
    }

    if (pStd != NULL)
    {
        for (i = 0; i < len; ++i)
        {
            stdValue = stdValue + (arr[i] - meanValue) * (arr[i] - meanValue);
        }

        stdValue = stdValue / (len - 1);

        stdValue = sqrtf(stdValue);

        *pStd = stdValue;
    }
}

void CalPtXYMeanVariance(Point_T PtArr[], int len, float *pMeanX, float *pVarianceX,
                         float *pMeanY, float *pVarianceY)
{
    int i            = 0;
    float meanValueX = 0, meanValueY = 0;
    float stdValueX = 0, stdValueY = 0;

    if (len <= 1)
    {
        if (pMeanX != NULL)
            *pMeanX = PtArr[0].x;
        if (pVarianceX != NULL)
            *pVarianceX = 0;
        if (pMeanY != NULL)
            *pMeanY = PtArr[0].y;
        if (pVarianceY != NULL)
            *pVarianceY = 0;
        return;
    }

    for (i = 0; i < len; ++i)
    {
        meanValueX = meanValueX + PtArr[i].x;
        meanValueY = meanValueY + PtArr[i].y;
    }

    meanValueX = meanValueX / len;
    meanValueY = meanValueY / len;

    if (pMeanX != NULL)
        *pMeanX = meanValueX;

    if (pMeanY != NULL)
        *pMeanY = meanValueY;

    if (pVarianceX != NULL)
    {
        for (i = 0; i < len; ++i)
            stdValueX = stdValueX + (PtArr[i].x - meanValueX) * (PtArr[i].x - meanValueX);

        stdValueX   = stdValueX / (len - 1);
        *pVarianceX = stdValueX;
    }

    if (pVarianceY != NULL)
    {
        for (i = 0; i < len; ++i)
            stdValueY = stdValueY + (PtArr[i].y - meanValueY) * (PtArr[i].y - meanValueY);

        stdValueY   = stdValueY / (len - 1);
        *pVarianceY = stdValueY;
    }
}

void CalMaxMinValue(float arr[], int len, float *pMax, float *pMin)
{
    int i          = 0;
    float maxvalue = 0;
    float minvalue = 0;

    if (len <= 0)
    {
        if (pMax != NULL)
        {
            *pMax = 0;
        }
        if (pMin != NULL)
        {
            *pMin = 0;
        }
        return;
    }

    maxvalue = arr[0];
    minvalue = arr[0];
    for (i = 0; i < len; ++i)
    {
        if (arr[i] > maxvalue)
        {
            maxvalue = arr[i];
        }
        if (arr[i] < minvalue)
        {
            minvalue = arr[i];
        }
    }

    if (pMax != NULL)
    {
        *pMax = maxvalue;
    }
    if (pMin != NULL)
    {
        *pMin = minvalue;
    }
}

/* Parameters */
const static float MinObjWidth = 0.5; /* mm */

const static float DistThresNear = 2400; /* mm */
const static float DistThresMid  = 3600; /* mm */
const static float DistThresFar  = 4500; /* mm */
const static int DistStatNum     = 4;

const static float SpeedThres = 4.0;

const static float RefiLimEntrRatio = 0.1;
const static float RefiLimExitRatio = 0.15;

/* [DistThresNear, DistThresMid] */
const static float RefiLimEntrMid = 0.25;
const static float RefiLimExitMid = 0.35;
/* [DistThresMid, DistThresFar] */
const static float RefiLimEntrFar = 0.15;
const static float RefiLimExitFar = 0.20;

const static unsigned char PeakThresLow1st_m = 116;
const static unsigned char PeakThresLow2nd_m = 116;
const static unsigned char PeakThresLow1st_f = 156;
const static unsigned char PeakThresLow2nd_f = 156;

const static float PeakStatWidth         = 1.0; /* [DistThresMid, DistThresFar] */
const static unsigned char PeakStatThres = 160;
const static int PeakNumThres            = 2;

/* Parameters for recognize right angle */
const static float RAngleDetectWidth     = 0.8;
const static float RAngleDetectSimiThres = 100; /* mm */
const static float RAngleDetectStepThres = 70;  /* mm */
const static float RAngleRefiLim         = 0.5;

void CalSubEdgeBy2nd_V3(Udata_Info_T *URadarInfo, int *pIdxEntr, int *pIdxExit,
                        int *pPointCnt)
{
    const int PointNum = URadarInfo->num;
    int RefiIdxEntr    = 0;
    int RefiIdxExit    = PointNum - 1;
    float SegmentLen   = 0;
    float theta        = 0;
    Point_T uv         = {0, 0};
    int hasRightAngle  = 0;
    int RefiIdxRA      = 0;
    float dist         = 0;
    float CurrLen = 0, LastLen = 0;
    unsigned char doRefine        = 0;
    float DistMean                = 0;
    float RefiLim                 = 0;
    unsigned char PeakThresLow1st = 0;
    unsigned char PeakThresLow2nd = 0;
    int PeakCnt                   = 0;
    int i                         = 0;

    /* Check the input */
    if (0 == URadarInfo->num)
    {
        *pIdxEntr  = 0;
        *pIdxExit  = 0;
        *pPointCnt = 0;
        return;
    }
    *pIdxEntr  = RefiIdxEntr;
    *pIdxExit  = RefiIdxExit;
    *pPointCnt = PointNum;

    /* Calculate the length of the point set */
    theta = (URadarInfo->curtheta[0] + URadarInfo->curtheta[PointNum - 1]) * 0.5f;
    uv.x  = cosf(theta);
    uv.y  = sinf(theta);
    SegmentLen =
        calcProjLen(&URadarInfo->obsPt[0], &URadarInfo->obsPt[PointNum - 1], &uv);

    // 起始和结束距离小于0.5m
    if (SegmentLen < MinObjWidth)
    {
        return;
    }

    /* Refine */
    /* Enter */
    theta = URadarInfo->curtheta[0];
    uv.x  = cosf(theta);
    uv.y  = sinf(theta);

    /* Detect right angle */
    hasRightAngle = 0;
    for (i = 0; i < PointNum; i++)
    {
        if (URadarInfo->value_2nd[i] < 5100)
        {
            break;
        }
    }

    if (i > 0 && i < PointNum - 1)
    {
        dist = calcProjLen(&URadarInfo->obsPt[0], &URadarInfo->obsPt[i], &uv);
        if (dist < RAngleDetectWidth)
        {
            // 两个点之间距离大于2cm，检测周期是50ms 3m/s = 3cm/10ms, 50ms = 15cm
            if (fabsf(URadarInfo->value_2nd[i] - URadarInfo->value[i - 1]) <
                    RAngleDetectSimiThres &&
                fabsf(URadarInfo->value[i] - URadarInfo->value[i - 1]) >
                    RAngleDetectStepThres &&
                fabsf(URadarInfo->value[i] - URadarInfo->value[i + 1]) <
                    RAngleDetectSimiThres)
            {
                hasRightAngle = i;
            }
        }
    }

    /* Refine right angle */
    RefiIdxRA = 0;
    if (hasRightAngle)
    {
        LastLen = 0;
        for (i = 0; i < PointNum; i++)
        {
            CurrLen = calcProjLen(&URadarInfo->obsPt[0], &URadarInfo->obsPt[i], &uv);
            if (CurrLen > RAngleRefiLim)
            {
                if ((CurrLen + LastLen) < RAngleRefiLim * 2)
                    RefiIdxRA = i;
                else
                    RefiIdxRA = i - 1;
                break;
            }
            LastLen = CurrLen;

            if (i == hasRightAngle)
            {
                RefiIdxRA = i;
                break;
            }
        }
    }

    /* General strategy */
    if (URadarInfo->start_spd < SpeedThres)
    {
        doRefine = 0;
        DistMean = calcArrMean_f(URadarInfo->value, rte_min(DistStatNum, PointNum));

        if (DistMean >= DistThresNear && DistMean < DistThresMid)
        {
            // 平均值在2.4 - 3.6 之间
            doRefine        = 1;
            RefiLim         = rte_min(RefiLimEntrMid, SegmentLen * RefiLimEntrRatio);
            PeakThresLow1st = PeakThresLow1st_m;
            PeakThresLow2nd = PeakThresLow2nd_m;
        }
        else if (DistMean >= DistThresMid && DistMean <= DistThresFar)
        {
            // 平均值在3.6 - 4.5之间
            PeakCnt = 0;
            for (i = 0; i < PointNum; i++)
            {
                CurrLen = calcProjLen(&URadarInfo->obsPt[0], &URadarInfo->obsPt[i], &uv);
                if (CurrLen >= PeakStatWidth)
                    break;
                if (URadarInfo->peak[i] > PeakStatThres ||
                    URadarInfo->peak_2nd[i] > PeakStatThres)
                    PeakCnt++;
            }
            if (PeakCnt >= PeakNumThres)
            {
                doRefine        = 1;
                RefiLim         = RefiLimEntrFar;
                PeakThresLow1st = PeakThresLow1st_f;
                PeakThresLow2nd = PeakThresLow2nd_f;
            }
        }

        if (doRefine)
        {
            LastLen = 0;
            for (i = 0; i < PointNum; i++)
            {
                CurrLen = calcProjLen(&URadarInfo->obsPt[0], &URadarInfo->obsPt[i], &uv);
                if (CurrLen > RefiLim)
                {
                    if ((CurrLen + LastLen) < RefiLim * 2)
                        RefiIdxEntr = i;
                    else
                        RefiIdxEntr = i - 1;
                    break;
                }
                LastLen = CurrLen;

                if (URadarInfo->peak[i] > PeakThresLow1st ||
                    URadarInfo->peak_2nd[i] > PeakThresLow2nd)
                {
                    RefiIdxEntr = i;
                    break;
                }
            }
        }
    }

    RefiIdxEntr = rte_max(RefiIdxEntr, RefiIdxRA);

    /* Exit */
    theta = URadarInfo->curtheta[PointNum - 1];
    uv.x  = cosf(theta);
    uv.y  = sinf(theta);

    /* Detect right angle */
    hasRightAngle = 0;
    for (i = PointNum - 1; i > 0; i--)
    {
        if (URadarInfo->value_2nd[i] < 5100)
            break;
    }
    if (i > 0 && i < PointNum - 1)
    {
        dist = calcProjLen(&URadarInfo->obsPt[PointNum - 1], &URadarInfo->obsPt[i], &uv);
        if (dist < RAngleDetectWidth)
        {
            if (fabsf(URadarInfo->value_2nd[i] - URadarInfo->value[i + 1]) <
                    RAngleDetectSimiThres &&
                fabsf(URadarInfo->value[i] - URadarInfo->value[i + 1]) >
                    RAngleDetectStepThres &&
                fabsf(URadarInfo->value[i] - URadarInfo->value[i - 1]) <
                    RAngleDetectSimiThres)
            {
                hasRightAngle = i;
            }
        }
    }

    /* Refine right angle */
    RefiIdxRA = PointNum - 1;
    if (hasRightAngle)
    {
        LastLen = 0;
        for (i = PointNum - 1; i > 0; i--)
        {
            CurrLen =
                calcProjLen(&URadarInfo->obsPt[PointNum - 1], &URadarInfo->obsPt[i], &uv);
            if (CurrLen > RAngleRefiLim)
            {
                if ((CurrLen + LastLen) < RAngleRefiLim * 2)
                    RefiIdxRA = i;
                else
                    RefiIdxRA = i + 1;
                break;
            }
            LastLen = CurrLen;

            if (i == hasRightAngle)
            {
                RefiIdxRA = i;
                break;
            }
        }
    }

    /* General strategy */
    if (URadarInfo->stop_spd < SpeedThres)
    {
        doRefine = 0;
        DistMean = calcArrMeanInv_f(&URadarInfo->value[PointNum - 1],
                                    rte_min(DistStatNum, PointNum));

        if (DistMean >= DistThresNear && DistMean < DistThresMid)
        {
            doRefine        = 1;
            RefiLim         = rte_min(RefiLimExitMid, SegmentLen * RefiLimExitRatio);
            PeakThresLow1st = PeakThresLow1st_m;
            PeakThresLow2nd = PeakThresLow2nd_m;
        }
        else if (DistMean >= DistThresMid && DistMean <= DistThresFar)
        {
            PeakCnt = 0;
            for (i = PointNum - 1; i > 0; i--)
            {
                CurrLen = calcProjLen(&URadarInfo->obsPt[PointNum - 1],
                                      &URadarInfo->obsPt[i], &uv);
                if (CurrLen >= PeakStatWidth)
                    break;
                if (URadarInfo->peak[i] > PeakStatThres ||
                    URadarInfo->peak_2nd[i] > PeakStatThres)
                    PeakCnt++;
            }
            if (PeakCnt >= PeakNumThres)
            {
                doRefine        = 1;
                RefiLim         = RefiLimExitFar;
                PeakThresLow1st = PeakThresLow1st_f;
                PeakThresLow2nd = PeakThresLow2nd_f;
            }
        }

        if (doRefine)
        {
            LastLen = 0;
            for (i = PointNum - 1; i > 0; i--)
            {
                CurrLen = calcProjLen(&URadarInfo->obsPt[PointNum - 1],
                                      &URadarInfo->obsPt[i], &uv);
                if (CurrLen > RefiLim)
                {
                    if ((CurrLen + LastLen) < RefiLim * 2)
                        RefiIdxExit = i;
                    else
                        RefiIdxExit = i + 1;
                    break;
                }
                LastLen = CurrLen;

                if (URadarInfo->peak[i] > PeakThresLow1st ||
                    URadarInfo->peak_2nd[i] > PeakThresLow2nd)
                {
                    RefiIdxExit = i;
                    break;
                }
            }
        }
    }

    RefiIdxExit = rte_min(RefiIdxExit, RefiIdxRA);

    /* Check the result */
    if (RefiIdxEntr >= RefiIdxExit)
    {
        *pIdxEntr  = 0;
        *pIdxExit  = PointNum - 1;
        *pPointCnt = PointNum;
    }
    else
    {
        *pIdxEntr  = RefiIdxEntr;
        *pIdxExit  = RefiIdxExit;
        *pPointCnt = RefiIdxExit - RefiIdxEntr + 1;
    }

    return;
}

void CalObs_part_onlyonce(Udata_Info_T *U_info, int *obj_num, int Udata_Idx,
                          float STDOFFSET, float UDELTA, float PATHDELTA,
                          U_FusionObj_T *pFusionObj)
{
    int i = 0, j = 0;
    // Udata_Info_T obs_temp;
    //  Accurate boundary value of secondary echo calculation
    int firstIdx_UData2nd = 0;
    int endIdx_Udata2end  = 0;
    int obs_realnum       = 0;

    // CalOutline_deviation_V2
    // float err1 = 0, err2 = 0;
    float err = 0;
    // The number of contour segments calculated at each time
    int contour_num = 0;
    // The contours calculated at each time are merged into the main contour Obj_InParam
    LineSeg_T contour_arr[CONTOUR_NUMMAX] = {{0, 0}, {0, 0}};

    int contour_wan[CONTOUR_NUMMAX]          = {0};
    int contour_qian[CONTOUR_NUMMAX]         = {0};
    int contour_qian_temp                    = 0;
    int contour_bai[CONTOUR_NUMMAX]          = {0};
    int contour_tenone[CONTOUR_NUMMAX]       = {0};
    int contour_tenone_temp                  = 0;
    int contour_is_over2dot5[CONTOUR_NUMMAX] = {0};
    int contour_attribute[CONTOUR_NUMMAX]    = {0};
    float contour_path[CONTOUR_NUMMAX]       = {0};
    int contour_is_Wholeobj[CONTOUR_NUMMAX]  = {0};
    float contour_theta[CONTOUR_NUMMAX]      = {0};

    float obs_theta = 0;

    float contour_path_gap = 0;
    //////////////////////////////////////////////////////////////////////////
    // initial
    // memset(&obs_temp, 0, sizeof(Udata_Info_T));
    memset(&contour_arr, 0, sizeof(contour_arr));

    if (Udata_Idx < 0 || Udata_Idx >= 4)
    {
        return;
    }

    //////////////////////////////////////////////////////////////////////////
    if (U_info->num <= 1)
    {
        memset(pFusionObj, 0, sizeof(U_FusionObj_T));
    }

    CalSubEdgeBy2nd_V3(U_info, &firstIdx_UData2nd, &endIdx_Udata2end, &obs_realnum);

    // firstIdx_UData2nd:  The subscript minimum is 0.
    // endIdx_Udata2end:   The maximum subscript is U_info.num-1
    // the last point will be kept in the CalOutline_deviation_V3 function.
    obs_realnum = U_info->num - firstIdx_UData2nd;
    CalOutline_deviation_V3(*U_info, firstIdx_UData2nd, obs_realnum, STDOFFSET, UDELTA,
                            PATHDELTA, Udata_Idx, &contour_num, contour_arr, &err);

    if (contour_num > 0)
    {
        // Calculating Obstacle Attributes
        // Ten thousand: the left side is 1.
        if (Udata_Idx == 1 || Udata_Idx == 2)
        {
            for (j = 0; j < contour_num; ++j) contour_wan[j] = 10000;
        }
        else
        {
            for (j = 0; j < contour_num; ++j) contour_wan[j] = 20000;
        }

        // CalMeanStdValue(obs_temp.curtheta, obs_temp.num, &obs_theta, NULL);
        CalMeanStdValue(&U_info->curtheta[firstIdx_UData2nd], obs_realnum, &obs_theta,
                        NULL);

        // 1000: 1 for near body, 2 for far body and roadside, 4 for 5510, 5 for the
        // latest
        if ((sf_curpath[Udata_Idx] - U_info->pathstart) > ATTRI_VD_PATH_TH)
        {
            contour_qian_temp = JudgeObsClass_V2(*U_info, firstIdx_UData2nd, obs_realnum,
                                                 obs_theta, contour_arr, contour_num);
            for (j = 0; j < contour_num; ++j)
            {
                contour_qian[j]         = contour_qian_temp;
                contour_is_over2dot5[j] = 1;
            }
        }
        else
        {
            for (j = 0; j < contour_num; ++j)
            {
                contour_qian[j]         = 5000;
                contour_is_over2dot5[j] = 0;
            }
        }

        // 100 bits: main direction output 100, this direction output 0
        CalObsMainDir(contour_arr, contour_num, obs_theta, contour_bai);

        // Ten bits, one bit: counter
        contour_tenone_temp = (*obj_num + 1) % SF_OBJ_NUM;
        for (j = 0; j < contour_num; ++j) contour_tenone[j] = contour_tenone_temp;

        for (j = 0; j < contour_num; ++j)
            contour_attribute[j] =
                contour_wan[j] + contour_qian[j] + contour_bai[j] + contour_tenone[j];

        if (Udata_Idx == 1 || Udata_Idx == 2)
        {
            for (j = 0; j < contour_num; ++j) contour_is_Wholeobj[j] = -2;
        }
        else
        {
            for (j = 0; j < contour_num; ++j) contour_is_Wholeobj[j] = -1;
        }

        for (j = 0; j < contour_num; ++j)
        {
            contour_path[j] = sf_curpath[Udata_Idx] + contour_path_gap;
            if (Udata_Idx == 0)
            {
                contour_path[j] += FSR_DELTAX - RSR_DELTAX;
            }
            else if (Udata_Idx == 1)
            {
                contour_path[j] += FSL_DELTAX - RSL_DELTAX;
            }

            contour_theta[j] = obs_theta;
        }
    }
    else
    {
        memset(&contour_attribute, 0, sizeof(contour_attribute));
        memset(&contour_path, 0, sizeof(contour_path));
        memset(&contour_is_over2dot5, 0, sizeof(contour_is_over2dot5));
        memset(&contour_is_Wholeobj, 0, sizeof(contour_is_Wholeobj));
        memset(&contour_theta, 0, sizeof(contour_theta));
    }

    //////////////////////////////////////////////////////////////////////////
    // Output final result
    pFusionObj->obj_num = contour_num;
    for (i = 0; i < contour_num; ++i)
    {
        pFusionObj->obj[i].pt1.x    = contour_arr[i].pt1.x;
        pFusionObj->obj[i].pt1.y    = contour_arr[i].pt1.y;
        pFusionObj->obj[i].pt2.x    = contour_arr[i].pt2.x;
        pFusionObj->obj[i].pt2.y    = contour_arr[i].pt2.y;
        pFusionObj->obj_attr[i]     = contour_attribute[i];
        pFusionObj->pathstart[i]    = contour_path[i];
        pFusionObj->is_Wholeobj[i]  = contour_is_Wholeobj[i];
        pFusionObj->is_over2dot5[i] = contour_is_over2dot5[i];
        pFusionObj->ang[i]          = contour_theta[i];
    }
}

void CalObs_whole(Udata_Info_T *U_info, int *obj_num, int Udata_Idx, uint8 isfront,
                  float STDOFFSET, float UDELTA, float PATHDELTA,
                  U_FusionObj_T *pFusionObj)
{
    int i = 0, j = 0;
    // Udata_Info_T obs_temp, obs_temp2;
    // Accurate boundary value of secondary echo calculation
    int firstIdx_UData2nd = 0;
    int endIdx_Udata2end  = 0;
    int obs_realnum       = 0;

    // CalOutline_deviation_V2
    float err       = 0;
    int contour_num = 0; // The number of contour segments calculated at each time
    LineSeg_T contour_arr[CONTOUR_NUMMAX] = {
        {0, 0}, {0, 0}}; // The contours calculated at each time are merged into the main
                         // contour Obj_InParam

    int contour_wan[CONTOUR_NUMMAX]    = {0};
    int contour_qian[CONTOUR_NUMMAX]   = {0};
    int contour_qian_temp              = 0;
    int contour_bai[CONTOUR_NUMMAX]    = {0};
    int contour_tenone[CONTOUR_NUMMAX] = {0};
    int contour_tenone_temp            = 0;
    // int contour_is_over2dot5[CONTOUR_NUMMAX] = { 0 };
    int contour_attribute[CONTOUR_NUMMAX]   = {0};
    float contour_path[CONTOUR_NUMMAX]      = {0};
    int contour_is_Wholeobj[CONTOUR_NUMMAX] = {0};
    float contour_theta[CONTOUR_NUMMAX]     = {0};

    float obs_theta        = 0;
    float contour_path_gap = 0;

    memset(&contour_arr, 0, sizeof(contour_arr));

    if (Udata_Idx < 0 || Udata_Idx >= 4)
        return;
    else
    {
        if (Udata_Idx == 0)
            contour_path_gap = FSR_DELTAX - RSR_DELTAX;
        else if (Udata_Idx == 1)
            contour_path_gap = FSL_DELTAX - RSL_DELTAX;
        else if (Udata_Idx == 2)
            contour_path_gap = 0;
        else
            contour_path_gap = 0;
    }
    //////////////////////////////////////////////////////////////////////////
    if (U_info->num <= 1)
        memset(pFusionObj, 0, sizeof(U_FusionObj_T));

    if (isfront)
    {
        // Accurate Boundary Location Calculated by Secondary Echo
        // CalSubEdgeBy2nd_V2(U_info->value, U_info->value_2nd, U_info->obsPt,
        // U_info->num, &firstIdx_UData2nd, &endIdx_Udata2end, &obs_realnum);
        CalSubEdgeBy2nd_V3(U_info, &firstIdx_UData2nd, &endIdx_Udata2end, &obs_realnum);
    }
    else
    {
        firstIdx_UData2nd = 0;
        endIdx_Udata2end  = U_info->num - 1;
        obs_realnum       = U_info->num;
    }

    //////////////////////////////////////////////////////////////////////////
    // Contour calculation
    CalOutline_deviation_V3(*U_info, firstIdx_UData2nd, obs_realnum, STDOFFSET, UDELTA,
                            PATHDELTA, Udata_Idx, &contour_num, contour_arr, &err);

    if (contour_num > 0)
    {
        *obj_num = *obj_num + 1;

        // Calculating Obstacle Attributes
        // Ten thousand: the left side is 1.
        if (Udata_Idx == 1 || Udata_Idx == 2)
        {
            for (j = 0; j < contour_num; ++j) contour_wan[j] = 10000;
        }
        else
        {
            for (j = 0; j < contour_num; ++j) contour_wan[j] = 20000;
        }

        // 1000 bits: 1 near body, 2 along road, 3 far body, 4 5510
        CalMeanStdValue(&U_info->curtheta[firstIdx_UData2nd], obs_realnum, &obs_theta,
                        NULL);

        contour_qian_temp = JudgeObsClass_V2(*U_info, firstIdx_UData2nd, obs_realnum,
                                             obs_theta, contour_arr, contour_num);
        for (j = 0; j < contour_num; ++j) contour_qian[j] = contour_qian_temp;

        // 100 bits: main direction output 100, this direction output 0
        CalObsMainDir(contour_arr, contour_num, obs_theta, contour_bai);

        // Ten bits, one bit: counter
        contour_tenone_temp = (*obj_num) % SF_OBJ_NUM;
        for (j = 0; j < contour_num; ++j) contour_tenone[j] = contour_tenone_temp;

        for (j = 0; j < contour_num; ++j)
            contour_attribute[j] =
                contour_wan[j] + contour_qian[j] + contour_bai[j] + contour_tenone[j];

        if (Udata_Idx == 1 || Udata_Idx == 2)
        {
            for (j = 0; j < contour_num; ++j) contour_is_Wholeobj[j] = 2;
        }
        else
        {
            for (j = 0; j < contour_num; ++j) contour_is_Wholeobj[j] = 1;
        }

        for (j = 0; j < contour_num; ++j)
        {
            contour_path[j]  = sf_curpath[Udata_Idx] + contour_path_gap;
            contour_theta[j] = obs_theta;
        }
    }

    //////////////////////////////////////////////////////////////////////////
    // Output final result
    pFusionObj->obj_num = contour_num;
    for (i = 0; i < contour_num; ++i)
    {
        pFusionObj->obj[i].pt1.x    = contour_arr[i].pt1.x;
        pFusionObj->obj[i].pt1.y    = contour_arr[i].pt1.y;
        pFusionObj->obj[i].pt2.x    = contour_arr[i].pt2.x;
        pFusionObj->obj[i].pt2.y    = contour_arr[i].pt2.y;
        pFusionObj->obj_attr[i]     = contour_attribute[i];
        pFusionObj->pathstart[i]    = contour_path[i];
        pFusionObj->is_Wholeobj[i]  = contour_is_Wholeobj[i];
        pFusionObj->is_over2dot5[i] = 0;
        pFusionObj->ang[i]          = contour_theta[i];
    }
}

uint8 CalLineSegAttribute_1sigama(Point_T Pt[], int num, Point_T *pPt_0, Point_T *pPt_1,
                                  Point_T *pPt_max, Point_T *pPt_min, Line_T *pLine,
                                  float *pStd_offset)
{
    int i;
    Point_T Pt_trans = {0, 0};
    int ptminIdx = 0, ptmaxIdx = 0, pt1Idx = 0, pt0Idx = 0;
    float aerfa    = 0;
    float sintemp1 = 0, costemp1 = 0, sintemp2 = 0, costemp2 = 0;
    float xtranstemp = 0;
    float offset_Min = 0, offset_Max = 0;
    float std_offset = 0, mean_offset = 0;
    int num_used     = 0;
    Line_T Line_used = {0, 0, 0, {0, 0}, {0, 0}};
    float aerfa_used = 0;
    Point_T Pt_mintemp, Pt_maxtemp;
    Point_T Pt_used[TIMENUM] = {0, 0};
    Line_T linetemp          = {0, 0, 0, {0, 0}, {0, 0}};

    CalLineSegAttribute(Pt, num, NULL, NULL, NULL, NULL, &linetemp, &mean_offset,
                        &std_offset);

    if (linetemp.b == 0)
        aerfa = PI_GEO / 2.0f;
    else
        aerfa = atanf(-linetemp.a / linetemp.b);

    sintemp1 = sinf(-PI_GEO / 2.0f - aerfa);
    costemp1 = cosf(-PI_GEO / 2.0f - aerfa);

    sintemp2 = sinf(PI_GEO / 2.0f - aerfa);
    costemp2 = cosf(PI_GEO / 2.0f - aerfa);

    // 1 times standard deviation correction
    offset_Min = mean_offset - std_offset;
    offset_Max = mean_offset + std_offset;

    // Filtering interference points with 1 times standard deviation

    for (i = 0; i < num; ++i)
    {
        if (aerfa >= -PI_GEO / 2.0f && aerfa <= 0)
        {
            xtranstemp = Pt[i].x * costemp1 - Pt[i].y * sintemp1;
        }
        else
        {
            xtranstemp = Pt[i].x * costemp2 - Pt[i].y * sintemp2;
        }

        if (xtranstemp >= offset_Min && xtranstemp <= offset_Max)
        {
            Pt_used[num_used].x = Pt[i].x;
            Pt_used[num_used].y = Pt[i].y;
            num_used            = num_used + 1;
        }
    }

    LineFitting_LimitedSquare2(Pt_used, num_used, &Line_used);
    // Before this function is calculated, it has been determined that there is no slope
    // of X=a in the straight line. Therefore, if the slope does not exist in
    // CalLineSegAttribute3, the double standard deviation point selection is problematic,
    // and the double standard deviation point selection is not adopted.
    if (Line_used.b == 0)
    {
        pPt_0->x     = Pt[pt0Idx].x;
        pPt_0->y     = Pt[pt0Idx].y;
        pPt_1->x     = Pt[pt1Idx].x;
        pPt_1->y     = Pt[pt1Idx].y;
        pPt_max->x   = Pt[ptmaxIdx].x;
        pPt_max->y   = Pt[ptmaxIdx].y;
        pPt_min->x   = Pt[ptminIdx].x;
        pPt_min->y   = Pt[ptminIdx].y;
        *pStd_offset = std_offset;
        return TRUE;
    }

    // Straight line direction
    if (Line_used.b == 0)
    {
        aerfa_used = PI_GEO / 2;
    }
    else
    {
        aerfa_used = atanf(-Line_used.a / Line_used.b);
    }

    sintemp1 = sinf(-PI_GEO / 2.0f - aerfa_used);
    costemp1 = cosf(-PI_GEO / 2.0f - aerfa_used);

    sintemp2 = sinf(PI_GEO / 2.0f - aerfa_used);
    costemp2 = cosf(PI_GEO / 2.0f - aerfa_used);

    // Calculate the Y direction first, and all the points are involved in calculating the
    // boundary value.
    for (i = 0; i < num; ++i)
    {
        // Transform the principal axis to the Y axis, point distribution along the Y
        // axis, all points are converted.
        //  Slope k <= 0
        if (aerfa_used >= -PI_GEO / 2.0f && aerfa_used <= 0)
        {
            Pt_trans.x = Pt[i].x * costemp1 - Pt[i].y * sintemp1;
            Pt_trans.y = Pt[i].x * sintemp1 + Pt[i].y * costemp1;
        }
        else
        {
            // Slope k > 0
            Pt_trans.x = Pt[i].x * costemp2 - Pt[i].y * sintemp2;
            Pt_trans.y = Pt[i].x * sintemp2 + Pt[i].y * costemp2;
        }

        if (i == 0)
        {
            Pt_mintemp.y = Pt_trans.y;
            Pt_maxtemp.y = Pt_trans.y;
            pt1Idx       = 0;
            pt0Idx       = 0;
        }
        else
        {
            if (Pt_trans.y > Pt_maxtemp.y)
            {
                Pt_maxtemp.y = Pt_trans.y;
                pt1Idx       = i;
            }
            if (Pt_trans.y < Pt_mintemp.y)
            {
                Pt_mintemp.y = Pt_trans.y;
                pt0Idx       = i;
            }
        }
    }

    // When X-direction is calculated, only points within one standard deviation are
    // involved in the calculation of boundary values.
    for (i = 0; i < num_used; ++i)
    {
        // Transform the principal axis to the Y axis. Points are distributed along the Y
        // axis. Points within one standard deviation are converted. Slope k <= 0
        if (aerfa_used >= -PI_GEO / 2.0f && aerfa_used <= 0)
        {
            Pt_trans.x = Pt_used[i].x * costemp1 - Pt_used[i].y * sintemp1;
            Pt_trans.y = Pt_used[i].x * sintemp1 + Pt_used[i].y * costemp1;
        }
        else
        {
            // Slope k > 0
            Pt_trans.x = Pt_used[i].x * costemp2 - Pt_used[i].y * sintemp2;
            Pt_trans.y = Pt_used[i].x * sintemp2 + Pt_used[i].y * costemp2;
        }

        if (i == 0)
        {
            Pt_mintemp.x = Pt_trans.x;
            Pt_maxtemp.x = Pt_trans.x;
            ptminIdx     = 0;
            ptmaxIdx     = 0;
        }
        else
        {
            if (Pt_trans.x < Pt_mintemp.x)
            {
                Pt_mintemp.x = Pt_trans.x;
                ptminIdx     = i;
            }
            if (Pt_trans.x > Pt_maxtemp.x)
            {
                Pt_maxtemp.x = Pt_trans.x;
                ptmaxIdx     = i;
            }
        }
    }

    // Result output
    if (pt0Idx < pt1Idx)
    {
        pPt_0->x   = Pt[pt0Idx].x;
        pPt_0->y   = Pt[pt0Idx].y;
        pPt_1->x   = Pt[pt1Idx].x;
        pPt_1->y   = Pt[pt1Idx].y;
        pPt_max->x = Pt_used[ptmaxIdx].x;
        pPt_max->y = Pt_used[ptmaxIdx].y;
        pPt_min->x = Pt_used[ptminIdx].x;
        pPt_min->y = Pt_used[ptminIdx].y;
    }
    else
    {
        pPt_0->x   = Pt[pt1Idx].x;
        pPt_0->y   = Pt[pt1Idx].y;
        pPt_1->x   = Pt[pt0Idx].x;
        pPt_1->y   = Pt[pt0Idx].y;
        pPt_max->x = Pt_used[ptminIdx].x;
        pPt_max->y = Pt_used[ptminIdx].y;
        pPt_min->x = Pt_used[ptmaxIdx].x;
        pPt_min->y = Pt_used[ptmaxIdx].y;
    }

    pLine->a     = Line_used.a;
    pLine->b     = Line_used.b;
    pLine->c     = Line_used.c;
    *pStd_offset = std_offset;
    return TRUE;
}

void CalSubOutline_V2(float curlocalx, float curlocaly, uint8 be_XYAxisInterchanged,
                      Point_T PtArr[TIMENUM], int num, Point_T *pPt_1_out,
                      Point_T *pPt_2_out)
{
    int i       = 0;
    Point_T Pt0 = {0, 0}, Pt1 = {0, 0}, Ptmax = {0, 0}, Ptmin = {0, 0};
    Line_T Line = {0, 0, 0, {0, 0}, {0, 0}}, Linenear = {0, 0, 0, {0, 0}, {0, 0}};
    float stdoffset = 0;
    int onside      = 0;
    Point_T Ptcur   = {0, 0};
    Point_T Ptnear  = {0, 0};
    float temp      = 0;

    if (NULL == pPt_1_out || NULL == pPt_2_out || num > TIMENUM)
    {
        pPt_1_out->x = 0;
        pPt_1_out->y = 0;
        pPt_2_out->x = 0;
        pPt_2_out->y = 0;
    }

    if (1U == be_XYAxisInterchanged)
    {
        // interchange the XY Axis
        for (i = 0; i < num; ++i)
        {
            temp       = PtArr[i].x;
            PtArr[i].x = PtArr[i].y;
            PtArr[i].y = temp;
        }
    }

    // Calculate the two endpoints of the fitting line segment, the two farthest points,
    // and the deviation error
    CalLineSegAttribute_1sigama(PtArr, num, &Pt0, &Pt1, &Ptmax, &Ptmin, &Line,
                                &stdoffset);

    Ptcur.x = curlocalx;
    Ptcur.y = curlocaly;
    onside  = GetPtLoc2(Pt0, Pt1, Ptcur);

    // Left
    if (onside == 1)
    {
        Ptnear.x = Ptmin.x;
        Ptnear.y = Ptmin.y;
    }
    else
    {
        Ptnear.x = Ptmax.x;
        Ptnear.y = Ptmax.y;
    }

    // The linear equation is determined b y the point oblique formula.
    // The point is near_x, near_y, and the direction vector is line_a line_b.
    // Linenear_a, linenear_b, linenear_c of the determined linear equation
    Linenear.a = Line.a;
    Linenear.b = Line.b;
    Linenear.c = -Linenear.a * Ptnear.x - Linenear.b * Ptnear.y;

    if (Linenear.a == 0 && Linenear.b == 0 && Linenear.c == 0)
    {
        if (1U == be_XYAxisInterchanged)
        {
            pPt_1_out->x = Pt0.y;
            pPt_1_out->y = Pt0.x;
            pPt_2_out->x = Pt1.y;
            pPt_2_out->y = Pt1.x;
        }
        else
        {
            pPt_1_out->x = Pt0.x;
            pPt_1_out->y = Pt0.y;
            pPt_2_out->x = Pt1.x;
            pPt_2_out->y = Pt1.y;
        }

#ifdef _ULTRADEBUG
        // printf("function CalSubOutline ERROR!\r\n");
#endif
        return;
    }

    GetNormalPt2(Linenear, Pt0, pPt_1_out);
    GetNormalPt2(Linenear, Pt1, pPt_2_out);

    if (1U == be_XYAxisInterchanged)
    {
        // recover the XY Axis
        for (i = 0; i < num; ++i)
        {
            temp       = PtArr[i].x;
            PtArr[i].x = PtArr[i].y;
            PtArr[i].y = temp;
        }

        temp         = pPt_1_out->x;
        pPt_1_out->x = pPt_1_out->y;
        pPt_1_out->y = temp;

        temp         = pPt_2_out->x;
        pPt_2_out->x = pPt_2_out->y;
        pPt_2_out->y = temp;
    }
}

static void PK_SF_A_URadar_Save(const Udata_Info_T &obs, int firstIdx, int realnum,
                                float StdOffsetTh, float udeltaTh, float pathdeltaTh,
                                int Udata_Idx, int flag)
{
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);
    snprintf(fileName, sizeof(fileName) - 1, "log/%d_%02d_%02d_%02d_%02d_%02d_%s",
             absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
             absStamp.tm_min, absStamp.tm_sec, "objover.txt");
    FILE *logFile = fopen(fileName, "w+");

    struct timeval tv;
    gettimeofday(&tv, (struct timezone *)0);
    fwrite(&tv, sizeof(tv), 1, logFile);
    fwrite(&obs, sizeof(Udata_Info_T), 1, logFile);
    fwrite(&firstIdx, sizeof(firstIdx), 1, logFile);
    fwrite(&realnum, sizeof(realnum), 1, logFile);
    fwrite(&StdOffsetTh, sizeof(StdOffsetTh), 1, logFile);
    fwrite(&udeltaTh, sizeof(udeltaTh), 1, logFile);
    fwrite(&pathdeltaTh, sizeof(pathdeltaTh), 1, logFile);
    fwrite(&Udata_Idx, sizeof(Udata_Idx), 1, logFile);
    fwrite(&flag, sizeof(flag), 1, logFile);
    fclose(logFile);
}

// version 3 is for the new interface of obs, which has the first index and the number
uint8 CalOutline_deviation_V3(Udata_Info_T obs, int firstIdx, int realnum,
                              float StdOffsetTh, float udeltaTh, float pathdeltaTh,
                              int Udata_Idx, int *pContour_num, LineSeg_T *pContour_arr,
                              float *pDeviation)
{
    int i = 0, j = 0;
    int num              = 0;
    float dis_squre      = 0;
    Point_T Pt_temp      = {0, 0};
    static int writeFile = 0;

    float err1 = 0, err2 = 0;
    int contour_num1 = 0; // The number of contour segments calculated at each time
    LineSeg_T contour_arr1[CONTOUR_NUMMAX] = {
        {0, 0}, {0, 0}};  // The contours calculated at each time are merged into the main
                          // contour Obj_InParam
    int contour_num2 = 0; // The number of contour segments calculated at each time
    LineSeg_T contour_arr2[CONTOUR_NUMMAX] = {
        {0, 0}, {0, 0}}; // The contours calculated at each time are merged into the main
                         // contour Obj_InParam

    float udelta          = 0;
    float pathdelta_squre = 0;
    float stdValueX       = 0;
    float meanValueX      = 0;
    float stdValueY       = 0;
    float meanValueY      = 0;

    Point_T Pt1_out = {0, 0}, Pt2_out = {0, 0};
    int beadd = 0;

    // CalLineSegAttribute Using temporary variables
    Point_T cur[TIMENUM];    // Point set currently processed
    float curudata[TIMENUM]; // Current ultrasonic value

    float Std_offset = 0;

    Line_T LineDeviation = {0, 0, 0, {0, 0}, {0, 0}};

    int contour_numIdx = 0;

    float sf_curlocalx_temp = 0, sf_curlocaly_temp = 0;

    if (Udata_Idx < 0 || Udata_Idx >= 4)
    {
        return 0;
    }

    sf_curlocalx_temp = sf_curlocalx[Udata_Idx];
    sf_curlocaly_temp = sf_curlocaly[Udata_Idx];

    for (i = 0; i < TIMENUM; ++i)
    {
        cur[i].x    = 0;
        cur[i].y    = 0;
        curudata[i] = 0;
    }

    *pContour_num = 0;

    if (realnum < 2)
    {
        return FALSE;
    }

    if (realnum == 2)
    {
        pContour_arr[*pContour_num].pt1.x = obs.obsPt[firstIdx].x;
        pContour_arr[*pContour_num].pt1.y = obs.obsPt[firstIdx].y;
        pContour_arr[*pContour_num].pt2.x = obs.obsPt[firstIdx + realnum - 1].x;
        pContour_arr[*pContour_num].pt2.y = obs.obsPt[firstIdx + realnum - 1].y;
        *pContour_num                     = 1;
        return TRUE;
    }

    //////////////////////////////////////////////////////////////////////////
    // forward
    for (i = firstIdx; i < firstIdx + realnum; ++i)
    {
        if (fpclassify(obs.obsPt[i].x) == FP_NAN ||
            fpclassify(obs.obsPt[i].x) == FP_INFINITE)
        {
            continue;
        }
        if (fpclassify(obs.obsPt[i].y) == FP_NAN ||
            fpclassify(obs.obsPt[i].y) == FP_INFINITE)
        {
            continue;
        }
        // 障碍物之间距离

        if (num == 0)
        {
            Pt_temp.x = obs.obsPt[i].x;
            Pt_temp.y = obs.obsPt[i].y;

            cur[num].x    = obs.obsPt[i].x;
            cur[num].y    = obs.obsPt[i].y;
            curudata[num] = obs.value[i];
            num           = 1;
            continue;
        }
        dis_squre = (obs.obsPt[i].x - Pt_temp.x) * (obs.obsPt[i].x - Pt_temp.x) +
                    (obs.obsPt[i].y - Pt_temp.y) * (obs.obsPt[i].y - Pt_temp.y);
        Pt_temp.x = obs.obsPt[i].x;
        Pt_temp.y = obs.obsPt[i].y;
        if (dis_squre < SAMPLE_DISTH * SAMPLE_DISTH && i != (firstIdx + realnum - 1))
        {
            continue;
        }

        CalLineSegAttribute(cur, num, NULL, NULL, NULL, NULL, NULL, NULL, &Std_offset);

        // 测量距离差
        udelta = fabsf(obs.value[i] - curudata[num - 1]);

        // 障碍物之间距离比测量距离大，说明车辆有移动
        if (dis_squre > ((udelta / 1000.f) * (udelta / 1000.f)))
            pathdelta_squre = dis_squre - (udelta / 1000.f) * (udelta / 1000.f);
        else
            pathdelta_squre = 0;

        // 方差小于0.08m, 测量距离小于240mm， 行驶距离小于50cm           STDOFFSET_TH,
        // UDELTA_TH, PATHDELTA_TH
        if (Std_offset < StdOffsetTh && num < TIMENUM && udelta < udeltaTh &&
            pathdelta_squre < pathdeltaTh * pathdeltaTh)
            beadd = 1;
        else
            beadd = 0;

        // February 23, 2018 15:41:38
        // This situation belongs to the situation that the first point is separated from
        // other points separately, and the solitary point will cause the fitting line to
        // be right-angled, so this situation does not divide the set of points.
        if (beadd == 0 && i == firstIdx + 1)
        {
            beadd = 1;
        }

        if (beadd && i != (firstIdx + realnum - 1))
        {
            cur[num].x    = obs.obsPt[i].x;
            cur[num].y    = obs.obsPt[i].y;
            curudata[num] = obs.value[i];
            num           = num + 1;
        }
        else
        {
            // To meet the requirement of continuous point set segmentation of obstacle
            // contour
            if (/*beadd &&*/ i == (firstIdx + realnum - 1))
            {
                cur[num].x    = obs.obsPt[i].x;
                cur[num].y    = obs.obsPt[i].y;
                curudata[num] = obs.value[i];
                num           = num + 1;
            }

            if (num > 1)
            {
                CalPtXYMeanVariance(cur, num, &meanValueX, &stdValueX, &meanValueY,
                                    &stdValueY);

                Pt1_out.x = 0.0, Pt1_out.y = 0.0;
                Pt2_out.x = 0.0, Pt2_out.y = 0.0;
                if (stdValueX < stdValueY)
                    CalSubOutline_V2(sf_curlocaly_temp, sf_curlocalx_temp, 1, cur, num,
                                     &Pt1_out, &Pt2_out);
                else
                    CalSubOutline_V2(sf_curlocalx_temp, sf_curlocaly_temp, 0, cur, num,
                                     &Pt1_out, &Pt2_out);

                dis_squre = (Pt1_out.x - Pt2_out.x) * (Pt1_out.x - Pt2_out.x) +
                            (Pt1_out.y - Pt2_out.y) * (Pt1_out.y - Pt2_out.y);
                if (dis_squre < SAMPLE_DISTH * SAMPLE_DISTH)
                {
                    log_warn("obj error! i = %d num %d start: %06f %06f end:%06f %06f\n",
                             i, num, cur[0].x, cur[0].y, cur[num - 1].x, cur[num - 1].y);
                    memset(&cur[0], 0, sizeof(Point_T) * (num));
                    memset(&curudata[0], 0, sizeof(float) * (num));
                    cur[0].x    = obs.obsPt[i].x;
                    cur[0].y    = obs.obsPt[i].y;
                    curudata[0] = obs.value[i];
                    num         = 1;
                    continue;
                }
                // Statistical deviation
                Getlineabc2(Pt1_out, Pt2_out, &LineDeviation);
                for (j = 0; j < num; j++) err1 = err1 + DistToPt2(LineDeviation, cur[j]);

                if (contour_num1 >= CONTOUR_NUMMAX && writeFile != 1)
                {
                    PK_SF_A_URadar_Save(obs, firstIdx, realnum, StdOffsetTh, udeltaTh,
                                        pathdeltaTh, Udata_Idx, 1);
                    writeFile = 1;
                }

                // For looping into contour_arr to prevent overflow
                contour_numIdx                     = contour_num1 % CONTOUR_NUMMAX;
                contour_arr1[contour_numIdx].pt1.x = Pt1_out.x;
                contour_arr1[contour_numIdx].pt1.y = Pt1_out.y;
                contour_arr1[contour_numIdx].pt2.x = Pt2_out.x;
                contour_arr1[contour_numIdx].pt2.y = Pt2_out.y;
                contour_num1                       = contour_num1 + 1;
            }
            else
            {
                // Processing of a single point
                // A non-terminal point connects the next point and a terminal point
                // connects the previous point.
                if (i >= firstIdx + 1 && i <= firstIdx + realnum - 2)
                {
                    if (contour_num1 >= CONTOUR_NUMMAX && writeFile != 2)
                    {
                        PK_SF_A_URadar_Save(obs, firstIdx, realnum, StdOffsetTh, udeltaTh,
                                            pathdeltaTh, Udata_Idx, 2);
                        writeFile = 2;
                    }
                    contour_numIdx                     = contour_num1 % CONTOUR_NUMMAX;
                    contour_arr1[contour_numIdx].pt1.x = obs.obsPt[i - 1].x;
                    contour_arr1[contour_numIdx].pt1.y = obs.obsPt[i - 1].y;
                    contour_arr1[contour_numIdx].pt2.x = obs.obsPt[i].x;
                    contour_arr1[contour_numIdx].pt2.y = obs.obsPt[i].y;
                    contour_num1                       = contour_num1 + 1;
                }
            }

            if (i == firstIdx + realnum - 1)
            {
                break;
            }

            // The following is to reinitialize the data
            // Here is a strategy to add a gap link between two segments, underlined as a
            // gap part.
            if (udelta < udeltaTh && pathdelta_squre < pathdeltaTh * pathdeltaTh)
            {
                //  ....._''''
                // The following code is added  ._''''
                cur[0].x    = cur[num - 1].x;
                cur[0].y    = cur[num - 1].y;
                curudata[0] = curudata[num - 1];
                memset(&cur[1], 0, sizeof(Point_T) * (num - 1));
                memset(&curudata[1], 0, sizeof(float) * (num - 1));
                cur[1].x    = obs.obsPt[i].x;
                cur[1].y    = obs.obsPt[i].y;
                curudata[1] = obs.value[i];
                num         = 2;
            }
            else
            {
                // The following code is added ''''
                memset(&cur[0], 0, sizeof(Point_T) * (num));
                memset(&curudata[0], 0, sizeof(float) * (num));
                cur[0].x    = obs.obsPt[i].x;
                cur[0].y    = obs.obsPt[i].y;
                curudata[0] = obs.value[i];
                num         = 1;
            }
        }
    } // end of for of the forward

    //////////////////////////////////////////////////////////////////////////
    // backward
    memset(&cur[0], 0, sizeof(Point_T) * (TIMENUM));
    memset(&curudata[0], 0, sizeof(float) * (TIMENUM));
    Pt_temp.x = 0;
    Pt_temp.y = 0;
    num       = 0;

    for (i = firstIdx + realnum - 1; i >= firstIdx; --i)
    {
        if (fpclassify(obs.obsPt[i].x) == FP_NAN ||
            fpclassify(obs.obsPt[i].x) == FP_INFINITE)
        {
            continue;
        }
        if (fpclassify(obs.obsPt[i].y) == FP_NAN ||
            fpclassify(obs.obsPt[i].y) == FP_INFINITE)
        {
            continue;
        }

        if (num == 0)
        {
            Pt_temp.x = obs.obsPt[i].x;
            Pt_temp.y = obs.obsPt[i].y;

            cur[num].x    = obs.obsPt[i].x;
            cur[num].y    = obs.obsPt[i].y;
            curudata[num] = obs.value[i];
            num           = 1;
            continue;
        }

        dis_squre = (obs.obsPt[i].x - Pt_temp.x) * (obs.obsPt[i].x - Pt_temp.x) +
                    (obs.obsPt[i].y - Pt_temp.y) * (obs.obsPt[i].y - Pt_temp.y);
        Pt_temp.x = obs.obsPt[i].x;
        Pt_temp.y = obs.obsPt[i].y;
        if (dis_squre < SAMPLE_DISTH * SAMPLE_DISTH &&
            i != firstIdx) // February 23, 2018 16:16:28, the next computer is len-1, May
                           // 2, 2018 12:54:27
            continue;

        CalLineSegAttribute(cur, num, NULL, NULL, NULL, NULL, NULL, NULL, &Std_offset);

        udelta = fabsf(obs.value[i] - curudata[num - 1]);
        if (dis_squre > ((udelta / 1000.f) * (udelta / 1000.f)))
            pathdelta_squre = dis_squre - (udelta / 1000.f) * (udelta / 1000.f);
        else
            pathdelta_squre = 0;

        if (Std_offset < StdOffsetTh && num < TIMENUM && udelta < udeltaTh &&
            pathdelta_squre < pathdeltaTh * pathdeltaTh)
            beadd = 1;
        else
            beadd = 0;

        // February 23, 2018 15:41:38
        // This situation belongs to the situation that the first point is separated from
        // other points separately, and the solitary point will cause the fitting line to
        // be right-angled, so this situation does not divide the set of points.
        if (beadd == 0 && i == firstIdx + realnum - 2)
            beadd = 1;

        if (beadd && i != firstIdx)
        {
            cur[num].x    = obs.obsPt[i].x;
            cur[num].y    = obs.obsPt[i].y;
            curudata[num] = obs.value[i];
            num           = num + 1;
        }
        else
        {
            // To meet the requirement of continuous point set segmentation of obstacle
            // contour
            if (/*beadd &&*/ i == firstIdx)
            {
                cur[num].x    = obs.obsPt[i].x;
                cur[num].y    = obs.obsPt[i].y;
                curudata[num] = obs.value[i];
                num           = num + 1;
            }

            if (num > 1)
            {
                CalPtXYMeanVariance(cur, num, &meanValueX, &stdValueX, &meanValueY,
                                    &stdValueY);

                Pt1_out.x = 0.0, Pt1_out.y = 0.0;
                Pt2_out.x = 0.0, Pt2_out.y = 0.0;
                if (stdValueX < stdValueY)
                    CalSubOutline_V2(sf_curlocaly_temp, sf_curlocalx_temp, 1, cur, num,
                                     &Pt1_out, &Pt2_out);
                else
                    CalSubOutline_V2(sf_curlocalx_temp, sf_curlocaly_temp, 0, cur, num,
                                     &Pt1_out, &Pt2_out);

                dis_squre = (Pt1_out.x - Pt2_out.x) * (Pt1_out.x - Pt2_out.x) +
                            (Pt1_out.y - Pt2_out.y) * (Pt1_out.y - Pt2_out.y);
                if (dis_squre < SAMPLE_DISTH * SAMPLE_DISTH)
                {
                    log_warn("obj error! i = %d num %d start: %06f %06f end:%06f %06f\n",
                             i, num, cur[0].x, cur[0].y, cur[num - 1].x, cur[num - 1].y);
                    memset(&cur[0], 0, sizeof(Point_T) * (num));
                    memset(&curudata[0], 0, sizeof(float) * (num));
                    cur[0].x    = obs.obsPt[i].x;
                    cur[0].y    = obs.obsPt[i].y;
                    curudata[0] = obs.value[i];
                    num         = 1;
                    continue;
                }

                // Statistical deviation
                Getlineabc2(Pt1_out, Pt2_out, &LineDeviation);
                for (j = 0; j < num; j++) err2 = err2 + DistToPt2(LineDeviation, cur[j]);

                if (contour_numIdx >= CONTOUR_NUMMAX && writeFile != 3)
                {
                    PK_SF_A_URadar_Save(obs, firstIdx, realnum, StdOffsetTh, udeltaTh,
                                        pathdeltaTh, Udata_Idx, 3);
                    writeFile = 3;
                }

                // For looping into contour_arr to prevent overflow
                contour_numIdx                     = contour_num2 % CONTOUR_NUMMAX;
                contour_arr2[contour_numIdx].pt1.x = Pt1_out.x;
                contour_arr2[contour_numIdx].pt1.y = Pt1_out.y;
                contour_arr2[contour_numIdx].pt2.x = Pt2_out.x;
                contour_arr2[contour_numIdx].pt2.y = Pt2_out.y;
                contour_num2                       = contour_num2 + 1;
            }
            else
            {
                // Processing of a single point
                // A non-terminal point connects the next point and a terminal point
                // connects the previous point.
                if (i >= firstIdx + 1 && i <= firstIdx + realnum - 2)
                {
                    if (contour_numIdx >= CONTOUR_NUMMAX && writeFile != 4)
                    {
                        PK_SF_A_URadar_Save(obs, firstIdx, realnum, StdOffsetTh, udeltaTh,
                                            pathdeltaTh, Udata_Idx, 4);
                        writeFile = 4;
                    }

                    contour_numIdx                     = contour_num2 % CONTOUR_NUMMAX;
                    contour_arr2[contour_numIdx].pt1.x = obs.obsPt[i + 1].x;
                    contour_arr2[contour_numIdx].pt1.y = obs.obsPt[i + 1].y;
                    contour_arr2[contour_numIdx].pt2.x = obs.obsPt[i].x;
                    contour_arr2[contour_numIdx].pt2.y = obs.obsPt[i].y;
                    contour_num2                       = contour_num2 + 1;
                }
            }

            // The following is to reinitialize the data
            // Here is a strategy to add a gap link between two segments, underlined as a
            // gap part.
            if (udelta < udeltaTh && pathdelta_squre < pathdeltaTh * pathdeltaTh)
            {
                //  ....._''''
                // The following code is added  ._''''
                cur[0].x    = cur[num - 1].x;
                cur[0].y    = cur[num - 1].y;
                curudata[0] = curudata[num - 1];
                memset(&cur[1], 0, sizeof(Point_T) * (num - 1));
                memset(&curudata[1], 0, sizeof(float) * (num - 1));
                cur[1].x    = obs.obsPt[i].x;
                cur[1].y    = obs.obsPt[i].y;
                curudata[1] = obs.value[i];
                num         = 2;
            }
            else
            {
                // The following code is added ''''
                memset(&cur[0], 0, sizeof(Point_T) * (num));
                memset(&curudata[0], 0, sizeof(float) * (num));
                cur[0].x    = obs.obsPt[i].x;
                cur[0].y    = obs.obsPt[i].y;
                curudata[0] = obs.value[i];
                num         = 1;
            }
        }
    } // end of for of the forward

    //////////////////////////////////////////////////////////////////////////
    // choose the better fitting
    if (err2 < err1)
    {
        for (i = 0; i < contour_num2; ++i)
        {
            // HOBOT_CHECK(i<CONTOUR_NUMMAX) << "i surpass the CONTOUR_NUMMAX";
            // 此处会出现数组越界，因此对i取余预算@2022.7.7
            int j                 = i % CONTOUR_NUMMAX;
            pContour_arr[j].pt1.x = contour_arr2[contour_num2 - j - 1].pt2.x;
            pContour_arr[j].pt1.y = contour_arr2[contour_num2 - j - 1].pt2.y;
            pContour_arr[j].pt2.x = contour_arr2[contour_num2 - j - 1].pt1.x;
            pContour_arr[j].pt2.y = contour_arr2[contour_num2 - j - 1].pt1.y;
        }
        *pContour_num = contour_num2;
        if (NULL != pDeviation)
            *pDeviation = err2;
    }
    else
    {
        for (i = 0; i < contour_num1; ++i)
        {
            // HOBOT_CHECK(i<CONTOUR_NUMMAX) << "i surpass the CONTOUR_NUMMAX";
            // 此处会出现数组越界，因此对i取余预算@2022.7.7
            int j                 = i % CONTOUR_NUMMAX;
            pContour_arr[j].pt1.x = contour_arr1[j].pt1.x;
            pContour_arr[j].pt1.y = contour_arr1[j].pt1.y;
            pContour_arr[j].pt2.x = contour_arr1[j].pt2.x;
            pContour_arr[j].pt2.y = contour_arr1[j].pt2.y;
        }
        *pContour_num = contour_num1;
        if (NULL != pDeviation)
            *pDeviation = err1;
    }

    if (*pContour_num > CONTOUR_NUMMAX)
    {
        *pContour_num = CONTOUR_NUMMAX;
    }
    writeFile = 0;
    return TRUE;
}

int JudgeObsClass_V2(Udata_Info_T obs, int firstIdx, int realnum, float obs_meantheta,
                     LineSeg_T contour_arr[CONTOUR_NUMMAX], int contour_num)
{
    int i;
    float meanValue     = 0;
    int validNum        = 0;
    float validRadio    = 0;
    float delta         = 0;
    float subcontourDir = 0;
    float k;
    int AttributeValue = 0;

    if (firstIdx < 0 || realnum < 0 || (firstIdx + realnum > TIMENUM))
        return 0;

    CalMeanStdValue(&obs.value[firstIdx], realnum, &meanValue, NULL);

    for (i = firstIdx; i < firstIdx + realnum; ++i)
    {
        if (obs.value_2nd[i] < SIDE_INVALID)
        {
            validNum = validNum + 1;
        }
    }

    validRadio = (float)validNum / (float)realnum;

    /*Prevent the short-distance section from being mischecked as car body due to too few
    number of short-distance sections, and add distance judgment to narrow the range.*/
    if (realnum < 15 && meanValue > UDATACLASS_TH2)
    {
        if (validRadio <= 0.5)
        {
            validRadio = validRadio / 3;
        }
    }

    // The purpose of adding obstacle orientation judgment is to prevent the interference
    // of sparse points on the edge contour of vertical vehicle head. Guarantee the course
    // angle of odometer between 0 and 2 pi
    while (obs_meantheta >= 2.0f * PI_GEO)
    {
        obs_meantheta = obs_meantheta - 2.0f * PI_GEO;
    }
    while (obs_meantheta <= 0)
    {
        obs_meantheta = obs_meantheta + 2.0f * PI_GEO;
    }
    if (obs_meantheta >= PI_GEO && obs_meantheta <= 2.0f * PI_GEO)
    {
        obs_meantheta = obs_meantheta - PI_GEO;
    }

    delta = 0;
    if (contour_num == 1)
    {
        if (fabsf(contour_arr[0].pt2.x - contour_arr[0].pt1.x) < 0.000001f)
        {
            subcontourDir = PI_GEO / 2.0f;
        }
        else
        {
            k = (contour_arr[0].pt2.y - contour_arr[0].pt1.y) /
                (contour_arr[0].pt2.x - contour_arr[0].pt1.x);
            subcontourDir = atanf(k);
            // Tangent function has a value range of [- pi / 2 , pi / 2], and PI is added
            // if it is less than 0.
            if (subcontourDir < 0)
            {
                subcontourDir = subcontourDir + PI_GEO;
            }
        }

        // When two straight lines intersect, the angle is small, that is to say, the
        // angle range is [0, pi/2]
        delta = fabsf(subcontourDir - obs_meantheta);
        if (delta >= PI_GEO / 2.0f && delta <= PI_GEO)
        {
            delta = PI_GEO - delta;
        }
    }

    // Judgment process
    if (meanValue > UDATACLASS_TH1)
    {
        // 5510
        AttributeValue = INVALID_5510;
    }
    else
    {
        if (validRadio < 0.08f &&
            delta <
                PI_GEO /
                    4 /*&& meanValue > UDATACLASS_TH2*/) // October 21, 2017, 22:10:15,
                                                         // shielding the condition
        {
            // CURB
            AttributeValue = CURB;
        }
        else
        {
            if (meanValue < UDATACLASS_TH2)
            {
                // Near vehicle body
                AttributeValue = NEAR_VEH_BODY;
            }
            else
            {
                // Far vehicle body
                AttributeValue = FAR_VEH_BODY;
            }
        }
    }

    return AttributeValue;
}

void CalObsMainDir(const LineSeg_T contour_arr[], int contour_num, float obs_meantheta,
                   int *pmaindir)
{
    int i                 = 0;
    float deltamin        = 0;
    float subcontourDir   = 0;
    float k               = 0;
    float delta           = 0;
    int idx               = 0;
    uint8 hadLongSeg_Flag = FALSE;
    float dis_squre       = 0;

    for (i = 0; i < contour_num; ++i)
    {
        pmaindir[i] = 0;
    }

    if (contour_num == 1)
    {
        *pmaindir = 100;
        return;
    }

    // Guarantee the course angle of odometry between 0 and 2 pi
    while (obs_meantheta >= 2.0f * PI_GEO)
    {
        obs_meantheta = obs_meantheta - 2.0f * PI_GEO;
    }
    while (obs_meantheta <= 0)
    {
        obs_meantheta = obs_meantheta + 2.0f * PI_GEO;
    }
    if (obs_meantheta >= PI_GEO && obs_meantheta <= 2.0f * PI_GEO)
    {
        obs_meantheta = obs_meantheta - PI_GEO;
    }

    // Traversal to determine whether there is a line segment greater than 0.4 meters of
    // the fixed value
    for (i = 0; i < contour_num; ++i)
    {
        dis_squre = (contour_arr[i].pt1.x - contour_arr[i].pt2.x) *
                        (contour_arr[i].pt1.x - contour_arr[i].pt2.x) +
                    (contour_arr[i].pt1.y - contour_arr[i].pt2.y) *
                        (contour_arr[i].pt1.y - contour_arr[i].pt2.y);

        if (dis_squre > DISMIN_MAINDIR_SQURE)
        {
            hadLongSeg_Flag = TRUE;
            break;
        }
    }

    deltamin = 99999;

    for (i = 0; i < contour_num; ++i)
    {
        // % Filtering the shorter contour, the longer contour can be used to judge the
        // direction.
        if (hadLongSeg_Flag == TRUE)
        {
            dis_squre = (contour_arr[i].pt1.x - contour_arr[i].pt2.x) *
                            (contour_arr[i].pt1.x - contour_arr[i].pt2.x) +
                        (contour_arr[i].pt1.y - contour_arr[i].pt2.y) *
                            (contour_arr[i].pt1.y - contour_arr[i].pt2.y);
            if (dis_squre < DISMIN_MAINDIR_SQURE)
                continue;
        }

        if (fabsf(contour_arr[i].pt2.x - contour_arr[i].pt1.x) < 0.000001f)
        {
            subcontourDir = PI_GEO / 2.0f;
        }
        else
        {
            k = (contour_arr[i].pt2.y - contour_arr[i].pt1.y) /
                (contour_arr[i].pt2.x - contour_arr[i].pt1.x);
            subcontourDir = atanf(k);
            // Tangent function has a value range of[-pi / 2, pi / 2], and PI is added if
            // it is less than 0.
            if (subcontourDir < 0)
            {
                subcontourDir = subcontourDir + PI_GEO;
            }
        }

        // Two straight lines intersect, and the angle range is [0, pi/2]
        delta = fabsf(subcontourDir - obs_meantheta);
        if (delta >= PI_GEO / 2.0f && delta <= PI_GEO)
        {
            delta = PI_GEO - delta;
        }
        if (delta < deltamin)
        {
            deltamin = delta;
            idx      = i;
        }
    }

    pmaindir[idx] = 100;
}

void ClearObs(Udata_Info_T *U_info, float cur_fsl, float cur_fsl_2nd, float cur_fsl_peak,
              float cur_fsl_peak_2nd, float cur_fsl_spd, Point_T curudata, float curpath,
              float curtheta)
{
    memset(U_info, 0, sizeof(Udata_Info_T));

    U_info->num          = 1;
    U_info->obsPt->x     = curudata.x;
    U_info->obsPt->y     = curudata.y;
    U_info->value[0]     = cur_fsl;
    U_info->value_2nd[0] = cur_fsl_2nd;
    U_info->peak[0]      = cur_fsl_peak;
    U_info->peak_2nd[0]  = cur_fsl_peak_2nd;
    U_info->start_spd    = cur_fsl_spd;
    U_info->stop_spd     = 0;
    U_info->curtheta[0]  = curtheta;
    U_info->pathstart    = curpath;
}

// Res = 1
// AX1__________________AX2
//           BX1______________________BX2

// Res = 2
// AX1__________________AX2
//                                  BX1______________________BX2

// Res = 3
//         AX1__________________AX2
// BX1_____________________________________BX2

// Res = 4
//                 AX1__________________AX2
// BX1______________________BX2

// Res = 5
//                                    AX1__________________AX2
// BX1______________________BX2

// Res = 6
//  AX1__________________________________AX2
//           BX1______________BX2

// Res = 7
// AX1______________________AX2
// BX1______________________BX2

//

// calculate the relation between the projected line segments
// one point should be paid attention: the line segments are vectors,which point from X1
// to X2
void CalLineSegOverlaid(float SegA_X1, float SegA_X2, float SegB_X1, float SegB_X2,
                        int *Res, float *LenOverlaid)
{
    *Res = 0;
    if (LenOverlaid != NULL)
    {
        *LenOverlaid = 0;
    }

    if (SegA_X2 <= SegB_X2)
    {
        if (SegA_X2 >= SegB_X1)
        {
            if (SegA_X1 < SegB_X1)
            {
                *Res = 1;
                if (LenOverlaid != NULL)
                {
                    *LenOverlaid = SegA_X2 - SegB_X1;
                }
            }
            else
            {
                if (SegA_X1 == SegB_X1)
                {
                    *Res = 7;
                    if (LenOverlaid != NULL)
                    {
                        *LenOverlaid = SegA_X2 - SegA_X1;
                    }
                }
                else
                {
                    *Res = 3;
                    if (LenOverlaid != NULL)
                    {
                        *LenOverlaid = SegA_X2 - SegA_X1;
                    }
                }
            }
        }
        else
        {
            *Res = 2;
            if (LenOverlaid != NULL)
            {
                *LenOverlaid = 0;
            }
        }
    }
    else
    {
        if (SegA_X1 <= SegB_X2)
        {
            if (SegA_X1 >= SegB_X1)
            {
                *Res = 4;
                if (LenOverlaid != NULL)
                {
                    *LenOverlaid = SegB_X2 - SegA_X1;
                }
            }
            else
            {
                *Res = 6;
                if (LenOverlaid != NULL)
                {
                    *LenOverlaid = SegB_X2 - SegB_X1;
                }
            }
        }
        else
        {
            *Res = 5;
            if (LenOverlaid != NULL)
            {
                *LenOverlaid = 0;
            }
        }
    }
}

void JudgeRearSeg_Bevaild(
    /*U_FusionObj_T * FusionObjLeftRear_Cur,*/ Udata_Info_T Rear_info,
    FusionObj_T FusionObj_RTE, uint8 isleft, uint8 *bedeleted)
{

    float PtStart_rx = 0, PtEnd_rx = 0;
    VehPos_T curPos = {0, 0, 0};
    int i           = 0;
    int num         = 0;
    float SegB_X1 = 0, SegB_X2 = 0;
    int relation       = 0;
    float len_overlaid = 0;

    if (isleft)
    {
        curPos.x     = sf_curlocalx[2];
        curPos.y     = sf_curlocaly[2];
        curPos.theta = sf_curtheta[2];
    }
    else
    {
        curPos.x     = sf_curlocalx[3];
        curPos.y     = sf_curlocaly[3];
        curPos.theta = sf_curtheta[3];
    }

    PtStart_rx = Project_PointTo1stPos_rx(curPos, Rear_info.obsPt[0]);
    PtEnd_rx   = Project_PointTo1stPos_rx(curPos, Rear_info.obsPt[Rear_info.num - 1]);

    *bedeleted = 0;

    num = rte_min(FusionObj_RTE.num, 10);

    for (i = SF_OBJ_NUM - 1; i >= SF_OBJ_NUM - num; --i)
    {
        SegB_X1 = Project_PointTo1stPos_rx(curPos, FusionObj_RTE.obj[i].pt1);
        SegB_X2 = Project_PointTo1stPos_rx(curPos, FusionObj_RTE.obj[i].pt2);
        if (SegB_X2 < PtStart_rx)
        {
            break;
        }
        CalLineSegOverlaid(PtStart_rx, PtEnd_rx, SegB_X1, SegB_X2, &relation,
                           &len_overlaid);
        if (relation == 1 || relation == 3 || relation == 4 || relation == 6 ||
            relation == 7)
        {
            *bedeleted = 1;
            break;
        }
    }
}
