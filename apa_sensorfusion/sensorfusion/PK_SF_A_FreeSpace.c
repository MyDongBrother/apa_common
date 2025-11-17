/**
 * @file PK_SF_A_FreeSpace.c
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

/*********************************************************************************************************
**Include common and project definition header
*********************************************************************************************************/
#include "PK_SF_A_FreeSpace.h"
#include "PK_SF.h"
#include "PK_SF_Subfun.h"
#include "Module_API.h"
#include "AVM_Buff.h"
#include "PK_Calibration.h"
#include "PK_SensorT2S.h"
/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "PK_Calibration.h"
#include "hobotlog/hobotlog.hpp"
/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/

/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */

#if ((PK_SF_A_FS_SW_MAJOR_VERSION != (3u)) || (PK_SF_A_FS_SW_MINOR_VERSION != (0u)) || \
     (PK_SF_A_FS_SW_PATCH_VERSION != (26U)))
#error "Version numbers of PK_SF_A_Fre.c and PK_SF.h are inconsistent!"
#endif

/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/
#pragma section all "CPU1.Private"

// 2. Odometry information

// 2.1 Currently processed odometer location information (matched with median filter) is
// different from CurPos, in order of FSR FSL RSL RSR
// FSR FSL RSL RSR世界坐标信息（用於中值滤波）。
float sf_curlocalx[4]   = {0};
float sf_curlocaly[4]   = {0};
float sf_curtheta[4]    = {0};
float sf_curpath[4]     = {0};
float sf_before_path[4] = {0};
// 2.2 Currently read odometry information
// 从RTE获取的后轴中心世界坐标信息
float sf_CurPos[4] = {0, 0, 0, 0}; // The odometry obtained per RTE

// 3. PK_SF_A_URadarLeft & PK_SF_A_URadarLeft
// 3.1 Number of Obstacles on the Left Side
// 传感器检测到的障碍物数量
int obj_num_fsl = 0;
int obj_num_fsr = 0;
int obj_num_rsl = 0;
int obj_num_rsr = 0;
/* 3.2 CalObs_whole or CalObs_part
   1:  Clear up the latest non-independent line segments in FusionObj
   0:  Clear the unknown type line segment with attribute 5 in FusionObj,
   and reserve some unknown line segments with length greater than 2.5 meters.*/
/*CalObs_whole(完整障碍物检测)还是CalObs_part(局部障碍物检测)
1：清除FusionObj中最新的非独立线段
0：清除FusionObj中属性为5的未知类型线段并保留一些长度大于2.5米的未知线段
*/
int in_CalObs_whole_l = 0;
int in_CalObs_whole_r = 0;

// 3.3 Whether to start CalObs_part in PK_SF_A_URadarLeft and PK_SF_A_URadarRight
// 1: Start CalObs_part in this round of calculation
// 0: CalObs_part was not started in this round of calculation
/*是否在PK_SF_A_URadarLeft和PK_SF_A_URadarRight中启用CalObs_part（局部障碍物检测）
1:本轮计算中启用CalObs_part
0:本轮计算中不启用CalObs_part
*/
int CalObs_part_Active_l = 0;
int CalObs_part_Active_r = 0;

// 4.  Information of ultrasonic detection points
// 4.1 12 Ultrasound Measurements:  udata_dir
// 4.2 8 indirect measurements:  udata_indir
// 4.3 Four quadratic echo data:  udata_2nd
/*4.超声检测点信息，
·12个超声测量值
·8个交叉测量值
·4个二次回波值
*/
U_RadarType sf_udata; // Ultrasound value read at each time
// 4.4 Information of ultrasonic detection points, coordinates of detection points (x, y)
// 超声波检测点信息，检测点坐标（x，y）
Point_T sf_curudata_detected[12] = {0};

// 5. macros for PK_SF_A_URadarLeft & PK_SF_A_URadarLeft
#define DISMIN_TH       0.01f
#define DISMAX_TH       0.5f
#define PATHMAX_TH      2.5f
#define PATHMAX_REAR_TH 1.5f

// CalOutline dedicated macro definition
// Mean deviation of fitting straight line
// 直线检测的方差阈值
#define STDOFFSET_TH 0.08f
/* The cutting point set is the maximum value of the numerical fluctuation of the
 ultrasonic wave and the cutting point set is larger than the maximum value.*/
// 切割点的超声波数值波动的阈值，超过此阈值并且小于切割点的最大里程距离阈值认为此障碍物结束
#define UDELTA_TH 240 // mm
// The maximum distance of odometer and the set of cutting points when it is larger than
// the maximum distance 切割点的最大里程距离
#define PATHDELTA_TH 0.6f

// 6.Obstacle Output
// Output of obstacle contour in process A
// A过程的障碍物轮廓输出

// obj_num obj_attr obj pathstart is_Wholeobj is_over2dot5 ang
// 障碍物数量，属性，障碍物， 起始里程， 是否完整障碍物，是否超过2.5，角度？？
U_FusionObj32_T FusionObjLeft;
U_FusionObj32_T FusionObjRight;

U_FusionObj32_T FusionObjLeftRear;
U_FusionObj32_T FusionObjRightRear;

// A Process RTE Interactive Interface
//	 obj attr ang odom num num_accu
FusionObj_T FusionObjLeft_RTE;
FusionObj_T FusionObjRight_RTE;
FusionObj_T FusionObjLeft_CCP;
FusionObj_T FusionObjRight_CCP;

FusionObj_T FusionObjLeftRear_RTE;
FusionObj_T FusionObjRightRear_RTE;
FusionObj_T FusionObjLeftRear_CCP;
FusionObj_T FusionObjRightRear_CCP;

// PK_SF_A_URadarLeft, PK_SF_A_URadarRight
// PK_SF_A_URadarLeftRear, PK_SF_A_URadarRightRear
/*数据格式：
   obsPt[](坐标) value[](超声距离)value_2nd[]; (二次回波距离)
   curtheta[](头朝向角度) peak[](峰值)，peak_2nd(二次回波峰值)
   num pathstart stop_spd*/

Udata_Info_T FSL_info;
Udata_Info_T FSR_info;
Udata_Info_T RSR_info;
Udata_Info_T RSL_info;

float RSL_curpathStart = 0;
float RSR_curpathStart = 0;

// 8. CCP
#define PATHVD \
    40.0f // Output of obstacle contour within 40 meters, beyond attribute 0;
          // 40米以内障碍物轮廓输出，超出属性0
          // FSR and RSR距离大约4米，垂直停车位的最短长度约为0.8米
#define URADAR_GAP_TH \
    3.4f // the distance between FSR and RSR is about 4m, and the shortest length of
         // Perpendicular car parking is about 0.8m
int LineSegCCP_FSL_total = 0; // The total number of lines that CCP needs to output
int LineSegCCP_FSL_cur   = 0; // Number of CCP segments currently output
int LineSegCCP_RSL_total = 0; // The total number of lines that CCP needs to output
int LineSegCCP_RSL_cur   = 0; // Number of CCP segments currently output
int LineSegCCP_FSR_total = 0; // The total number of lines that CCP needs to output
int LineSegCCP_FSR_cur   = 0; // Number of CCP segments currently output
int LineSegCCP_RSR_total = 0; // The total number of lines that CCP needs to output
int LineSegCCP_RSR_cur   = 0; // Number of CCP segments currently output

// 9. parameter of the last object deleted by loop in current cycle
// 当前循环中被删除的障碍物的参数
uint8 sf_loopflag_left    = 0;
uint8 sf_loopflag_right   = 0;
float sf_loopflag_left_S  = 0.0f;
float sf_loopflag_right_S = 0.0f;

/*********************************************************************************************************
**Definition of main function of auto parking
*********************************************************************************************************/
void PK_SF_A_FreeSpace(void)
{
    float udata_dir12[12] = {0}; // Current Ultrasound Value for Computation
    float udata_indir8[8] = {0}; // Current Ultrasound Value for Computation

    // step 1cm
    // float dis[4] = { 0,0,0,0 };
    static Point_T curPt = {0, 0};
    static float EPSILON = 0.02f;
    // float tmp_path[4];
    //////////////////////////////////////////////////////////////////////////
    // get filter data
    // the matched location and udata .
    // sf_curlocalx,sf_curlocaly,sf_curtheta,sf_curpath
    // cur_fsr,cur_fsr_2nd,cur_fsl,cur_fsl_2nd,cur_rsr,cur_rsr_2nd,cur_rsl,cur_rsl_2nd

    udata_dir12[0]  = cur_fsr;      // 对应宋pro-6号FSR
    udata_dir12[1]  = cur_fsl;      // 对应宋pro-1号FSL
    udata_dir12[2]  = cur_rsl;      // 对应宋pro-12号RLS
    udata_dir12[3]  = cur_rsr;      // 对应宋pro-7号RRS
    udata_dir12[4]  = sf_udata.FOL; // 对应宋pro-2号FL
    udata_dir12[5]  = sf_udata.FCL; // 对应宋pro-3号FLM
    udata_dir12[6]  = sf_udata.FCR; // 对应宋pro-4号FRM
    udata_dir12[7]  = sf_udata.FOR; // 对应宋pro-5号FR
    udata_dir12[8]  = sf_udata.ROL; // 对应宋pro-11号RL
    udata_dir12[9]  = sf_udata.RCL; // 对应宋pro-10号RLM
    udata_dir12[10] = sf_udata.RCR; // 对应宋pro-9号RRM
    udata_dir12[11] = sf_udata.ROR; // 对应宋pro-8号RR
    udata_indir8[0] = sf_udata.FCL_FOL;
    udata_indir8[1] = sf_udata.FCL_FCR;
    udata_indir8[2] = sf_udata.FCR_FCL;
    udata_indir8[3] = sf_udata.FCR_FOR;
    udata_indir8[4] = sf_udata.RCL_ROL;
    udata_indir8[5] = sf_udata.RCL_RCR;
    udata_indir8[6] = sf_udata.RCR_RCL;
    udata_indir8[7] = sf_udata.RCR_ROR;
    CalAllDetected_12(udata_dir12, udata_indir8, sf_curudata_detected);

    LOGD << "obstacle location detected by FSR x: " << sf_curudata_detected[0].x
         << " y: " << sf_curudata_detected[0].y;
    LOGD << "obstacle location detected by FSL x: " << sf_curudata_detected[1].x
         << " y: " << sf_curudata_detected[1].y;
    LOGD << "obstacle location detected by RSL x: " << sf_curudata_detected[2].x
         << " y: " << sf_curudata_detected[2].y;
    LOGD << "obstacle location detected by RSR x: " << sf_curudata_detected[3].x
         << " y: " << sf_curudata_detected[3].y;

    //////////////////////////////////////////////////////////////////////////
    // odometer and sonar data are extracted in the step of 1cm
    // 里程表和声纳数据以1cm的步长提取

    if (sf_ultrasMotorPos[FSL_IDX][3] < -9999.0f ||
        sf_ultrasMotorPos[FSR_IDX][3] < -9999.0f ||
        sf_ultrasMotorPos[RSL_IDX][3] < -9999.0f ||
        sf_ultrasMotorPos[RSR_IDX][3] < -9999.0f)
    {
        PK_SF_A_FreeSpace_Clear();
        return;
    }
    else if (LOCAT_STILL == RTE_PK_Location_Get_Vel_MoveMentSt()) // 车辆是静止状态？
    {
        return;
    }

    float diffx = fabsf(curPt.x - sf_ultrasMotorPos[FSL_IDX][0]);
    float diffy = fabsf(curPt.y - sf_ultrasMotorPos[FSL_IDX][1]);
    if (diffx > EPSILON || diffy > EPSILON) // 当前坐标比上次移动超过阈值
    {
        if (diffx < 1.0 && diffy < 1.0)
        {
            PK_SF_A_URadar();
            PK_SF_A_Vision();
        }

        curPt.x = sf_ultrasMotorPos[FSL_IDX][0];
        curPt.y = sf_ultrasMotorPos[FSL_IDX][1];
    }
    // following is comment out by xls@2022.4.29
    //  else
    //  {
    //  	curPt.x = curPt.x;
    //  }
}

void PK_SF_A_URadar(void)
{
    sf_loopflag_left    = 0;
    sf_loopflag_right   = 0;
    sf_loopflag_left_S  = 0;
    sf_loopflag_right_S = 0;
    U_FusionObj_T FusionObjLeft_Cur;
    U_FusionObj_T FusionObjRight_Cur;
    U_FusionObj_T FusionObjLeftRear_Cur;
    U_FusionObj_T FusionObjRightRear_Cur;

    memset(&FusionObjLeft_Cur, 0, sizeof(FusionObjLeft_Cur));
    memset(&FusionObjRight_Cur, 0, sizeof(FusionObjRight_Cur));
    memset(&FusionObjLeftRear_Cur, 0, sizeof(FusionObjLeftRear_Cur));
    memset(&FusionObjRightRear_Cur, 0, sizeof(FusionObjRightRear_Cur));

    PK_SF_A_URadar_Left(&FusionObjLeft_Cur);
    PK_SF_A_URadar_Right(&FusionObjRight_Cur);

    PK_SF_A_URadar_LeftRear(&FusionObjLeftRear_Cur);
    PK_SF_A_URadar_RightRear(&FusionObjRightRear_Cur);

    // CCP will update when there are objects extracted in CalObs_whole function
    if (in_CalObs_whole_l && FusionObjLeft_Cur.obj_num > 0)
    {
        LineSegCCP_FSL_total = LineSegCCP_FSL_total + FusionObjLeft_Cur.obj_num;
        UpdateObj_CCP(&FusionObjLeft_CCP, FusionObjLeft_Cur);
    }

    if (in_CalObs_whole_r && FusionObjRight_Cur.obj_num > 0)
    {
        LineSegCCP_FSR_total = LineSegCCP_FSR_total + FusionObjRight_Cur.obj_num;
        UpdateObj_CCP(&FusionObjRight_CCP, FusionObjRight_Cur);
    }

    if (FusionObjLeftRear_Cur.obj_num > 0)
    {
        LineSegCCP_RSL_total = LineSegCCP_RSL_total + FusionObjLeftRear_Cur.obj_num;
        UpdateObj_CCP(&FusionObjLeftRear_CCP, FusionObjLeftRear_Cur);

        // add the objects at the beginning between the gap of FSL and RSL
        // 开始的时候将FSL and RSL之间的障碍物信息也update到FusionObjLeft_CCP  add by
        // xls@2022.4.29
        if ((sf_curpath[2] - sf_before_path[2] > 0.0f) &&
            (sf_curpath[2] - sf_before_path[2] < URADAR_GAP_TH))
        {
            LineSegCCP_FSL_total = LineSegCCP_FSL_total + FusionObjLeftRear_Cur.obj_num;
            UpdateObj_CCP(&FusionObjLeft_CCP, FusionObjLeftRear_Cur);
        }
    }

    if (FusionObjRightRear_Cur.obj_num > 0)
    {
        LineSegCCP_RSR_total = LineSegCCP_RSR_total + FusionObjRightRear_Cur.obj_num;
        UpdateObj_CCP(&FusionObjRightRear_CCP, FusionObjRightRear_Cur);

        // add the objects at the beginning between the gap of FSR and RSR
        // 开始的时候将FSR and RSR之间的障碍物信息也update到FusionObjRight_CCP  add by
        // xls@2022.4.29
        if ((sf_curpath[3] - sf_before_path[3] > 0.0f) &&
            (sf_curpath[3] - sf_before_path[3] < URADAR_GAP_TH))
        {
            LineSegCCP_FSR_total = LineSegCCP_FSR_total + FusionObjRightRear_Cur.obj_num;
            UpdateObj_CCP(&FusionObjRight_CCP, FusionObjRightRear_Cur);
        }
    }

    // Clear up the temporary line segments generated in the previous cycle
    // Update external output interface variables
    // FusionObjLeft, FusionObjRight,FusionObjLeftRear and FusionObjRightRear  are for RTE
    // update.
    if (CalObs_part_Active_l || in_CalObs_whole_l)
    {
        ClearPartObj(&FusionObjLeft, in_CalObs_whole_l);
        // UpdateObj(&FusionObjLeft, FusionObjLeft_Cur);
        UpdateObj_V2(&FusionObjLeft, FusionObjLeft_Cur, 0U, &sf_loopflag_left,
                     &sf_loopflag_left_S);
    }

    if (CalObs_part_Active_r || in_CalObs_whole_r)
    {
        ClearPartObj(&FusionObjRight, in_CalObs_whole_r);
        UpdateObj_V2(&FusionObjRight, FusionObjRight_Cur, 0U, &sf_loopflag_right,
                     &sf_loopflag_right_S);
    }

    UpdateObj_V2(&FusionObjLeftRear, FusionObjLeftRear_Cur, 0U, NULL, NULL);
    UpdateObj_V2(&FusionObjRightRear, FusionObjRightRear_Cur, 0U, NULL, NULL);

    // add the objects at the beginning between the gap of FSL and RSL
    if ((sf_curpath[2] - sf_before_path[2] > 0.0f) &&
        (sf_curpath[2] - sf_before_path[2] < URADAR_GAP_TH) &&
        FusionObjLeftRear_Cur.obj_num > 0)
    {
        UpdateObj_V2(&FusionObjLeft, FusionObjLeftRear_Cur, 1U, &sf_loopflag_left,
                     &sf_loopflag_left_S);
    }

    // add the objects at the beginning between the gap of FSR and RSR
    if ((sf_curpath[3] - sf_before_path[3] > 0.0f) &&
        (sf_curpath[3] - sf_before_path[3] < URADAR_GAP_TH) &&
        FusionObjRightRear_Cur.obj_num > 0)
    {
        UpdateObj_V2(&FusionObjRight, FusionObjRightRear_Cur, 1U, &sf_loopflag_right,
                     &sf_loopflag_right_S);
    }

    //////////////////////////////////////////////////////////////////////////
    //  output result and state
    //////////////////////////////////////////////////////////////////////////
    PK_SF_A_URadar_Log();
}

void PK_SF_A_URadar_Left(U_FusionObj_T *FusionObjLeft_Cur)
{
    float udata_delta   = 0;
    float obs_udataMean = 0;
    //////////////////////////////////////////////////////////////////////////
    // Output Initialization
    memset(FusionObjLeft_Cur, 0, sizeof(U_FusionObj_T));

    //////////////////////////////////////////////////////////////////////////
    // process simulation

    // Change of Ultrasound in One Cycle
    if (FSL_info.num >= 1)
    {
        udata_delta = fabsf(cur_fsl - FSL_info.value[FSL_info.num - 1]) / 1000.0f;
    }
    else
    {
        udata_delta = 0;
    }

    /* Start the main process 当前FSL变化值<=最大距离(0.5m)，
    并且时间序列小于TIMENUM(100),认为此obstacle并未结束
    add by xls@2022.4.29
    */
    if ((udata_delta <= DISMAX_TH && FSL_info.num < TIMENUM))
    {
        in_CalObs_whole_l = 0;

        FSL_info.obsPt[FSL_info.num].x   = sf_curudata_detected[1].x;
        FSL_info.obsPt[FSL_info.num].y   = sf_curudata_detected[1].y;
        FSL_info.value[FSL_info.num]     = cur_fsl;
        FSL_info.value_2nd[FSL_info.num] = cur_fsl_2nd;
        FSL_info.curtheta[FSL_info.num]  = sf_curtheta[1];
        FSL_info.peak[FSL_info.num]      = cur_fsl_peak;
        FSL_info.peak_2nd[FSL_info.num]  = cur_fsl_peak_2nd;
        FSL_info.stop_spd                = cur_fsl_spd;
        FSL_info.num                     = FSL_info.num + 1;
        if (FSL_info.num == 1)
        { // 记录发现障碍物时的path和速度 add by xls@2022.4.29
            FSL_info.pathstart = sf_curpath[1] + FSL_DELTAX - RSL_DELTAX;
            FSL_info.start_spd = cur_fsl_spd;
        }

        // 计算超声波测量值的均值
        CalMeanStdValue(&FSL_info.value[0], FSL_info.num, &obs_udataMean, NULL);
        if ((obs_udataMean / 1000) > 5.0f || FSL_info.num <= 2)
        { // 均值大于5m，并且数量小于2，认为此障碍物无效 add by xls@2022.4.29
            return;
        }

        if (FSL_info.num % 5 == 0) // zhouzj 250ms
        {
            CalObs_part_onlyonce(&FSL_info, &obj_num_fsl, 1, STDOFFSET_TH, UDELTA_TH,
                                 PATHDELTA_TH, FusionObjLeft_Cur);
            CalObs_part_Active_l = 1;
        }
        else
        {
            CalObs_part_Active_l = 0;
        }
    }
    else
    { // 此次障碍物结束add by xls@2022.4.29
        // 100个点计算平均值和整体形状
        FSL_info.stop_spd = cur_fsl_spd;

        in_CalObs_whole_l = 1;

        CalMeanStdValue(&FSL_info.value[0], FSL_info.num, &obs_udataMean, NULL);
        if (FSL_info.num <= 1 ||
            (obs_udataMean / 1000) >
                5.0f) // 均值大于5m，并且数量小于2，认为此障碍物无效 add by xls@2022.4.29
        {
            ClearObs(&FSL_info, cur_fsl, cur_fsl_2nd, cur_fsl_peak, cur_fsl_peak_2nd,
                     cur_fsl_spd, sf_curudata_detected[1], sf_curpath[1], sf_curtheta[1]);
            return;
        }
        // 将障碍物计算后保存FusionObjLeft_Cur   add by xls@2022.4.29
        CalObs_whole(&FSL_info, &obj_num_fsl, 1, 1, STDOFFSET_TH, UDELTA_TH, PATHDELTA_TH,
                     FusionObjLeft_Cur);

        // Empty reset array
        ClearObs(&FSL_info, cur_fsl, cur_fsl_2nd, cur_fsl_peak, cur_fsl_peak_2nd,
                 cur_fsl_spd, sf_curudata_detected[1], sf_curpath[1], sf_curtheta[1]);
    }
}

void PK_SF_A_URadar_Right(U_FusionObj_T *FusionObjRight_Cur)
{
    float udata_delta   = 0;
    float obs_udataMean = 0;
    //////////////////////////////////////////////////////////////////////////
    // Output Initialization
    memset(FusionObjRight_Cur, 0, sizeof(U_FusionObj_T));

    //////////////////////////////////////////////////////////////////////////
    // process simulation

    // Change of Ultrasound in One Cycle
    if (FSR_info.num >= 1)
    {
        udata_delta = fabsf(cur_fsr - FSR_info.value[FSR_info.num - 1]) / 1000.0f;
    }
    else
    {
        udata_delta = 0;
    }

    // Start the main process
    if ((udata_delta <= DISMAX_TH && FSR_info.num < TIMENUM))
    {
        in_CalObs_whole_r = 0;

        FSR_info.obsPt[FSR_info.num].x   = sf_curudata_detected[0].x;
        FSR_info.obsPt[FSR_info.num].y   = sf_curudata_detected[0].y;
        FSR_info.value[FSR_info.num]     = cur_fsr;
        FSR_info.value_2nd[FSR_info.num] = cur_fsr_2nd;
        FSR_info.curtheta[FSR_info.num]  = sf_curtheta[0];
        FSR_info.peak[FSR_info.num]      = cur_fsr_peak;
        FSR_info.peak_2nd[FSR_info.num]  = cur_fsr_peak_2nd;
        FSR_info.stop_spd                = cur_fsr_spd;
        FSR_info.num                     = FSR_info.num + 1;
        if (FSR_info.num == 1)
        {
            FSR_info.pathstart = sf_curpath[0] + FSR_DELTAX - RSR_DELTAX;
            FSR_info.start_spd = cur_fsr_spd;
        }

        CalMeanStdValue(&FSR_info.value[0], FSR_info.num, &obs_udataMean, NULL);
        if ((obs_udataMean / 1000) > 5.0f || FSR_info.num <= 2) // 20180712
        // if (FSR_info.value[0] > 5000.0f || FSR_info.num <= 2)
        {
            return;
        }

        if (FSR_info.num % 5 == 0)
        {
            // CalObs_part(FSR_info, &obj_num_fsr, 0, FusionObjRight_Cur);
            CalObs_part_onlyonce(&FSR_info, &obj_num_fsr, 0, STDOFFSET_TH, UDELTA_TH,
                                 PATHDELTA_TH, FusionObjRight_Cur);
            CalObs_part_Active_r = 1;
        }
        else
        {
            CalObs_part_Active_r = 0;
        }
    }
    else
    {
        FSR_info.stop_spd = cur_fsr_spd;

        in_CalObs_whole_r = 1;

        CalMeanStdValue(&FSR_info.value[0], FSR_info.num, &obs_udataMean, NULL);
        if (FSR_info.num <= 1 || (obs_udataMean / 1000) > 5.0f) // 20180712
        // if(FSR_info.num <= 1 || FSR_info.value[0] > 5000.0f)
        {
            ClearObs(&FSR_info, cur_fsr, cur_fsr_2nd, cur_fsr_peak, cur_fsr_peak_2nd,
                     cur_fsr_spd, sf_curudata_detected[0], sf_curpath[0], sf_curtheta[0]);
            return;
        }

        CalObs_whole(&FSR_info, &obj_num_fsr, 0, 1, STDOFFSET_TH, UDELTA_TH, PATHDELTA_TH,
                     FusionObjRight_Cur);

        // Empty reset array
        ClearObs(&FSR_info, cur_fsr, cur_fsr_2nd, cur_fsr_peak, cur_fsr_peak_2nd,
                 cur_fsr_spd, sf_curudata_detected[0], sf_curpath[0], sf_curtheta[0]);
    }
}

void PK_SF_A_URadar_LeftRear(U_FusionObj_T *FusionObjLeftRear_Cur)
{
    float udata_delta          = 0;
    float obs_udataMean        = 0;
    float disPathStart_rsl     = 0;
    int RSL_path_devided       = 0;
    int RSL_locatStill_devided = 0;
    uint8 bedeleted            = 0;
    //////////////////////////////////////////////////////////////////////////
    // Output Initialization
    memset(FusionObjLeftRear_Cur, 0, sizeof(U_FusionObj_T));

    //////////////////////////////////////////////////////////////////////////
    // process simulation

    // Change of Ultrasound in One Cycle
    if (RSL_info.num >= 1)
    {
        udata_delta = fabsf(cur_rsl - RSL_info.value[RSL_info.num - 1]) / 1000.0f;
    }
    else
    {
        udata_delta = 0;
    }

    if (RSL_info.num > 2)
        disPathStart_rsl = sf_curpath[2] - RSL_curpathStart;
    else
    {
        RSL_curpathStart = sf_curpath[2];
        disPathStart_rsl = 0;
    }

    if (disPathStart_rsl >= PATHMAX_REAR_TH)
        RSL_path_devided = 1;
    else
        RSL_path_devided = 0;

    /* After increasing the number of people who find parking spaces to step on the
    parking lot, they are forced to export the surrounding obstacle environment.*/
    if (RTE_PK_SlotDetect_Get_SlotShape() != PK_SLOT_NO &&
        RTE_PK_Location_Get_Vel_MoveMentSt() == LOCAT_STILL)
        RSL_locatStill_devided = 1;
    else
        RSL_locatStill_devided = 0;

    // Start the main process
    if ((udata_delta <= DISMAX_TH && RSL_info.num < TIMENUM) && RSL_path_devided == 0 &&
        RSL_locatStill_devided == 0)
    {
        RSL_info.obsPt[RSL_info.num].x   = sf_curudata_detected[2].x;
        RSL_info.obsPt[RSL_info.num].y   = sf_curudata_detected[2].y;
        RSL_info.value[RSL_info.num]     = cur_rsl;
        RSL_info.value_2nd[RSL_info.num] = cur_rsl_2nd;
        RSL_info.curtheta[RSL_info.num]  = sf_curtheta[2];
        RSL_info.peak[RSL_info.num]      = cur_rsl_peak;
        RSL_info.peak_2nd[RSL_info.num]  = cur_rsl_peak_2nd;
        RSL_info.stop_spd                = cur_rsl_spd;
        RSL_info.num                     = RSL_info.num + 1;
        if (RSL_info.num == 1)
        {
            RSL_info.pathstart = sf_curpath[2] /* + RSL_DELTAX*/;
            RSL_info.start_spd = cur_rsl_spd;
        }
    }
    else
    {
        RSL_info.stop_spd = cur_rsl_spd;

        CalMeanStdValue(&RSL_info.value[0], RSL_info.num, &obs_udataMean, NULL);
        if (RSL_info.num <= 1 || (obs_udataMean / 1000) > 5.0f)
        // if(RSL_info.num <= 1 || RSL_info.value[0] > 5000.0f)
        {
            ClearObs(&RSL_info, cur_rsl, cur_rsl_2nd, cur_rsl_peak, cur_rsl_peak_2nd,
                     cur_rsl_spd, sf_curudata_detected[2], sf_curpath[2], sf_curtheta[2]);
            return;
        }
        // 根据线段关系（主要是与前探头）判断是否需要舍弃  add by xls@2022.4.29
        JudgeRearSeg_Bevaild(/*FusionObjLeftRear_Cur,*/ RSL_info, FusionObjLeft_RTE, 1,
                             &bedeleted);
        if (bedeleted)
        {
            ClearObs(&RSL_info, cur_rsl, cur_rsl_2nd, cur_rsl_peak, cur_rsl_peak_2nd,
                     cur_rsl_spd, sf_curudata_detected[2], sf_curpath[2], sf_curtheta[2]);
            return;
        }

        CalObs_whole(&RSL_info, &obj_num_rsl, 2, 0, STDOFFSET_TH, UDELTA_TH, PATHDELTA_TH,
                     FusionObjLeftRear_Cur);

        // Empty reset array
        ClearObs(&RSL_info, cur_rsl, cur_rsl_2nd, cur_rsl_peak, cur_rsl_peak_2nd,
                 cur_rsl_spd, sf_curudata_detected[2], sf_curpath[2], sf_curtheta[2]);
    }
}

void PK_SF_A_URadar_RightRear(U_FusionObj_T *FusionObjRightRear_Cur)
{
    float udata_delta          = 0;
    float obs_udataMean        = 0;
    float disPathStart_rsr     = 0;
    int RSR_path_devided       = 0;
    int RSR_locatStill_devided = 0;
    uint8 bedeleted            = 0;
    //////////////////////////////////////////////////////////////////////////
    // Output Initialization
    memset(FusionObjRightRear_Cur, 0, sizeof(U_FusionObj_T));

    //////////////////////////////////////////////////////////////////////////
    // process simulation

    // Change of Ultrasound in One Cycle
    if (RSR_info.num >= 1)
    {
        udata_delta = fabsf(cur_rsr - RSR_info.value[RSR_info.num - 1]) / 1000.0f;
    }
    else
    {
        udata_delta = 0;
    }

    // 从起始点开始的距离
    if (RSR_info.num > 2)
        disPathStart_rsr = sf_curpath[3] - RSR_curpathStart;
    else
    {
        RSR_curpathStart = sf_curpath[3];
        disPathStart_rsr = 0;
    }

    // 移动距离超过1.5m
    if (disPathStart_rsr >= PATHMAX_REAR_TH)
        RSR_path_devided = 1;
    else
        RSR_path_devided = 0;

    /* After increasing the number of people who find parking spaces to step on the
    parking lot, they are forced to export the surrounding obstacle environment.*/
    if (RTE_PK_SlotDetect_Get_SlotShape() != PK_SLOT_NO &&
        RTE_PK_Location_Get_Vel_MoveMentSt() == LOCAT_STILL)
        RSR_locatStill_devided = 1;
    else
        RSR_locatStill_devided = 0;

    // Start the main process
    if ((udata_delta <= DISMAX_TH && RSR_info.num < TIMENUM) && RSR_path_devided == 0 &&
        RSR_locatStill_devided == 0)
    { // 此次障碍物未结束并且未要求强制输出   add by xls@2022.4.29
        RSR_info.obsPt[RSR_info.num].x   = sf_curudata_detected[3].x;
        RSR_info.obsPt[RSR_info.num].y   = sf_curudata_detected[3].y;
        RSR_info.value[RSR_info.num]     = cur_rsr;
        RSR_info.value_2nd[RSR_info.num] = cur_rsr_2nd;
        RSR_info.curtheta[RSR_info.num]  = sf_curtheta[3];
        RSR_info.peak[RSR_info.num]      = cur_rsr_peak;
        RSR_info.peak_2nd[RSR_info.num]  = cur_rsr_peak_2nd;
        RSR_info.stop_spd                = cur_rsr_spd;
        RSR_info.num                     = RSR_info.num + 1;
        if (RSR_info.num == 1)
        {
            RSR_info.pathstart = sf_curpath[3];
            RSR_info.start_spd = cur_rsr_spd;
        }
    }
    else
    {
        RSR_info.stop_spd = cur_rsr_spd;

        CalMeanStdValue(&RSR_info.value[0], RSR_info.num, &obs_udataMean, NULL);
        if (RSR_info.num <= 1 || (obs_udataMean / 1000) > 5.0f)
        // 采集障碍物的数量小于2，或者障碍物的平均距离大于5，认为检测无效 add by
        // xls@2022.4.29
        {
            ClearObs(&RSR_info, cur_rsr, cur_rsr_2nd, cur_rsr_peak, cur_rsr_peak_2nd,
                     cur_rsr_spd, sf_curudata_detected[3], sf_curpath[3], sf_curtheta[3]);
            return;
        }
        // 根据线段关系（主要是与前探头）判断是否需要舍弃  add by xls@2022.4.29
        JudgeRearSeg_Bevaild(/*FusionObjLeftRear_Cur,*/ RSR_info, FusionObjRight_RTE, 0,
                             &bedeleted);
        if (bedeleted)
        {
            ClearObs(&RSR_info, cur_rsr, cur_rsr_2nd, cur_rsr_peak, cur_rsr_peak_2nd,
                     cur_rsr_spd, sf_curudata_detected[3], sf_curpath[3], sf_curtheta[3]);
            return;
        }
        CalObs_whole(&RSR_info, &obj_num_rsr, 3, 0, STDOFFSET_TH, UDELTA_TH, PATHDELTA_TH,
                     FusionObjRightRear_Cur);

        // Empty reset array
        ClearObs(&RSR_info, cur_rsr, cur_rsr_2nd, cur_rsr_peak, cur_rsr_peak_2nd,
                 cur_rsr_spd, sf_curudata_detected[3], sf_curpath[3], sf_curtheta[3]);
    }
}

void PK_SF_A_FreeSpace_Clear(void)
{
    PK_SF_A_FreeSpace_Init();
    TimeSeries_PK_SF = 0;
}

// each cycle will only clean the last obj if the condition is ok
void PK_SF_A_ClearRteObj(FusionObj_T *FusionObj, U_FusionObj32_T *U_FusionObj)
{
    int j = 0;

    for (j = SF_OBJ_NUM - 1; j > SF_OBJ_NUM - FusionObj->num; j--)
    {
        FusionObj->ang[j]       = FusionObj->ang[j - 1];
        FusionObj->attr[j]      = FusionObj->attr[j - 1];
        FusionObj->odom[j]      = FusionObj->odom[j - 1];
        FusionObj->obj[j].pt1.x = FusionObj->obj[j - 1].pt1.x;
        FusionObj->obj[j].pt1.y = FusionObj->obj[j - 1].pt1.y;
        FusionObj->obj[j].pt2.x = FusionObj->obj[j - 1].pt2.x;
        FusionObj->obj[j].pt2.y = FusionObj->obj[j - 1].pt2.y;

        U_FusionObj->ang[j]          = U_FusionObj->ang[j - 1];
        U_FusionObj->is_over2dot5[j] = U_FusionObj->is_over2dot5[j - 1];
        U_FusionObj->is_Wholeobj[j]  = U_FusionObj->is_Wholeobj[j - 1];
        U_FusionObj->obj[j].pt1.x    = U_FusionObj->obj[j - 1].pt1.x;
        U_FusionObj->obj[j].pt1.y    = U_FusionObj->obj[j - 1].pt1.y;
        U_FusionObj->obj[j].pt2.x    = U_FusionObj->obj[j - 1].pt2.x;
        U_FusionObj->obj[j].pt2.y    = U_FusionObj->obj[j - 1].pt2.y;
        U_FusionObj->obj_attr[j]     = U_FusionObj->obj_attr[j - 1];
        U_FusionObj->pathstart[j]    = U_FusionObj->pathstart[j - 1];
    }

    FusionObj->ang[SF_OBJ_NUM - FusionObj->num]       = 0;
    FusionObj->attr[SF_OBJ_NUM - FusionObj->num]      = 0;
    FusionObj->odom[SF_OBJ_NUM - FusionObj->num]      = 0;
    FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt1.x = 0;
    FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt1.y = 0;
    FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt2.x = 0;
    FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt2.y = 0;

    U_FusionObj->ang[SF_OBJ_NUM - FusionObj->num]          = 0;
    U_FusionObj->is_over2dot5[SF_OBJ_NUM - FusionObj->num] = 0;
    U_FusionObj->is_Wholeobj[SF_OBJ_NUM - FusionObj->num]  = 0;
    U_FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt1.x    = 0;
    U_FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt1.y    = 0;
    U_FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt2.x    = 0;
    U_FusionObj->obj[SF_OBJ_NUM - FusionObj->num].pt2.y    = 0;
    U_FusionObj->obj_attr[SF_OBJ_NUM - FusionObj->num]     = 0;

    U_FusionObj->pathstart[SF_OBJ_NUM - FusionObj->num] = 0;

    FusionObj->num      = FusionObj->num - 1;
    FusionObj->num_accu = FusionObj->num_accu - 1;

    U_FusionObj->obj_num = U_FusionObj->obj_num - 1;
}

// when COM = OFF, we should clear some objects which are in front of our vehicle.
float CurLocal[4] = {0, 0, 0, 0};
void PK_SF_A_FreeSpace_OffClear(void)
{
    float rx = 0;
    VehPos_T curpos;

    RTE_PK_Location_Get_CurPos(CurLocal);

    // because sf_curlocalx will stop update when getfilterdata_v2 off
    // curpos.x = sf_curlocalx[0];  //current location of fsr, it is ok for all of the
    // probes. no need to make the difference between them. curpos.y = sf_curlocaly[0];
    // curpos.theta = sf_curtheta[0];
    curpos.x = CurLocal[0]; // current location of fsr, it is ok for all of the probes. no
                            // need to make the difference between them.
    curpos.y     = CurLocal[1];
    curpos.theta = CurLocal[2];

    // clear the free space objs
    if (FusionObjLeft.obj_num > 0)
    {
        rx = Project_PointTo1stPos_rx(curpos, FusionObjLeft.obj[SF_OBJ_NUM - 1].pt1);
        if (rx > 0)
        {
            PK_SF_A_ClearRteObj(&FusionObjLeft_RTE, &FusionObjLeft);
        }
    }

    if (FusionObjRight.obj_num > 0)
    {
        rx = Project_PointTo1stPos_rx(curpos, FusionObjRight.obj[SF_OBJ_NUM - 1].pt1);
        if (rx > 0)
        {
            PK_SF_A_ClearRteObj(&FusionObjRight_RTE, &FusionObjRight);
            // obj_num_fsr = obj_num_fsr - 1;
        }
    }

    if (FusionObjLeftRear.obj_num > 0)
    {
        rx = Project_PointTo1stPos_rx(curpos, FusionObjLeftRear.obj[SF_OBJ_NUM - 1].pt1);
        if (rx > 0)
        {
            PK_SF_A_ClearRteObj(&FusionObjLeftRear_RTE, &FusionObjLeftRear);
            // obj_num_rsl = obj_num_rsl - 1;
        }
    }

    if (FusionObjRightRear.obj_num > 0)
    {
        rx = Project_PointTo1stPos_rx(curpos, FusionObjRightRear.obj[SF_OBJ_NUM - 1].pt1);
        if (rx > 0)
        {
            PK_SF_A_ClearRteObj(&FusionObjRightRear_RTE, &FusionObjRightRear);
            // obj_num_rsr = obj_num_rsr - 1;
        }
    }

    /* 3.2 CalObs_whole or CalObs_part
       1: Clear up the latest non-independent line segments in Fusion Obj
       0: Clear the unknown type line segment with attribute 5 in FusionObj,
    and reserve some unknown line segments with length greater than 2.5 meters.*/
    in_CalObs_whole_l = 0;
    in_CalObs_whole_r = 0;

    // 4.  Information of ultrasonic detection points
    // 4.1 12 Ultrasound Measurements  :  udata_dir
    // 4.2 8 indirect measurements     :  udata_indir
    // 4.3 Four quadratic echo data  :  udata_2nd
    memset(&sf_udata, 0, sizeof(sf_udata));
    // 4.4 Information of ultrasonic detection points, coordinates of detection points (x,
    // y)
    memset(sf_curudata_detected, 0, sizeof(sf_curudata_detected));

    // A process data storage
    memset(&FSR_info, 0, sizeof(FSR_info));
    memset(&FSL_info, 0, sizeof(FSL_info));
    memset(&RSR_info, 0, sizeof(RSR_info));
    memset(&RSL_info, 0, sizeof(RSL_info));
    memset(sf_ultrasMotorPos, 0, sizeof(sf_ultrasMotorPos));

    RSL_curpathStart = 0;
    RSR_curpathStart = 0;

    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left_Raw(&FusionObjLeft_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right_Raw(&FusionObjRight_RTE);

    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left_Raw(&FusionObjLeftRear_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right_Raw(&FusionObjRightRear_RTE);
}

void PK_SF_A_FreeSpace_Init(void)
{

    /* 2.  Odometer information
       2.1 Currently processed odometer location information
       (matched with median filtering) is different from CurPos.*/
    memset(sf_curlocalx, 0, sizeof(sf_curlocalx));
    memset(sf_curlocaly, 0, sizeof(sf_curlocaly));
    memset(sf_curtheta, 0, sizeof(sf_curtheta));
    memset(sf_before_path, 0, sizeof(sf_before_path));
    memset(sf_curpath, 0, sizeof(sf_curpath));

    // 2.2 Currently read odometer information
    memset(sf_CurPos, 0, sizeof(sf_CurPos));

    // 3. PK_SF_A_URadarLeft & PK_SF_A_URadarLeft
    // 3.1 Number of Obstacles on the Left Side
    obj_num_fsl = 0;
    obj_num_fsr = 0;
    obj_num_rsl = 0;
    obj_num_rsr = 0;
    /* 3.2 CalObs_whole or CalObs_part
       1:  Clear up the latest non-independent line segments in Fusion Obj
       0:  Clear the unknown type line segment with attribute 5 in FusionObj,
           and reserve some unknown line segments with length greater than 2.5 meters.*/
    in_CalObs_whole_l = 0;
    in_CalObs_whole_r = 0;

    // 4. Information of ultrasonic detection points
    // 4.1 12 Ultrasound Measurements  :  udata_dir
    // 4.2 8 indirect measurements     :  udata_indir
    // 4.3 Four quadratic echo data   :  udata_2nd
    memset(&sf_udata, 0, sizeof(sf_udata));
    // 4.4 Information of ultrasonic detection points, coordinates of detection points (x,
    // y)
    memset(sf_curudata_detected, 0, sizeof(sf_curudata_detected));

    // 6.Obstacle Output
    // Output of obstacle contour in process A
    memset(&FusionObjLeft, 0, sizeof(FusionObjLeft));
    memset(&FusionObjRight, 0, sizeof(FusionObjRight));

    memset(&FusionObjLeftRear, 0, sizeof(FusionObjLeftRear));
    memset(&FusionObjRightRear, 0, sizeof(FusionObjRightRear));
    // A Process RTE Interactive Interface
    memset(&FusionObjLeft_RTE, 0, sizeof(FusionObjLeft_RTE));
    memset(&FusionObjRight_RTE, 0, sizeof(FusionObjLeft_RTE));

    memset(&FusionObjLeftRear_RTE, 0, sizeof(FusionObjLeftRear_RTE));
    memset(&FusionObjRightRear_RTE, 0, sizeof(FusionObjRightRear_RTE));

    memset(&FusionObjLeft_CCP, 0, sizeof(FusionObjLeft_CCP));
    memset(&FusionObjRight_CCP, 0, sizeof(FusionObjLeft_CCP));

    memset(&FusionObjLeftRear_CCP, 0, sizeof(FusionObjLeftRear_CCP));
    memset(&FusionObjRightRear_CCP, 0, sizeof(FusionObjRightRear_CCP));

    // A process data storage
    memset(&FSR_info, 0, sizeof(FSR_info));
    memset(&FSL_info, 0, sizeof(FSL_info));
    memset(&RSR_info, 0, sizeof(RSR_info));
    memset(&RSL_info, 0, sizeof(RSR_info));
    memset(sf_ultrasMotorPos, 0, sizeof(sf_ultrasMotorPos));

    RSL_curpathStart = 0;
    RSR_curpathStart = 0;

    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left_Raw(&FusionObjLeft_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right_Raw(&FusionObjRight_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left_Raw(&FusionObjLeftRear_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right_Raw(&FusionObjRightRear_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left(&FusionObjLeft_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right(&FusionObjRight_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left(&FusionObjLeftRear_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right(&FusionObjRightRear_RTE);

    cur_fsr     = 0;
    cur_fsr_2nd = 0; // The value of quadratic echo
    cur_fsl     = 0;
    cur_fsl_2nd = 0; // The value of quadratic echo
    cur_rsr     = 0;
    cur_rsr_2nd = 0; // The value of quadratic echo
    cur_rsl     = 0;
    cur_rsl_2nd = 0; // The value of quadratic echo

    PK_SENSORFUSION_A_ARR_NUM_CCP = 0;

    sf_loopflag_left    = 0;
    sf_loopflag_right   = 0;
    sf_loopflag_left_S  = 0;
    sf_loopflag_right_S = 0;
}

void PK_SF_A_Vision(void) {}

void ClearPartObj(U_FusionObj32_T *FusionObj, int in_CalObs_whole)
{
    int i = 0, j = 0;
    int tempattr = 0;
    int clearNum = 0;
    int index1   = FusionObj->obj_num;

    for (i = SF_OBJ_NUM - 1; i >= SF_OBJ_NUM - index1; --i)
    {
        /*
            %is_Wholeobj
            % 0: initial value
            % 1: On the right CalObs_whole gets
            % 2: On the left CalObs_whole gets
            % -1:On the right CalObs_part gets
            % -2:On the left CalObs_part gets
        */

        if (in_CalObs_whole == 1)
        {
            /*  CalObs_whole In the calculation process, all is_Wholeobj=1 or-2
            segments obtained before CalObs_part need to be deleted.*/
            if (FusionObj->is_Wholeobj[i] == -2 || FusionObj->is_Wholeobj[i] == -1)
            {
                FusionObj->obj_num         = FusionObj->obj_num - 1;
                FusionObj->obj_attr[i]     = 0;
                FusionObj->obj[i].pt1.x    = 0;
                FusionObj->obj[i].pt1.y    = 0;
                FusionObj->obj[i].pt2.x    = 0;
                FusionObj->obj[i].pt2.y    = 0;
                FusionObj->pathstart[i]    = 0;
                FusionObj->is_Wholeobj[i]  = 0;
                FusionObj->is_over2dot5[i] = 0;
                FusionObj->ang[i]          = 0;

                clearNum = clearNum + 1;
            }
        }
        else
        {
            /* CalObs_part From the calculation process,
            two types should be deleted.
            one is the object which has the temporary attribute = 5
            the other one is the object of which the temporary attribute is not equal to
            5,but the length is larger than 2.5 meters. In this situation it should be
            deleted in each cycle and updated by the new outline*/
            tempattr = (FusionObj->obj_attr[i] / 1000) % 10;
            if (tempattr == 5 || FusionObj->is_over2dot5[i] == 1)
            {
                FusionObj->obj_num         = FusionObj->obj_num - 1;
                FusionObj->obj_attr[i]     = 0;
                FusionObj->obj[i].pt1.x    = 0;
                FusionObj->obj[i].pt1.y    = 0;
                FusionObj->obj[i].pt2.x    = 0;
                FusionObj->obj[i].pt2.y    = 0;
                FusionObj->pathstart[i]    = 0;
                FusionObj->is_Wholeobj[i]  = 0;
                FusionObj->is_over2dot5[i] = 0;
                FusionObj->ang[i]          = 0;

                clearNum = clearNum + 1;
            }
        }
    }

    // Move after deletion.
    if (clearNum > 0)
    {
        for (j = SF_OBJ_NUM - 1; j >= clearNum; --j)
        {
            FusionObj->obj_attr[j]     = FusionObj->obj_attr[j - clearNum];
            FusionObj->obj[j].pt1.x    = FusionObj->obj[j - clearNum].pt1.x;
            FusionObj->obj[j].pt1.y    = FusionObj->obj[j - clearNum].pt1.y;
            FusionObj->obj[j].pt2.x    = FusionObj->obj[j - clearNum].pt2.x;
            FusionObj->obj[j].pt2.y    = FusionObj->obj[j - clearNum].pt2.y;
            FusionObj->pathstart[j]    = FusionObj->pathstart[j - clearNum];
            FusionObj->is_Wholeobj[j]  = FusionObj->is_Wholeobj[j - clearNum];
            FusionObj->is_over2dot5[j] = FusionObj->is_over2dot5[j - clearNum];
            FusionObj->ang[j]          = FusionObj->ang[j - clearNum];
        }

        for (j = 0; j < clearNum; ++j)
        {
            FusionObj->obj_attr[j]     = 0;
            FusionObj->obj[j].pt1.x    = 0;
            FusionObj->obj[j].pt1.y    = 0;
            FusionObj->obj[j].pt2.x    = 0;
            FusionObj->obj[j].pt2.y    = 0;
            FusionObj->pathstart[j]    = 0;
            FusionObj->is_Wholeobj[j]  = 0;
            FusionObj->is_over2dot5[j] = 0;
            FusionObj->ang[j]          = 0;
        }
    }
}

void UpdateObj(U_FusionObj32_T *FusionObj, U_FusionObj_T FusionObj_Cur)
{
    int i = 0, j = 0;
    int num = 0;

    num = FusionObj_Cur.obj_num;

    for (i = 0; i < num; ++i)
    {
        // Output all updated data
        if (FusionObj->obj_num < SF_OBJ_NUM)
        {
            // The inversion method is consistent with the previous RAM storage method,
            // with a leading bit.
            for (j = SF_OBJ_NUM - 1 - FusionObj->obj_num; j < SF_OBJ_NUM - 1; ++j)
            {
                FusionObj->obj_attr[j]     = FusionObj->obj_attr[j + 1];
                FusionObj->obj[j].pt1.x    = FusionObj->obj[j + 1].pt1.x;
                FusionObj->obj[j].pt1.y    = FusionObj->obj[j + 1].pt1.y;
                FusionObj->obj[j].pt2.x    = FusionObj->obj[j + 1].pt2.x;
                FusionObj->obj[j].pt2.y    = FusionObj->obj[j + 1].pt2.y;
                FusionObj->is_Wholeobj[j]  = FusionObj->is_Wholeobj[j + 1];
                FusionObj->pathstart[j]    = FusionObj->pathstart[j + 1];
                FusionObj->is_over2dot5[j] = FusionObj->is_over2dot5[j + 1];
                FusionObj->ang[j]          = FusionObj->ang[j + 1];
            }
            FusionObj->obj_attr[SF_OBJ_NUM - 1]     = FusionObj_Cur.obj_attr[i];
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.x    = FusionObj_Cur.obj[i].pt1.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.y    = FusionObj_Cur.obj[i].pt1.y;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.x    = FusionObj_Cur.obj[i].pt2.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.y    = FusionObj_Cur.obj[i].pt2.y;
            FusionObj->is_Wholeobj[SF_OBJ_NUM - 1]  = FusionObj_Cur.is_Wholeobj[i];
            FusionObj->pathstart[SF_OBJ_NUM - 1]    = FusionObj_Cur.pathstart[i];
            FusionObj->is_over2dot5[SF_OBJ_NUM - 1] = FusionObj_Cur.is_over2dot5[i];
            FusionObj->ang[SF_OBJ_NUM - 1]          = FusionObj_Cur.ang[i];

            FusionObj->obj_num = FusionObj->obj_num + 1;
        }
        else
        {
            FusionObj->obj_num = SF_OBJ_NUM;
            for (j = 0; j < SF_OBJ_NUM - 1; ++j)
            {
                FusionObj->obj_attr[j]     = FusionObj->obj_attr[j + 1];
                FusionObj->obj[j].pt1.x    = FusionObj->obj[j + 1].pt1.x;
                FusionObj->obj[j].pt1.y    = FusionObj->obj[j + 1].pt1.y;
                FusionObj->obj[j].pt2.x    = FusionObj->obj[j + 1].pt2.x;
                FusionObj->obj[j].pt2.y    = FusionObj->obj[j + 1].pt2.y;
                FusionObj->is_Wholeobj[j]  = FusionObj->is_Wholeobj[j + 1];
                FusionObj->pathstart[j]    = FusionObj->pathstart[j + 1];
                FusionObj->is_over2dot5[j] = FusionObj->is_over2dot5[j + 1];
                FusionObj->ang[j]          = FusionObj->ang[j + 1];
            }

            FusionObj->obj_attr[SF_OBJ_NUM - 1]     = FusionObj_Cur.obj_attr[i];
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.x    = FusionObj_Cur.obj[i].pt1.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.y    = FusionObj_Cur.obj[i].pt1.y;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.x    = FusionObj_Cur.obj[i].pt2.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.y    = FusionObj_Cur.obj[i].pt2.y;
            FusionObj->is_Wholeobj[SF_OBJ_NUM - 1]  = FusionObj_Cur.is_Wholeobj[i];
            FusionObj->pathstart[SF_OBJ_NUM - 1]    = FusionObj_Cur.pathstart[i];
            FusionObj->is_over2dot5[SF_OBJ_NUM - 1] = FusionObj_Cur.is_over2dot5[i];
            FusionObj->ang[SF_OBJ_NUM - 1]          = FusionObj_Cur.ang[i];
        }
    }
}

void UpdateObj_V2(U_FusionObj32_T *FusionObj, const U_FusionObj_T &FusionObj_Cur,
                  uint8 beUpdated_ByOdo, uint8 *beLooped_Flag, float *beLooped_S)
{
    int i = 0, j = 0;
    int num             = 0;
    float cur_pathstart = 0;
    int insertidx       = 0; // the range of insertidx is from 0
    uint8 beFront_Flag  = 0; // if beFront_Flag is true, the position of the inserted
                             // object is beyond all of the other objects

    num       = FusionObj_Cur.obj_num;
    insertidx = SF_OBJ_NUM - 1;

    if (NULL != beLooped_Flag)
        *beLooped_Flag = 0U;
    if (NULL != beLooped_S)
        *beLooped_S = 0.0f;

    if (num <= 0)
        return;

    // when it's updated by rear sonar data, beUpdated_ByOdo should be true.
    if (beUpdated_ByOdo)
    {
        cur_pathstart = FusionObj_Cur.pathstart[0];
        // first step: search the insert position by the parameter: pathstart
        if (cur_pathstart <= FusionObj->pathstart[SF_OBJ_NUM - FusionObj->obj_num])
        {
            beFront_Flag = 1U;
            // the position of the inserted object is beyond all of the other objects
            if (FusionObj->obj_num < SF_OBJ_NUM)
            {
                insertidx = SF_OBJ_NUM - FusionObj->obj_num - 1;
            }
            else
            {
                insertidx = 0;
            }
        }
        else
        {
            beFront_Flag = 0U;
            // the position of the inserted object is between them or at the last
            // position.
            for (i = SF_OBJ_NUM - 1; i >= SF_OBJ_NUM - FusionObj->obj_num; i--)
            {
                if (cur_pathstart > FusionObj->pathstart[i])
                {
                    insertidx = i;
                    break;
                }
            }
        }
    }

    // when update the front sonar data, beFront_Flag = 0.
    if (0U == beFront_Flag)
    {
        for (i = 0; i < num; ++i)
        {
            // Output all updated data
            if (FusionObj->obj_num < SF_OBJ_NUM)
            {
                // The inversion method is consistent with the previous RAM storage
                // method, with a leading bit.
                for (j = SF_OBJ_NUM - 1 - FusionObj->obj_num; j <= insertidx - 1; ++j)
                {
                    FusionObj->obj_attr[j]     = FusionObj->obj_attr[j + 1];
                    FusionObj->obj[j].pt1.x    = FusionObj->obj[j + 1].pt1.x;
                    FusionObj->obj[j].pt1.y    = FusionObj->obj[j + 1].pt1.y;
                    FusionObj->obj[j].pt2.x    = FusionObj->obj[j + 1].pt2.x;
                    FusionObj->obj[j].pt2.y    = FusionObj->obj[j + 1].pt2.y;
                    FusionObj->is_Wholeobj[j]  = FusionObj->is_Wholeobj[j + 1];
                    FusionObj->pathstart[j]    = FusionObj->pathstart[j + 1];
                    FusionObj->is_over2dot5[j] = FusionObj->is_over2dot5[j + 1];
                    FusionObj->ang[j]          = FusionObj->ang[j + 1];
                }
                FusionObj->obj_attr[insertidx]     = FusionObj_Cur.obj_attr[i];
                FusionObj->obj[insertidx].pt1.x    = FusionObj_Cur.obj[i].pt1.x;
                FusionObj->obj[insertidx].pt1.y    = FusionObj_Cur.obj[i].pt1.y;
                FusionObj->obj[insertidx].pt2.x    = FusionObj_Cur.obj[i].pt2.x;
                FusionObj->obj[insertidx].pt2.y    = FusionObj_Cur.obj[i].pt2.y;
                FusionObj->is_Wholeobj[insertidx]  = FusionObj_Cur.is_Wholeobj[i];
                FusionObj->pathstart[insertidx]    = FusionObj_Cur.pathstart[i];
                FusionObj->is_over2dot5[insertidx] = FusionObj_Cur.is_over2dot5[i];
                FusionObj->ang[insertidx]          = FusionObj_Cur.ang[i];

                FusionObj->obj_num = FusionObj->obj_num + 1;
            }
            else
            {
                FusionObj->obj_num = SF_OBJ_NUM;
                if (NULL != beLooped_Flag)
                    *beLooped_Flag = 1U;
                if (NULL != beLooped_S)
                    *beLooped_S = FusionObj->pathstart[0];

                for (j = 0; j <= insertidx - 1; ++j)
                {
                    FusionObj->obj_attr[j]     = FusionObj->obj_attr[j + 1];
                    FusionObj->obj[j].pt1.x    = FusionObj->obj[j + 1].pt1.x;
                    FusionObj->obj[j].pt1.y    = FusionObj->obj[j + 1].pt1.y;
                    FusionObj->obj[j].pt2.x    = FusionObj->obj[j + 1].pt2.x;
                    FusionObj->obj[j].pt2.y    = FusionObj->obj[j + 1].pt2.y;
                    FusionObj->is_Wholeobj[j]  = FusionObj->is_Wholeobj[j + 1];
                    FusionObj->pathstart[j]    = FusionObj->pathstart[j + 1];
                    FusionObj->is_over2dot5[j] = FusionObj->is_over2dot5[j + 1];
                    FusionObj->ang[j]          = FusionObj->ang[j + 1];
                }

                FusionObj->obj_attr[insertidx]     = FusionObj_Cur.obj_attr[i];
                FusionObj->obj[insertidx].pt1.x    = FusionObj_Cur.obj[i].pt1.x;
                FusionObj->obj[insertidx].pt1.y    = FusionObj_Cur.obj[i].pt1.y;
                FusionObj->obj[insertidx].pt2.x    = FusionObj_Cur.obj[i].pt2.x;
                FusionObj->obj[insertidx].pt2.y    = FusionObj_Cur.obj[i].pt2.y;
                FusionObj->is_Wholeobj[insertidx]  = FusionObj_Cur.is_Wholeobj[i];
                FusionObj->pathstart[insertidx]    = FusionObj_Cur.pathstart[i];
                FusionObj->is_over2dot5[insertidx] = FusionObj_Cur.is_over2dot5[i];
                FusionObj->ang[insertidx]          = FusionObj_Cur.ang[i];
            }
        }
    }
    else
    {
        // update rear sonar data at the situation that there are already objects in
        // FusionObj extracted by the front sonar data.
        for (i = num - 1; i >= 0; --i)
        {
            // insert the objects in front of the FusionObj objects.
            // if the container size is SF_OBJ_NUM, stop updating the rear objects since
            // the front sonar objects are more important
            if (FusionObj->obj_num < SF_OBJ_NUM && insertidx >= 0)
            {
                FusionObj->obj_attr[insertidx]     = FusionObj_Cur.obj_attr[i];
                FusionObj->obj[insertidx].pt1.x    = FusionObj_Cur.obj[i].pt1.x;
                FusionObj->obj[insertidx].pt1.y    = FusionObj_Cur.obj[i].pt1.y;
                FusionObj->obj[insertidx].pt2.x    = FusionObj_Cur.obj[i].pt2.x;
                FusionObj->obj[insertidx].pt2.y    = FusionObj_Cur.obj[i].pt2.y;
                FusionObj->is_Wholeobj[insertidx]  = FusionObj_Cur.is_Wholeobj[i];
                FusionObj->pathstart[insertidx]    = FusionObj_Cur.pathstart[i];
                FusionObj->is_over2dot5[insertidx] = FusionObj_Cur.is_over2dot5[i];
                FusionObj->ang[insertidx]          = FusionObj_Cur.ang[i];

                FusionObj->obj_num = FusionObj->obj_num + 1;
                insertidx          = insertidx - 1;
            }
            else
            {
            }
        }
    }
}

void UpdateObj_CCP(FusionObj_T *FusionObj, U_FusionObj_T FusionObj_Cur)
{
    int i = 0, j = 0;
    int num = 0;

    num = FusionObj_Cur.obj_num;
    for (i = 0; i < num; ++i)
    {
        // Output all updated data
        if (FusionObj->num < SF_OBJ_NUM)
        {
            // The inversion method is consistent with the previous RAM storage method,
            // with a leading bit.
            for (j = SF_OBJ_NUM - 1 - FusionObj->num; j < SF_OBJ_NUM - 1; ++j)
            {
                FusionObj->ang[j]       = FusionObj->ang[j + 1];
                FusionObj->attr[j]      = FusionObj->attr[j + 1];
                FusionObj->odom[j]      = FusionObj->odom[j + 1];
                FusionObj->obj[j].pt1.x = FusionObj->obj[j + 1].pt1.x;
                FusionObj->obj[j].pt1.y = FusionObj->obj[j + 1].pt1.y;
                FusionObj->obj[j].pt2.x = FusionObj->obj[j + 1].pt2.x;
                FusionObj->obj[j].pt2.y = FusionObj->obj[j + 1].pt2.y;
            }

            FusionObj->ang[SF_OBJ_NUM - 1]       = FusionObj_Cur.ang[i];
            FusionObj->attr[SF_OBJ_NUM - 1]      = FusionObj_Cur.obj_attr[i];
            FusionObj->odom[SF_OBJ_NUM - 1]      = FusionObj_Cur.pathstart[i];
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.x = FusionObj_Cur.obj[i].pt1.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.y = FusionObj_Cur.obj[i].pt1.y;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.x = FusionObj_Cur.obj[i].pt2.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.y = FusionObj_Cur.obj[i].pt2.y;

            FusionObj->num      = FusionObj->num + 1;
            FusionObj->num_accu = FusionObj->num_accu + 1;
        }
        else
        {
            FusionObj->num      = SF_OBJ_NUM;
            FusionObj->num_accu = FusionObj->num_accu + 1;
            for (j = 0; j < SF_OBJ_NUM - 1; ++j)
            {
                FusionObj->ang[j]       = FusionObj->ang[j + 1];
                FusionObj->attr[j]      = FusionObj->attr[j + 1];
                FusionObj->odom[j]      = FusionObj->odom[j + 1];
                FusionObj->obj[j].pt1.x = FusionObj->obj[j + 1].pt1.x;
                FusionObj->obj[j].pt1.y = FusionObj->obj[j + 1].pt1.y;
                FusionObj->obj[j].pt2.x = FusionObj->obj[j + 1].pt2.x;
                FusionObj->obj[j].pt2.y = FusionObj->obj[j + 1].pt2.y;
            }

            FusionObj->ang[SF_OBJ_NUM - 1]       = FusionObj_Cur.ang[i];
            FusionObj->attr[SF_OBJ_NUM - 1]      = FusionObj_Cur.obj_attr[i];
            FusionObj->odom[SF_OBJ_NUM - 1]      = FusionObj_Cur.pathstart[i];
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.x = FusionObj_Cur.obj[i].pt1.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt1.y = FusionObj_Cur.obj[i].pt1.y;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.x = FusionObj_Cur.obj[i].pt2.x;
            FusionObj->obj[SF_OBJ_NUM - 1].pt2.y = FusionObj_Cur.obj[i].pt2.y;
        }
    }
}

void Set_PK_SENSORFUSION_A_ARR_CCP(LineSeg_T Segm, int dir)
{
    PK_SENSORFUSION_A_ARR_CCP[0] = Segm.pt1.x;
    PK_SENSORFUSION_A_ARR_CCP[1] = Segm.pt1.y;
    PK_SENSORFUSION_A_ARR_CCP[2] = Segm.pt2.x;
    PK_SENSORFUSION_A_ARR_CCP[3] = Segm.pt2.y;
    PK_SENSORFUSION_A_ARR_NUM_CCP++;
    PK_SENSORFUSION_A_ARR_DIR_CCP = dir;
}

void Set_PK_SENSORFUSION_ARear_ARR_CCP(LineSeg_T Segm, int dir)
{
    PK_SENSORFUSION_B_ARR_CCP[0] = Segm.pt1.x;
    PK_SENSORFUSION_B_ARR_CCP[1] = Segm.pt1.y;
    PK_SENSORFUSION_B_ARR_CCP[2] = Segm.pt2.x;
    PK_SENSORFUSION_B_ARR_CCP[3] = Segm.pt2.y;
    PK_SENSORFUSION_B_ARR_NUM_CCP++;
    PK_SENSORFUSION_B_ARR_DIR_CCP = dir;
}

void Set_SF_A_URadar_ccp(void)
{
    static int last_send_index_left = 0, last_send_index_right = 0;
    LineSeg_T new_segm;
    int set_left_flag = 0;
    int leftNum       = FusionObjLeft_CCP.num_accu;
    int rightNum      = FusionObjRight_CCP.num_accu;
    int dir           = 0;
    int sendIndex     = 0;
    int delta_num     = 0;

    static int outputTimesNum_LeftFront =
        0; // Statistics of the number of times used to output CCP
    static int outputTimesNum_RightFront = 0;

    if (leftNum == 0)
    {
        last_send_index_left = 0;
    }
    if (rightNum == 0)
    {
        last_send_index_right = 0;
    }

    if (last_send_index_left < leftNum)
    {
        delta_num = leftNum - last_send_index_left;
        sendIndex = SF_OBJ_NUM - delta_num;
        new_segm  = FusionObjLeft_CCP.obj[sendIndex];
        dir       = FusionObjLeft_CCP.attr[sendIndex];
        if (outputTimesNum_LeftFront % 2 == 1)
        {
            outputTimesNum_LeftFront = outputTimesNum_LeftFront + 1;
        }
        else
        {
            Set_PK_SENSORFUSION_A_ARR_CCP(new_segm, dir);
            last_send_index_left++;
            set_left_flag            = 1;
            outputTimesNum_LeftFront = outputTimesNum_LeftFront + 1;

#ifdef _ULTRADEBUG
            printf("FunA_CCP_LOG: %03d :FrontLeft  %06f %06f %06f %06f %04d\n",
                   TestSFAResDataNum_Front + 1, new_segm.pt1.x, new_segm.pt1.y,
                   new_segm.pt2.x, new_segm.pt2.y, dir);
            TestSFAResData_Front[TestSFAResDataNum_Front][0]   = new_segm.pt1.x;
            TestSFAResData_Front[TestSFAResDataNum_Front][1]   = new_segm.pt1.y;
            TestSFAResData_Front[TestSFAResDataNum_Front][2]   = new_segm.pt2.x;
            TestSFAResData_Front[TestSFAResDataNum_Front][3]   = new_segm.pt2.y;
            TestSFAResData_Front_Attr[TestSFAResDataNum_Front] = dir;
            TestSFAResDataNum_Front = TestSFAResDataNum_Front + 1;
#endif
        }
    }

    if (set_left_flag == 0 && last_send_index_right < rightNum)
    {
        delta_num = rightNum - last_send_index_right;
        sendIndex = SF_OBJ_NUM - delta_num;
        new_segm  = FusionObjRight_CCP.obj[sendIndex];
        dir       = FusionObjRight_CCP.attr[sendIndex];
        if (outputTimesNum_RightFront % 2 == 1)
        {
            outputTimesNum_RightFront = outputTimesNum_RightFront + 1;
        }
        else
        {
            Set_PK_SENSORFUSION_A_ARR_CCP(new_segm, dir);
            last_send_index_right++;
            outputTimesNum_RightFront = outputTimesNum_RightFront + 1;

#ifdef _ULTRADEBUG
            printf("FunA_CCP_LOG: %03d :FrontRight  %06f %06f %06f %06f %04d\n",
                   TestSFAResDataNum_Front + 1, new_segm.pt1.x, new_segm.pt1.y,
                   new_segm.pt2.x, new_segm.pt2.y, dir);
            TestSFAResData_Front[TestSFAResDataNum_Front][0]   = new_segm.pt1.x;
            TestSFAResData_Front[TestSFAResDataNum_Front][1]   = new_segm.pt1.y;
            TestSFAResData_Front[TestSFAResDataNum_Front][2]   = new_segm.pt2.x;
            TestSFAResData_Front[TestSFAResDataNum_Front][3]   = new_segm.pt2.y;
            TestSFAResData_Front_Attr[TestSFAResDataNum_Front] = dir;
            TestSFAResDataNum_Front = TestSFAResDataNum_Front + 1;
#endif
        }
    }
}

// #ifdef _ULTRADEBUG
void Set_SF_A_URadar_Rear_ccp(void)
{
    static int last_send_index_leftrear = 0, last_send_index_rightrear = 0;
    LineSeg_T new_segm;
    int set_left_flag = 0;
    int leftNum       = FusionObjLeftRear_CCP.num_accu;
    int rightNum      = FusionObjRightRear_CCP.num_accu;
    int dir           = 0;
    // static int send_dir = 0; // for the last segment to send in alternative way
    int sendIndex = 0;
    int delta_num = 0;

    static int outputTimesNum_LeftRear =
        0; // Statistics of the number of times used to output CCP
    static int outputTimesNum_RightRear = 0;

    if (leftNum == 0)
    {
        last_send_index_leftrear = 0;
    }
    if (rightNum == 0)
    {
        last_send_index_rightrear = 0;
    }

    if (last_send_index_leftrear < leftNum)
    {
        delta_num = leftNum - last_send_index_leftrear;
        sendIndex = SF_OBJ_NUM - delta_num;
        new_segm  = FusionObjLeftRear_CCP.obj[sendIndex];
        dir       = FusionObjLeftRear_CCP.attr[sendIndex];
        if (outputTimesNum_LeftRear % 2 == 1)
        {
            outputTimesNum_LeftRear = outputTimesNum_LeftRear + 1;
        }
        else
        {
            Set_PK_SENSORFUSION_ARear_ARR_CCP(new_segm, dir);
            last_send_index_leftrear++;
            set_left_flag           = 1;
            outputTimesNum_LeftRear = outputTimesNum_LeftRear + 1;

#ifdef _ULTRADEBUG
            printf("FunA_CCP_LOG: %d :RearLeft  %\t%f\t %f\t %f\t %f\t %d\t \r\n",
                   TestSFAResDataNum_Rear + 1, new_segm.pt1.x, new_segm.pt1.y,
                   new_segm.pt2.x, new_segm.pt2.y, dir);
            TestSFAResData_Rear[TestSFAResDataNum_Rear][0]   = new_segm.pt1.x;
            TestSFAResData_Rear[TestSFAResDataNum_Rear][1]   = new_segm.pt1.y;
            TestSFAResData_Rear[TestSFAResDataNum_Rear][2]   = new_segm.pt2.x;
            TestSFAResData_Rear[TestSFAResDataNum_Rear][3]   = new_segm.pt2.y;
            TestSFAResData_Rear_Attr[TestSFAResDataNum_Rear] = dir;
            TestSFAResDataNum_Rear                           = TestSFAResDataNum_Rear + 1;
#endif
        }
    }

    if (set_left_flag == 0 && last_send_index_rightrear < rightNum)
    {
        delta_num = rightNum - last_send_index_rightrear;
        sendIndex = SF_OBJ_NUM - delta_num;
        new_segm  = FusionObjRightRear_CCP.obj[sendIndex];
        dir       = FusionObjRightRear_CCP.attr[sendIndex];
        if (outputTimesNum_RightRear % 2 == 1)
        {
            outputTimesNum_RightRear = outputTimesNum_RightRear + 1;
        }
        else
        {
            Set_PK_SENSORFUSION_ARear_ARR_CCP(new_segm, dir);
            last_send_index_rightrear++;
            outputTimesNum_RightRear = outputTimesNum_RightRear + 1;

#ifdef _ULTRADEBUG
            printf("FunA_CCP_LOG: %d :RearRight  %\t%f\t %f\t %f\t %f\t %d\t \r\n",
                   TestSFAResDataNum_Rear + 1, new_segm.pt1.x, new_segm.pt1.y,
                   new_segm.pt2.x, new_segm.pt2.y, dir);
            TestSFAResData_Rear[TestSFAResDataNum_Rear][0]   = new_segm.pt1.x;
            TestSFAResData_Rear[TestSFAResDataNum_Rear][1]   = new_segm.pt1.y;
            TestSFAResData_Rear[TestSFAResDataNum_Rear][2]   = new_segm.pt2.x;
            TestSFAResData_Rear[TestSFAResDataNum_Rear][3]   = new_segm.pt2.y;
            TestSFAResData_Rear_Attr[TestSFAResDataNum_Rear] = dir;
            TestSFAResDataNum_Rear                           = TestSFAResDataNum_Rear + 1;
#endif
        }
    }
}
// #endif

void PK_SF_A_URadar_Log(void)
{
    int i = 0;
    // The followings are comment out by xls@2022.4.30
    // int LineSegCCP_FSL_delta = 0, LineSegCCP_FSR_delta = 0;
    // int LineSegCCP_RSL_delta = 0, LineSegCCP_RSR_delta = 0;

    // Front
    FusionObjLeft_RTE.num      = FusionObjLeft.obj_num;
    FusionObjLeft_RTE.num_accu = LineSegCCP_FSL_total;

    FusionObjRight_RTE.num      = FusionObjRight.obj_num;
    FusionObjRight_RTE.num_accu = LineSegCCP_FSR_total;

    for (i = 0; i < SF_OBJ_NUM; ++i)
    {
        FusionObjLeft_RTE.ang[i]       = FusionObjLeft.ang[i];
        FusionObjLeft_RTE.attr[i]      = FusionObjLeft.obj_attr[i];
        FusionObjLeft_RTE.odom[i]      = FusionObjLeft.pathstart[i];
        FusionObjLeft_RTE.obj[i].pt1.x = FusionObjLeft.obj[i].pt1.x;
        FusionObjLeft_RTE.obj[i].pt1.y = FusionObjLeft.obj[i].pt1.y;
        FusionObjLeft_RTE.obj[i].pt2.x = FusionObjLeft.obj[i].pt2.x;
        FusionObjLeft_RTE.obj[i].pt2.y = FusionObjLeft.obj[i].pt2.y;
        if (i >= (SF_OBJ_NUM - FusionObjLeft_RTE.num))
        {
            if (sf_curpath[1] - FusionObjLeft.pathstart[i] > PATHVD)
            {
                FusionObjLeft_RTE.attr[i] = 0;
            }
        }

        FusionObjRight_RTE.ang[i]       = FusionObjRight.ang[i];
        FusionObjRight_RTE.attr[i]      = FusionObjRight.obj_attr[i];
        FusionObjRight_RTE.odom[i]      = FusionObjRight.pathstart[i];
        FusionObjRight_RTE.obj[i].pt1.x = FusionObjRight.obj[i].pt1.x;
        FusionObjRight_RTE.obj[i].pt1.y = FusionObjRight.obj[i].pt1.y;
        FusionObjRight_RTE.obj[i].pt2.x = FusionObjRight.obj[i].pt2.x;
        FusionObjRight_RTE.obj[i].pt2.y = FusionObjRight.obj[i].pt2.y;
        if (i >= (SF_OBJ_NUM - FusionObjRight_RTE.num))
        {
            if (sf_curpath[0] - FusionObjRight.pathstart[i] > PATHVD)
            {
                FusionObjRight_RTE.attr[i] = 0;
            }
        }
    }
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Left_Raw(&FusionObjLeft_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Right_Raw(&FusionObjRight_RTE);

    // Rear
    FusionObjLeftRear_RTE.num      = FusionObjLeftRear.obj_num;
    FusionObjLeftRear_RTE.num_accu = LineSegCCP_RSL_total;

    FusionObjRightRear_RTE.num      = FusionObjRightRear.obj_num;
    FusionObjRightRear_RTE.num_accu = LineSegCCP_RSR_total;

    for (i = 0; i < SF_OBJ_NUM; ++i)
    {
        FusionObjLeftRear_RTE.ang[i]       = FusionObjLeftRear.ang[i];
        FusionObjLeftRear_RTE.attr[i]      = FusionObjLeftRear.obj_attr[i];
        FusionObjLeftRear_RTE.odom[i]      = FusionObjLeftRear.pathstart[i];
        FusionObjLeftRear_RTE.obj[i].pt1.x = FusionObjLeftRear.obj[i].pt1.x;
        FusionObjLeftRear_RTE.obj[i].pt1.y = FusionObjLeftRear.obj[i].pt1.y;
        FusionObjLeftRear_RTE.obj[i].pt2.x = FusionObjLeftRear.obj[i].pt2.x;
        FusionObjLeftRear_RTE.obj[i].pt2.y = FusionObjLeftRear.obj[i].pt2.y;
        if (i >= (SF_OBJ_NUM - FusionObjLeftRear_RTE.num))
        {
            if (sf_curpath[2] - FusionObjLeftRear.pathstart[i] > PATHVD)
            {
                FusionObjLeftRear_RTE.attr[i] = 0;
            }
        }

        FusionObjRightRear_RTE.ang[i]       = FusionObjRightRear.ang[i];
        FusionObjRightRear_RTE.attr[i]      = FusionObjRightRear.obj_attr[i];
        FusionObjRightRear_RTE.odom[i]      = FusionObjRightRear.pathstart[i];
        FusionObjRightRear_RTE.obj[i].pt1.x = FusionObjRightRear.obj[i].pt1.x;
        FusionObjRightRear_RTE.obj[i].pt1.y = FusionObjRightRear.obj[i].pt1.y;
        FusionObjRightRear_RTE.obj[i].pt2.x = FusionObjRightRear.obj[i].pt2.x;
        FusionObjRightRear_RTE.obj[i].pt2.y = FusionObjRightRear.obj[i].pt2.y;
        if (i >= (SF_OBJ_NUM - FusionObjRightRear_RTE.num))
        {
            if (sf_curpath[3] - FusionObjRightRear.pathstart[i] > PATHVD)
            {
                FusionObjRightRear_RTE.attr[i] = 0;
            }
        }
    }

    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Left_Raw(&FusionObjLeftRear_RTE);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_A_Rear_Right_Raw(&FusionObjRightRear_RTE);

#ifdef _ULTRADEBUG
    memcpy(&FusionObjLeft_RTE_Debug, &FusionObjLeft_RTE, sizeof(FusionObjLeft_RTE_Debug));
    memcpy(&FusionObjRight_RTE_Debug, &FusionObjRight_RTE,
           sizeof(FusionObjRight_RTE_Debug));
    memcpy(&FusionObjLeftRear_RTE_Debug, &FusionObjLeft_RTE,
           sizeof(FusionObjLeftRear_RTE_Debug));
    memcpy(&FusionObjRightRear_RTE_Debug, &FusionObjRight_RTE,
           sizeof(FusionObjRightRear_RTE_Debug));
#endif

    Set_SF_A_URadar_ccp();
    Set_SF_A_URadar_Rear_ccp();
}
