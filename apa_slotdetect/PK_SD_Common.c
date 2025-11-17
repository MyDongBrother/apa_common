/**
 * @file PK_SD_Common.c
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
#include "Module_API.h"
#include "Rte_ComIF.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "hobotlog/hobotlog.hpp"
/*********************************************************************************************************
**Include headers of the component
*********************************************************************************************************/
#include "PK_SD_Common.h"

/*********************************************************************************************************
**Include other headers
*********************************************************************************************************/
#include <string.h>
#include <math.h>
/*********************************************************************************************************
** Definition of structures and enumerated variables
*********************************************************************************************************/

/*********************************************************************************************************
** Declaration for common used variables
*********************************************************************************************************/

FusionObj_T g_FS_ObsInfo_A;
int Is_BC_Update_Bst = 0;
int Is_DE_Update_Bst = 0;
FusionObj_T g_FS_RearObsInfo_A;
// Point_T g_CenPtCB_A,g_CenPtDE_A;
// float g_ThetaCB_A,g_ThetaDE_A;//  slot info from process A
FusionObj_T g_FS_ObsInfo_B;

/**********************************************************************
 * add by xls@ 2022-06-08：check wether there is space to park on the
 * right side of LEFT VEHICLE or on the left side of the RIGHT VEHICLE
 * if yes return 1,else return 0;
 * slot shape according to vehicle length and depth is treated as constant
 **********************************************************************/
// 计算Veh_1_MainLine的垂线，计算车身后方障碍物各个点到过pt1垂线的距离，
// 若是平行车位所有障碍物距离大于阈值（暂定2.8米，平行车位6.4米）则认为可以泊车

int check_vehicle_space(LineSeg_T current_Vehicle_MainLine, int SlotDir,
                        PK_SlotShapeType *current_slotshape, SlotObj_T *current_slotobj,
                        VehPos_T *current_targpos)
{

    float current_vehicle_len =
        Cal_Dis_Pt2Pt(current_Vehicle_MainLine.pt1, current_Vehicle_MainLine.pt2);
    const float uradar_compensation =
        1.2; // 由于超声波发散角的问题，会导致计算出的车身长度偏长1.2左右
    const float min_vehicle_length = 3.0; // 参考五菱mini ev
    const float min_vehicle_width  = 1.5; // 参考五菱mini ev
    float min_park_space           = 0.6; // 两车之间的间隙
    float park_depth, park_width;         // 车位的宽度和深度
    if (current_vehicle_len > min_vehicle_length + uradar_compensation)
    { // 是平行车位
        park_width = 6.4;
        park_depth = 2.4;
        if (SlotDir == 0)
        {
            *current_slotshape = PK_SLOT_LEFT_PARA;
        }
        else
        {
            *current_slotshape = PK_SLOT_RIGHT_PARA;
        }
    }
    else if (current_vehicle_len > min_vehicle_width)
    { // 是垂直车位
        park_width = 2.8;
        park_depth = 5.0;
        if (SlotDir == 0)
        {
            *current_slotshape = PK_SLOT_LEFT_VERT;
        }
        else
        {
            *current_slotshape = PK_SLOT_RIGHT_VERT;
        }
    }
    else
    {
        return 0;
    }

    float pt1_x = current_Vehicle_MainLine.pt1.x;
    float pt1_y = current_Vehicle_MainLine.pt1.y;
    float pt2_x = current_Vehicle_MainLine.pt2.x;
    float pt2_y = current_Vehicle_MainLine.pt2.y;
    LOGW << "current_vehicle_len:" << current_vehicle_len;
    LOGW << "Veh_1_MainLine.pt1.x :" << pt1_x << "Veh_1_MainLine.pt1.y :" << pt1_y;
    LOGW << "Veh_1_MainLine.pt2.x :" << pt2_x << "Veh_1_MainLine.pt2.y :" << pt2_y;

    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);
    VehPos_T curVehPos;
    memcpy(&curVehPos, curpos, sizeof(VehPos_T));
    float rx = Project_PointTo1stPos_rx(curVehPos, current_Vehicle_MainLine.pt1);
    float ry = Project_PointTo1stPos_ry(curVehPos, current_Vehicle_MainLine.pt1);
    LOGW << "Veh_1_MainLine.pt1 projection on vehicle position rx:" << rx;
    LOGW << "Veh_1_MainLine.pt1 projection on vehicle position ry:" << ry;
    LOGW << "curVehPos:"
         << "  " << curVehPos.x << "  " << curVehPos.y << "  " << curVehPos.theta;
    if (rx > 0)
    {
        LOGW << "Too near from the vehicle, doesn't start to detect!";
        return 0;
    }

    // x,y向量的法向量是-y,x
    LineSeg_T perpendicular_Line;
    perpendicular_Line.pt1.x = pt1_x;
    perpendicular_Line.pt1.y = pt1_y;
    perpendicular_Line.pt2.x = pt1_x + pt1_y - pt2_y;
    perpendicular_Line.pt2.y = pt1_y + pt2_x - pt1_x;

    if ((perpendicular_Line.pt2.x - perpendicular_Line.pt1.x) * (pt2_x - pt1_x) +
            (perpendicular_Line.pt2.y - perpendicular_Line.pt1.y) * (pt2_y - pt1_y) >
        1e-5)
    {
        LOGE << "perpendicular_Line calculation goes error, it may leads to detection "
                "failure!!!";
        return 0;
    };

    FusionObj_T temp_FS_Obj;
    if (SlotDir == 0)
    {
        LineSeg_T current_line;
        float pt2line_dist;
        Relation_T pt2line_dir;
        int flag = 0;
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&temp_FS_Obj);
        int front_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = front_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt1.x < pt1_x - min_park_space)
            { // x小于mainline下端点,判断该点是否位于泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt2,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Left URadar detection: there is obstacle in parking space "
                            "on the right of left vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw(&temp_FS_Obj);
        int rear_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = rear_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt1.x < pt1_x - min_park_space)
            { // x小于mainline下端点,判断该点是否位于泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt2,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Rear left URadar detection: there is obstacle in parking "
                            "space on the right of left vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        if (flag)
        {
            LOGW << "on the right of the LEFT VEHICLE, there is NO enough space to park!";
            return 0;
        }
        else
        {
            LOGW << "on the  right of the LEFT VEHICLE,  there is enough space to park!";

            current_slotobj->ptA.x = pt1_x - (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptA.y = pt1_y - (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptB.x =
                pt1_x - (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptB.y =
                pt1_y - (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptE.x = pt1_x - (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptE.y = pt1_y - (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_y - pt1_y);

            current_slotobj->ptC.x = current_slotobj->ptB.x -
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptC.y = current_slotobj->ptB.y +
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;
            current_slotobj->ptD.x = current_slotobj->ptE.x -
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptD.y = current_slotobj->ptE.y +
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;

            memcpy(&current_slotobj->ptF, &current_Vehicle_MainLine.pt1, sizeof(Point_T));
            Point_T mid_point;

            if (*current_slotshape == PK_SLOT_LEFT_PARA)
            {
                mid_point.x =
                    pt1_x - (min_park_space + park_width / 2 - uradar_compensation / 2 +
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt1_y - (min_park_space + park_width / 2 - uradar_compensation / 2 +
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x =
                    mid_point.x - (pt2_y - pt1_y) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->y =
                    mid_point.y + (pt2_x - pt1_x) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->theta = atan2((pt2_y - pt1_y), (pt2_x - pt1_x));
            }
            else
            {
                mid_point.x =
                    pt1_x - (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt1_y - (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x = mid_point.x - (pt2_y - pt1_y) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->y = mid_point.y + (pt2_x - pt1_x) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->theta =
                    atan2((pt2_y - pt1_y), (pt2_x - pt1_x)) - PI / 2.0;
            }
            return 1;
        }
    }
    else
    {

        LineSeg_T current_line;
        float pt2line_dist;
        Relation_T pt2line_dir;
        int flag = 0;
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&temp_FS_Obj);
        int front_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = front_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt1.x < pt1_x - min_park_space)
            { // x小于mainline下端点,判断该点是否位于泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt2,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Right URadar detection: there is obstacle in parking space "
                            "on the left of right vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw(&temp_FS_Obj);
        int rear_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = rear_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt1.x < pt1_x - min_park_space)
            { // x小于mainline下端点,判断该点是否位于泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt2,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Rear right URadar detection: there is obstacle in parking "
                            "space on the left of right vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        if (flag)
        {
            LOGW
                << "on the left of the RIGHT VEHICLE, there are no enough space to park!";
            return 0;
        }
        else
        {
            LOGW << "on the  left of the RIGHT VEHICLE,  there are space to park!";
            current_slotobj->ptA.x = pt1_x - (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptA.y = pt1_y - (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptB.x =
                pt1_x - (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptB.y =
                pt1_y - (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptE.x = pt1_x - (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptE.y = pt1_y - (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_y - pt1_y);

            current_slotobj->ptC.x = current_slotobj->ptB.x +
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptC.y = current_slotobj->ptB.y -
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;
            current_slotobj->ptD.x = current_slotobj->ptE.x +
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptD.y = current_slotobj->ptE.y -
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;

            memcpy(&current_slotobj->ptF, &current_Vehicle_MainLine.pt1, sizeof(Point_T));
            Point_T mid_point;

            if (*current_slotshape == PK_SLOT_RIGHT_PARA)
            {
                mid_point.x =
                    pt1_x - (min_park_space + park_width / 2 - uradar_compensation / 2 +
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt1_y - (min_park_space + park_width / 2 - uradar_compensation / 2 +
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x =
                    mid_point.x + (pt2_y - pt1_y) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->y =
                    mid_point.y - (pt2_x - pt1_x) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->theta = atan2((pt2_y - pt1_y), (pt2_x - pt1_x));
            }
            else
            {
                mid_point.x =
                    pt1_x - (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt1_y - (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x = mid_point.x + (pt2_y - pt1_y) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->y = mid_point.y - (pt2_x - pt1_x) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->theta =
                    atan2((pt2_y - pt1_y), (pt2_x - pt1_x)) + PI / 2.0;
            }

            return 1;
        }
    }
}
/**********************************************************************
 * add by xls@ 2022-06-10：check wether there is space to park on the
 * left side of LEFT VEHICLE or on the right side of the RIGHT VEHICLE
 * if yes return 1,else return 0;
 * slot shape according to vehicle length and depth is treated as constant
 **********************************************************************/
// 计算Veh_1_MainLine的垂线，计算车身后方障碍物各个点到过pt1垂线的距离，
// 若是平行车位所有障碍物距离大于阈值（暂定2.8米，平行车位6.4米）则认为可以泊车
int check_vehicle_other_space(LineSeg_T current_Vehicle_MainLine, int SlotDir,
                              PK_SlotShapeType *current_slotshape,
                              SlotObj_T *current_slotobj, VehPos_T *current_targpos)
{

    // 计算常规参数和车位类型
    float current_vehicle_len =
        Cal_Dis_Pt2Pt(current_Vehicle_MainLine.pt1, current_Vehicle_MainLine.pt2);
    const float uradar_compensation =
        1.2; // 由于超声波发散角的问题，会导致计算出的车身长度偏长1.2左右
    const float min_vehicle_length = 3.0; // 参考五菱mini ev
    const float min_vehicle_width  = 1.5; // 参考五菱mini ev
    float min_park_space           = 0.6; // 两车之间的间隙
    float park_depth, park_width;         // 车位的宽度和深度
    if (current_vehicle_len > min_vehicle_length + uradar_compensation)
    { // 是平行车位
        park_width = 6.4;
        park_depth = 2.4;
        if (SlotDir == 0)
        {
            *current_slotshape = PK_SLOT_LEFT_PARA;
        }
        else
        {
            *current_slotshape = PK_SLOT_RIGHT_PARA;
        }
    }
    else if (current_vehicle_len > min_vehicle_width)
    { // 是垂直车位
        park_width = 2.8;
        park_depth = 5.0;
        if (SlotDir == 0)
        {
            *current_slotshape = PK_SLOT_LEFT_VERT;
        }
        else
        {
            *current_slotshape = PK_SLOT_RIGHT_VERT;
        }
    }
    else
    {
        return 0;
    }

    float pt1_x = current_Vehicle_MainLine.pt1.x;
    float pt1_y = current_Vehicle_MainLine.pt1.y;
    float pt2_x = current_Vehicle_MainLine.pt2.x;
    float pt2_y = current_Vehicle_MainLine.pt2.y;

    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);
    VehPos_T curVehPos;
    memcpy(&curVehPos, curpos, sizeof(VehPos_T));
    float rx = Project_PointTo1stPos_rx(curVehPos, current_Vehicle_MainLine.pt2);
    float ry = Project_PointTo1stPos_ry(curVehPos, current_Vehicle_MainLine.pt2);
    LOGW << "Veh_1_MainLine.pt2 projection on vehicle position rx:" << rx;
    LOGW << "Veh_1_MainLine.pt2 projection on vehicle position ry:" << ry;
    LOGW << "curVehPos:"
         << "  " << curVehPos.x << "  " << curVehPos.y << "  " << curVehPos.theta;
    if (rx > -(park_width + min_park_space))
    {
        LOGW << "Too near from the vehicle, doesn't start to detect!";
        return 0;
    }

    LineSeg_T perpendicular_Line;
    perpendicular_Line.pt1.x = pt2_x;
    perpendicular_Line.pt1.y = pt2_y;
    perpendicular_Line.pt2.x = pt2_x + pt1_y - pt2_y;
    perpendicular_Line.pt2.y = pt2_y + pt2_x - pt1_x;
    if ((perpendicular_Line.pt2.x - perpendicular_Line.pt1.x) * (pt2_x - pt1_x) +
            (perpendicular_Line.pt2.y - perpendicular_Line.pt1.y) * (pt2_y - pt1_y) >
        1e-5)
    {
        LOGE << "perpendicular_Line calculation goes error, it may leads to detection "
                "failure!!!";
        return 0;
    }

    FusionObj_T temp_FS_Obj;
    if (SlotDir == 0)
    {
        LineSeg_T current_line;
        float pt2line_dist;
        Relation_T pt2line_dir;
        int flag = 0;
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left_Raw(&temp_FS_Obj);
        int front_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = front_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt2.x > pt2_x + min_park_space)
            { // x小于mainline下端点,判断该点是否在目标泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt1,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Left URadar detection: there is obstacle in parking space "
                            "on the left of left vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left_Raw(&temp_FS_Obj);
        int rear_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = rear_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt2.x > pt2_x + min_park_space)
            { // x小于mainline下端点,判断该点是否在目标泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt1,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Rear left URadar detection: there is obstacle in parking "
                            "space on the left of left vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        if (flag)
        {
            LOGW << "on the left of the LEFT VEHICLE, there is NO enough space to park!";
            return 0;
        }
        else
        {
            LOGW << "on the  left of the LEFT VEHICLE,  there is enough space to park!";

            current_slotobj->ptF.x = pt2_x + (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptF.y = pt2_y + (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptE.x =
                pt2_x + (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptE.y =
                pt2_y + (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptB.x = pt2_x + (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptB.y = pt2_y + (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_y - pt1_y);

            current_slotobj->ptD.x = current_slotobj->ptE.x -
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptD.y = current_slotobj->ptE.y +
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;
            current_slotobj->ptC.x = current_slotobj->ptB.x -
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptC.y = current_slotobj->ptB.y +
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;

            memcpy(&current_slotobj->ptA, &current_Vehicle_MainLine.pt2, sizeof(Point_T));
            Point_T mid_point;

            if (*current_slotshape == PK_SLOT_LEFT_PARA)
            {
                mid_point.x =
                    pt2_x + (min_park_space + park_width / 2 - uradar_compensation / 2 -
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt2_y + (min_park_space + park_width / 2 - uradar_compensation / 2 -
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x =
                    mid_point.x - (pt2_y - pt1_y) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->y =
                    mid_point.y + (pt2_x - pt1_x) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->theta = atan((pt2_y - pt1_y) / (pt2_x - pt1_x));
            }
            else
            {
                mid_point.x =
                    pt2_x + (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt2_y + (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x = mid_point.x - (pt2_y - pt1_y) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->y = mid_point.y + (pt2_x - pt1_x) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->theta =
                    atan((pt2_y - pt1_y) / (pt2_x - pt1_x)) - PI / 2.0;
            }

            return 1;
        }
    }
    else
    {

        LineSeg_T current_line;
        float pt2line_dist;
        Relation_T pt2line_dir;
        int flag = 0;
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right_Raw(&temp_FS_Obj);
        int front_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = front_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt2.x > pt2_x + min_park_space)
            { // x小于mainline下端点,判断该点是否位于泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt1,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Right URadar detection: there is obstacle in parking space "
                            "on the right of right vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right_Raw(&temp_FS_Obj);
        int rear_StartIndex =
            temp_FS_Obj.num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - temp_FS_Obj.num;
        for (int current_index = rear_StartIndex; current_index < SF_OBJ_NUM;
             current_index++)
        {
            memcpy(&current_line, &temp_FS_Obj.obj[current_index], sizeof(current_line));
            if (current_line.pt2.x > pt2_x + min_park_space)
            { // x小于mainline下端点,判断该点是否位于泊车区域内
                Get_Dist_Dir_Pt2SegLine(perpendicular_Line, current_line.pt1,
                                        &pt2line_dist, &pt2line_dir);
                float mainline_dist1, mainline_dist2;
                Relation_T mainline_dir1, mainline_dir2;
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt1,
                                        &mainline_dist1, &mainline_dir1);
                Get_Dist_Dir_Pt2SegLine(current_Vehicle_MainLine, current_line.pt2,
                                        &mainline_dist2, &mainline_dir2);
                if ((pt2line_dist < park_width + min_park_space) &&
                    (mainline_dist1 < park_depth - 0.2 ||
                     mainline_dist2 < park_depth - 0.2))
                {
                    LOGW << "Rear right URadar detection: there is obstacle in parking "
                            "space on the right of right vehicle!";
                    flag = 1;
                    break;
                }
            }
        }
        if (flag)
        {
            LOGW << "on the right of the RIGHT VEHICLE, there are no enough space to "
                    "park!";
            return 0;
        }
        else
        {
            LOGW << "on the  right of the RIGHT VEHICLE,  there are space to park!";
            current_slotobj->ptF.x = pt2_x + (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptF.y = pt2_y + (2 * min_park_space + park_width) /
                                                 current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptE.x =
                pt2_x + (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptE.y =
                pt2_y + (min_park_space + park_width - uradar_compensation / 2) /
                            current_vehicle_len * (pt2_y - pt1_y);
            current_slotobj->ptB.x = pt2_x + (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_x - pt1_x);
            current_slotobj->ptB.y = pt2_y + (min_park_space - uradar_compensation / 2) /
                                                 current_vehicle_len * (pt2_y - pt1_y);

            current_slotobj->ptD.x = current_slotobj->ptE.x +
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptD.y = current_slotobj->ptE.y -
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;
            current_slotobj->ptC.x = current_slotobj->ptB.x +
                                     (pt2_y - pt1_y) / current_vehicle_len * park_depth;
            current_slotobj->ptC.y = current_slotobj->ptB.y -
                                     (pt2_x - pt1_x) / current_vehicle_len * park_depth;

            memcpy(&current_slotobj->ptA, &current_Vehicle_MainLine.pt2, sizeof(Point_T));
            Point_T mid_point;

            if (*current_slotshape == PK_SLOT_RIGHT_PARA)
            {
                mid_point.x =
                    pt2_x + (min_park_space + park_width / 2 - uradar_compensation / 2 -
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt2_y + (min_park_space + park_width / 2 - uradar_compensation / 2 -
                             (VEHICLE_LEN / 2 - REAR_SUSPENSION)) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x =
                    mid_point.x + (pt2_y - pt1_y) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->y =
                    mid_point.y - (pt2_x - pt1_x) / current_vehicle_len * VEHICLE_WID / 2;
                current_targpos->theta = atan((pt2_y - pt1_y) / (pt2_x - pt1_x));
            }
            else
            {
                mid_point.x =
                    pt2_x + (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_x - pt1_x);
                mid_point.y =
                    pt2_y + (min_park_space + park_width / 2 - uradar_compensation / 2) /
                                current_vehicle_len * (pt2_y - pt1_y);
                current_targpos->x = mid_point.x + (pt2_y - pt1_y) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->y = mid_point.y - (pt2_x - pt1_x) / current_vehicle_len *
                                                       (VEHICLE_LEN - REAR_SUSPENSION);
                current_targpos->theta =
                    atan((pt2_y - pt1_y) / (pt2_x - pt1_x)) + PI / 2.0;
            }
            return 1;
        }
    }
}

/**********************************************************************
 * add by xls@ 2022-05-14：calculate adjust value according to
 * URadar data of song plus which collected by Long Yunxiang.
 * we assume FSL and FSR are symmetric, RSL,RSR are symmetric.
 **********************************************************************/

void adjust_veh_1_mainline(LineSeg_T *Veh_1_MainLine)
{
    // theta=>  angle between vehicle and x axis,anti-clockwise is positive!
    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);
    Point_T midpoint = {0};
    midpoint.x       = (Veh_1_MainLine->pt1.x + Veh_1_MainLine->pt2.x) / 2.0;
    midpoint.y       = (Veh_1_MainLine->pt1.y + Veh_1_MainLine->pt2.y) / 2.0;

    LineSeg_T veh_segline    = {0};
    veh_segline.pt1.x        = curpos[0];
    veh_segline.pt1.y        = curpos[1];
    veh_segline.pt2.x        = curpos[0] + cos(curpos[2]);
    veh_segline.pt2.y        = curpos[1] + sin(curpos[2]);
    const float adjust_ratio = 0.5;
    float dist               = 0.0;
    Relation_T dir;
    // Here it has been simplified, we assume that steering angle doesn't change!
    Get_Dist_Dir_Pt2PointLine(veh_segline.pt1, veh_segline.pt2, midpoint, &dist, &dir);
    float value = get_adjust_value_RSL((dist - VEHICLE_WID / 2.0) * 100, REAR_OFFSET) /
                  100.0; // 1st vehicle line
    float seg_length = Get_Segment_Len(*Veh_1_MainLine);
    float delta_x = value / seg_length * (Veh_1_MainLine->pt2.x - Veh_1_MainLine->pt1.x);
    float delta_y = value / seg_length * (Veh_1_MainLine->pt2.y - Veh_1_MainLine->pt1.y);
    Veh_1_MainLine->pt2.x = Veh_1_MainLine->pt2.x - delta_x * adjust_ratio;
    Veh_1_MainLine->pt2.y = Veh_1_MainLine->pt2.y - delta_y * adjust_ratio;
}

void adjust_veh_2_mainline(LineSeg_T *Veh_2_MainLine)
{
    // theta=>  angle between vehicle and x axis,anti-clockwise is positive!
    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);
    Point_T midpoint = {0};
    midpoint.x       = (Veh_2_MainLine->pt1.x + Veh_2_MainLine->pt2.x) / 2.0;
    midpoint.y       = (Veh_2_MainLine->pt1.y + Veh_2_MainLine->pt2.y) / 2.0;

    LineSeg_T veh_segline    = {0};
    veh_segline.pt1.x        = curpos[0];
    veh_segline.pt1.y        = curpos[1];
    veh_segline.pt2.x        = curpos[0] + cos(curpos[2]);
    veh_segline.pt2.y        = curpos[1] + sin(curpos[2]);
    const float adjust_ratio = 0.5;
    float dist               = 0.0;
    Relation_T dir;
    // Here it has been simplified, we assume that steering angle doesn't change!
    Get_Dist_Dir_Pt2PointLine(veh_segline.pt1, veh_segline.pt2, midpoint, &dist, &dir);
    float value = get_adjust_value_FSL((dist - VEHICLE_WID / 2.0) * 100, FRONT_OFFSET) /
                  100.0; // 2st vehicle line
    float seg_length = Get_Segment_Len(*Veh_2_MainLine);
    float delta_x = value / seg_length * (Veh_2_MainLine->pt2.x - Veh_2_MainLine->pt1.x);
    float delta_y = value / seg_length * (Veh_2_MainLine->pt2.y - Veh_2_MainLine->pt1.y);
    Veh_2_MainLine->pt1.x = Veh_2_MainLine->pt1.x + delta_x * adjust_ratio;
    Veh_2_MainLine->pt1.y = Veh_2_MainLine->pt1.y + delta_y * adjust_ratio;
}

float get_adjust_value_FSL(float sensor_value, int direction)
{

    // direction: 0-> front, 1-> back
    if (direction == FRONT_OFFSET)
    {
        if (sensor_value < 30.0)
        {
            return 0.0;
        }
        else if (sensor_value < 100.0)
        {
            float value = 0.4286 * sensor_value + 17.14 - sensor_value * cos(FSL_AERFA);
            return value;
        }
        else if (sensor_value < 130.0)
        {
            float value = 2.0 * sensor_value - 140.0 - sensor_value * cos(FSL_AERFA);
            return value;
        }
        else if (sensor_value < 225.0)
        {
            float value = 100.0 - sensor_value * cos(FSL_AERFA);
            return value;
        }
        else if (sensor_value < 255.0)
        {
            float value = 393.3 - 1.333 * sensor_value - sensor_value * cos(FSL_AERFA);
            return value;
        }
        else
        {
            return 0.0;
        }
    }
    else
    {
        if (sensor_value < 30.0)
        {
            return 0.0;
        }
        else if (sensor_value < 110.0)
        {
            float value = sensor_value * cos(FSL_AERFA) + 0.125 * sensor_value + 16.25;
            return value;
        }
        else if (sensor_value < 140.0)
        {
            float value = sensor_value * cos(FSL_AERFA) + 0.5 * sensor_value - 25.0;
            return value;
        }
        else if (sensor_value < 225.0)
        {
            float value = sensor_value * cos(FSL_AERFA) + 80.0 - 0.2 * sensor_value;
            return value;
        }
        else if (sensor_value < 255.0)
        {
            float value = sensor_value * cos(FSL_AERFA) + 297.5 - sensor_value * 1.1667;
            return value;
        }
        else
        {
            return 0.0;
        }
    }
}

float get_adjust_value_RSL(float sensor_value, int direction)
{

    // direction: 0-> front, 1-> back
    if (direction == FRONT_OFFSET)
    {
        if (sensor_value < 30.0)
        {
            return 0.0;
        }
        else if (sensor_value < 170.0)
        {
            float value = 0.2143 * sensor_value + 13.57;
            return value;
        }
        else if (sensor_value < 190.0)
        {
            float value = 1.5 * sensor_value - 205;
            return value;
        }
        else if (sensor_value < 210.0)
        {
            float value = 80.0;
            return value;
        }
        else if (sensor_value < 255.0)
        {
            float value = 395.0 - 1.5 * sensor_value;
            return value;
        }
        else
        {
            return 0.0;
        }
    }
    else
    {
        if (sensor_value < 30.0)
        {
            return 0.0;
        }
        else if (sensor_value < 170.0)
        {
            float value = 0.2143 * sensor_value + 13.57;
            return value;
        }
        else if (sensor_value < 190.0)
        {
            float value = 1.5 * sensor_value - 205;
            return value;
        }
        else if (sensor_value < 255.0)
        {
            float value = 238.333 - sensor_value * 0.8333;
            return value;
        }
        else
        {
            return 0.0;
        }
    }
}
/**********************************************************************
 * When find the slot get rid of the excess obj
 * ********************************************************************/
FusionObj_T get_rid_of_excess_obj(SlotInfo_T *U_Slot_Array, int U_Slot_Num,
                                  FusionObj_T *Side_FS_Obj, TRIM_OBJ_DIR SlotDirection)
{
    FusionObj_T Side_FS_Obj_trimmed;
    memcpy(&Side_FS_Obj_trimmed, Side_FS_Obj, sizeof(FusionObj_T));
    Side_FS_Obj_trimmed.num      = Side_FS_Obj->num;
    Side_FS_Obj_trimmed.num_accu = Side_FS_Obj->num_accu;
    int start_index = Side_FS_Obj->num > SF_OBJ_NUM ? 0 : SF_OBJ_NUM - Side_FS_Obj->num;

    for (int i = 0; i < U_Slot_Num; i++)
    {

        if (SlotDirection == TRIM_LEFT)
        {
            if (U_Slot_Array[i].slotshap == PK_SLOT_RIGHT_VERT ||
                U_Slot_Array[i].slotshap == PK_SLOT_RIGHT_PARA)
            {
                continue;
            }
        }
        if (SlotDirection == TRIM_RIGHT)
        {
            if (U_Slot_Array[i].slotshap == PK_SLOT_LEFT_VERT ||
                U_Slot_Array[i].slotshap == PK_SLOT_LEFT_PARA)
            {
                continue;
            }
        }
        double B_x, B_y, C_x, C_y, D_x, D_y, E_x, E_y;
        if (U_Slot_Array[i].is_vision_slot)
        {
            B_x           = U_Slot_Array[i].avm_point.near_rear.x;
            B_y           = U_Slot_Array[i].avm_point.near_rear.y;
            C_x           = U_Slot_Array[i].avm_point.far_rear.x;
            C_y           = U_Slot_Array[i].avm_point.far_rear.y;
            D_x           = U_Slot_Array[i].avm_point.far_front.x;
            D_y           = U_Slot_Array[i].avm_point.far_front.y;
            E_x           = U_Slot_Array[i].avm_point.near_front.x;
            E_y           = U_Slot_Array[i].avm_point.near_front.y;
            float dist_BE = Cal_Dis_Pt2Pt(U_Slot_Array[i].avm_point.near_rear,
                                          U_Slot_Array[i].avm_point.near_front);
            if (expand_slot_width > dist_BE)
            {
                B_x = B_x - (E_x - B_x) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                B_y = B_y - (E_y - B_y) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                C_x = C_x - (E_x - B_x) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                C_y = C_y - (E_y - B_y) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                D_x = D_x + (E_x - B_x) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                D_y = D_y + (E_y - B_y) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                E_x = E_x + (E_x - B_x) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
                E_y = E_y + (E_y - B_y) / dist_BE * (expand_slot_width - dist_BE) / 2.0;
            }
        }
        else
        {
            B_x = U_Slot_Array[i].slotobj.ptB.x;
            B_y = U_Slot_Array[i].slotobj.ptB.y;
            C_x = U_Slot_Array[i].slotobj.ptC.x;
            C_y = U_Slot_Array[i].slotobj.ptC.y;
            D_x = U_Slot_Array[i].slotobj.ptD.x;
            D_y = U_Slot_Array[i].slotobj.ptD.y;
            E_x = U_Slot_Array[i].slotobj.ptE.x;
            E_y = U_Slot_Array[i].slotobj.ptE.y;
        }

        for (int j = start_index; j < SF_OBJ_NUM; j++)
        {
            double pt1_x = Side_FS_Obj->obj[j].pt1.x;
            double pt1_y = Side_FS_Obj->obj[j].pt1.y;
            double pt2_x = Side_FS_Obj->obj[j].pt2.x;
            double pt2_y = Side_FS_Obj->obj[j].pt2.y;
            double crossBE =
                (E_x - B_x) * (pt2_x - pt1_x) + (E_y - B_y) * (pt2_y - pt1_y);

            if (crossBE < 0)
            {
                // 在R档检测障碍物反向计算
                pt1_x = Side_FS_Obj->obj[j].pt2.x;
                pt1_y = Side_FS_Obj->obj[j].pt2.y;
                pt2_x = Side_FS_Obj->obj[j].pt1.x;
                pt2_y = Side_FS_Obj->obj[j].pt1.y;
            }

            // 过(x1,y1)和(x2,y2)两点直线的一般式为(y2-y1)x+(x1-x2)y+x2y1-x1y2=0
            // 即A = y2 -y1, B = x1 - x2, C = x2y1 - x1y2,
            double A_far  = (B_y - C_y);
            double B_far  = (C_x - B_x);
            double C_far  = B_x * C_y - C_x * B_y;
            double A_near = (E_y - D_y);
            double B_near = (D_x - E_x);
            double C_near = E_x * D_y - D_x * E_y;

            double pt1_line_far  = A_far * pt1_x + B_far * pt1_y + C_far;
            double pt1_line_near = A_near * pt1_x + B_near * pt1_y + C_near;
            double pt2_line_far  = A_far * pt2_x + B_far * pt2_y + C_far;
            double pt2_line_near = A_near * pt2_x + B_near * pt2_y + C_near;
            int pt1_is_in        = 0;
            int pt2_is_in        = 0;
            if (pt1_line_far * pt1_line_near < 0)
            {
                pt1_is_in = 1;
            }
            if (pt2_line_far * pt2_line_near < 0)
            {
                pt2_is_in = 1;
            }

            if (pt1_is_in == 1 && pt2_is_in == 1)
            {
                // 整个线段处于车位内
                //  TODO:设置为无穷远处

                LOGD << "pt1_is_in==1 && pt2_is_in==1";
                LOGD << "pt1.x: " << pt1_x << " pt1.y: " << pt1_y << " pt2.x: " << pt2_x
                     << " pt2.y: " << pt2_y;
                LOGD << "B.x: " << B_x << " B.y: " << B_y << " C.x: " << C_x
                     << " C.y: " << C_y;
                LOGD << "E.x: " << E_x << " E.y: " << E_y << " D.x: " << D_x
                     << " D.y: " << D_y;
                double distance_pt1_line_far =
                    fabs(pt1_line_far) / sqrt(pow2(A_far) + pow2(B_far));
                double distance_pt1_line_near =
                    fabs(pt1_line_near) / sqrt(pow2(A_near) + pow2(B_near));
                double distance_pt2_line_far =
                    fabs(pt2_line_far) / sqrt(pow2(A_far) + pow2(B_far));
                double distance_pt2_line_near =
                    fabs(pt2_line_near) / sqrt(pow2(A_near) + pow2(B_near));
                double projectOnBE_pt1_pt2 =
                    fabs(crossBE) / sqrt(pow2(E_x - B_x) + pow2(E_y - B_y));
                // double distance_BE = sqrt(pow2(B_x-E_x)+pow2(B_y-E_y));
                // HOBOT_CHECK(distance_pt1_line_far< distance_BE) <<
                // "distance_pt1_line_far:"
                //              <<distance_pt1_line_far << " distance_BE:" << distance_BE;
                // HOBOT_CHECK(distance_pt1_line_near< distance_BE) <<
                // "distance_pt1_line_near:"
                //              <<distance_pt1_line_near << " distance_BE:" <<
                //              distance_BE;
                // HOBOT_CHECK(distance_pt2_line_far< distance_BE) <<
                // "distance_pt2_line_far:"
                //              <<distance_pt2_line_far << " distance_BE:" << distance_BE;
                // HOBOT_CHECK(distance_pt2_line_near< distance_BE) <<
                // "distance_pt2_line_near:"
                //              <<distance_pt2_line_near << " distance_BE:" <<
                //              distance_BE;

                if (distance_pt1_line_far > 1.0 && distance_pt1_line_near > 1.0)
                {
                    LOGD << "THERE IS OBJ IN SLOT! CANNOT REMOVE!";
                    continue;
                }
                if (distance_pt2_line_far > 1.0 && distance_pt2_line_near > 1.0)
                {
                    LOGD << "THERE IS OBJ IN SLOT! CANNOT REMOVE!";
                    continue;
                }
                if (projectOnBE_pt1_pt2 > 0.8)
                {
                    LOGD << "THERE IS OBJ IN SLOT! CANNOT REMOVE!";
                    continue;
                }
                // Side_FS_Obj_trimmed.attr[j] =
                // Side_FS_Obj->attr[j]/10000*10000+4000+Side_FS_Obj->attr[j]%1000;//千位改为4(for
                // 5510) 其余保持不变
                // memcpy(&Side_FS_Obj_trimmed.obj[j],&inf_far,sizeof(LineSeg_T));
            }
            else if (pt1_is_in == 1 && pt2_is_in == 0)
            {
                // 线段前半部分位于车位内
                double distance_pt1_line_near =
                    fabs(pt1_line_near) / sqrt(pow(A_near, 2.0) + pow(B_near, 2));
                if (distance_pt1_line_near > 1.0)
                {
                    LOGD << "THERE IS OBJ IN SLOT! CANNOT REMOVE!";
                    continue;
                }
                LOGD << "pt1_is_in==1 && pt2_is_in==0";
                LOGD << "pt1.x: " << pt1_x << " pt1.y: " << pt1_y << " pt2.x: " << pt2_x
                     << " pt2.y: " << pt2_y;
                LOGD << "B.x: " << B_x << " B.y: " << B_y << " C.x: " << C_x
                     << " C.y: " << C_y;
                LOGD << "E.x: " << E_x << " E.y: " << E_y << " D.x: " << D_x
                     << " D.y: " << D_y;
                // TODO:从交点处裁断，只需修改obj的pt1坐标即可
                double intersection_x, intersection_y;
                int ret =
                    cal_lines_intersection(pt1_x, pt1_y, pt2_x, pt2_y, E_x, E_y, D_x, D_y,
                                           &intersection_x, &intersection_y);
                if (ret)
                {
                    Side_FS_Obj_trimmed.obj[j].pt1.x = intersection_x;
                    Side_FS_Obj_trimmed.obj[j].pt1.y = intersection_y;
                }

                LOGD << "intersection_x:" << intersection_x
                     << " intersection_y: " << intersection_y;
            }
            else if (pt1_is_in == 0 && pt2_is_in == 1)
            { // 线段后半部分位于车位内
                float distance_pt2_line_far =
                    fabs(pt2_line_far) / sqrt(pow2(A_far) + pow2(B_far));
                if (distance_pt2_line_far > 1.0)
                {
                    LOGD << "THERE IS OBJ IN SLOT! CANNOT REMOVE!";
                    continue;
                }
                LOGD << "pt1_is_in==0 && pt2_is_in==1";
                LOGD << "pt1.x: " << pt1_x << " pt1.y: " << pt1_y << " pt2.x: " << pt2_x
                     << " pt2.y: " << pt2_y;
                LOGD << "B.x: " << B_x << " B.y: " << B_y << " C.x: " << C_x
                     << " C.y: " << C_y;
                LOGD << "E.x: " << E_x << " E.y: " << E_y << " D.x: " << D_x
                     << " D.y: " << D_y;
                // TODO:从交点处裁断，只需修改obj的pt2坐标即可
                double intersection_x, intersection_y;
                int ret =
                    cal_lines_intersection(pt1_x, pt1_y, pt2_x, pt2_y, B_x, B_y, C_x, C_y,
                                           &intersection_x, &intersection_y);
                if (ret)
                {
                    Side_FS_Obj_trimmed.obj[j].pt2.x = intersection_x;
                    Side_FS_Obj_trimmed.obj[j].pt2.y = intersection_y;
                }
                LOGD << "intersection_x:" << intersection_x
                     << " intersection_y: " << intersection_y;
            }
        }
    }
    // for(int i=0; i<U_Slot_Num; i++){
    //  if(U_Slot_Array[i].is_vision_slot==1 && U_Slot_Array[i].has_stopper!=0 ){
    //      if(U_Slot_Array[i].slotshap == PK_SLOT_LEFT_PARA || U_Slot_Array[i].slotshap
    //      == PK_SLOT_RIGHT_PARA){
    //          if(start_index-i<0) continue;
    //          if(U_Slot_Array[i].has_stopper==1){
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt1.x =
    //              U_Slot_Array[i].avm_point.near_front.x;
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt1.y =
    //              U_Slot_Array[i].avm_point.near_front.y;
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt2.x =
    //              U_Slot_Array[i].avm_point.far_front.x;
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt2.y =
    //              U_Slot_Array[i].avm_point.far_front.y; Side_FS_Obj_trimmed.num++;
    //              Side_FS_Obj_trimmed.num_accu++;
    //          }else if(U_Slot_Array[i].has_stopper==2){
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt1.x =
    //              U_Slot_Array[i].avm_point.near_rear.x;
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt1.y =
    //              U_Slot_Array[i].avm_point.near_rear.y;
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt2.x =
    //              U_Slot_Array[i].avm_point.far_rear.x;
    //              Side_FS_Obj_trimmed.obj[start_index-i].pt2.y =
    //              U_Slot_Array[i].avm_point.far_rear.y; Side_FS_Obj_trimmed.num++;
    //              Side_FS_Obj_trimmed.num_accu++;
    //          }else{
    //              continue;
    //          }

    //      }
    //      if(U_Slot_Array[i].slotshap == PK_SLOT_LEFT_PARA){
    //          Side_FS_Obj_trimmed.attr[start_index-i]=110000+(Side_FS_Obj_trimmed.num)%SF_OBJ_NUM;
    //      }else if(U_Slot_Array[i].slotshap == PK_SLOT_RIGHT_PARA){
    //          Side_FS_Obj_trimmed.attr[start_index-i]=210000+(Side_FS_Obj_trimmed.num)%SF_OBJ_NUM;
    //      }
    //  }

    // }
    return Side_FS_Obj_trimmed;
}

int cal_lines_intersection(double line1_pt1_x, double line1_pt1_y, double line1_pt2_x,
                           double line1_pt2_y, double line2_pt1_x, double line2_pt1_y,
                           double line2_pt2_x, double line2_pt2_y, double *x, double *y)
{

    double A_line1 = (line1_pt1_y - line1_pt2_y);
    double B_line1 = (line1_pt2_x - line1_pt1_x);
    double C_line1 = line1_pt1_x * line1_pt2_y - line1_pt2_x * line1_pt1_y;

    double A_line2 = (line2_pt1_y - line2_pt2_y);
    double B_line2 = (line2_pt2_x - line2_pt1_x);
    double C_line2 = line2_pt1_x * line2_pt2_y - line2_pt2_x * line2_pt1_y;

    double m = A_line1 * B_line2 - A_line2 * B_line1;
    if (fabs(m) < 0.0001)
    {
        return 0;
    }
    else
    {
        *x = (C_line2 * B_line1 - C_line1 * B_line2) / m;
        *y = (C_line1 * A_line2 - C_line2 * A_line1) / m;
        return 1;
    }
}

int check_slot_exist(SlotObj_T *slotobj, SlotInfo_T *U_Slot_Array, int U_Slot_Num,
                     PK_SlotShapeType slotshap)
{
    float center_x =
        (slotobj->ptB.x + slotobj->ptC.x + slotobj->ptD.x + slotobj->ptE.x) / 4.0;
    float center_y =
        (slotobj->ptB.y + slotobj->ptC.y + slotobj->ptD.y + slotobj->ptE.y) / 4.0;
    for (int i = 0; i < U_Slot_Num; i++)
    {
        SlotObj_T cur_slotobj = U_Slot_Array[i].slotobj;
        float cur_center_x = (cur_slotobj.ptB.x + cur_slotobj.ptC.x + cur_slotobj.ptD.x +
                              cur_slotobj.ptE.x) /
                             4.0;
        float cur_center_y = (cur_slotobj.ptB.y + cur_slotobj.ptC.y + cur_slotobj.ptD.y +
                              cur_slotobj.ptE.y) /
                             4.0;
        float distance =
            sqrt(pow2(cur_center_x - center_x) + pow2(cur_center_y - center_y));
        if (slotshap == PK_SLOT_LEFT_VERT || slotshap == PK_SLOT_RIGHT_VERT)
        {
            if (distance <= MIN_PARK_LEN_VERTICAL * 0.8)
            {
                return 1;
            }
        }

        if (slotshap == PK_SLOT_LEFT_PARA || slotshap == PK_SLOT_RIGHT_PARA)
        {
            if (distance <= MIN_PARK_LEN_PARALLEL * 0.8)
            {
                return 1;
            }
        }
    }
    return 0;
}

//	20180206: check whether line segments interfere with vehicle
// 0- no cross; 1-cross; -1 input error;
int IsSegmentInsertTargPos(const LineSeg_T &segment, const VehPos_T &targpos,
                           const PK_SlotShapeType SlotShap)
{
    Point_T cur_pt;

    Relation_T left_dir, right_dir, bottom_dir;
    Relation_T left_cross_dir, right_cross_dir, bottom_cross_dir;
    int pt_out_left = 0, pt_out_right = 0; //  if the segment point are out of vechicle

    Rect_T vehCornerPt; //   vehicle's 4 corner point
    Get_Veh_CornerPt(targpos, &vehCornerPt, 0);

    for (int i = 0; i < 2; i++)
    {
        if (i == 0)
        {
            memcpy(&cur_pt, &segment.pt1, sizeof(cur_pt));
        }
        else
        {
            memcpy(&cur_pt, &segment.pt2, sizeof(cur_pt));
        }

        if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
        {
            left_cross_dir  = PK_ON_LEFT;
            right_cross_dir = PK_ON_RIGHT;
            Get_Dist_Dir_Pt2PointLine(vehCornerPt.pt_rl, vehCornerPt.pt_rr, cur_pt, NULL,
                                      &left_dir); // CornerPt.pt_rl,CornerPt.pt_rr,cur_pt
            Get_Dist_Dir_Pt2PointLine(vehCornerPt.pt_fl, vehCornerPt.pt_fr, cur_pt, NULL,
                                      &right_dir); // CornerPt.pt_fl,CornerPt.pt_fr,cur_pt
            if (SlotShap == PK_SLOT_LEFT_PARA)
            {
                Get_Dist_Dir_Pt2PointLine(
                    vehCornerPt.pt_rl, vehCornerPt.pt_fl, cur_pt, NULL,
                    &bottom_dir); // CornerPt.pt_rl,CornerPt.pt_fl,cur_pt
                bottom_cross_dir = PK_ON_RIGHT;
            }
            else
            {
                Get_Dist_Dir_Pt2PointLine(
                    vehCornerPt.pt_rr, vehCornerPt.pt_fr, cur_pt, NULL,
                    &bottom_dir); // CornerPt.pt_rr,CornerPt.pt_fr,cur_pt
                bottom_cross_dir = PK_ON_LEFT;
            }
        }
        else if (SlotShap == PK_SLOT_LEFT_VERT)
        {
            left_cross_dir   = PK_ON_LEFT;
            right_cross_dir  = PK_ON_RIGHT;
            bottom_cross_dir = PK_ON_RIGHT;
            Get_Dist_Dir_Pt2PointLine(vehCornerPt.pt_rr, vehCornerPt.pt_fr, cur_pt, NULL,
                                      &left_dir); // CornerPt.pt_rr,CornerPt.pt_fr,cur_pt
            Get_Dist_Dir_Pt2PointLine(vehCornerPt.pt_rl, vehCornerPt.pt_fl, cur_pt, NULL,
                                      &right_dir); // CornerPt.pt_rl,CornerPt.pt_fl,cur_pt
            Get_Dist_Dir_Pt2PointLine(
                vehCornerPt.pt_rr, vehCornerPt.pt_rl, cur_pt, NULL,
                &bottom_dir); // CornerPt.pt_rr,CornerPt.pt_rl,cur_pt
        }
        else if (SlotShap == PK_SLOT_RIGHT_VERT)
        {
            left_cross_dir   = PK_ON_RIGHT;
            right_cross_dir  = PK_ON_LEFT;
            bottom_cross_dir = PK_ON_RIGHT;
            Get_Dist_Dir_Pt2PointLine(vehCornerPt.pt_rl, vehCornerPt.pt_fl, cur_pt, NULL,
                                      &left_dir); // CornerPt.pt_rl,CornerPt.pt_fl,cur_pt
            Get_Dist_Dir_Pt2PointLine(vehCornerPt.pt_rr, vehCornerPt.pt_fr, cur_pt, NULL,
                                      &right_dir); // CornerPt.pt_rr,CornerPt.pt_fr,cur_pt
            Get_Dist_Dir_Pt2PointLine(
                vehCornerPt.pt_rr, vehCornerPt.pt_rl, cur_pt, NULL,
                &bottom_dir); // CornerPt.pt_rr,CornerPt.pt_rl,cur_pt
        }
        else //  input error
        {
            return -1;
        }
        //  judge cross
        if (bottom_dir == bottom_cross_dir)
        {
            if (left_dir == left_cross_dir &&
                right_dir ==
                    right_cross_dir) //  at least one point of segment insert vehicle
            {
                return 1;
            }
            else if (left_dir != left_cross_dir && right_dir == right_cross_dir)
            {
                pt_out_left = 1;
            }
            else if (left_dir == left_cross_dir && right_dir != right_cross_dir)
            {
                pt_out_right = 1;
            }
        }
    }
    return (pt_out_left && pt_out_right) ? 1 : 0;
}

// 20170516: check whether the target pose is within BCDE
//  20171008: enhance the constraints that check whether corners of the vehicle interfere
//  with slot
// return: 0-outside?ê?1-inside
Relation_T IsTargPosInSlot(Point_T Slot_B, Point_T Slot_C, Point_T Slot_D, Point_T Slot_E,
                           VehPos_T TargPos)
{
    Quad_T quad;
    Point_T targ_pt;
    memcpy(&quad.pt_fl, &Slot_B, sizeof(Point_T));
    memcpy(&quad.pt_fr, &Slot_C, sizeof(Point_T));
    memcpy(&quad.pt_rr, &Slot_D, sizeof(Point_T));
    memcpy(&quad.pt_rl, &Slot_E, sizeof(Point_T));
    memcpy(&targ_pt, &TargPos, sizeof(Point_T));
    // check whether the target pose is within BCDE
    return (Is_Point_In_Quad(quad, targ_pt));
}

//  20171008￡ocheck whether vehicle interfere with slot
// return￡o0-no interference￡?1-interference
int IsTargPosVehInSertSlot(VehPos_T TargPos, SlotObj_T SlotObj)
{
    Rect_T CornerPt; //  corner points of vehicle
    int i;
    Point_T VehLineA, VehLineB;
    Get_Veh_CornerPt(TargPos, &CornerPt,
                     0.0f); //  the free space around targpos can't be too small

    for (i = 0; i < 4; i++)
    {
        if (i == 0)
        {
            memcpy(&VehLineA, &CornerPt.pt_fl, sizeof(VehLineA));
            memcpy(&VehLineB, &CornerPt.pt_fr, sizeof(VehLineB));
        }
        else if (i == 1)
        {
            memcpy(&VehLineA, &CornerPt.pt_fr, sizeof(VehLineA));
            memcpy(&VehLineB, &CornerPt.pt_rr, sizeof(VehLineB));
        }
        else if (i == 2)
        {
            memcpy(&VehLineA, &CornerPt.pt_rr, sizeof(VehLineA));
            memcpy(&VehLineB, &CornerPt.pt_rl, sizeof(VehLineB));
        }
        else
        {
            memcpy(&VehLineA, &CornerPt.pt_fl, sizeof(VehLineA));
            memcpy(&VehLineB, &CornerPt.pt_rl, sizeof(VehLineB));
        }
        if (Is_TwoSegment_Cross(VehLineA, VehLineB, SlotObj.ptB, SlotObj.ptC) ||
            Is_TwoSegment_Cross(VehLineA, VehLineB, SlotObj.ptD, SlotObj.ptE))
        {
            return 1;
        }
    }
    return 0;
}

//  require the insert segment length bigger than certain dist
float Get_Segment_Insert_TargPos_Len(LineSeg_T segment, VehPos_T targpos,
                                     PK_SlotShapeType SlotShap)
{
    Rect_T CornerPt;
    float insert_dist = 0;
    LineSeg_T Seg_Cross;
    float slot_theta, segm_theta, cross_angle;
    Get_Veh_CornerPt(
        targpos, &CornerPt,
        -0.2f); //  care more about the middle part to avoid disturbed situations

    if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
    {
        slot_theta = targpos.theta;
    }
    else if (SlotShap == PK_SLOT_LEFT_VERT)
    {
        slot_theta = targpos.theta + PI / 2.0f;
    }
    else if (SlotShap == PK_SLOT_RIGHT_VERT)
    {
        slot_theta = targpos.theta - PI / 2.0f;
    }
    else // input error
    {
        return 0;
    }
    if (RectClipSegment(CornerPt, segment, &Seg_Cross))
    {
        insert_dist = Get_Segment_Len(Seg_Cross);
        segm_theta =
            atan2f(Seg_Cross.pt2.y - Seg_Cross.pt1.y, Seg_Cross.pt2.x - Seg_Cross.pt1.x);
        cross_angle = fabsf(segm_theta - slot_theta);
        insert_dist *= fabsf(cosf(cross_angle));
    }
    return insert_dist;
}

//  use the rear side radar to check the detected slot
// 1- pass; 0- fail;
int Rear_Side_Radar_Check_Slot(VehPos_T targPos, PK_SlotShapeType slotShap)
{
    int LineDir = 0;
    int i, StartIndex;
    LineSeg_T segment;
    // const float valid_segment_len = 0.3f;
    float insert_len_Sum = 0;
    float insert_len_min = 0;
    //  get RS radar sensorfusion data
    if (slotShap == PK_SLOT_LEFT_PARA || slotShap == PK_SLOT_LEFT_VERT)
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Left(&g_FS_RearObsInfo_A);
    }
    else
    {
        RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Rear_Right(&g_FS_RearObsInfo_A);
    }
    if (slotShap == PK_SLOT_LEFT_PARA || slotShap == PK_SLOT_RIGHT_PARA)
    {
        insert_len_min = 1.5f;
    }
    else
    {
        insert_len_min = 1.0f;
    }

    //  find the cross segment to tarpos
    StartIndex = SF_OBJ_NUM - g_FS_RearObsInfo_A.num;
    for (i = StartIndex; i < SF_OBJ_NUM; i++)
    {
        if (g_FS_RearObsInfo_A.attr[i] == 0) //  invalid line segments
        {
            continue;
        }
        LineDir = g_FS_RearObsInfo_A.attr[i] / 10000;
        if (((slotShap == PK_SLOT_LEFT_PARA || slotShap == PK_SLOT_LEFT_VERT) &&
             LineDir == 2) ||
            ((slotShap == PK_SLOT_RIGHT_PARA || slotShap == PK_SLOT_RIGHT_VERT) &&
             LineDir == 1)) //  ignore segments not in the slot's direction
        {
            continue;
        }
        memcpy(&segment, &g_FS_RearObsInfo_A.obj[i], sizeof(segment));
        insert_len_Sum += Get_Segment_Insert_TargPos_Len(segment, targPos, slotShap);
    }

    if (insert_len_Sum > insert_len_min)
    {
        return 1;
    }
    return 0;
}

// calculate middle point
Point_T Get_Middle_Point(Point_T spt, Point_T ept)
{
    Point_T pt;
    pt.x = 0.5f * (spt.x + ept.x);
    pt.y = 0.5f * (spt.y + ept.y);
    return pt;
}

/**********************************************************************************
Description     :Get Vector's angle. From pt1 toward pt2
Owner           :HYB
Modefied Date   :2018.11.27
Parameter[In]   :Line Segment's two point: pt1, pt2
Return          :The angle of input vector
***********************************************************************************/

float Get_Segment_Angle(Point_T pt1, Point_T pt2)
{
    float dx, dy, angle;
    dx    = pt2.x - pt1.x;
    dy    = pt2.y - pt1.y;
    angle = atan2f(dy, dx);
    return angle;
}

Vec2_T Norm_Linesegment(Point_T spt, Point_T ept)
{
    float len;
    Vec2_T vec;

    len = Cal_Dis_Pt2Pt(spt, ept);
    if (len > 1e-2f)
    {
        vec.vx = (ept.x - spt.x) / len;
        vec.vy = (ept.y - spt.y) / len;
    }
    else
    {
        vec.vx = 0;
        vec.vy = 0;
    }
    return vec;
}

float Get_Average_angle_from_4AVMpts(Point_T NF, Point_T FF, Point_T NR, Point_T FR)
{
    Vec2_T Vec_F, Vec_R;
    Vec2_T TargPos_Vec;

    Vec_F          = Norm_Linesegment(FF, NF);
    Vec_R          = Norm_Linesegment(FR, NR);
    TargPos_Vec.vx = Vec_F.vx + Vec_R.vx;
    TargPos_Vec.vy = Vec_F.vy + Vec_R.vy;

    return atan2f(TargPos_Vec.vy, TargPos_Vec.vx);
}

// Fit a line using min least square method. Notice that ptSet will be modified during
// calculation.
int LineFittingLeastSquare(Point_T *ptSet, int ptNum, LineSeg_T *line)
{
    if (line == NULL)
    {
        return 0;
    }

    if (ptNum < 2)
    {
        return 0;
    }

    int i;
    float yAver = 0.0f, xAver = 0.0f, xSquaSum = 0.0f, xySum = 0.0f, a = 0.0f, b = 0.0f,
          tmp = 0.0f, xOffset = 0.0f, yOffset = 0.0f;

    Point_T pt1, pt2;

    // Initialize offset for transformation
    xOffset = ptSet[0].x;
    yOffset = ptSet[0].y;

    // Transform all points to avoid overflow during calculation
    for (i = 0; i < ptNum; ++i)
    {
        ptSet[i].x = ptSet[i].x - xOffset;
        ptSet[i].y = ptSet[i].y - yOffset;
    }

    for (i = 0; i < ptNum; ++i)
    {
        xAver += ptSet[i].x;
        yAver += ptSet[i].y;

        xSquaSum += (ptSet[i].x * ptSet[i].x);
        xySum += (ptSet[i].x * ptSet[i].y);
    }

    xAver /= ptNum;
    yAver /= ptNum;

    // cal parameters
    tmp = xSquaSum - ptNum * xAver * xAver;
    if (tmp < 1e-8f)
    {
        return 0;
    }

    b = (yAver * xSquaSum - xAver * xySum) / tmp;
    a = (xySum - ptNum * xAver * yAver) / tmp;

    pt1.x = ptSet[0].x;
    pt1.y = pt1.x * a + b;

    pt2.x = ptSet[ptNum - 1].x;
    pt2.y = pt2.x * a + b;

    // transform the points to origin coordinate
    pt1.x += xOffset;
    pt1.y += yOffset;
    pt2.x += xOffset;
    pt2.y += yOffset;

    memcpy(&(line->pt1), &pt1, sizeof(Point_T));
    memcpy(&(line->pt2), &pt2, sizeof(Point_T));

    return 1;
}