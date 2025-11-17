#include "PathPlan_Func.h"
#include "SystemPara.h"
#include "PK_PathPlanP.h"
#include "PathPlan_Tools.h"
#include "PathPlan_Obj.h"
#include "Record_Log.h"

enum
{
    AB,
    BC,
    CD,
    DE,
    EF,
    EGMAX
};

//  modify by zzx 1114
//  declarations of inner functions
int circle_colli(obj_para EXtr_obj[1], path_para EXtr_path[1]);
int circle_inner(obj_para EXtr_obj[1], path_para EXtr_path[1]);
int circle_collision(obj_para EXtr_obj[1], path_para EXtr_path[1]);
int rec_collision(obj_para EXtr_obj[1], float x_out[4], float y_out[4]);
int Exclude_obj(obj_para EXtr_obj[1], path_para EXtr_path[1]);
int Path_ParaCal(const float path[5], path_para EXtr_path[1], const float swell);

// int Path_ParaCal1(const float path[5], path_para EXtr_path[1], float wid, float length,
// float hr, const float swell);//add 20230921

int obj_ParaCal(float obj[4], obj_para EXtr_obj[1]);
int Point_CircleLine(obj_para EXtr_obj[1], path_para EXtr_path[1], int status[4],
                     float xs[][2], float ys[][2]);
void Get_Limit(float point[3], float limit[4], float limit_dist, float limit_len);

float PointToLine(const float line[4], const float point[2])
{
    float a = line[3] - line[1];
    float b = line[0] - line[2];
    float c = -b * line[1] - a * line[0];

    return fabsf(a * point[0] + b * point[1] + c) / sqrtf(a * a + b * b);
}

/***********************************************************
Function Description : PathPlan_PathVerify-->>Determine if the path interferes with
obstacles Edition : 2017.05.18 Input list : Output list : return cross://0-not
interfere;1-interfere;
***********************************************************/
int PK_PathPlan_PathVerify(const float path[TRAJITEM_LEN], const int obj_num,
                           const float obj[][4], const float swell)
{
    float x_out[4], y_out[4];
    int i, j, cross_obs = 0, cross_temp = 0;
    path_para EXtr_path[1];
    obj_para EXtr_obj[1];
    float tempobj[96][4] = {{0}};
    float temppath[5]    = {0};

    if (obj_num <= 0)
    {
        return 0;
    }

    for (i = 0; i < obj_num; i++) // Avoid calculation errors if the value is too large
    {
        tempobj[i][0] = obj[i][0] - path[0];
        tempobj[i][1] = obj[i][1] - path[1];
        tempobj[i][2] = obj[i][2] - path[0];
        tempobj[i][3] = obj[i][3] - path[1];
    }

    temppath[0] = 0;
    temppath[1] = 0;
    temppath[2] = path[2];
    temppath[3] = path[3];
    temppath[4] = path[4];

    Path_ParaCal(temppath, EXtr_path, swell); // Extract path
    for (i = 0; i < 4; i++)
    {
        x_out[i] = EXtr_path[0].x_out[i];
        y_out[i] = EXtr_path[0].y_out[i];
    }

    for (j = 0; j < obj_num; j++)
    {
        obj_ParaCal(tempobj[j], EXtr_obj); // Extract obj
        if (fabsf(EXtr_path[0].r) >= 1)    // circle
        {
            cross_temp = circle_collision(EXtr_obj, EXtr_path);
            if (cross_temp == 1)
            {
                cross_obs = j + 1;
                break;
            }
            else
            {
                cross_obs = 0;
            }
        }
        else // straight line
        {
            cross_temp = rec_collision(EXtr_obj, x_out, y_out);
            if (cross_temp == 1)
            {
                cross_obs = j + 1;
                break;
            }
            else
            {
                cross_obs = 0;
            }
        }
    }
    return cross_obs;
}

int PathPlan_PathVerify2(const float path[TRAJITEM_LEN], const int obj_num,
                         const float obj[][4], const float swell, int type)
{
    return PK_PathPlan_PathVerify(path, obj_num, obj, swell);
}

/*
HAVECHANGE 202390928
完善直线路径校验
*/
float PathPlan_LandMark(float path[TRAJITEM_LEN], int obj_num, const float obj[][4],
                        float swell)
{
    float x0, y0, theta0, ds, r, ftheta, x_start, y_start, theta_start, ds_final,
        accuracy = 0.03f;
    float ftheta0, theta_end, x_mid, y_mid, theta_mid, ds_mid = 0, ds_mid1 = 1,
                                                       path_mid[5], ds_temp, ds_start = 0;
    float x_out[4], y_out[4];
    int i, j, cross_obs = 0, cross_temp = 0;
    path_para EXtr_path[1], EXtr_pathmid[1];
    ObjPara EXtr_obj[96]; // 存储障碍物校验数据和校验结果
    float tempobj[96][4] = {{0}};
    float temppath[5]    = {0};

    for (i = 0; i < obj_num; i++) // Avoid calculation errors if the value is too large
    {
        tempobj[i][0] = obj[i][0] - path[0];
        tempobj[i][1] = obj[i][1] - path[1];
        tempobj[i][2] = obj[i][2] - path[0];
        tempobj[i][3] = obj[i][3] - path[1];
    }

    temppath[0] = 0;
    temppath[1] = 0;
    temppath[2] = path[2];
    temppath[3] = path[3];
    temppath[4] = path[4];

    Path_ParaCal(temppath, EXtr_path, swell);
    x0     = EXtr_path[0].x0;
    y0     = EXtr_path[0].y0;
    theta0 = EXtr_path[0].theta0;

    x_start     = x0;
    y_start     = y0;
    theta_start = theta0;

    ds     = EXtr_path[0].ds;
    r      = EXtr_path[0].r;
    ftheta = EXtr_path[0].ftheta;

    // ds_final = ds;    //  redundant operation to be deleted
    for (i = 0; i < 4; i++)
    {
        x_out[i] = EXtr_path[0].x_out[i];
        y_out[i] = EXtr_path[0].y_out[i];
    }
    for (j = 0; j < obj_num; j++)
    {
        obj_ParaCal(tempobj[j], &EXtr_obj[j].extr_para); // 获取障碍物校验数据
        if (fabsf(r) >= 1)
        {
            cross_temp = circle_collision(&EXtr_obj[j].extr_para, EXtr_path);
            if (cross_temp == 1)
            {
                cross_obs            = 1;
                EXtr_obj[j].is_cross = 1;
            }
            else
            {
                EXtr_obj[j].is_cross = 0;
            }
        }
        else
        {
            cross_temp = rec_collision(&EXtr_obj[j].extr_para, x_out, y_out);
            if (cross_temp == 1)
            {
                cross_obs            = 1;
                EXtr_obj[j].is_cross = 1;
            }
            else
            {
                EXtr_obj[j].is_cross = 0;
            }
        }
    }
    if (cross_obs == 0)
    {
        ds_final = ds;
    }
    else // If interfere,using dichotomy
    {
        if (fabsf(r) >= 1)
        {
            x_mid =
                cosf(ds / 2 / r + ftheta) * fabsf(r) + x_start - fabsf(r) * cosf(ftheta);
            y_mid =
                sinf(ds / 2 / r + ftheta) * fabsf(r) + y_start - fabsf(r) * sinf(ftheta);
            theta_mid = theta_start + ds / 2 / r;
            theta_end = theta_start + ds / r;
        }
        else
        {
            x_mid     = cosf(theta_start) * ds / 2 + x_start;
            y_mid     = sinf(theta_start) * ds / 2 + y_start;
            theta_mid = theta_start;
            theta_end = theta_start;
        }
        while (fabsf(ds_mid - ds_mid1) > accuracy) // The min ds 0.05
        {
            ds_mid1 = ds_mid;
            if (fabsf(r) >= 1)
            {
                ds_mid = r * (theta_mid - theta0);
            }
            else
            {
                ds_mid = sign(ds) *
                         sqrtf((x_mid - x0) * (x_mid - x0) + (y_mid - y0) * (y_mid - y0));
            }
            path_mid[0] = x0;
            path_mid[1] = y0;
            path_mid[2] = theta0;
            path_mid[3] = ds_mid;
            path_mid[4] = r;
            Path_ParaCal(path_mid, EXtr_pathmid, swell);
            // cout << EXtr_pathmid[0].x0 << EXtr_pathmid[0].y0 << endl;
            for (j = 0; j < obj_num; j++)
            {
                if (fabsf(r) >= 1)
                {
                    if (EXtr_obj[j].is_cross ==
                        1) // 1.判断障碍物之前的校验结果，结果为不通过则重新校验
                    {
                        cross_temp =
                            circle_collision(&EXtr_obj[j].extr_para, EXtr_pathmid);
                    }
                    else // 通过，返回循环开头
                    {
                        cross_obs = 0;
                        continue;
                    }

                    if (cross_temp == 1) // 2.校验结果仍不通过，跳出循环
                    {
                        cross_obs = 1;
                        break;
                    }
                    else // 通过则更改障碍物校验结果
                    {
                        cross_obs            = 0;
                        EXtr_obj[j].is_cross = 2;
                    }
                }
                else
                {
                    if (EXtr_obj[j].is_cross == 1)
                    {
                        cross_temp =
                            rec_collision(&EXtr_obj[j].extr_para, EXtr_pathmid[0].x_out,
                                          EXtr_pathmid[0].y_out);
                    }
                    else
                    {
                        cross_obs = 0;
                        continue;
                    }

                    if (cross_temp == 1)
                    {
                        cross_obs = 1;
                        break;
                    }
                    else
                    {
                        cross_obs            = 0;
                        EXtr_obj[j].is_cross = 2;
                    }
                }
            }
            if (cross_obs == 0) // If no interference,the midpoint become stpoint
            {
                for (j = 0; j < obj_num; j++)
                {
                    EXtr_obj[j].is_cross =
                        EXtr_obj[j].is_cross == 2 ? 1 : EXtr_obj[j].is_cross;
                }

                x_start     = x_mid;
                y_start     = y_mid;
                theta_start = theta_mid;

                if (fabsf(r) >= 1)
                {
                    ds_temp = r * (theta_end - theta_start);
                    ftheta0 = -sign(r) * PI / 2 + theta_start;
                    x_mid   = cosf(ds_temp / 2 / r + ftheta0) * fabsf(r) + x_start -
                            fabsf(r) * cosf(ftheta0);
                    y_mid = sinf(ds_temp / 2 / r + ftheta0) * fabsf(r) + y_start -
                            fabsf(r) * sinf(ftheta0);
                }
                else
                {
                    ds_temp = ds_mid - ds_start;
                    x_mid   = cosf(theta_start) * ds_temp / 2 + x_start;
                    y_mid   = sinf(theta_start) * ds_temp / 2 + y_start;
                }
                theta_mid = (theta_start + theta_end) / 2;
                ds_start  = ds_mid;
            }
            else // If interference,the midpoint become finpoint
            {
                for (j = 0; j < obj_num; j++)
                {
                    EXtr_obj[j].is_cross =
                        EXtr_obj[j].is_cross == 2 ? 0 : EXtr_obj[j].is_cross;
                }

                theta_end = theta_mid;

                if (fabsf(r) >= 1)
                {
                    ds_temp = r * (theta_end - theta_start);
                    ftheta0 = -sign(r) * PI / 2 + theta_start;
                    x_mid   = cosf(ds_temp / 2 / r + ftheta0) * fabsf(r) + x_start -
                            fabsf(r) * cosf(ftheta0);
                    y_mid = sinf(ds_temp / 2 / r + ftheta0) * fabsf(r) + y_start -
                            fabsf(r) * sinf(ftheta0);
                }
                else
                {
                    ds_temp = ds_mid - ds_start;
                    x_mid   = cosf(theta_start) * ds_temp / 2 + x_start;
                    y_mid   = sinf(theta_start) * ds_temp / 2 + y_start;
                }
                theta_mid = (theta_start + theta_end) / 2;
            }
            // cout << cross_obs << endl;
        }
        // ds_final = r*(theta_start - theta0);
        if (fabsf(r) >= 1)
        {
            ds_final = r * (theta_start - theta0);
        }
        else
        {
            ds_final = ds_start;
        }
    }
    path[3] = sign(path[3]) * fabsf(ds_final); // change the length of input path

    return ds_final;
}

float PathPlan_LandMark2(float path[TRAJITEM_LEN], int obj_num, const float obj[][4],
                         float swell, int type)
{
    float ds_final1 = PathPlan_LandMark(path, obj_num, obj, swell);
    return ds_final1;
}
/***********************************************************
Function Description : rec_collision-->>Determine if the obj interferes with the rectangle
Edition : 2017.05.18
Input list :
            EXtr_obj:the obstacle after extracting
            x_out coordinate of the rectangle
            y_out coordinate of the rectangle
Output list :
            cross:Whether the obstacle interfere with the rectangle//0-no;1-yes
***********************************************************/
int rec_collision(obj_para EXtr_obj[1], float x_out[4], float y_out[4])
{
    float obj[4], theta_obs, dis_obs, xc_obj[4], yc_obj[4], xt1, yt1, xt2, yt2, xmax,
        xmin;
    int cross = 0, status_temp[2];
    float temp1, temp2, temp3, temp4;
    int i;
    obj[0]    = EXtr_obj[0].x1;
    obj[1]    = EXtr_obj[0].y1;
    obj[2]    = EXtr_obj[0].x2;
    obj[3]    = EXtr_obj[0].y2;
    theta_obs = EXtr_obj[0].theta_obs;
    dis_obs   = EXtr_obj[0].dis_obs;

    for (i = 0; i < 4; i++)
    {
        xc_obj[i] =
            (x_out[i] - obj[0]) * cosf(theta_obs) + (y_out[i] - obj[1]) * sinf(theta_obs);
        yc_obj[i] = -(x_out[i] - obj[0]) * sinf(theta_obs) +
                    (y_out[i] - obj[1]) * cosf(theta_obs);
    }
    for (i = 0; i < 4; i++)
    {
        if (i == 0)
        {
            xt1 = xc_obj[0];
            xt2 = xc_obj[3];
            yt1 = yc_obj[0];
            yt2 = yc_obj[3];
        }
        else
        {
            xt1 = xc_obj[i - 1];
            xt2 = xc_obj[i];
            yt1 = yc_obj[i - 1];
            yt2 = yc_obj[i];
        }
        if (yt1 * yt2 <= 0)
        {
            if (yt1 == yt2)
            {
                xmax = rte_max(xt1, xt2);
                xmin = rte_min(xt1, xt2);
            }
            else
            {
                xmax = yt1 * (xt1 - xt2) / (yt2 - yt1) + xt1;
                xmin = xmax;
            }
            if ((xmax <= dis_obs && xmax >= 0) || (xmin <= dis_obs && xmin >= 0))
            {
                cross = 1;
                break;
            }
            else
            {
                cross = 0;
            }
        }
        else
        {
            cross = 0;
        }
    }
    if (cross == 0) // Determine if the obstacle is inside the rectangle
    {
        for (i = 0; i < 2; i++)
        {
            temp1 = (obj[2 * i + 1] - y_out[0]) * (x_out[1] - x_out[0]) -
                    (obj[2 * i] - x_out[0]) * (y_out[1] - y_out[0]);
            temp2 = (obj[2 * i + 1] - y_out[1]) * (x_out[2] - x_out[1]) -
                    (obj[2 * i] - x_out[1]) * (y_out[2] - y_out[1]);
            temp3 = (obj[2 * i + 1] - y_out[2]) * (x_out[3] - x_out[2]) -
                    (obj[2 * i] - x_out[2]) * (y_out[3] - y_out[2]);
            temp4 = (obj[2 * i + 1] - y_out[3]) * (x_out[0] - x_out[3]) -
                    (obj[2 * i] - x_out[3]) * (y_out[0] - y_out[3]);
            if (temp1 >= 0 && temp2 > 0 && temp3 > 0 && temp4 > 0)
            {
                status_temp[i] = 1;
            }
            else
            {
                status_temp[i] = 0;
            }
        }
        if (status_temp[0] == 1 && status_temp[1] == 1)
        {
            cross = 1;
        }
        else
        {
            cross = 0;
        }
    }
    return cross;
}

/***********************************************************
    Function Description : circle_collision-->>Determine if the obj interferes with the
arc Edition : 2017.05.18 Input list : EXtr_obj:the obstacle after extracting EXtr_path:the
path after extracting Output list : return status_final://0-no interference;1-interference
***********************************************************/
int circle_collision(obj_para EXtr_obj[1], path_para EXtr_path[1])
{
    float xv_end[3][4] = {{0.0f}}, yv_end[3][4] = {{0.0f}};
    int status_temp, status_final;
    int i;

    for (i = 0; i < 4; i++)
    {
        xv_end[0][i] = EXtr_path[0].xv_end[0][i];
        yv_end[0][i] = EXtr_path[0].yv_end[0][i];

        xv_end[1][i] = EXtr_path[0].xv_end[1][i];
        yv_end[1][i] = EXtr_path[0].yv_end[1][i];

        xv_end[2][i] = EXtr_path[0].xv_end[2][i];
        yv_end[2][i] = EXtr_path[0].yv_end[2][i];
    }
    status_temp = Exclude_obj(EXtr_obj, EXtr_path);
    if (status_temp == 0) // Obvious interference
    {
        status_final = 0;
    }
    else
    {
        //        status_temp = circle_colli(EXtr_obj,EXtr_path);//Determine if the
        //        obstacle intersects the arc if (status_temp == 1)
        //        {
        //            status_final = 1;
        //        }
        //        else
        //        {
        //            status_temp = circle_inner(EXtr_obj, EXtr_path);//Determine if the
        //            obstacle is inside the arc if (status_temp == 1)
        //            {
        //                status_final = 1;
        //            }
        //            else
        //            {
        //                //Determine if the obstacle interferes with the finpos
        //                status_temp = rec_collision(EXtr_obj, xv_end,yv_end);
        //                if (status_temp == 1)
        //                    status_final = 1;
        //                else
        //                    status_final = 0;
        //            }
        //        }

        status_final = (rec_collision(EXtr_obj, xv_end[0], yv_end[0]) ||
                        rec_collision(EXtr_obj, xv_end[1], yv_end[1]) ||
                        rec_collision(EXtr_obj, xv_end[2], yv_end[2])) ||
                       circle_colli(EXtr_obj, EXtr_path) ||
                       circle_inner(EXtr_obj, EXtr_path);
    }
    return status_final;
}

/***********************************************************
Function Description : Exclude_obj-->>Excluding the obvious interference
Edition : 2017.05.18
Input list :

Output list :
            return status://0-no interference;1-interference
***********************************************************/
int Exclude_obj(obj_para EXtr_obj[1], path_para EXtr_path[1])
{
    int status_temp, status, i;
    float RR[2], temp1, temp2, temp3, temp4;
    float Xo[2], Yo[2], x_out[4], y_out[4];
    float r, rr[2], x_c, y_c; // ds,
    float A, B, C;
    float d; // The distance from the center of the circle to the line where the obstacle
             // is located
    float angle_total;
    Xo[0] = EXtr_obj[0].x1;
    Xo[1] = EXtr_obj[0].x2;
    Yo[0] = EXtr_obj[0].y1;
    Yo[1] = EXtr_obj[0].y2;
    A     = EXtr_obj[0].A;
    B     = EXtr_obj[0].B;
    C     = EXtr_obj[0].C;
    x_c   = EXtr_path[0].x_c;
    y_c   = EXtr_path[0].y_c;
    r     = EXtr_path[0].r;
    for (i = 0; i < 4; i++)
    {
        x_out[i] = EXtr_path[0].x_out[i];
        y_out[i] = EXtr_path[0].y_out[i];
    }
    for (i = 0; i < 2; i++)
    {
        RR[i] = sqrtf((Xo[i] - x_c) * (Xo[i] - x_c) +
                      (Yo[i] - y_c) *
                          (Yo[i] - y_c)); // The distance between the ends of the obstacle
                                          // from the center of the circle
        rr[i] = EXtr_path[0].rr[i];
    }
    if (r > 0)
    {
        temp1 = (x_out[0] - x_c) * (Yo[0] - y_c) - (Xo[0] - x_c) * (y_out[0] - y_c);
        temp2 = (x_out[0] - x_c) * (Yo[1] - y_c) - (Xo[1] - x_c) * (y_out[0] - y_c);
        temp3 = (x_out[1] - x_c) * (Yo[0] - y_c) - (Xo[0] - x_c) * (y_out[1] - y_c);
        temp4 = (x_out[1] - x_c) * (Yo[1] - y_c) - (Xo[1] - x_c) * (y_out[1] - y_c);
    }
    else
    {
        temp1 = (x_out[2] - x_c) * (Yo[0] - y_c) - (Xo[0] - x_c) * (y_out[2] - y_c);
        temp2 = (x_out[2] - x_c) * (Yo[1] - y_c) - (Xo[1] - x_c) * (y_out[2] - y_c);
        temp3 = (x_out[3] - x_c) * (Yo[0] - y_c) - (Xo[0] - x_c) * (y_out[3] - y_c);
        temp4 = (x_out[3] - x_c) * (Yo[1] - y_c) - (Xo[1] - x_c) * (y_out[3] - y_c);
    }
    angle_total = EXtr_path[0].angle_total;
    if (angle_total <= PI)
    {
        if ((temp1 > 0 && temp2 > 0) || (temp3 < 0 && temp4 < 0)) // cross product
        {
            status_temp = 0;
        }
        else
        {
            status_temp = 1;
        }
    }
    else
    {
        if ((temp1 > 0 && temp2 > 0) && (temp3 < 0 && temp4 < 0))
        {
            status_temp = 0;
        }
        else
        {
            status_temp = 1;
        }
    }

    d = fabsf(A * x_c + B * y_c + C) / sqrtf(A * A + B * B); // 圆心到线段距离

    // 圆心到线段端点的距离小于内径, 或者圆心到线段的距离大于外径, 或者
    if (((RR[0] < rr[0]) && (RR[1] < rr[0])) || d > rr[1] || status_temp == 0)
    {
        status = 0;
    }
    else
    {
        status = 1;
    }
    return status;
}

/***********************************************************
Function Description : circle_colli-->> whether intersect
Edition : 2017.05.18
Input list :

Output list :
            return status_circle0://0,1
***********************************************************/
int circle_colli(obj_para EXtr_obj[1], path_para EXtr_path[1])
{
    float temp1, temp2;
    float xs[2][2], ys[2][2];
    int i, k;
    int status_temp, status[2], status1[2][2] = {{0}}, status_circle0;
    float dtheta, x_c, y_c, x_start2[2], y_start2[2], x_end2[2], y_end2[2];
    float x1, y1, x2, y2;

    x1 = EXtr_obj[0].x1;
    x2 = EXtr_obj[0].x2;
    y1 = EXtr_obj[0].y1;
    y2 = EXtr_obj[0].y2;
    for (i = 0; i < 2; i++)
    {
        x_start2[i] = EXtr_path[0].x_start2[i];
        y_start2[i] = EXtr_path[0].y_start2[i];
        x_end2[i]   = EXtr_path[0].x_end2[i];
        y_end2[i]   = EXtr_path[0].y_end2[i];
    }
    dtheta = EXtr_path[0].dtheta;
    x_c    = EXtr_path[0].x_c;
    y_c    = EXtr_path[0].y_c;
    Point_CircleLine(EXtr_obj, EXtr_path, status, xs, ys); // caculate the intersection
    for (k = 0; k < 2; k++)
    {
        if (status[k] == 0)
        {
            for (i = 0; i < 2;
                 i++) // Determine if the intersection is on the arc and the line
            {
                if ((xs[k][i] - x1) * (xs[k][i] - x2) +
                        (ys[k][i] - y1) * (ys[k][i] - y2) >
                    0)
                {
                    status1[k][i] = 0;
                }
                else
                {
                    temp1 = ((x_c - x_start2[k]) * (y_end2[k] - y_start2[k])) -
                            (x_end2[k] - x_start2[k]) * (y_c - y_start2[k]);
                    temp2 = ((xs[k][i] - x_start2[k]) * (y_end2[k] - y_start2[k])) -
                            (x_end2[k] - x_start2[k]) * (ys[k][i] - y_start2[k]);
                    if ((dtheta <= PI && temp1 * temp2 <= 0) ||
                        (dtheta >= PI && temp1 * temp2 >= 0))
                    {
                        status_temp = 1;
                    }
                    else
                    {
                        status_temp = 0;
                    }
                    status1[k][i] = status_temp;
                }
            }
        }

        if (status1[k][0] == 0 && status1[k][1] == 0)
        {
            status[k] = 0; // not intersect
        }
        else
        {
            status[k] = 1;
            break;
        }
    }

    if (((status[0] == 0) || (status[0] == -1)) &&
        ((status[1] == 0) || (status[1] == -1)))
    {
        status_circle0 = 0;
    }
    else
    {
        status_circle0 = 1; // intersect
    }
    return status_circle0;
}

/***********************************************************
Function Description : circle_inner-->>Determine if the obstacle is in the arc area
Edition : 2017.05.18
Input list :

Output list :
            return status_circle://0: in;1-out
***********************************************************/
int circle_inner(obj_para EXtr_obj[1], path_para EXtr_path[1])
{
    float Xo[2], Yo[2], dis_d, dx_e, dy_e, x1_e, y1_e;
    float x_start3, y_start3, x_end3, y_end3, dtheta_e, ds, r, x_c,
        y_c; //, ftheta, dtheta;
    float rr[2], x_start2[2], y_start2[2], x_end2[2], y_end2[2], xv_start[4], yv_start[4],
        xv_end[4], yv_end[4];
    float temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
    int status[2], status_circle;
    int i;
    Xo[0]    = EXtr_obj[0].x1;
    Xo[1]    = EXtr_obj[0].x2;
    Yo[0]    = EXtr_obj[0].y1;
    Yo[1]    = EXtr_obj[0].y2;
    x_c      = EXtr_path[0].x_c;
    y_c      = EXtr_path[0].y_c;
    ds       = EXtr_path[0].ds;
    r        = EXtr_path[0].r;
    dtheta_e = EXtr_path[0].dtheta_e;
    x_start3 = EXtr_path[0].x_start3;
    y_start3 = EXtr_path[0].y_start3;
    x_end3   = EXtr_path[0].x_end3;
    y_end3   = EXtr_path[0].y_end3;
    for (i = 0; i < 2; i++)
    {
        rr[i]       = EXtr_path[0].rr[i];
        x_start2[i] = EXtr_path[0].x_start2[i];
        y_start2[i] = EXtr_path[0].y_start2[i];
        x_end2[i]   = EXtr_path[0].x_end2[i];
        y_end2[i]   = EXtr_path[0].y_end2[i];
    }
    for (i = 0; i < 4; i++)
    {
        xv_start[i] = EXtr_path[0].xv_start[i];
        yv_start[i] = EXtr_path[0].yv_start[i];
        xv_end[i]   = EXtr_path[0].xv_end[0][i];
        yv_end[i]   = EXtr_path[0].yv_end[0][i];
    }
    for (i = 0; i < 2; i++)
    {
        dis_d = sqrtf((Xo[i] - x_c) * (Xo[i] - x_c) + (Yo[i] - y_c) * (Yo[i] - y_c));
        dx_e  = (Xo[i] - x_c) / dis_d;
        dy_e  = (Yo[i] - y_c) / dis_d;
        x1_e  = x_c + rr[1] * dx_e;
        y1_e  = y_c + rr[1] * dy_e;
        if (dis_d <= rr[1] && dis_d >= rr[0])
        {
            if (ds > 0)
            {
                temp1 = (x_c - x_start3) * (y_end2[1] - y_start3) -
                        (x_end2[1] - x_start3) * (y_c - y_start3);
                temp2 = (x1_e - x_start3) * (y_end2[1] - y_start3) -
                        (x_end2[1] - x_start3) * (y1_e - y_start3);
            }
            else
            {
                temp1 = (x_c - x_start2[1]) * (y_end3 - y_start2[1]) -
                        (x_end3 - x_start2[1]) * (y_c - y_start2[1]);
                temp2 = (x1_e - x_start2[1]) * (y_end3 - y_start2[1]) -
                        (x_end3 - x_start2[1]) * (y1_e - y_start2[1]);
            }
            if ((dtheta_e <= PI && temp1 * temp2 <= 0) ||
                (dtheta_e >= PI && temp1 * temp2 >= 0))
            {
                status[i] = 1;
            }
            else
            {
                status[i] = 0;
            }
            if (status[i] == 1)
            {
                if (r > 0 && ds > 0)
                {
                    temp1 = (xv_start[2] - xv_start[3]) * (Yo[i] - yv_start[3]) -
                            (Xo[i] - xv_start[3]) * (yv_start[2] - yv_start[3]);
                    temp2 = (xv_start[0] - xv_start[3]) * (Yo[i] - yv_start[3]) -
                            (Xo[i] - xv_start[3]) * (yv_start[0] - yv_start[3]);
                    temp3 = (xv_end[0] - xv_end[1]) * (Yo[i] - yv_end[1]) -
                            (Xo[i] - xv_end[1]) * (yv_end[0] - yv_end[1]);
                    temp4 = (x_end2[0] - xv_end[2]) * (Yo[i] - yv_end[2]) -
                            (Xo[i] - xv_end[2]) * (y_end2[0] - yv_end[2]);
                }
                if (r > 0 && ds < 0)
                {
                    temp1 = (xv_end[2] - xv_end[3]) * (Yo[i] - yv_end[3]) -
                            (Xo[i] - xv_end[3]) * (yv_end[2] - yv_end[3]);
                    temp2 = (xv_end[0] - xv_end[3]) * (Yo[i] - yv_end[3]) -
                            (Xo[i] - xv_end[3]) * (yv_end[0] - yv_end[3]);
                    temp3 = (xv_start[0] - xv_start[1]) * (Yo[i] - yv_start[1]) -
                            (Xo[i] - xv_start[1]) * (yv_start[0] - yv_start[1]);
                    temp4 = (x_start2[0] - xv_start[2]) * (Yo[i] - yv_start[2]) -
                            (Xo[i] - xv_start[2]) * (y_start2[0] - yv_start[2]);
                }
                if (r < 0 && ds > 0)
                {
                    temp1 = (xv_start[0] - xv_start[1]) * (Yo[i] - yv_start[1]) -
                            (Xo[i] - xv_start[1]) * (yv_start[0] - yv_start[1]);
                    temp2 = (xv_start[0] - xv_start[3]) * (Yo[i] - yv_start[3]) -
                            (Xo[i] - xv_start[3]) * (yv_start[0] - yv_start[3]);
                    temp3 = (Xo[i] - xv_end[2]) * (yv_end[3] - yv_end[2]) -
                            (xv_end[3] - xv_end[2]) * (Yo[i] - yv_end[2]);
                    temp4 = (Xo[i] - xv_end[1]) * (y_end2[0] - yv_end[1]) -
                            (x_end2[0] - xv_end[1]) * (Yo[i] - yv_end[1]);
                }
                if (r < 0 && ds < 0)
                {
                    temp1 = (xv_end[0] - xv_end[1]) * (Yo[i] - yv_end[1]) -
                            (Xo[i] - xv_end[1]) * (yv_end[0] - yv_end[1]);
                    temp2 = (xv_end[0] - xv_end[3]) * (Yo[i] - yv_end[3]) -
                            (Xo[i] - xv_end[3]) * (yv_end[0] - yv_end[3]);
                    temp3 = -(xv_start[3] - xv_start[2]) * (Yo[i] - yv_start[2]) +
                            (Xo[i] - xv_start[2]) * (yv_start[3] - yv_start[2]);
                    temp4 = -(x_start2[0] - xv_start[1]) * (Yo[i] - yv_start[1]) +
                            (Xo[i] - xv_start[1]) * (y_start2[0] - yv_start[1]);
                }
                if ((temp1 >= 0 && temp2 >= 0) ||
                    (temp3 >= 0 &&
                     temp4 <=
                         0)) // Determine if the obstacle endpoint is in two special areas
                {
                    status[i] = 0;
                }
                else
                {
                    status[i] = 1;
                }
            }
        }
        else
        {
            status[i] = 0;
        }
    }
    if (status[0] == 1 && status[1] == 1)
    {
        status_circle = 1;
    }
    else
    {
        status_circle = 0;
    }
    return status_circle;
}

/***********************************************************
Function Description : Path_ParaCal-->>Extract the path
Edition : 2017.05.18
Input list :
            path
Output list :
            EXtr_path
***********************************************************/
int Path_ParaCal(const float path[5], path_para EXtr_path[1],
                 const float swell) //  swell is the whole swell
{
    float len, width, h;
    float xv1[4], yv1[4], xv[2], yv[2];
    int i;

    if (fabsf(path[4]) < 1)
    {
        len   = VEHICLE_LEN;
        width = VEHICLE_WID;
        h     = REAR_SUSPENSION;
    }
    else
    {
        len   = VEHICLE_LEN - 0.75;
        width = VEHICLE_WID;
        h     = REAR_SUSPENSION - 0.3;
    }

    len    = len + swell;
    width  = width + swell;
    h      = h + swell / 2.0f;
    xv1[0] = len - h;
    xv1[1] = -h;
    xv1[2] = -h;
    xv1[3] = len - h;
    yv1[0] = width / 2;
    yv1[1] = width / 2;
    yv1[2] = -width / 2;
    yv1[3] = -width / 2;

    if (fabs(path[3]) > PATH_LENGTH_MAX + 0.001f)
    {
        if (fabs(EXtr_path[0].x0 - EXtr_path[0].x_end) < PATH_LENGTH_MAX &&
            fabs(EXtr_path[0].y0 - EXtr_path[0].y_end) < PATH_LENGTH_MAX)
        {
            PathPlan_CheckTrajLen("Path_ParaCal too long!", path[3]);
        }
    }

    EXtr_path[0].x0     = path[0];
    EXtr_path[0].y0     = path[1];
    EXtr_path[0].theta0 = path[2];
    EXtr_path[0].ds     = path[3];
    EXtr_path[0].r      = path[4];
    EXtr_path[0].dtheta = fabsf(path[3] / path[4]);

    if (fabsf(path[4]) < 1)
    {
        EXtr_path[0].x_end     = cosf(path[2]) * path[3] + path[0];
        EXtr_path[0].y_end     = sinf(path[2]) * path[3] + path[1];
        EXtr_path[0].theta_end = path[2];
    }
    else
    {
        EXtr_path[0].ftheta = -sign(path[4]) * PI / 2 + path[2];
        EXtr_path[0].x_c    = path[0] - fabsf(path[4]) * cosf(EXtr_path[0].ftheta);
        EXtr_path[0].y_c    = path[1] - fabsf(path[4]) * sinf(EXtr_path[0].ftheta);
        EXtr_path[0].x_end =
            cosf(path[3] / path[4] + EXtr_path[0].ftheta) * fabsf(path[4]) +
            EXtr_path[0].x_c; // x0 - fabsf(r)*cosf(ftheta);
        EXtr_path[0].y_end =
            sinf(path[3] / path[4] + EXtr_path[0].ftheta) * fabsf(path[4]) +
            EXtr_path[0].y_c; // y0 - fabsf(r)*sinf(ftheta);
        EXtr_path[0].theta_end = path[2] + path[3] / path[4];
    }

    for (i = 0; i < 4; i++)
    {
        EXtr_path[0].xv_start[i] =
            xv1[i] * cosf(path[2]) - yv1[i] * sinf(path[2]) + path[0];
        EXtr_path[0].yv_start[i] =
            xv1[i] * sinf(path[2]) + yv1[i] * cosf(path[2]) + path[1];
        EXtr_path[0].xv_end[0][i] = xv1[i] * cosf(EXtr_path[0].theta_end) -
                                    yv1[i] * sinf(EXtr_path[0].theta_end) +
                                    EXtr_path[0].x_end;
        EXtr_path[0].yv_end[0][i] = xv1[i] * sinf(EXtr_path[0].theta_end) +
                                    yv1[i] * cosf(EXtr_path[0].theta_end) +
                                    EXtr_path[0].y_end;
    }

    /*添加终点障碍检测矩形****************************/
    if (fabsf(path[4]) < 1)
    {
        for (i = 0; i < 4; i++)
        {
            EXtr_path[0].xv_end[1][i] = EXtr_path[0].xv_end[0][i];
            EXtr_path[0].yv_end[1][i] = EXtr_path[0].yv_end[0][i];

            EXtr_path[0].xv_end[2][i] = EXtr_path[0].xv_end[0][i];
            EXtr_path[0].yv_end[2][i] = EXtr_path[0].yv_end[0][i];
        }
    }
    else
    {
        float width1 = VEHICLE_WID - 1.2 + swell;
        float len1   = VEHICLE_LEN + swell;
        float h1     = REAR_SUSPENSION + swell / 2.0f;
        float xv2[4] = {0.0f}, yv2[4] = {0.0f};
        xv2[0] = len1 - h1;
        xv2[1] = -h1;
        xv2[2] = -h1;
        xv2[3] = len1 - h1;
        yv2[0] = width1 / 2;
        yv2[1] = width1 / 2;
        yv2[2] = -width1 / 2;
        yv2[3] = -width1 / 2;

        float width2 = VEHICLE_WID - 0.8 + swell;
        float len2   = VEHICLE_LEN - 0.15 + swell;
        float h2     = REAR_SUSPENSION + swell / 2.0f;
        float xv3[4] = {0.0f}, yv3[4] = {0.0f};
        xv3[0] = len2 - h2;
        xv3[1] = -h2;
        xv3[2] = -h2;
        xv3[3] = len2 - h2;
        yv3[0] = width2 / 2;
        yv3[1] = width2 / 2;
        yv3[2] = -width2 / 2;
        yv3[3] = -width2 / 2;

        for (i = 0; i < 4; i++)
        {
            EXtr_path[0].xv_end[1][i] = xv2[i] * cosf(EXtr_path[0].theta_end) -
                                        yv2[i] * sinf(EXtr_path[0].theta_end) +
                                        EXtr_path[0].x_end;
            EXtr_path[0].yv_end[1][i] = xv2[i] * sinf(EXtr_path[0].theta_end) +
                                        yv2[i] * cosf(EXtr_path[0].theta_end) +
                                        EXtr_path[0].y_end;

            EXtr_path[0].xv_end[2][i] = xv3[i] * cosf(EXtr_path[0].theta_end) -
                                        yv3[i] * sinf(EXtr_path[0].theta_end) +
                                        EXtr_path[0].x_end;
            EXtr_path[0].yv_end[2][i] = xv3[i] * sinf(EXtr_path[0].theta_end) +
                                        yv3[i] * cosf(EXtr_path[0].theta_end) +
                                        EXtr_path[0].y_end;
        }
    }

    /***********************************************/

    if (fabsf(path[4]) >= 1)
    {
        if (path[4] > 0)
        {
            xv[0] = 0;
            xv[1] = len - h;
            yv[0] = width / 2;
            yv[1] = -width / 2;
        }
        else
        {
            xv[0] = 0;
            xv[1] = len - h;
            yv[0] = -width / 2;
            yv[1] = width / 2;
        }
        for (i = 0; i < 2; i++) // Start and end coordinates of the inner and outer arc
        {
            EXtr_path[0].x_start2[i] =
                xv[i] * cosf(path[2]) - yv[i] * sinf(path[2]) + path[0];
            EXtr_path[0].y_start2[i] =
                xv[i] * sinf(path[2]) + yv[i] * cosf(path[2]) + path[1];
            EXtr_path[0].x_end2[i] = xv[i] * cosf(EXtr_path[0].theta_end) -
                                     yv[i] * sinf(EXtr_path[0].theta_end) +
                                     EXtr_path[0].x_end;
            EXtr_path[0].y_end2[i] = xv[i] * sinf(EXtr_path[0].theta_end) +
                                     yv[i] * cosf(EXtr_path[0].theta_end) +
                                     EXtr_path[0].y_end;
        }
    }

    if (path[3] >= 0)
    {
        EXtr_path[0].x_out[0] = EXtr_path[0].xv_end[0][0];
        EXtr_path[0].x_out[1] = EXtr_path[0].xv_start[1];
        EXtr_path[0].x_out[2] = EXtr_path[0].xv_start[2];
        EXtr_path[0].x_out[3] = EXtr_path[0].xv_end[0][3];
        EXtr_path[0].y_out[0] = EXtr_path[0].yv_end[0][0];
        EXtr_path[0].y_out[1] = EXtr_path[0].yv_start[1];
        EXtr_path[0].y_out[2] = EXtr_path[0].yv_start[2];
        EXtr_path[0].y_out[3] = EXtr_path[0].yv_end[0][3];
    }
    else
    {
        EXtr_path[0].x_out[0] = EXtr_path[0].xv_start[0];
        EXtr_path[0].x_out[1] = EXtr_path[0].xv_end[0][1];
        EXtr_path[0].x_out[2] = EXtr_path[0].xv_end[0][2];
        EXtr_path[0].x_out[3] = EXtr_path[0].xv_start[3];
        EXtr_path[0].y_out[0] = EXtr_path[0].yv_start[0];
        EXtr_path[0].y_out[1] = EXtr_path[0].yv_end[0][1];
        EXtr_path[0].y_out[2] = EXtr_path[0].yv_end[0][2];
        EXtr_path[0].y_out[3] = EXtr_path[0].yv_start[3];
    }

    for (i = 0; i < 2; i++)
    {
        EXtr_path[0].rr[i] = sqrtf((EXtr_path[0].x_start2[i] - EXtr_path[0].x_c) *
                                       (EXtr_path[0].x_start2[i] - EXtr_path[0].x_c) +
                                   (EXtr_path[0].y_start2[i] - EXtr_path[0].y_c) *
                                       (EXtr_path[0].y_start2[i] - EXtr_path[0].y_c));
    }
    EXtr_path[0].x_start3 =
        EXtr_path[0].x_c + EXtr_path[0].rr[1] * cosf(EXtr_path[0].ftheta);
    EXtr_path[0].y_start3 =
        EXtr_path[0].y_c + EXtr_path[0].rr[1] * sinf(EXtr_path[0].ftheta);
    EXtr_path[0].x_end3 =
        EXtr_path[0].x_c +
        EXtr_path[0].rr[1] * cosf(EXtr_path[0].ftheta + path[3] / path[4]);
    EXtr_path[0].y_end3 =
        EXtr_path[0].y_c +
        EXtr_path[0].rr[1] * sinf(EXtr_path[0].ftheta + path[3] / path[4]);
    EXtr_path[0].dtheta_e = atanf((len - h) / (fabsf(EXtr_path[0].r) + width)) +
                            fabsf(EXtr_path[0].ds / EXtr_path[0].r);
    EXtr_path[0].angle_total = atanf((len - h) / fabsf(EXtr_path[0].r)) +
                               atanf(h / fabsf(EXtr_path[0].r)) +
                               fabsf(EXtr_path[0].ds / EXtr_path[0].r);
    return 1;
}

/***********************************************************
Function Description : obj_ParaCal-->>Extract the path
Edition : 2017.05.18
Input list :
            obj
Output list :
            EXtr_obj
***********************************************************/
int obj_ParaCal(float obj[4], obj_para EXtr_obj[1])
{

    EXtr_obj[0].x1 = obj[0];
    EXtr_obj[0].x2 = obj[2];
    EXtr_obj[0].y1 = obj[1];
    EXtr_obj[0].y2 = obj[3];
    EXtr_obj[0].A  = EXtr_obj[0].y2 - EXtr_obj[0].y1;
    EXtr_obj[0].B  = EXtr_obj[0].x1 - EXtr_obj[0].x2;
    EXtr_obj[0].C  = EXtr_obj[0].y1 * EXtr_obj[0].x2 -
                    EXtr_obj[0].y2 * EXtr_obj[0].x1; // Oblique equation Ax+By+C=0
    EXtr_obj[0].theta_obs =
        atan2f(EXtr_obj[0].y2 - EXtr_obj[0].y1, EXtr_obj[0].x2 - EXtr_obj[0].x1);
    EXtr_obj[0].dis_obs =
        sqrtf((EXtr_obj[0].x2 - EXtr_obj[0].x1) * (EXtr_obj[0].x2 - EXtr_obj[0].x1) +
              (EXtr_obj[0].y2 - EXtr_obj[0].y1) * (EXtr_obj[0].y2 - EXtr_obj[0].y1));
    return 1;
}

int RightSide_PointLine(float Px, float Py, float x1, float y1, float x2, float y2)
{
    float temp;
    int status_right;
    temp = (x2 - x1) * (Py - y1) + (Px - x1) * (y2 - y1);
    if (temp > 0)
    {
        status_right = 0;
    }
    else
    {
        status_right = 1;
    }
    return status_right;
}

/***********************************************************
Function Description : Point_CircleLine-->>Calculate the intersection of the line and the
circle Edition : 2017.05.18 Input list : EXtr_obj EXtr_path Output list : status:-1:not
intersect;0-intersect xs:The x coordinate of the intersection ys:The y coordinate of the
intersection
***********************************************************/
int Point_CircleLine(obj_para EXtr_obj[1], path_para EXtr_path[1], int status[2],
                     float xs[][2], float ys[][2])
{
    float D[2], E[2], F[2];
    float a, b, c, delta;
    int i, k;
    float x_c, y_c, rr[2];
    float A, B, C;
    A   = EXtr_obj[0].A;
    B   = EXtr_obj[0].B;
    C   = EXtr_obj[0].C;
    x_c = EXtr_path[0].x_c;
    y_c = EXtr_path[0].y_c;

    for (i = 0; i < 2; i++)
    {
        rr[i] = EXtr_path[0].rr[i];
        D[i]  = -2 * x_c;
        E[i]  = -2 * y_c;
        F[i] =
            x_c * x_c + y_c * y_c - rr[i] * rr[i]; // Circular equation: x^2+y^2+Dx+Ey+F=0
    }

    for (k = 0; k < 2; k++)
    {
        if (fabsf(B) > fabsf(A))
        {
            a     = 1 + (A * A) / (B * B);
            b     = 2 * A * C / (B * B) + D[k] - A * E[k] / B;
            c     = C * C / (B * B) - C * E[k] / B + F[k];
            delta = b * b - 4 * a * c; // simultaneous equations:ax^2+bx+c = 0
            if (delta < 0)
            {
                status[k] = -1;
            }
            else
            {
                status[k] = 0;
                xs[k][0]  = (-b + sqrtf(delta)) / (2 * a);
                xs[k][1]  = (-b - sqrtf(delta)) / (2 * a);
                ys[k][0]  = (-C - A * (xs[k][0])) / B;
                ys[k][1]  = (-C - A * (xs[k][1])) / B;
            }
        }
        else
        {
            status[k] = 0;
            a         = 1 + (B * B) / (A * A);
            b         = 2 * B * C / (A * A) + E[k] - B * D[k] / A;
            c         = C * C / (A * A) - C * D[k] / A + F[k];
            delta     = b * b - 4 * a * c;
            if (delta < 0)
            {
                status[k] = -1;
            }
            else
            {
                status[k] = 0;
                ys[k][0]  = (-b + sqrtf(delta)) / (2 * a);
                ys[k][1]  = (-b - sqrtf(delta)) / (2 * a);
                xs[k][0]  = (-C - B * (ys[k][0])) / A;
                xs[k][1]  = (-C - B * (ys[k][1])) / A;
            }
        }
    }
    return 1;
}

// 检查车位有效性（可泊空间是否满足泊车要求）
int PathPlan_UpdateSlotSwell(PlanDataCase &PlanDataVer)
{
    const float side_dist_min = 0.1f; //  for a slot min free side dist around the targpos

    float front_dist    = 0;
    float rear_dist     = 0;
    float min_side_dist = 0;
    int result          = 1;

    switch (PlanDataVer.slotshape)
    {
        case 1:
        case 2: // parallel slot
            rear_dist =
                Project_PosTo1st_rx(PlanDataVer.finpoint, PlanDataVer.pointb_danger) +
                PlanDataVer.h; // has sign
            front_dist =
                Project_PosTo1st_rx(PlanDataVer.finpoint, PlanDataVer.pointf_danger) -
                PlanDataVer.len + PlanDataVer.h;
            min_side_dist = rte_min(fabsf(front_dist), fabsf(rear_dist));
            PlanDataVer.swell =
                rte_max(rte_min(((min_side_dist - 0.4f) * 0.5f + 0.8f), 2.0f), 0.9f) *
                g_local_swell;
            if (front_dist < side_dist_min || rear_dist > -side_dist_min)
            {
                result = -1;
            }
            break;

        default: //  for verticle slot and angle slot

            float distBC =
                Project_PosTo1st_ry(PlanDataVer.finpoint, PlanDataVer.pointb_danger);
            rear_dist =
                sign(distBC) *
                (fabsf(distBC) -
                 PlanDataVer.width / 2); //  use the sign() to make sure that slotB and
                                         //  slotE stay in different side of targpos
            float distDE =
                Project_PosTo1st_ry(PlanDataVer.finpoint, PlanDataVer.pointf_danger);
            front_dist = sign(distDE) * (fabsf(distDE) - PlanDataVer.width / 2);

            min_side_dist = rte_min(fabsf(front_dist), fabsf(rear_dist));
            PlanDataVer.swell =
                rte_max(rte_min(((min_side_dist - 0.3f) * 0.5f + 0.85f), 2.0f), 0.9f) *
                g_local_swell;
            if (PlanDataVer.swell >
                min_side_dist) //  20180726: if min_side_dist is too small,there's a bug
            {
                PlanDataVer.swell = 0.9 * min_side_dist;
            }

            if (front_dist * rear_dist > 0 ||
                fabsf(rear_dist) < PlanDataVer.swell / 2.0f ||
                fabsf(front_dist) < PlanDataVer.swell / 2.0f)
            {
                if (fabs(distBC) + fabs(distDE) > PlanDataVer.width + PlanDataVer.swell)
                {
                    printf(
                        "PathPlan_UpdateSlotSwell Vert Para failed! Vision and ultra "
                        "offset too much!\n");
                }
                result = -1;
            }
            break;
    }

    PlanDataVer.swell      = rte_max(PlanDataVer.swell, 0.35f);
    PlanDataVer.front_dist = fabsf(front_dist);
    PlanDataVer.rear_dist  = fabsf(rear_dist);
    return result;
}

// 限位杆处理
int PathPlan_BuildObjsByStopper(const SlotInfo_T &slot, LineSeg_T parkBars[2])
{
    const float stopper_offset     = 0.9;
    const float stopper_veh_offset = 0.15;
    const float collision_offset   = 0.45;

    if ((slot.slotshap != PK_SLOT_RIGHT_PARA && slot.slotshap != PK_SLOT_LEFT_PARA) ||
        slot.has_stopper == 0 || slot.is_vision_slot == 0)
    {
        return 0;
    }

    float target2Avm = 0.0;
    Point_T target;
    target.x = slot.targpos.x;
    target.y = slot.targpos.y;

    float rspoint[2], obj[4], finpoint[3];
    memcpy(finpoint, &slot.targpos, sizeof(finpoint));

    // bcAvmOffset > 0 车后轴短，可能在车位内  bcAvmOffset < 0 车后轴长，可能超出车位
    // obj_slot bc
    target2Avm =
        REAR_SUSPENSION + collision_offset - (stopper_offset + stopper_veh_offset);

    RevConvert(finpoint, (float *)&slot.avm_point.near_rear, rspoint);
    rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + target2Avm);
    //    rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) - collision_offset);
    //    Convert(finpoint, rspoint, (float *)&obj[0]);

    //    RevConvert(finpoint, (float *)&obj[0], rspoint);
    //    rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 0.8);
    //    Convert(finpoint, rspoint, (float *)&obj[2]);

    Get_Limit(finpoint, obj, rspoint[0], 1.2);
    memcpy(&parkBars[0], obj, sizeof(obj));

    // obj_slot de
    Get_Dist_Dir_Pt2PointLine(slot.avm_point.near_front, slot.avm_point.far_front, target,
                              &target2Avm, NULL);
    target2Avm = 2 * (stopper_offset + collision_offset);

    RevConvert(finpoint, (float *)&slot.avm_point.near_front, rspoint);
    rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + target2Avm);
    //    rspoint[1] = sign(rspoint[1]) * (fabs(rspoint[1]) - collision_offset);
    //    Convert(finpoint, rspoint, (float *)&obj[0]);

    //    RevConvert(finpoint, (float *)&obj[0], rspoint);
    //    rspoint[0] = sign(rspoint[0]) * (fabs(rspoint[0]) + 0.8);
    //    Convert(finpoint, rspoint, (float *)&obj[2]);
    Get_Limit(finpoint, obj, rspoint[0], 1.5);
    memcpy(&parkBars[1], obj, sizeof(obj));

    return 2;
}

/**
 * @brief 更新停车位边缘的距离偏差信息，用于检测障碍物和更新停车位形状。
 *
 * @param rightSlot 表示车位的方向，1 为右侧停车位，-1 为左侧停车位。
 * @param curSeg 当前障碍物的线段表示（包含两点坐标）。
 * @param slotobj 停车位形状信息。
 * @param slotshape 停车位的类型，0无效，1-2 表示平行车位，其他值表示垂直车位。
 * @param targPos 停车目标点（车辆停泊的参考位置）。
 * @param limit 边界限制，用于约束障碍物检测的范围。
 * @param diff 存储停车位各边的距离偏差信息。
 * @param curSegType (可选) 当前线段的类型指针，用于标识特殊的障碍物类型。
 * @return 如果障碍物在车位中间区域返回非 0 值，否则返回 0。
 */

static int UpdateSlotEdge(int slotId, const int rightSlot, const LineSeg_T &curSeg,
                          const SlotObj_T &slotobj, const int slotshape,
                          const float targPos[3], const float limit[2], float diff[EGMAX],
                          int *curSegType = NULL)
{
    float rspoint[3];
    RevConvert(targPos, (const float *)&curSeg.pt1, rspoint);
    float rx1 = rspoint[0];
    float ry1 = rspoint[1];

    RevConvert(targPos, (const float *)&curSeg.pt2, rspoint);
    float rx2 = rspoint[0];
    float ry2 = rspoint[1];

    float rear_limit, front_limit, bottom_limit;
    if (slotshape <= PK_SLOT_RIGHT_PARA)
    {
        rear_limit   = -(REAR_SUSPENSION + 0.5f); // 车位底部障碍物有效范围
        front_limit  = (VEHICLE_LEN - REAR_SUSPENSION + 0.5f);
        bottom_limit = -VEHICLE_WID * 0.25;
    }
    else
    {
        rear_limit   = -(VEHICLE_WID * 0.5f); // 车位底部障碍物有效范围
        front_limit  = (VEHICLE_WID * 0.5f);
        bottom_limit = -(REAR_SUSPENSION);
    }

    if ((rx1 > limit[0] && rx2 > limit[0]))
    {
        return 0;
    }

    // HAVECHANGE Tangsj 忽略车位内距超声波2.6m以外的障碍物
    //    float obj_ds_max = VEHICLE_WID/2 + 2.6;
    //    float obj_ry1 = Project_PosTo1st_ry(CurPos, (const float *)&curSeg.pt1);
    //    float obj_ry2 = Project_PosTo1st_ry(CurPos, (const float *)&curSeg.pt2);
    //    if(fabs(obj_ry1) > obj_ds_max && fabs(obj_ry2) > obj_ds_max) { return 0; }

    //  make sure obs in the same direction of slot
    // 1 在车位中心两侧
    if (ry1 * ry2 <= 0 &&
        ((rx1 >= 0 && rx1 <= limit[0]) || (rx2 >= 0 && rx2 <= limit[0]) ||
         (rx1 <= 0 && rx1 >= limit[1]) || (rx2 < 0 && rx2 >= limit[1])) &&
        (rx1 > bottom_limit || rx2 > bottom_limit))
    {
        printf(
            "UpdateSlotEdge Failed! obj in slot, cross with target! %d "
            "obj:%04f,%04f,%04f,%04f %04f,%04f,%04f,%04f\n",
            slotId, curSeg.pt1.x, curSeg.pt1.y, curSeg.pt2.x, curSeg.pt2.y, rx1, rx2, ry1,
            ry2);
        return 1;
    }

    float tempdist0[6];
    Relation_T dir0[6];

    Get_Dist_Dir_Pt2PointLine(slotobj.ptC, slotobj.ptB, curSeg.pt1, &tempdist0[0],
                              &dir0[0]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptC, slotobj.ptB, curSeg.pt2, &tempdist0[1],
                              &dir0[1]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptE, slotobj.ptD, curSeg.pt1, &tempdist0[2],
                              &dir0[2]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptE, slotobj.ptD, curSeg.pt2, &tempdist0[3],
                              &dir0[3]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptC, slotobj.ptD, curSeg.pt1, &tempdist0[4],
                              &dir0[4]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptC, slotobj.ptD, curSeg.pt2, &tempdist0[5],
                              &dir0[5]);

    if (dir0[0] == dir0[1] && dir0[0] == PK_ON_LEFT && rightSlot == 1)
    {
        return 0;
    }
    if (dir0[2] == dir0[3] && dir0[2] == PK_ON_LEFT && rightSlot == 1)
    {
        return 0;
    }
    if (dir0[0] == dir0[1] && dir0[0] == PK_ON_RIGHT && rightSlot == -1)
    {
        return 0;
    }
    if (dir0[2] == dir0[3] && dir0[2] == PK_ON_RIGHT && rightSlot == -1)
    {
        return 0;
    }

    float distAB = 0.0;
    float distEF = 0.0;
    float distBC = 0.0;
    float distDE = 0.0;
    float distCD = 0.0;

    float tempdist1[4];
    Relation_T dir1[4];
    Get_Dist_Dir_Pt2PointLine(slotobj.ptA, slotobj.ptB, curSeg.pt1, &tempdist1[0],
                              &dir1[0]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptA, slotobj.ptB, curSeg.pt2, &tempdist1[1],
                              &dir1[1]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptE, slotobj.ptF, curSeg.pt1, &tempdist1[2],
                              &dir1[2]);
    Get_Dist_Dir_Pt2PointLine(slotobj.ptE, slotobj.ptF, curSeg.pt2, &tempdist1[3],
                              &dir1[3]);

    if ((dir0[0] != dir0[1] || dir0[2] != dir0[3]) &&
        (rx1 > bottom_limit || rx2 > bottom_limit))
    {
        Point_T pointB, pointE;
        float rsPoint[2];

        RevConvert(targPos, (float *)&slotobj.ptB, rsPoint);
        rsPoint[0] = limit[0];
        Convert(targPos, rsPoint, (float *)&pointB);

        RevConvert(targPos, (float *)&slotobj.ptE, rsPoint);
        rsPoint[0] = limit[0];
        Convert(targPos, rsPoint, (float *)&pointE);

        auto cbCrossed = Is_TwoSegment_Cross(slotobj.ptC, pointB, curSeg.pt1, curSeg.pt2);
        auto edCrossed = Is_TwoSegment_Cross(slotobj.ptD, pointE, curSeg.pt1, curSeg.pt2);

        // 右侧车位
        if (rightSlot == 1)
        {
            if (dir0[0] != dir0[1])
            {
                if (cbCrossed != 0)
                {
                    // 离B最远的距离
                    if (dir0[0] == PK_ON_RIGHT)
                    {
                        distBC = rte_max(tempdist0[0], distBC);
                    }
                    if (dir0[1] == PK_ON_RIGHT)
                    {
                        distBC = rte_max(tempdist0[1], distBC);
                    }
                    if (dir1[0] == PK_ON_LEFT)
                    {
                        distAB = rte_max(tempdist1[0], distAB);
                    }
                    if (dir1[1] == PK_ON_LEFT)
                    {
                        distAB = rte_max(tempdist1[1], distAB);
                    }
                }
            }
            else
            {
                if (edCrossed != 0)
                {
                    // 离ED最远的距离
                    if (dir0[2] == PK_ON_RIGHT)
                    {
                        distDE = rte_max(tempdist0[2], distDE);
                    }
                    if (dir0[3] == PK_ON_RIGHT)
                    {
                        distDE = rte_max(tempdist0[3], distDE);
                    }
                    if (dir1[2] == PK_ON_LEFT)
                    {
                        distEF = rte_max(tempdist1[2], distEF);
                    }
                    if (dir1[3] == PK_ON_LEFT)
                    {
                        distEF = rte_max(tempdist1[3], distEF);
                    }
                }
            }
        }
        else
        {
            // DE两侧
            if (dir0[0] != dir0[1])
            {
                if (cbCrossed != 0)
                {
                    if (dir0[0] == PK_ON_LEFT)
                    {
                        distBC = rte_max(tempdist0[0], distBC);
                    }
                    if (dir0[1] == PK_ON_LEFT)
                    {
                        distBC = rte_max(tempdist0[1], distBC);
                    }
                    if (dir1[0] == PK_ON_RIGHT)
                    {
                        distAB = rte_max(tempdist1[0], distAB);
                    }
                    if (dir1[1] == PK_ON_RIGHT)
                    {
                        distAB = rte_max(tempdist1[1], distAB);
                    }
                }
            }
            else
            {
                if (edCrossed != 0)
                {
                    if (dir0[2] == PK_ON_LEFT)
                    {
                        distDE = rte_max(tempdist0[2], distDE);
                    }
                    if (dir0[3] == PK_ON_LEFT)
                    {
                        distDE = rte_max(tempdist0[3], distDE);
                    }
                    if (dir1[2] == PK_ON_RIGHT)
                    {
                        distEF = rte_max(tempdist1[2], distEF);
                    }
                    if (dir1[3] == PK_ON_RIGHT)
                    {
                        distEF = rte_max(tempdist1[3], distEF);
                    }
                }
            }
        }

        diff[AB] = rte_max(diff[AB], distAB);
        diff[BC] = rte_max(diff[BC], distBC);
        diff[DE] = rte_max(diff[DE], distDE);
        diff[EF] = rte_max(diff[EF], distEF);
    }
    else if (rx1 > bottom_limit || rx2 > bottom_limit)
    {
        // 在车位内
        Point_T target1 = {targPos[0], targPos[1]};
        Point_T target2 = {targPos[0] + cosf(targPos[2]), targPos[1] + sinf(targPos[2])};

        Relation_T dir2;
        Get_Dist_Dir_Pt2PointLine(target1, target2, curSeg.pt1, NULL, &dir2);

        distBC = rte_max(fabs(tempdist0[0]), fabs(tempdist0[1]));
        distDE = rte_max(fabs(tempdist0[2]), fabs(tempdist0[3]));

        // 右边车位, PT1在左边
        if ((rightSlot == 1 && dir2 == PK_ON_LEFT) ||
            (rightSlot == -1 && dir2 == PK_ON_RIGHT))
        {
            diff[BC] = rte_max(diff[BC], distBC);

            if (dir1[0] == PK_ON_LEFT && rightSlot == 1)
            {
                distAB = rte_max(tempdist1[0], distAB);
            }
            if (dir1[1] == PK_ON_LEFT && rightSlot == 1)
            {
                distAB = rte_max(tempdist1[1], distAB);
            }
            if (dir1[0] == PK_ON_RIGHT && rightSlot == -1)
            {
                distAB = rte_max(tempdist1[0], distAB);
            }
            if (dir1[1] == PK_ON_RIGHT && rightSlot == -1)
            {
                distAB = rte_max(tempdist1[1], distAB);
            }
            // distAB = rte_max(fabs(tempdist1[0]), fabs(tempdist1[1]));
            diff[AB] = rte_max(diff[AB], distAB);
        }
        else if ((rightSlot == 1 && dir2 == PK_ON_RIGHT) ||
                 (rightSlot == -1 && dir2 == PK_ON_LEFT))
        {
            diff[DE] = rte_max(diff[DE], distDE);
            if (dir1[2] == PK_ON_LEFT && rightSlot == 1)
            {
                distEF = rte_max(tempdist1[2], distEF);
            }
            if (dir1[3] == PK_ON_LEFT && rightSlot == 1)
            {
                distEF = rte_max(tempdist1[3], distEF);
            }
            if (dir1[2] == PK_ON_RIGHT && rightSlot == -1)
            {
                distEF = rte_max(tempdist1[2], distEF);
            }
            if (dir1[3] == PK_ON_RIGHT && rightSlot == -1)
            {
                distEF = rte_max(tempdist1[3], distEF);
            }
            // distEF = rte_max(fabs(tempdist1[2]), fabs(tempdist1[3]));
            diff[EF] = rte_max(diff[EF], distEF);
        }
    }

    if (rx1 < 0)
    {
        if (rightSlot == 1 && ry1 < -rear_limit && ry1 > -front_limit &&
            dir0[4] == PK_ON_LEFT)
        {
            distCD      = rte_max(fabs(tempdist0[4]), distCD);
            *curSegType = 30000;
        }

        if (rightSlot == -1 && ry1 > rear_limit && ry1 < front_limit &&
            dir0[4] == PK_ON_RIGHT)
        {
            distCD      = rte_max(fabs(tempdist0[4]), distCD);
            *curSegType = 30000;
        }
    }

    if (rx2 < 0)
    {
        if (rightSlot == 1 && ry2 < -rear_limit && ry2 > -front_limit &&
            dir0[5] == PK_ON_LEFT)
        {
            distCD      = rte_max(fabs(tempdist0[5]), distCD);
            *curSegType = 30000;
        }

        if (rightSlot == -1 && ry2 > rear_limit && ry2 < front_limit &&
            dir0[5] == PK_ON_RIGHT)
        {
            distCD      = rte_max(fabs(tempdist0[5]), distCD);
            *curSegType = 30000;
        }
    }

    diff[CD] = rte_max(diff[CD], distCD);
    return 0;
}

static void UpdateDistToAvm(const PlanDataCase planData[1], const SlotInfo_T &slot,
                            float avmDist[3])
{
    float bc[4], de[4], cd[4];
    if (planData[1].is_vision_slot == 0)
    {
        memcpy(bc, planData[0].obj_slot[1], sizeof(Point_T));
        memcpy(cd, planData[0].obj_slot[2], sizeof(Point_T));
        memcpy(de, planData[0].obj_slot[3], sizeof(Point_T));
    }
    else
    {
        memcpy(&bc[0], (float *)&slot.avm_point.near_rear, sizeof(Point_T));
        memcpy(&bc[2], (float *)&slot.avm_point.far_rear, sizeof(Point_T));
        memcpy(&cd[0], (float *)&slot.avm_point.far_rear, sizeof(Point_T));
        memcpy(&cd[2], (float *)&slot.avm_point.far_front, sizeof(Point_T));
        memcpy(&de[0], (float *)&slot.avm_point.far_front, sizeof(Point_T));
        memcpy(&de[2], (float *)&slot.avm_point.near_front, sizeof(Point_T));
    }

    avmDist[0] = PK_PointToLineDist(bc, planData[0].finpoint);
    avmDist[1] = PK_PointToLineDist(cd, planData[0].finpoint);
    avmDist[2] = PK_PointToLineDist(de, planData[0].finpoint);
}

// 将障碍物存入PlanDataCase中
static void UpdateSlotObjDanger(PlanDataCase PlanData_Ver[1], const float slotB_Pos[3],
                                const float rect_pr_possb[4],
                                const int FS_ObjDir[FS_OBJ_ARR_NUM],
                                const float FS_Obj[FS_OBJ_ARR_NUM][4])
{
    int temp_objdDir;
    float rline_pr[4] = {0}, rline_pr_temp[4] = {0}, temp_obj[4] = {0};
    LineSeg_T objsegm;
    VehPos_T targPos;

    memcpy(&targPos, PlanData_Ver[0].finpoint, sizeof(targPos));

    // get the sensor fusion objects near slot
    for (uint32_t i = 0; i < FS_OBJ_ARR_NUM;
         i++) //  20180703: should set a condition: if (return_sta == 1) to avoid
              //  unnecessary calculation, and the initialization value should be set to 0
    {
        if (FS_ObjDir[i] != OBJTYPE_NONE)
        {
            rline_pr[0] = Project_PosTo1st_rx(slotB_Pos, FS_Obj[i]);
            rline_pr[1] = Project_PosTo1st_ry(slotB_Pos, FS_Obj[i]);
            rline_pr[2] = Project_PosTo1st_rx(slotB_Pos, &FS_Obj[i][2]);
            rline_pr[3] = Project_PosTo1st_ry(slotB_Pos, &FS_Obj[i][2]);
            memcpy(rline_pr_temp, rline_pr, sizeof(rline_pr));
            memcpy(&objsegm, FS_Obj[i], sizeof(objsegm));
            if (PK_RectLineCross(rect_pr_possb, rline_pr_temp) == 1 &&
                PK_PosLineSegCross_Check(targPos, 1, &objsegm, OBJ_SWELL) ==
                    0) // 在之前已进行了过滤处理，安全考虑车辆周围的障碍物也要考虑
            {
                if (rline_pr[0] > rline_pr[2])
                {
                    if (sign(Project_PosTo1st_ry(PlanData_Ver[0].stpoint, FS_Obj[i])) !=
                        sign(Project_PosTo1st_ry(PlanData_Ver[0].stpoint, &FS_Obj[i][2])))
                    {
                        temp_objdDir = FS_ObjDir[i] / 10000 > 1 ? 1 : 2;
                    }
                    else
                    {
                        temp_objdDir = FS_ObjDir[i] / 10000;
                    }
                    memcpy(&temp_obj[0], &FS_Obj[i][2], sizeof(float) * 2);
                    memcpy(&temp_obj[2], &FS_Obj[i][0], sizeof(float) * 2);
                }
                else
                {
                    temp_objdDir = FS_ObjDir[i] / 10000;
                    memcpy(&temp_obj[0], &FS_Obj[i][0], sizeof(float) * 2);
                    memcpy(&temp_obj[2], &FS_Obj[i][2], sizeof(float) * 2);
                }

                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][0] =
                    temp_obj[0];
                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][1] =
                    temp_obj[1];
                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][2] =
                    temp_obj[2];
                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][3] =
                    temp_obj[3];
                PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] =
                    temp_objdDir; //  dir
                PlanData_Ver[0].obj_danger_num++;
            }
            else if (FS_ObjDir[i] / 10000 == 3) // 平行车位底部障碍物
            {
                temp_objdDir = 3; // PathPlan_SlotDir(slot.slotshap) == 2 ? 1 : 2;

                memcpy(&temp_obj[0], &FS_Obj[i][0], sizeof(float) * 2);
                memcpy(&temp_obj[2], &FS_Obj[i][2], sizeof(float) * 2);

                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][0] =
                    temp_obj[0];
                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][1] =
                    temp_obj[1];
                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][2] =
                    temp_obj[2];
                PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num][3] =
                    temp_obj[3];
                PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] =
                    temp_objdDir; //  dir
                PlanData_Ver[0].obj_danger_num++;
            }
        }
    }
}

// 将障碍物存入PlanDataCase中
static void UpdateSlotObjDanger1(PlanDataCase PlanData_Ver[1], const float slotB_Pos[3],
                                 const float rect_pr_possb[4],
                                 const int FS_ObjDir[FS_OBJ_ARR_NUM],
                                 const float FS_Obj[FS_OBJ_ARR_NUM][4])
{
    enum
    {
        LINE_AB = 0,
        LINE_BC,
        LINE_CD,
        LINE_DE,
        LINE_EF
    };

    LineSeg_T lineSlot[5];
    for (int i = 0; i < 5; i++)
    {
        memcpy(&lineSlot[i], PlanData_Ver[0].obj_slot[i], sizeof(LineSeg_T));
    }

    Point_T basePoint;
    float baseDist = 0.0;
    Relation_T targDir;
    memcpy((float *)&basePoint, PlanData_Ver[0].finpoint, sizeof(Point_T)); // target
    Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_CD], basePoint, &baseDist, &targDir);

    Vec2_T cdvec;
    cdvec.vx = lineSlot[LINE_CD].pt2.x - lineSlot[LINE_CD].pt1.x;
    cdvec.vy = lineSlot[LINE_CD].pt2.y - lineSlot[LINE_CD].pt1.y;
    Normalize_Vec2(&cdvec);

    Point_T midPoint;
    midPoint.x = 0.5f * (lineSlot[LINE_CD].pt2.x + lineSlot[LINE_CD].pt1.x);
    midPoint.y = 0.5f * (lineSlot[LINE_CD].pt2.y + lineSlot[LINE_CD].pt1.y);

    int tempDir[FS_OBJ_ARR_NUM];
    for (uint32_t i = 0; i < FS_OBJ_ARR_NUM; i++)
    {
        tempDir[i] = OBJTYPE_NONE;
        if (FS_ObjDir[i] != OBJTYPE_NONE)
        {
            float lineDist[2];
            Relation_T lineDir[2];
            LineSeg_T objLine;
            memcpy(&objLine, FS_Obj[i], sizeof(LineSeg_T));
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_CD], objLine.pt1, &lineDist[0],
                                    &lineDir[0]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_CD], objLine.pt2, &lineDist[1],
                                    &lineDir[1]);

            // CD 外侧
            if (lineDir[0] == lineDir[1] && lineDir[1] != targDir)
            {
                continue;
            }

            // CD 内侧但是距离太远
            if (lineDir[0] == lineDir[1] && lineDir[1] == targDir)
            {
                if (rte_min(lineDist[0], lineDist[1]) > 20.0f)
                {
                    continue;
                }
            }

            Vec2_T objvec[2];
            objvec[0].vx = objLine.pt1.x - midPoint.x;
            objvec[0].vy = objLine.pt1.y - midPoint.y;
            objvec[1].vx = objLine.pt2.x - midPoint.x;
            objvec[1].vy = objLine.pt2.y - midPoint.y;

            float pt1x = Get_Dot_Vec2(cdvec, objvec[0]);
            float pt2x = Get_Dot_Vec2(cdvec, objvec[1]);
            // CD 前后方向太远
            if (pt1x * pt2x > 0.0001 && rte_min(fabs(pt1x), fabs(pt2x)) > 10.0f)
            {
                continue;
            }
            tempDir[i] = FS_ObjDir[i];
        }
    }

    Relation_T pointCDir, pointADir;
    memcpy((float *)&basePoint, (float *)&lineSlot[LINE_CD].pt1,
           sizeof(Point_T)); // C point
    Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_AB], basePoint, &baseDist, &pointCDir);
    memcpy((float *)&basePoint, (float *)&lineSlot[LINE_AB].pt1,
           sizeof(Point_T)); // A point
    float distAtoBC = 0.0;
    Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_BC], basePoint, &distAtoBC, &pointADir);
    for (uint32_t i = 0; i < FS_OBJ_ARR_NUM; i++)
    {
        if (tempDir[i] != OBJTYPE_NONE)
        {
            float lineDist[4];
            Relation_T lineDir[4];
            LineSeg_T objLine;
            memcpy((float *)&objLine, FS_Obj[i], sizeof(LineSeg_T));
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_AB], objLine.pt1, &lineDist[0],
                                    &lineDir[0]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_AB], objLine.pt2, &lineDist[1],
                                    &lineDir[1]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_BC], objLine.pt1, &lineDist[2],
                                    &lineDir[2]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_BC], objLine.pt2, &lineDist[3],
                                    &lineDir[3]);

            // AB 外侧
            bool conditonAB = (lineDir[0] == lineDir[1] && lineDir[1] == pointCDir);
            conditonAB =
                conditonAB || (lineDir[0] == pointCDir && lineDir[1] == PK_ON_THE_LINE);
            conditonAB =
                conditonAB || (lineDir[1] == pointCDir && lineDir[0] == PK_ON_THE_LINE);

            // BC 外侧
            bool conditonBC = (lineDir[2] == lineDir[3] && lineDir[2] == pointADir);
            conditonBC =
                conditonBC || (lineDir[2] == pointADir && lineDir[3] == PK_ON_THE_LINE);
            conditonBC =
                conditonBC || (lineDir[3] == pointADir && lineDir[2] == PK_ON_THE_LINE);

            if (conditonAB && conditonBC && lineDist[2] < distAtoBC + OBJ_SWELL * 0.5 &&
                lineDist[3] < distAtoBC + OBJ_SWELL * 0.5)
            {
                tempDir[i] = OBJTYPE_NONE;
            }
        }
    }

    Relation_T pointDDir, pointFDir;
    memcpy((float *)&basePoint, (float *)&lineSlot[LINE_CD].pt2,
           sizeof(Point_T)); // D point
    Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_EF], basePoint, &baseDist, &pointDDir);
    float distFtoDE = 0.0;
    memcpy((float *)&basePoint, (float *)&lineSlot[LINE_EF].pt2,
           sizeof(Point_T)); // F point
    Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_DE], basePoint, &distFtoDE, &pointFDir);
    for (uint32_t i = 0; i < FS_OBJ_ARR_NUM; i++)
    {
        if (tempDir[i] != OBJTYPE_NONE)
        {
            float lineDist[4];
            Relation_T lineDir[4];
            LineSeg_T objLine;
            memcpy((float *)&objLine, FS_Obj[i], sizeof(LineSeg_T));
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_EF], objLine.pt1, &lineDist[0],
                                    &lineDir[0]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_EF], objLine.pt2, &lineDist[1],
                                    &lineDir[1]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_DE], objLine.pt1, &lineDist[2],
                                    &lineDir[2]);
            Get_Dist_Dir_Pt2SegLine(lineSlot[LINE_DE], objLine.pt2, &lineDist[3],
                                    &lineDir[3]);

            // EF 外侧
            bool conditonEF = (lineDir[0] == lineDir[1] && lineDir[1] == pointDDir);
            conditonEF =
                conditonEF || (lineDir[0] == pointDDir && lineDir[1] == PK_ON_THE_LINE);
            conditonEF =
                conditonEF || (lineDir[1] == pointDDir && lineDir[0] == PK_ON_THE_LINE);

            // DE 外侧
            bool conditonDE = (lineDir[2] == lineDir[3] && lineDir[2] == pointFDir);
            conditonDE =
                conditonDE || (lineDir[2] == pointFDir && lineDir[3] == PK_ON_THE_LINE);
            conditonDE =
                conditonDE || (lineDir[3] == pointFDir && lineDir[2] == PK_ON_THE_LINE);

            if (conditonEF && conditonDE && lineDist[2] < distFtoDE + OBJ_SWELL * 0.5 &&
                lineDist[3] < distFtoDE + OBJ_SWELL * 0.5)
            {
                tempDir[i] = OBJTYPE_NONE;
            }
        }
    }

    for (uint32_t i = 0; i < FS_OBJ_ARR_NUM;
         i++) //  20180703: should set a condition: if (return_sta == 1) to avoid
              //  unnecessary calculation, and the initialization value should be set to 0
    {
        if (tempDir[i] == OBJTYPE_NONE)
        {
            continue;
        }

        if (sign(Project_PosTo1st_ry(PlanData_Ver[0].stpoint, FS_Obj[i])) !=
            sign(Project_PosTo1st_ry(PlanData_Ver[0].stpoint, &FS_Obj[i][2])))
        {
            PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] =
                tempDir[i] / 10000 > 1 ? 1 : 2;
        }
        else
        {
            PlanData_Ver[0].obj_danger_dir[PlanData_Ver[0].obj_danger_num] =
                tempDir[i] / 10000;
        }

        memcpy(PlanData_Ver[0].obj_danger[PlanData_Ver[0].obj_danger_num], FS_Obj[i],
               sizeof(LineSeg_T));
        PlanData_Ver[0].obj_danger_num++;
    }
}

// 将障碍物存入PlanDataCase中
void PathPlan_BuildSlotObjDanger(PlanDataCase PlanData_Ver[1], const float slotB_Pos[3],
                                 const float rect_pr_possb[4],
                                 const int FS_ObjDir[FS_OBJ_ARR_NUM],
                                 const float FS_Obj[FS_OBJ_ARR_NUM][4],
                                 const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                 const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    VehPos_T targPos;

    memcpy(&targPos, PlanData_Ver[0].finpoint, sizeof(targPos));
    PlanData_Ver[0].obj_danger_num = 0;

    UpdateSlotObjDanger1(PlanData_Ver, slotB_Pos, rect_pr_possb, FS_ObjDir, FS_Obj);

    UpdateSlotObjDanger1(PlanData_Ver, slotB_Pos, rect_pr_possb, FS_ObjDir_Rear,
                         FS_Obj_Rear);
}

static void UpdateSlotWithEdge(const float finpoint[3], const float diff[5],
                               SlotObj_T &slotObj)
{
    auto MovePointFunc = [&finpoint](const int index, const float diff, float point[2]) {
        float rspoint[2];
        RevConvert(finpoint, point, rspoint);
        rspoint[index] = sign(rspoint[index]) * (fabs(rspoint[index]) + diff);
        Convert(finpoint, rspoint, point);
    };

    if (fabs(diff[AB]) > ZERO_FLOAT)
    {
        MovePointFunc(0, diff[AB], (float *)&slotObj.ptA);
        MovePointFunc(0, diff[AB], (float *)&slotObj.ptB);
    }

    if (fabs(diff[BC]) > ZERO_FLOAT)
    {
        MovePointFunc(1, diff[BC], (float *)&slotObj.ptA);
        MovePointFunc(1, diff[BC], (float *)&slotObj.ptB);
        MovePointFunc(1, diff[BC], (float *)&slotObj.ptC);
    }

    if (fabs(diff[CD]) > ZERO_FLOAT)
    {
        MovePointFunc(0, diff[CD], (float *)&slotObj.ptC);
        MovePointFunc(0, diff[CD], (float *)&slotObj.ptD);
    }

    if (fabs(diff[DE]) > ZERO_FLOAT)
    {
        MovePointFunc(1, diff[DE], (float *)&slotObj.ptD);
        MovePointFunc(1, diff[DE], (float *)&slotObj.ptE);
        MovePointFunc(1, diff[DE], (float *)&slotObj.ptF);
    }

    if (fabs(diff[EF]) > ZERO_FLOAT)
    {
        MovePointFunc(0, diff[EF], (float *)&slotObj.ptE);
        MovePointFunc(0, diff[EF], (float *)&slotObj.ptF);
    }
}

void AddSlotObjDanger(const PlanDataCase planData[1], const int FS_ObjDir[FS_OBJ_ARR_NUM],
                      const float FS_Obj[FS_OBJ_ARR_NUM][4],
                      const int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                      const float FS_Obj_Rear[FS_OBJ_ARR_NUM][4],
                      int dangeObjDir[FS_OBJ_ARR_NUM], float dangerObj[FS_OBJ_ARR_NUM][4])
{
    const float dist_diff = 0.20f;
    int count             = 0;
    for (int k = 0; k < 2; k++)
    {
        for (uint32_t i = 0; i < FS_OBJ_ARR_NUM && count < FS_OBJ_ARR_NUM; i++)
        {
            int dir = (k == 0) ? FS_ObjDir[i] : FS_ObjDir_Rear[i];
            if (dir != 0)
            {
                int j = 0;
                float *p1 =
                    (k == 0) ? (float *)&FS_Obj[i][0] : (float *)&FS_Obj_Rear[i][0];
                float *p2 =
                    (k == 0) ? (float *)&FS_Obj[i][2] : (float *)&FS_Obj_Rear[i][2];
                while (j < 5)
                {
                    if (Calc_Dist_Line2Line(p1, p2, (float *)&planData[0].obj_slot[j][0],
                                            (float *)&planData[0].obj_slot[j][2]) <
                        dist_diff)
                    {
                        break;
                    }
                    j++;
                }

                if (j < 5)
                {
                    continue;
                }

                j = 0;
                while (j < planData[0].obj_danger_num)
                {
                    if (Calc_Dist_Line2Line(p1, p2, (float *)&planData[0].obj_slot[j][0],
                                            (float *)&planData[0].obj_slot[j][2]) <
                        dist_diff)
                    {
                        break;
                    }
                    j++;
                }

                if (j < planData[0].obj_danger_num)
                {
                    continue;
                }

                dangeObjDir[count] = FS_ObjDir[i];
                memcpy(dangerObj[count], FS_Obj[i], sizeof(float) * 4);
                count++;
            }
        }
    }
}

/*
功能描述：设置车位最大泊车可行驶区域
函数名称：ExpandSlotObjs
INTPUT:     车位slot
OUTPUT:     车位slot.slotobj
*/
int ExpandSlotObjs(SlotInfo_T &slot, const float stpoint[3])
{
    float expand_para_ab, expand_para_bc, expand_para_cd, expand_para_de, expand_para_ef;
    float expand_vert_ab, expand_vert_bc, expand_vert_cd, expand_vert_de, expand_vert_ef;

    float left_limit = 5550, right_limit = 5550, front_limit = 2550, rear_limit = 2550;
    U_RadarType uradar;
    RTE_PD_Get_U_Radar(&uradar); // 超声波实时反馈数据
    int minDist = 5500;

    minDist = rte_min(uradar.FOL, minDist);
    minDist = rte_min(uradar.FCL, minDist);
    minDist = rte_min(uradar.FCR, minDist);
    minDist = rte_min(uradar.FOR, minDist);
    minDist = rte_min(uradar.ROL, minDist);
    minDist = rte_min(uradar.RCL, minDist);
    minDist = rte_min(uradar.RCR, minDist);
    minDist = rte_min(uradar.ROR, minDist);
    minDist = rte_min(uradar.FSL, minDist);
    minDist = rte_min(uradar.FSR, minDist);
    minDist = rte_min(uradar.RSL, minDist);
    minDist = rte_min(uradar.RSR, minDist);

    if (minDist <= 250)
    {
        return PATHPLAN_SUCCESS + 1;
    } // ????????????

    const float max_para_cd_offset = 1.2; // 0.6 - 1.2
    const float max_para_bc_offset = 2.5;
    const float max_para_de_offset = 6.5;
    expand_para_ab = 0.01; // 0.01;//VEHICLE_WID / 10; //have bug? 当 expand_para_ab 和
                           // expand_para_ef = 0 时有BUG
    expand_para_bc = REAR_SUSPENSION + max_para_bc_offset;
    expand_para_cd = VEHICLE_WID / 2 + max_para_cd_offset;
    expand_para_de = VEHICLE_LEN - REAR_SUSPENSION + max_para_de_offset;
    expand_para_ef = expand_para_ab;

    expand_vert_ab = VEHICLE_LEN - REAR_SUSPENSION - 3.5;
    expand_vert_bc = VEHICLE_WID / 2 + EXPAND_SLOTOBJ_CB_VERT;
    expand_vert_cd = REAR_SUSPENSION + 0.6;
    expand_vert_de = VEHICLE_WID / 2 + EXPAND_SLOTOBJ_DE_VERT;
    expand_vert_ef = expand_vert_ab;

    if (slot.is_vision_slot == 1 && uradar.FCL < 2000 && uradar.FCR < 2000)
    {
        float rtheta = Project_PosTo1st_rtheta(stpoint, &slot.targpos.x);

        if (fabs(rtheta) > PI * 0.4 && fabs(rtheta) < PI * 0.6)
        {
            expand_vert_bc = VEHICLE_WID / 2 + 3 * EXPAND_SLOTOBJ_CB_VERT;
        }
    }

    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        left_limit  = rte_min(uradar.FSL, uradar.RSL);
        right_limit = rte_min(uradar.FSR, uradar.RSR);
    }
    else
    {
        left_limit  = rte_min(uradar.FSL, uradar.FOL);
        right_limit = rte_min(uradar.FSR, uradar.ROR);
    }

    // 平行车位左右两边有障碍物
    if (slot.is_vision_slot == 2 && slot.slotshap <= PK_SLOT_RIGHT_PARA &&
        rte_max(left_limit, right_limit) < 3000.0f)
    {
        return PATHPLAN_SUCCESS + 1;
    }

    if (slot.is_vision_slot == 2 && slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        if (slot.slotshap == PK_SLOT_RIGHT_PARA &&
            left_limit > 3000.0f) // 平行车位，入口在左边
        {
            front_limit = rte_min(uradar.FCL, uradar.FCR);
            front_limit = rte_min(front_limit, uradar.FOL);
            rear_limit  = rte_min(uradar.RCL, uradar.RCR);
            rear_limit  = rte_min(rear_limit, uradar.ROL);

            front_limit =
                rte_max((front_limit < 2000.0f ? front_limit : 3000.0f), 100.0f) / 1000;
            rear_limit =
                rte_max((rear_limit < 2000.0f ? rear_limit : 3000.0f), 100.0f) / 1000;
            right_limit = uradar.RSR / 1000.0f;

            expand_para_ab = VEHICLE_WID / 2 + 0.5;
            expand_para_bc = REAR_SUSPENSION + rte_min(rear_limit, expand_para_cd);
            expand_para_cd = VEHICLE_WID / 2 + rte_min(right_limit, expand_para_cd);
            expand_para_de =
                VEHICLE_LEN - REAR_SUSPENSION + rte_min(front_limit, expand_para_de);
            expand_para_ef = expand_para_ab;
        }
        else if (slot.slotshap == PK_SLOT_LEFT_PARA &&
                 right_limit > 3000.0f) // 平行车位，入口在右边
        {
            front_limit = rte_min(uradar.FCR, uradar.FCR);
            front_limit = rte_min(front_limit, uradar.FOR);
            rear_limit  = rte_min(uradar.RCR, uradar.RCL);
            rear_limit  = rte_min(rear_limit, uradar.ROR);

            front_limit =
                rte_max((front_limit < 2000.0f ? front_limit : 3000.0f), 100.0f) / 1000;
            rear_limit =
                rte_max((rear_limit < 2000.0f ? rear_limit : 3000.0f), 100.0f) / 1000;
            left_limit = uradar.RSL / 1000.0f;

            expand_para_ab = VEHICLE_WID / 2 + 0.5;
            expand_para_bc = REAR_SUSPENSION + rte_min(rear_limit, expand_para_cd);
            expand_para_cd = VEHICLE_WID / 2 + rte_min(left_limit, expand_para_cd);
            expand_para_de =
                VEHICLE_LEN - REAR_SUSPENSION + rte_min(front_limit, expand_para_de);
            expand_para_ef = expand_para_ab;
        }
        else
        {
            return PATHPLAN_SUCCESS + 1;
        }
    }

    if (slot.is_vision_slot == 2 && slot.slotshap > PK_SLOT_RIGHT_PARA)
    {
        if (slot.slotshap == PK_SLOT_RIGHT_VERT) // 垂直车位
        {
            left_limit  = rte_min(2000.0f, left_limit);
            right_limit = rte_min(2000.0f, right_limit);

            left_limit =
                rte_max((uradar.FOL < left_limit ? uradar.FOL : left_limit), 100.0f) /
                1000;
            right_limit =
                rte_max((uradar.FOR < right_limit ? uradar.FOR : right_limit), 100.0f) /
                1000;

            expand_vert_ab = VEHICLE_LEN - REAR_SUSPENSION + 0.5;
            expand_vert_bc = VEHICLE_WID / 2 + left_limit;
            expand_vert_cd = REAR_SUSPENSION + 0.6;
            expand_vert_de = VEHICLE_WID / 2 + right_limit;
            expand_vert_ef = expand_vert_ab;
        }
        else if (slot.slotshap == PK_SLOT_LEFT_VERT)
        {
            left_limit  = rte_min(2000.0f, left_limit);
            right_limit = rte_min(2000.0f, right_limit);

            left_limit =
                rte_max((uradar.FOL < left_limit ? uradar.FOL : left_limit), 100.0f) /
                1000;
            right_limit =
                rte_max((uradar.FOR < right_limit ? uradar.FOR : right_limit), 100.0f) /
                1000;

            expand_vert_ab = VEHICLE_LEN - REAR_SUSPENSION + 0.5;
            expand_vert_bc = VEHICLE_WID / 2 + right_limit;
            expand_vert_cd = REAR_SUSPENSION + 0.6;
            expand_vert_de = VEHICLE_WID / 2 + left_limit;
            expand_vert_ef = expand_vert_ab;
        }
    }

    if (slot.is_vision_slot == 0)
    {
        return PATHPLAN_SUCCESS;
    }

    float slotObj[5][4];
    SlotObj_Convert_float(slot.slotobj, slotObj);

    float finpoint[3];
    memcpy(finpoint, &slot.targpos, sizeof(finpoint));

    float diff[EGMAX] = {0, 0, 0, 0, 0};
    float offset      = 0.0;
    if (slot.slotshap == PK_SLOT_RIGHT_PARA || slot.slotshap == PK_SLOT_LEFT_PARA)
    {
        offset = PointToLine(slotObj[0], finpoint);
        if (offset > expand_para_ab)
        {
            diff[AB] = expand_para_ab - offset;
        }

        offset = PointToLine(slotObj[1], finpoint);
        if (offset < expand_para_bc)
        {
            diff[BC] = expand_para_bc - offset;
        }

        offset = PointToLine(slotObj[2], finpoint);
        if (offset < expand_para_cd)
        {
            diff[CD] = expand_para_cd - offset;
        }

        offset = PointToLine(slotObj[3], finpoint);
        if (offset < expand_para_de)
        {
            diff[DE] = expand_para_de - offset;
        }

        offset = PointToLine(slotObj[4], finpoint);
        if (offset > expand_para_ef)
        {
            diff[EF] = expand_para_ef - offset;
        }

        if (slot.slotshap == PK_SLOT_RIGHT_PARA)
        {
            finpoint[2] = Round_PI(finpoint[2] + PI_2);
        }
        else
        {
            finpoint[2] = Round_PI(finpoint[2] - PI_2);
        }

        UpdateSlotWithEdge(finpoint, diff, slot.slotobj);
    }
    else
    {
        offset = PointToLine(slotObj[0], finpoint);
        if (offset > expand_vert_ab)
        {
            diff[AB] = expand_vert_ab - offset;
        }

        offset = PointToLine(slotObj[1], finpoint);
        if (offset < expand_vert_bc)
        {
            diff[BC] = expand_vert_bc - offset;
        }

        offset = PointToLine(slotObj[2], finpoint);
        if (offset < expand_vert_cd)
        {
            diff[CD] = expand_vert_cd - offset;
        }

        offset = PointToLine(slotObj[3], finpoint);
        if (offset < expand_vert_de)
        {
            diff[DE] = expand_vert_de - offset;
        }

        offset = PointToLine(slotObj[4], finpoint);
        if (offset > expand_vert_ef)
        {
            diff[EF] = expand_vert_ef - offset;
        }

        UpdateSlotWithEdge(finpoint, diff, slot.slotobj);
    }

    return PATHPLAN_SUCCESS;
}

static int NormalizeRect(const float bcde[4][2], const float target[3], const float fact,
                         float avmRect[4])
{
    float rspoint[3];
    avmRect[0] = avmRect[1] = avmRect[2] = avmRect[3] = 0.0f;
    int slotValid                                     = 0;
    for (int i = 0; i < 4; i++)
    {
        RevConvert(target, bcde[i], rspoint);
        rspoint[0] = rspoint[0] * fact;
        rspoint[1] = rspoint[1] * fact;
        if (rspoint[0] < -0.001f && rspoint[1] < -0.001f)
        {
            avmRect[0] = rspoint[0];
            avmRect[1] = rspoint[1];
            slotValid  = slotValid | 0x01;
        }

        if (rspoint[0] > 0.001f && rspoint[1] > 0.001f)
        {
            avmRect[2] = rspoint[0];
            avmRect[3] = rspoint[1];
            slotValid  = slotValid | 0x02;
        }
    }

    if (slotValid != 3)
    {
        // usleep(10000);
        printf(
            "NormalizeRect Failed! bj:%04f,%04f,%04f,%04f,%04f,%04f,%04f,%04f "
            "target:%04f,%04f,%04f\n",
            bcde[0][0], bcde[0][1], bcde[1][0], bcde[1][1], bcde[2][0], bcde[2][1],
            bcde[3][0], bcde[3][1], target[0], target[1], target[2]);
        return 1;
    }

    return 0;
}

static int CheckObjs(const PlanDataCase &planData, const int objDirs[FS_OBJ_ARR_NUM],
                     const float objs[FS_OBJ_ARR_NUM][4])
{
    for (int i = 0; i < FS_OBJ_ARR_NUM; i++)
    {
        if (objDirs[i] == 0)
        {
            continue;
        }
        if (pow2(objs[i][0] - objs[i][2]) + pow2(objs[i][1] - objs[i][3]) < 1.5)
        {
            continue;
        }

        float rspoint[2][2];
        RevConvert(planData.finpoint, &objs[i][0], rspoint[0]);
        RevConvert(planData.finpoint, &objs[i][2], rspoint[1]);

        if (rspoint[0][0] < 0 && rspoint[1][0] < 0)
        {
            continue;
        }
        if (rspoint[0][0] > VEHICLE_LEN - REAR_SUSPENSION &&
            rspoint[1][0] > VEHICLE_LEN - REAR_SUSPENSION)
        {
            continue;
        }

        if (rspoint[0][1] * rspoint[1][1] < -0.0001)
        {
            return 0;
        }

        if (rte_min(fabs(rspoint[0][1]), fabs(rspoint[1][1])) < VEHICLE_WID / 4)
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief 根据车位附近的障碍物更新停车位边缘（可泊空间）。
 *
 * @param slot SlotInfo_T结构，包含停车位的详细信息。
 * @param FS_ObjDir 前方传感器检测到的障碍物方向数组。
 * @param FS_Obj 前方传感器检测到的障碍物信息数组，每个障碍物由一个四元组表示。
 * @param FS_ObjDir_Rear 后方传感器检测到的障碍物方向数组。
 * @param FS_Obj_Rear 后方传感器检测到的障碍物信息数组，每个障碍物由一个四元组表示。
 * @return
 * 成功时返回PATHPLAN_SUCCESS；否则，返回错误代码，其中高字节表示问题来源（前/后传感器），低字节表示障碍物索引。
 */
int UpdateSlotWithObsInSlot(SlotInfo_T &slot, int FS_ObjDir[FS_OBJ_ARR_NUM],
                            float FS_Obj[FS_OBJ_ARR_NUM][4],
                            int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                            float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    // 定义垂直和平行泊车位边界的限制
    const float vert_limits[2] = {float(VEHICLE_LEN - REAR_SUSPENSION + 1.0 + 1.0),
                                  float(-REAR_SUSPENSION * 0.7)};
    const float para_limits[2] = {float(VEHICLE_WID * 0.5f + 1.5f),
                                  -float(VEHICLE_WID * 0.5f + 0.6f)};

    // 初始化泊车位矩形和目标点
    float finpoint[3];
    memcpy(finpoint, &slot.targpos, sizeof(finpoint));

    // 确定泊车方向的偏移量和标志
    int offset    = 0; //(PathPlan_SlotDir(slot.slotshap) == 2) ? SF_OBJ_NUM : 0;
    int rightFlag = (PathPlan_SlotDir(slot.slotshap) == 2) ? 1 : -1;

    // 如果是平行停车位，调整目标点的朝向
    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        finpoint[2] = Round_PI(finpoint[2] + rightFlag * PI_2);
    }

    float distToRect[3] = {0.0, 0.0, 0.0};
    if (slot.is_vision_slot != 0)
    {
        float rspoint[2][2];
        RevConvert(finpoint, (float *)&slot.avm_point.near_rear, rspoint[0]);
        RevConvert(finpoint, (float *)&slot.avm_point.near_front, rspoint[1]);
        distToRect[0] = 0.5f * (rspoint[0][0] + rspoint[1][0]) + 0.1f; // TO BE
        distToRect[1] = rspoint[0][1];                                 // TO BC
        distToRect[2] = rspoint[1][1];                                 // TO DE
    }

    // 初始化边缘偏移数组
    float edgeOffset[EGMAX] = {0, 0, 0, 0, 0};

    float curpos[4];
    RTE_PK_Location_Get_CurPos(curpos);

    float targety = Project_PosTo1st_ry(curpos, finpoint);
    // 处理前后传感器检测到的障碍物
    for (int j = 0; j < 2; j++)
    {
        for (int i = 0; i < FS_OBJ_ARR_NUM; i++)
        {
            int objDir = 0;
            LineSeg_T seg;

            // 获取障碍物方向
            objDir = (j == 0) ? FS_ObjDir[i + offset] : FS_ObjDir_Rear[i + offset];
            if (objDir == 0)
            {
                continue;
            }

            // 获取障碍物信息
            if (j == 0)
            {
                memcpy(&seg, &FS_Obj[i + offset], sizeof(seg));
            }
            else
            {
                memcpy(&seg, &FS_Obj_Rear[i + offset], sizeof(seg));
            }

            float segLen = Get_Segment_Len(seg);
            if (segLen < PathPlanConstCfg::valid_obj_len)
            {
                if (j == 0)
                {
                    FS_ObjDir[i + offset] = 0;
                }
                else
                {
                    FS_ObjDir_Rear[i + offset] = 0;
                }
                continue;
            }

            if (slot.is_vision_slot == 0 && segLen < 1.3 *
                                                         PathPlanConstCfg::valid_obj_len /*&& PK_PosLineSegCross_Check(slot.targpos,1,&seg,0.6f) != 0*/) // HAVECHANGE Tangsj 2023/6/26
            {
                if (j == 0)
                {
                    FS_ObjDir[i + offset] = 0;
                }
                else
                {
                    FS_ObjDir_Rear[i + offset] = 0;
                }
                continue;
            }

            float disty1 = Project_PosTo1st_ry(curpos, (float *)&seg.pt1);
            float disty2 = Project_PosTo1st_ry(curpos, (float *)&seg.pt2);
            if (sign(disty1) * sign(disty2) < ZERO_FLOAT)
            {
                continue;
            }

            if (sign(disty1) * sign(targety) < ZERO_FLOAT)
            {
                continue;
            }

            // 视觉停车位时，检查障碍物是否与停车位矩形相交
            if (slot.is_vision_slot != 0)
            {
                float linePoint[4];
                RevConvert(finpoint, (float *)&seg.pt1, &linePoint[0]);
                RevConvert(finpoint, (float *)&seg.pt2, &linePoint[2]);

                // 超出车位线
                if (sign(linePoint[1]) * sign(linePoint[3]) > ZERO_FLOAT &&
                    slot.slotshap > PK_SLOT_RIGHT_PARA)
                {
                    if (fabs(linePoint[0]) > distToRect[0])
                    {
                        if (fabs(linePoint[1]) < VEHICLE_WID * 0.5 + 0.75f)
                        {
                            linePoint[0] = distToRect[0];
                        }
                        else
                        {
                            linePoint[0] = distToRect[0] - 0.8;
                        }

                        Convert(finpoint, &linePoint[0], (float *)&seg.pt1);
                    }

                    if (fabs(linePoint[2]) > distToRect[0])
                    {
                        if (fabs(linePoint[1]) < VEHICLE_WID * 0.5 + 0.75f)
                        {
                            linePoint[2] = distToRect[0];
                        }
                        else
                        {
                            linePoint[2] = distToRect[0] - 0.8;
                        }

                        Convert(finpoint, &linePoint[2], (float *)&seg.pt2);
                    }
                }
            }

            // 根据泊车类型更新停车位边缘信息
            int result = PATHPLAN_SUCCESS;
            if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
            {
                result = UpdateSlotEdge(slot.slot_index, rightFlag, seg, slot.slotobj,
                                        slot.slotshap, finpoint, para_limits, edgeOffset,
                                        &FS_ObjDir[i + offset]);
            }
            else
            {
                result = UpdateSlotEdge(slot.slot_index, rightFlag, seg, slot.slotobj,
                                        slot.slotshap, finpoint, vert_limits, edgeOffset,
                                        &FS_ObjDir[i + offset]);
            }

            // 如果更新失败，返回错误
            if (result != PATHPLAN_SUCCESS)
            {
                return (2 + j) * 256 + (i + 1);
            }
        }
    }

    // 处理停车位边缘调整和验证
    if (slot.slotshap <= PK_SLOT_RIGHT_PARA)
    {
        // 构建停车栏杆并更新停车位边缘
        LineSeg_T parkBars[2];
        int counter = PathPlan_BuildObjsByStopper(slot, parkBars);
        if (counter == 1)
        {
            UpdateSlotEdge(slot.slot_index, rightFlag, parkBars[0], slot.slotobj,
                           slot.slotshap, finpoint, para_limits, edgeOffset);
        }
        else if (counter == 2)
        {
            UpdateSlotEdge(slot.slot_index, rightFlag, parkBars[0], slot.slotobj,
                           slot.slotshap, finpoint, para_limits, edgeOffset);
            UpdateSlotEdge(slot.slot_index, rightFlag, parkBars[1], slot.slotobj,
                           slot.slotshap, finpoint, para_limits, edgeOffset);
        }

        // 调整边缘偏移方向
        if (edgeOffset[BC] > ZERO_FLOAT)
        {
            edgeOffset[BC] = -edgeOffset[BC];
        }
        if (edgeOffset[CD] > ZERO_FLOAT)
        {
            edgeOffset[CD] = -edgeOffset[CD];
        }
        if (edgeOffset[DE] > ZERO_FLOAT)
        {
            edgeOffset[DE] = -edgeOffset[DE];
        }

        // 更新停车位信息并验证停车位长度
        UpdateSlotWithEdge(finpoint, edgeOffset, slot.slotobj);
        float slotlen = Cal_Dis_Pt2Pt(slot.slotobj.ptC, slot.slotobj.ptD);
        if (slotlen < VEHICLE_LEN + PathPlanConstCfg::short_path_len)
        {
            printf("UpdateSlotWithObsInSlot para slotId:%d len:%f\n", slot.slot_index,
                   slotlen);
            return 3;
        }
    }
    else
    {
        // 处理垂直停车位边缘调整和验证
        if (edgeOffset[BC] > ZERO_FLOAT)
        {
            edgeOffset[BC] = -edgeOffset[BC];
        }
        if (edgeOffset[CD] > ZERO_FLOAT)
        {
            edgeOffset[CD] = -edgeOffset[CD];
        }
        if (edgeOffset[DE] > ZERO_FLOAT)
        {
            edgeOffset[DE] = -edgeOffset[DE];
        }
        UpdateSlotWithEdge(finpoint, edgeOffset, slot.slotobj);
        float slotlen = Cal_Dis_Pt2Pt(slot.slotobj.ptC, slot.slotobj.ptD);
        if (slotlen < VEHICLE_WID + g_local_swell)
        {
            printf("UpdateSlotWithObsInSlot vert slotId:%d wide:%f\n", slot.slot_index,
                   slotlen);
            return 3;
        }
    }

    // 返回成功
    return PATHPLAN_SUCCESS;
}

int PathPlan_UpdateParaSlotCDWithObs(PlanDataCase &PlanData,
                                     int FS_ObjDir[FS_OBJ_ARR_NUM],
                                     float FS_Obj[FS_OBJ_ARR_NUM][4],
                                     int FS_ObjDir_Rear[FS_OBJ_ARR_NUM],
                                     float FS_Obj_Rear[FS_OBJ_ARR_NUM][4])
{
    float baseC[3];
    baseC[0] = PlanData.obj_slot[2][0];
    baseC[1] = PlanData.obj_slot[2][1];
    baseC[2] = PlanData.finpoint[2];

    LineSeg_T line;
    memcpy(&line, &PlanData.obj_slot[2][0], sizeof(line));
    float distCD      = Get_Segment_Len(line);
    float distCDCheck = rte_min(distCD, VEHICLE_LEN + 3 * g_local_swell);

    float minDist = (PlanData.left_fac < -0.01) ? -g_local_swell : g_local_swell;
    int offset    = (PlanData.left_fac > ZERO_FLOAT) ? 0 : SF_OBJ_NUM;

    for (int dir = 0; dir < 2; dir++)
    {
        int *objdir    = (dir == 0) ? FS_ObjDir : FS_ObjDir_Rear;
        float *objdata = (dir == 0) ? &FS_Obj[0][0] : &FS_Obj_Rear[0][0];

        for (int i = 0; i < SF_OBJ_NUM; i++)
        {
            if (objdir[i + offset] == 0)
            {
                break;
            }

            memcpy(&line, &objdata[i * 4], sizeof(line));
            if (Get_Segment_Len(line) < PathPlanConstCfg::valid_obj_len)
            {
                continue;
            }

            float rspoint[2][2];
            RevConvert(baseC, &objdata[(i + offset) * 4], rspoint[0]);
            RevConvert(baseC, &objdata[(i + offset) * 4 + 2], rspoint[1]);

            if (rspoint[0][0] <= -0.01 && rspoint[1][0] <= -0.01)
            {
                continue;
            }

            if (rspoint[0][0] >= distCDCheck && rspoint[1][0] >= distCDCheck)
            {
                continue;
            }

            if (PlanData.left_fac < -0.01)
            {
                float offset = rte_max(rspoint[0][1], rspoint[1][1]);
                if (fabs(offset) > VEHICLE_WID / 2)
                {
                    continue;
                }
                minDist = rte_max(offset, minDist);
            }
            else
            {
                float offset = rte_min(rspoint[0][1], rspoint[1][1]);
                if (fabs(offset) > VEHICLE_WID / 2)
                {
                    continue;
                }
                minDist = rte_min(offset, minDist);
            }
        }
    }

    SlotObj_T slotobj;
    SlotObj_Convert_struct(&slotobj, PlanData.obj_slot);

    float newPoint[2];
    newPoint[0] = 0;
    newPoint[1] = minDist;
    Convert(baseC, newPoint, (float *)&slotobj.ptC);

    newPoint[0] = distCD;
    newPoint[1] = minDist;
    Convert(baseC, newPoint, (float *)&slotobj.ptD);

    float ry = Project_PosTo1st_ry(PlanData.finpoint, (float *)&slotobj.ptC);
    if (fabs(ry) < VEHICLE_WID / 2)
    {
        return -2;
    }

    SlotObj_Convert_float(slotobj, PlanData.obj_slot);

    return 1;
}

// 0917 check all available paths whether their traj's stpos is curpos
uint8_t Check_TrajStartPos_Is_CurPos(MultiPlanInfo &multiPlans, float curPos[4],
                                     uint8_t IsTrajStPos[MAX_PARKED_SLOTS])
{
    int i;
    float dist, dtheta;
    int CoincideFlag = 1;
    VehPos_T TrajStPos, CurVehPos;
    memcpy(&CurVehPos, curPos, sizeof(CurVehPos));
    for (i = 0; i < MAX_PARKED_SLOTS; i++)
    {
        IsTrajStPos[i] = 1;
    }
    for (i = 0; i < multiPlans.slotNum; i++)
    {
        dtheta = Round_PI(multiPlans.slotPlans[i].Act_traj[0][2] - curPos[2]);
        if (fabsf(dtheta) > 0.08f)
        {
            IsTrajStPos[i] = 0;
            CoincideFlag   = 0;
        }
        else
        {
            memcpy(&TrajStPos, multiPlans.slotPlans[i].Act_traj[0], sizeof(TrajStPos));
            dist = Cal_PowDist_Pos2Pos(TrajStPos, CurVehPos);
            if (dist > 0.01f) // 10cm
            {
                IsTrajStPos[i] = 0;
                CoincideFlag   = 0;
            }
        }
    }
    return CoincideFlag;
}

int PK_Check_Path(const int Path_Num, const float Path[][5], const float stpoint[3],
                  const float finpoint[3], const int obj_num, const float Envi_obj[][4],
                  const float swell)
{
    int i, Path_outliner   = 0;
    float EndPos[3], x_div = 0, y_div = 0;

    if (Path_Num < 1)
    {
        return -1;
    }

    float tempswell = rte_max(swell, 0.00); //  0930: single swell to both side swell
    // printf("PK_Check_Path %f\n", swell);

    x_div = fabsf(Path[0][0] - stpoint[0]);
    y_div = fabsf(Path[0][1] - stpoint[1]);

    if (x_div > PathPlanConstCfg::div_err || y_div > PathPlanConstCfg::div_err)
    {
        Path_outliner = (Path_outliner + 1) << 16;
        ASSERT_INFO((Path_outliner > 0), "PK_Check_Path");
    }

    if (Path_outliner > 0)
    {
        return Path_outliner;
    }

    int result = 0;
    for (i = 0; i < Path_Num; i++)
    {
        result = PathPlan_PathVerify2(Path[i], obj_num, Envi_obj, tempswell);

        if (result != PATHPLAN_SUCCESS)
        {
            Path_outliner = ((i + 1) << 16) | (result + 1);
            break;
        }

        PK_Get_Path_EndPos(Path[i], EndPos);
        int checkDiv = 0;
        if (i < (Path_Num - 1))
        {
            checkDiv = PathPlan_CheckPosDiv(EndPos, Path[i + 1]);
        }
        else
        {
            checkDiv = PathPlan_CheckPosDiv(EndPos, finpoint);
        }

        if (checkDiv != 0)
        {
            Path_outliner = (i + 1) << 16 | (checkDiv << 8);
            break;
        }
    }

    return Path_outliner;
}

int PK_Check_Path_simple(int Path_Num, float Path[][5], int obj_num, float Envi_obj[][4],
                         float swell, float stpoint[3], float finpoint[3],
                         int *IsNarrowSlot)
{
    int i, Path_outliner = 0;

    if (Path_Num < 1)
    {
        return -1;
    }

    for (i = 0; i < Path_Num; i++)
    {
        Path[i][0] = Path[i][0] - stpoint[0];
        Path[i][1] = Path[i][1] - stpoint[1];
    }

    for (i = 0; i < obj_num; i++)
    {
        Envi_obj[i][0] = Envi_obj[i][0] - stpoint[0];
        Envi_obj[i][1] = Envi_obj[i][1] - stpoint[1];
        Envi_obj[i][2] = Envi_obj[i][2] - stpoint[0];
        Envi_obj[i][3] = Envi_obj[i][3] - stpoint[1];
    }

    finpoint[0] = finpoint[0] - stpoint[0];
    finpoint[1] = finpoint[1] - stpoint[1];

    int result = 0;
    for (i = 0; i < Path_Num; i++)
    {
        result = PathPlan_PathVerify2(Path[i], obj_num, Envi_obj, swell);

        if (result != PATHPLAN_SUCCESS)
        {
            Path_outliner = Path_outliner + 1;
            if (Path[i][3] > 0 && i > 0 && i <= 3) // ds > 0
            {
                *IsNarrowSlot = 1;
            }
            break;
        }
    }

    for (i = 0; i < Path_Num; i++)
    {
        Path[i][0] = Path[i][0] + stpoint[0];
        Path[i][1] = Path[i][1] + stpoint[1];
    }

    for (i = 0; i < obj_num; i++)
    {
        Envi_obj[i][0] = Envi_obj[i][0] + stpoint[0];
        Envi_obj[i][1] = Envi_obj[i][1] + stpoint[1];
        Envi_obj[i][2] = Envi_obj[i][2] + stpoint[0];
        Envi_obj[i][3] = Envi_obj[i][3] + stpoint[1];
    }

    finpoint[0] = finpoint[0] + stpoint[0];
    finpoint[1] = finpoint[1] + stpoint[1];

    return (Path_outliner == 0) ? PATHPLAN_SUCCESS : (i + 1);
}

/*********************************************************************
Function description : PathPlan_DrawTrajectory-->> to discrete the path into many vehicle
postures

Calibration state    : NO -->>
Edition              : 2016/3/6  1.0 : based on the geometry calculation
                     : 2018/1/16 1.1 : add start ds
Input list:
dsmin: the minimum step distance
traject: the input path
Position_arr: the posture array on the path
Output list:
return:  points num
**********************************************************************/
int PathPlan_DrawTrajectory(const float dsmin, const float traject[5],
                            float Position_arr[30][4], const float start_ds)
{
    int i;
    float step_th, start_th;
    float Xr, Yr, ftheta;
    float x0, y0, theta0, r, ds;
    int points = 1;
    float floor_ds, remian_ds;
    float cos_theta0, sin_theta0, cos_ftheta, sin_ftheta; //  avoid repeat calculation

    x0     = traject[0];
    y0     = traject[1];
    theta0 = traject[2];
    r      = traject[4];
    ds     = traject[3] - start_ds;

    float dsminv = dsmin;
    floor_ds     = (float)(int)(fabsf(ds / dsminv));
    remian_ds    = fabsf(ds) - fabsf(floor_ds * dsminv);

    if (fabsf(remian_ds) < 0.005f)
    {
        points = (int)floor_ds;
    }
    else
    {
        points = 1 + (int)floor_ds;
    }

    if (fabsf(ds) <= fabsf(dsminv))
    {
        points = 1;
    }

    if (points > 30)
    {
        points = 30;
        dsminv = fabsf(ds / 30.0f);
    }

    ftheta     = -sign(r) * PI / 2 + theta0;
    cos_theta0 = cosf(theta0);
    sin_theta0 = sinf(theta0);
    cos_ftheta = cosf(ftheta);
    sin_ftheta = sinf(ftheta);
    if (fabsf(r) < 1) // line state
    {
        for (i = 0; i < points - 1; i++)
        {
            Position_arr[i][0] =
                (i + 1) * cos_theta0 * (dsminv + start_ds) * sign(ds) + x0;
            Position_arr[i][1] =
                (i + 1) * sin_theta0 * (dsminv + start_ds) * sign(ds) + y0;
            Position_arr[i][2] = theta0;
            // Position_arr[i][3]=(i+1)*dsmin*sign(ds)+start_ds;//big_bug fix 2018.4
            // (i+1)*(dsmin+ds)*sign(ds)
            Position_arr[i][3] =
                (i + 1) * (dsminv + start_ds) * sign(ds); // CSJ:  bug fix 2018.8.9
        }

        Position_arr[points - 1][0] = cos_theta0 * ds + x0;
        Position_arr[points - 1][1] = sin_theta0 * ds + y0;
        Position_arr[points - 1][2] = theta0;
        Position_arr[points - 1][3] = ds;
    }
    else
    {
        // dth>0 left
        Xr       = x0 - fabsf(r) * cos_ftheta;
        Yr       = y0 - fabsf(r) * sin_ftheta;
        step_th  = fabsf(dsminv / r) * sign(ds * r);
        start_th = fabsf(start_ds / r) * sign(ds * r);
        for (i = 0; i < points - 1; i++)
        {
            Position_arr[i][0] =
                cosf((i + 1) * step_th + start_th + ftheta) * fabsf(r) + Xr;
            Position_arr[i][1] =
                sinf((i + 1) * step_th + start_th + ftheta) * fabsf(r) + Yr;
            Position_arr[i][2] = (i + 1) * step_th + start_th + theta0;
            Position_arr[i][3] = (i + 1) * dsminv * sign(ds) + start_ds;
        }

        Position_arr[points - 1][0] = cosf((ds + start_ds) / r + ftheta) * fabsf(r) + Xr;
        Position_arr[points - 1][1] = sinf((ds + start_ds) / r + ftheta) * fabsf(r) + Yr;
        Position_arr[points - 1][2] = theta0 + (ds + start_ds) / r;
        Position_arr[points - 1][3] = ds + start_ds;
    }
    return points;
}

/*
add 2023/4/10 Tangsj
功能描述：设置车辆限制线段
函数名称：Get_Limit
INTPUT:     限制基准点point,限制线段距基准点距离limit_dist，限制线段长度limit_len
OUTPUT:     车辆限制线段limit
*/
void Get_Limit(float point[3], float limit[4], float limit_dist, float limit_len)
{
    float tem_limit[4] = {limit_dist, limit_len / 2, limit_dist, -limit_len / 2};

    Convert(point, tem_limit, limit);
    Convert(point, &tem_limit[2], &limit[2]);
}

float PathPlan_LandMarkByStopper(float path[5], const float obj[][2])
{
    // add Tangsj 2023/7/6 --后轮线段
    float ds_final, path_dist, endPos[3], whell_limit[4];
    Point_T Stopper_A, Stopper_B, Whell_A, Whell_B;

    path_dist = path[3];
    path[3]   = sign(path_dist) * 0.05;

    while (fabs(path_dist) >= fabs(path[3]))
    {
        PK_Get_Path_EndPos(path, endPos);
        Get_Limit(endPos, whell_limit, -(WHELL_RADIUS - 0.01), VEHICLE_WID / 2);
        memcpy(&Stopper_A, obj[0], sizeof(Stopper_A));
        memcpy(&Stopper_B, &obj[0][2], sizeof(Stopper_B));
        memcpy(&Whell_A, whell_limit, sizeof(Whell_A));
        memcpy(&Whell_B, &whell_limit[2], sizeof(Whell_B));

        if (Is_TwoSegment_Cross(Stopper_A, Stopper_B, Whell_A, Whell_B) == 1)
        {
            ds_final = path[3] - sign(path_dist) * 0.05;
            return ds_final;
        }
        if (path[3] == path_dist)
        {
            ds_final = path[3];
            return ds_final;
        }

        path[3] += sign(path_dist) * 0.05;
    }

    path[3] = path_dist;
    PK_Get_Path_EndPos(path, endPos);
    Get_Limit(endPos, whell_limit, -(WHELL_RADIUS - 0.01), VEHICLE_WID / 2);
    memcpy(&Stopper_A, obj[0], sizeof(Stopper_A));
    memcpy(&Stopper_B, &obj[0][2], sizeof(Stopper_B));
    memcpy(&Whell_A, whell_limit, sizeof(Whell_A));
    memcpy(&Whell_B, &whell_limit[2], sizeof(Whell_B));

    if (Is_TwoSegment_Cross(Stopper_A, Stopper_B, Whell_A, Whell_B) == 1)
    {
        ds_final = path[3] - sign(path_dist) * 0.05;
        return ds_final;
    }

    ds_final = path[3];
    return ds_final;
}

// 获取线段斜率（弧度）
float Get_LineSegment_Theta(const float p0[2], const float p1[2])
{
    float theta = atan2f(p1[1] - p0[1], p1[0] - p0[0]);
    return Round_PI(theta);
}

/*
功能描述：将线段延长或缩短到指定长度
函数名称：Extend_LineSegment
INTPUT:     线段起点p0,线段终点p1，指定长度line_len
OUTPUT:     线段终点p1
RETURN:     线段斜率（rad）
*/
float Extend_LineSegment(const float p0[2], float p1[2], float line_len)
{
    float theta = Get_LineSegment_Theta(p0, p1);
    if (line_len == 0)
    {
        return theta;
    }

    float len       = Cal_Dis_Pt2Pt(p0, p1);
    float extend_ds = sign(line_len) * (fabs(line_len) - sign(line_len) * len);

    // 计算新的端点坐标
    p1[0] = p1[0] + extend_ds * cosf(theta);
    p1[1] = p1[1] + extend_ds * sinf(theta);

    return theta;
}

void PathPlan_UpdateSideDist(const PlanDataCase planData[1], const SlotInfo_T &slot,
                             float avmDist[3])
{
    float bc[4], de[4], cd[4];
    if (planData[1].is_vision_slot == 0)
    {
        memcpy(bc, planData[0].obj_slot[1], sizeof(Point_T));
        memcpy(cd, planData[0].obj_slot[2], sizeof(Point_T));
        memcpy(de, planData[0].obj_slot[3], sizeof(Point_T));
    }
    else
    {
        memcpy(&bc[0], (float *)&slot.avm_point.near_rear, sizeof(Point_T));
        memcpy(&bc[2], (float *)&slot.avm_point.far_rear, sizeof(Point_T));
        memcpy(&cd[0], (float *)&slot.avm_point.far_rear, sizeof(Point_T));
        memcpy(&cd[2], (float *)&slot.avm_point.far_front, sizeof(Point_T));
        memcpy(&de[0], (float *)&slot.avm_point.far_front, sizeof(Point_T));
        memcpy(&de[2], (float *)&slot.avm_point.near_front, sizeof(Point_T));
    }

    avmDist[0] = PK_PointToLineDist(bc, planData[0].finpoint);
    avmDist[1] = PK_PointToLineDist(cd, planData[0].finpoint);
    avmDist[2] = PK_PointToLineDist(de, planData[0].finpoint);
}

/**
 * @brief 根据膨胀量调整障碍物的边界，适应车辆与停车位形状和方向。
 *
 * @param swell 膨胀量，用于调整障碍物的边界。
 * @param stPos 起始位置（参考点），用于计算障碍物相对于车辆的方位。
 * @param finPos 停车终点位置，用于判断障碍物方向和位置关系。
 * @param obj_num 障碍物的数量。
 * @param obj 障碍物边界数组，每个障碍物由 4 个浮点值定义（两个点的坐标）。
 * @param obj_dir
 * 障碍物的方向数组，定义每个障碍物相对于车辆或停车位的方向（1：车辆右侧，2：车辆左侧，3：平行车位底部）。
 * @param slotshap 停车位的类型（如平行车位或垂直车位）。
 *
 * @details
 * 根据停车位的方向（左侧或右侧）、障碍物的相对位置、障碍物与停车位的方向关系，动态调整障碍物的边界。
 */
void PathPlan_ExpandObjBySwell(const float swell, const float avmDist[3],
                               PlanDataCase planData[1])
{
    Relation_T dir0;
    float tempdist0;
    Point_T LinePt_A, LinePt_B, Pt_C;
    int slot_dir = PathPlan_SlotDir(planData[0].slotshape);

    memcpy(&Pt_C, planData[0].stpoint, sizeof(Point_T));
    Point_T bcdePoints[4];
    memcpy((float *)&bcdePoints[0], &planData[0].obj_slot[1][0], sizeof(Point_T));
    memcpy((float *)&bcdePoints[1], &planData[0].obj_slot[1][2], sizeof(Point_T));
    memcpy((float *)&bcdePoints[2], &planData[0].obj_slot[3][0], sizeof(Point_T));
    memcpy((float *)&bcdePoints[3], &planData[0].obj_slot[3][2], sizeof(Point_T));

    float bry = Project_PosTo1st_ry(planData[0].finpoint, &planData[0].obj_slot[1][0]);
    float ery = Project_PosTo1st_ry(planData[0].finpoint, &planData[0].obj_slot[4][0]);

    for (int i = 0; i < planData[0].obj_danger_num; i++)
    {
        int paraobj = 0;
        if (planData[0].obj_danger_dir[i] == OBJTYPE_NONE)
        {
            ASSERT(false);
            continue;
        }
        if (planData[0].obj_danger_dir[i] == OBJTYPE_BOTTOM)
        {
            paraobj = 1;
        }

        memcpy(&LinePt_A, &planData[0].obj_danger[i][0], sizeof(Point_T));
        memcpy(&LinePt_B, &planData[0].obj_danger[i][2], sizeof(Point_T));

        Get_Dist_Dir_Pt2PointLine(LinePt_A, LinePt_B, Pt_C, &tempdist0, &dir0);

        if (sign(Project_PosTo1st_ry(planData[0].stpoint,
                                     &planData[0].obj_danger[i][0])) ==
            sign(Project_PosTo1st_ry(planData[0].stpoint, planData[0].finpoint)))
        {
            planData[0].obj_danger_dir[i] =
                (slot_dir == OBJTYPE_LEFT) ? OBJTYPE_LEFT : OBJTYPE_RIGHT;
        }
        else
        {
            planData[0].obj_danger_dir[i] =
                (slot_dir == OBJTYPE_LEFT) ? OBJTYPE_RIGHT : OBJTYPE_LEFT;
        }

        // int is_extend = (slot_dir != obj_dir[i]) ? 1 : 0;//车位对向障碍物延长

        int dir = (planData[0].obj_danger_dir[i] == 1) ? -1 : 1;

        float rx1 =
            Project_PosTo1st_rx(planData[0].stpoint, &planData[0].obj_danger[i][0]);
        float ry1 =
            Project_PosTo1st_ry(planData[0].stpoint, &planData[0].obj_danger[i][0]);
        float rx2 =
            Project_PosTo1st_rx(planData[0].stpoint, &planData[0].obj_danger[i][2]);
        float ry2 =
            Project_PosTo1st_ry(planData[0].stpoint, &planData[0].obj_danger[i][2]);

        if ((dir0 == PK_ON_LEFT &&
             ((((rx1 > 0 && fabs(ry1) < VEHICLE_WID / 2 && fabs(ry1) < fabs(ry2)) ||
                (rx2 < 0 && fabs(ry2) < VEHICLE_WID / 2 && fabs(ry1) > fabs(ry2))) &&
               sign(ry1) == sign(ry2)) ||
              (sign(ry1) != sign(ry2) && ry1 > 0))) ||
            (dir0 == PK_ON_RIGHT &&
             ((((rx2 < 0 && fabs(ry2) < VEHICLE_WID / 2 && fabs(ry1) > fabs(ry2)) ||
                (rx1 > 0 && fabs(ry1) < VEHICLE_WID / 2 && fabs(ry1) < fabs(ry2))) &&
               sign(ry1) == sign(ry2)) ||
              (sign(ry1) != sign(ry2) && ry1 < 0))))
        {
            dir = -dir;
            // is_extend = 1;//车正前方、正后方障碍物
        }

        float angle    = Get_LineSegment_Theta(&planData[0].obj_danger[i][0],
                                               &planData[0].obj_danger[i][2]);
        float objSwell = swell;
        if (planData[0].is_vision_slot != 0 && paraobj != 1 &&
            (planData[0].slotshape == PK_SLOT_LEFT_VERT ||
             planData[0].slotshape == PK_SLOT_RIGHT_VERT))
        {
            Point_T objPoint[2];
            float objRs[2][2];

            memcpy((float *)&objPoint[0], &planData[0].obj_danger[i][0], sizeof(Point_T));
            memcpy((float *)&objPoint[1], &planData[0].obj_danger[i][2], sizeof(Point_T));

            int bcCross = Is_TwoSegment_Cross(bcdePoints[0], bcdePoints[1], objPoint[0],
                                              objPoint[1]);
            int deCross = Is_TwoSegment_Cross(bcdePoints[2], bcdePoints[3], objPoint[0],
                                              objPoint[1]);
            // avmDist[0] BC距离  avmDist[2] DE距离
            if (bcCross != 0) // 和BC或者DE相交
            {
                RevConvert(planData[0].finpoint, &planData[0].obj_danger[i][0], objRs[0]);
                RevConvert(planData[0].finpoint, &planData[0].obj_danger[i][2], objRs[1]);
                PathPlan_ClipLineSegmenty(objRs[0], objRs[1],
                                          (avmDist[0] - 0.05f) * sign(bry),
                                          -sign(bry) * 1.0f);
                objSwell = 0.0f;
                Convert(planData[0].finpoint, objRs[0], &planData[0].obj_danger[i][0]);
                Convert(planData[0].finpoint, objRs[1], &planData[0].obj_danger[i][2]);
            }
            else if (deCross != 0)
            {
                RevConvert(planData[0].finpoint, &planData[0].obj_danger[i][0], objRs[0]);
                RevConvert(planData[0].finpoint, &planData[0].obj_danger[i][2], objRs[1]);
                PathPlan_ClipLineSegmenty(objRs[0], objRs[1],
                                          (avmDist[2] - 0.05f) * sign(ery),
                                          -sign(ery) * 1.0f);
                objSwell = 0.0f;
                Convert(planData[0].finpoint, objRs[0], &planData[0].obj_danger[i][0]);
                Convert(planData[0].finpoint, objRs[1], &planData[0].obj_danger[i][2]);
            }
        }

        //      if( is_extend )//车位对向障碍物及车正前方、正后方障碍物延长处理
        //      {
        planData[0].obj_danger[i][0] =
            planData[0].obj_danger[i][0] - objSwell * cosf(angle);
        planData[0].obj_danger[i][1] =
            planData[0].obj_danger[i][1] - objSwell * sin(angle);

        planData[0].obj_danger[i][2] =
            planData[0].obj_danger[i][2] + objSwell * cosf(angle);
        planData[0].obj_danger[i][3] =
            planData[0].obj_danger[i][3] + objSwell * sin(angle);
        //      }

        if (paraobj == 1)
        {
            planData[0].obj_danger[i][0] = planData[0].obj_danger[i][0] +
                                           cos(angle + dir * PI / 2) * (objSwell + 0.05);
            planData[0].obj_danger[i][1] = planData[0].obj_danger[i][1] +
                                           sin(angle + dir * PI / 2) * (objSwell + 0.05);

            planData[0].obj_danger[i][2] = planData[0].obj_danger[i][2] +
                                           cos(angle + dir * PI / 2) * (objSwell + 0.05);
            planData[0].obj_danger[i][3] = planData[0].obj_danger[i][3] +
                                           sin(angle + dir * PI / 2) * (objSwell + 0.05);
        }
        else
        {
            planData[0].obj_danger[i][0] =
                planData[0].obj_danger[i][0] + cos(angle + dir * PI / 2) * objSwell;
            planData[0].obj_danger[i][1] =
                planData[0].obj_danger[i][1] + sin(angle + dir * PI / 2) * objSwell;

            planData[0].obj_danger[i][2] =
                planData[0].obj_danger[i][2] + cos(angle + dir * PI / 2) * objSwell;
            planData[0].obj_danger[i][3] =
                planData[0].obj_danger[i][3] + sin(angle + dir * PI / 2) * objSwell;
        }
    }
}
