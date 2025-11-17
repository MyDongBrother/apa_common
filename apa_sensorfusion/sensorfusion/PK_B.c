#include "PK_BP.h"
#include "Rte_Func.h"
#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathFunc.h"
#include "PK_SF_Subfun.h"
#include "Rte.h"
#include "SystemPara.h"
#include "Record_Buff.h"
#include "hobotlog/hobotlog.hpp"
#include "Rte_ComIF.h"

/*********************************************************************************************************
**Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */

#if ((PK_B_SW_MAJOR_VERSION != (0u)) || (PK_B_SW_MINOR_VERSION != (0u)) || \
     (PK_B_SW_PATCH_VERSION != (35U)))
#error "Version numbers of PK_B.c and PK_B.h are inconsistent!"
#endif

#pragma section all "CPU1.Private"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copy From A Process
#define A_LINE_LEFT  50000 // l
#define A_LINE_RIGHT 60000 // r
// B Process, Out of Slot Area
#define B_OUT_LEFT  10000 // l
#define B_OUT_RIHGT 20000 // r
// B Process, Vertical Park in Slot Area
#define B_CHECK_LEFT  10001 // l
#define B_CHECK_RIHGT 20001 // r
// B Process, Parallel Park in Slot Area
#define B_LINE_FRONT 30001 // front
#define B_LINE_REAR  40001 // rear

#define VEHICLE_MOVE_DIST 0.02f // vehicle move mini distance
// For B1 Function
#define STAT_SIZE          40
#define CENT_POINT_THR     15     // 30
#define CENT_POINT_PERCENT 0.3f   // 0.35f
#define OUT_POINT_THR      15     // 20
#define OUT_POINT_PERCENT  0.2f   // 0.15f
#define MAX_REAR_DIST      (0.9f) // Parallel Park Rear distance thresh

// For B2 Function
#define FIT_STEP      10     // Fit new line use points num
#define MOVE_STEP     2      // Every MOVE_STEP points, start calculate
#define SIMILAR       0.965f // 15deg Use cos value, Get two lines similar
#define KERNEL_RADIUS 6      // Median filter kernel radius, real size 2*KERNEL_RADIUS+1
#define SET_SAVE_SIZE 120    // num of radar sets size
#define RADAR_NUM     20     // num of original radar value
#define ADJUSTANGLE   25     // For Vertical park,vehicle and tarpos angle thresh
#define CHECK_SIDE_RANGE \
    1.5f // PointCheck side distance,chanage 0.9f to 1.5f,2018/11/14,change it for
         // pathplan
#define CHECK_FR_RANGE 2.3f // for parallel park, front and rear move thresh
// #define B_UPDATE_DIST_THRS 0.5f// Park in slot, But no radar value, increase distance
#define B_EXPAND_RESERVE_LENS \
    0.5f // move in a deep length, radar have no value, slot expand a reserve length
#define B_UPDATE_RADIAN_THRS \
    0.0436f // radian difference for B process, AVM park corner update use thresh
#define VEHICLE_REAR_SAFE_DIST 0.35f // car tyres and gear lever saft distance

#define DIST_INIT           100000  // calculate minimum distance initialize value
#define AVM_CORNER_MIN_DIST 1.0f    // OTO output avm park corner, minimum distance
#define AVM_OBJ_MIN_DIST    0.1f    // OTO output avm park lever lenght, minimum distance
#define GET_A_LINE_RATIO    5       // get ratio area all line for B process initialize
#define B_LINE_RATIO        12      // get B tarpos line ratio
#define RADAR_338_ANGLE_COS 0.9397f // 0.6428f//  338 radar field angle cos value
#define RADAR_338_DIST_THR \
    1000 // 338 radar reachable distance, > this value expression invalid value.
#define RADAR_313_DIST_THR \
    5000 // 313 radar reachable distance, > this value expression invalid value.
#define ROTATE_ANGLE_COS_THR \
    0.9397f                 // just use this angle line for calculate,for rotate slot park
#define UNIT_MM_TO_M 0.001f // unit mm convert to m
#define PARA_PARK_EXTEND \
    10 // for para park,extend BC line for remove A process lines inside park area.
#define CHECK_CD_RANGE 0.6f // for PointCheck,move CD line to a small range
#define PARA_CHECK_BC_RANGE \
    -0.5f                       // for para park, use side radar PointCheck value thresh.
#define FIT_LINE_COS_THR 0.906f // when fit line and park axls angle, cos value thresh.
#define IGNORE_LINE_THR  0.07f  // ignore small line when fitline process
#define LINE_LEN_NO_ZERO 0.001f // line lenght > this value, express not a zero.
#define REAR_STAT_THR \
    6 // rear radar statistics thresh,middle weight as 2, side weight as 1
#define REAR_STAT_PERCENT \
    0.7f // rear radar update statistics percent,70% is good for code
#define POINT2_DIST_THR    0.2f // when vehicle in parking, radar get 2 point distance thresh
#define AVM_REACHABLE_DIST 4.0f // OTO output avm point, max distance
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For Fit Line struct type
typedef struct _Idx_Val_T
{
    int sidx;     // set start idx
    int eidx;     // set end idx
    LineSeg_T ls; // Fitline start and end point
} Idx_Val_T;

// Radar original scan set
typedef struct _Array_T
{
    float data[RADAR_NUM];
    int num; // array size
} Array_T;

// Real object location set
typedef struct _Pointset_T
{
    Point_T set[SET_SAVE_SIZE];
    int real_num; // arry real size
    int stat_num; // stat num
} Pointset_T;

// point and value and line and axls similar(cos)
typedef struct _Point_Val_T
{
    Point_T pt; //
    float val;
    float simi;
} Point_Val_T;

// line segment and value struct
typedef struct _Vec_Val_T
{
    Vec2_T vc;
    float val;
} Vec_Val_T;

// Just for B1 Process
static int g_front_stat[STAT_SIZE]; // front radar project value stat array
static int g_rear_stat[STAT_SIZE];  // rear radar project value stat array
static float g_front_conf[4];       // every radar confidence stat array
static float g_rear_conf[4];

static float g_last_path; // save last odo, judge vehicle is moving?

static Array_T g_radarl; // rsl radar scan value
static Array_T g_radarr; // rsr radar scan value

static Pointset_T g_setl;  // rsl radar scan object location set
static Pointset_T g_setfl; // rsl radar scan object location set
static Pointset_T g_setr;  // rsr radar scan object location set
static Pointset_T g_setfr; // rsr radar scan object location set

// Global fit line, left,right line save in [0, SF_OBJ_NUM-2],front and rear saved in
// SF_OBJ_NUM-1;
static FusionObj_T g_lf_fus; // left and front
static FusionObj_T g_rr_fus; // right and rear

static Idx_Val_T g_cur_timl; // left current fit line
static Idx_Val_T g_cur_timr; // right current fit line

static Point_Val_T g_min_lenl; // left nearest line,and nearest point
static Point_Val_T g_min_lenr; // right nearest line, and nearest point

static int g_avm_b_flag;     // B process avm update flag
static Avm_Pot_T g_avm_a_pt; // A process avm park point

// park slot,ABCDEF
static SlotObj_T g_opark_area; // original slot
static SlotObj_T g_npark_area; // mirror slot
static const SlotObj_T &GetNParkArea() { return g_npark_area; }

static float g_new_bc_length; // slot bc length
static float g_new_ed_length; // slot ed length
static Vec2_T g_ori_norm_cd;  // vector cd
static int g_mov_stat;
static Point_Val_T g_rear_radar;

static float g_front_val; // in B1 process, rear side radar update value

static VehPos_T g_last_tar;   // last tarpos
static SlotObj_T g_last_park; // last slot park

static float g_veh_tar_dist; // current vehicle and tarpos project distance
static int g_update_B_flag;  // tarpos and slot update flag,when =0,B process no update,
                             // =1,start update

static VehPos_T g_tar_vel; // global tarpos

static LineSeg_T g_tar_axls; // global tarpos axle
static const LineSeg_T &GetInitTarg() { return g_tar_axls; }
static float g_lef_dist; // left slot line and tarpos distance
static float g_rig_dist; // right slot line and tarpos distance

static Vec2_T g_norm_bc; // park slot bc unit vector
static Vec2_T g_norm_cd; // park slot cd unit vector
static float g_bc_theta; // park slot bc angle

static int g_is_big_park;   // A process slot park is big park
static float g_tar_cd_dist; // A process tarpos and slot cd distance
static int
    g_bfirst_rev; // path plan have 2 or 1 forward or backward, or move in a deep distance

static LineSeg_T g_ori_mov_bc; // for B1,bc external expansion
static LineSeg_T g_ori_mov_ed; // for B1,ed external expansion
static VehPos_T g_cur_veh;     // cur vehicle pos

static Point_T g_front_start; // for B1, front project start point
static Point_T g_rear_start;  // for B1, rear project start point

static LineSeg_T g_park_cd;   // park slot cd line
static Vec_Val_T g_front_mov; // parallel park front move distance and confidence
static Vec_Val_T g_rear_mov;  // parallel park rear move distance and confidence
static float
    g_b_update_dist_thr; // B process, if update B avm point, use this value as thresh
static int g_index_obs_flag  = -1;
static int Update_B_Flag_ccp = 0;
static FusionObj_T FS_ObsInfo_Left_B, FS_ObsInfo_Right_B;

static int Point_Check(const SlotObj_T park, const Point_T obs, const float side_thr,
                       const float be_thr, const float cd_thr);

#ifdef PK_SlotDetect_OffLineTest
int g_IsBSlotUpata = 0;
SlotObj_T g_SlotObj_After;
VehPos_T g_TargPos_After;

SlotObj_T g_SlotObj_before;
VehPos_T g_TargPos_before;
#endif
// Initialize all global variable
void PK_SF_B_Clear(void)
{
    g_last_path = .0f;

    memset(&g_radarl, 0, sizeof(g_radarl));
    memset(&g_radarr, 0, sizeof(g_radarr));

    memset(&g_setl, 0, sizeof(g_setl));
    memset(&g_setr, 0, sizeof(g_setr));

    memset(&g_setfl, 0, sizeof(g_setfl));
    memset(&g_setfr, 0, sizeof(g_setfr));

    memset(&g_lf_fus, 0, sizeof(g_lf_fus));
    memset(&g_rr_fus, 0, sizeof(g_rr_fus));

    // clean RTE
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Left(&g_lf_fus);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Right(&g_rr_fus);

    g_cur_timl.sidx = -1;
    g_cur_timl.eidx = -1;
    g_cur_timr.sidx = -1;
    g_cur_timr.eidx = -1;

    g_update_B_flag   = 0;
    Update_B_Flag_ccp = 0;

    g_is_big_park = 0;

    g_avm_b_flag = 0;
    memset(&g_avm_a_pt, 0, sizeof(g_avm_a_pt));

    g_min_lenl.val = DIST_INIT;
    g_min_lenr.val = DIST_INIT;
    g_veh_tar_dist = DIST_INIT;

    g_front_mov.val = 0;
    g_rear_mov.val  = 0;

    g_mov_stat       = 0;
    g_rear_radar.val = -1;

    memset(g_front_stat, 0, sizeof(g_front_stat));
    memset(g_rear_stat, 0, sizeof(g_rear_stat));

    memset(g_front_conf, 0, sizeof(g_front_conf));
    memset(g_rear_conf, 0, sizeof(g_rear_conf));

    g_bfirst_rev = 0;

    g_b_update_dist_thr = 0.6f; // as default
#ifdef PK_B_DEBUG
    all_num  = 0;
    side_num = 0;
    mid_num  = 0;
#endif
}

// Add radar value
void Add_Value(Array_T *arr, float val)
{
    if (arr->num >= RADAR_NUM)
    {
        for (int i = 1; i < RADAR_NUM; ++i)
        {
            arr->data[i - 1] = arr->data[i];
        }
        --(arr->num);
    }

    arr->data[arr->num] = val;
    ++(arr->num);
}
// Add Point value
void Add_Point(Pointset_T *ps, Point_T pt)
{
    int i;
    if (ps->real_num >= SET_SAVE_SIZE)
    {
        for (i = 1; i < SET_SAVE_SIZE; ++i)
        {
            ps->set[i - 1] = ps->set[i];
        }
        --(ps->real_num);
    }

    ps->set[ps->real_num] = pt;
    ++(ps->real_num);
    ++(ps->stat_num);
}

// Move FusionObj value
void Mov_FusObj(FusionObj_T *fus)
{
    if (fus->num < SF_OBJ_NUM - 2)
    {
        return;
    }

    // (SF_OBJ_NUM - 1) area reserved for parallel park global line
    for (int i = 1; i < SF_OBJ_NUM - 2; ++i)
    {
        fus->obj[i - 1]  = fus->obj[i];
        fus->attr[i - 1] = fus->attr[i];
    }
    fus->num -= 1;
}

// calculate 2 point spt->ept unit vector
static Vec2_T Norm_Lineseq(Point_T spt, Point_T ept)
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
// calculate middle point
static Point_T Get_Mid_Point(Point_T spt, Point_T ept)
{
    Point_T pt;
    pt.x = 0.5f * (spt.x + ept.x);
    pt.y = 0.5f * (spt.y + ept.y);
    return pt;
}

// like matlab code, calculate radar -> object location
static void Get_Object_Location(float aerfa, float deltax, float deltay, float radar,
                                VehPos_T vel, Point_T *obs)
{
    float sintmp1 = sinf(aerfa);
    float costmp1 = cosf(aerfa);

    float sintmp2 = sinf(vel.theta);
    float costmp2 = cosf(vel.theta);

    float xDetect = deltax + (radar * costmp1 * UNIT_MM_TO_M);
    float yDetect = deltay + (radar * sintmp1 * UNIT_MM_TO_M);

    obs->x = xDetect * costmp2 - yDetect * sintmp2 + vel.x;
    obs->y = xDetect * sintmp2 + yDetect * costmp2 + vel.y;
}

// https://www.cnblogs.com/ranjiewen/p/5985847.html
// Fit line
// pData: point set
// num: set num
// fit_point: output line segment and other attribute
static void Polyfit_Line(const Point_T *pData, const int num, LineSeg_T *fit_point)
{
    float sum_x2 = 0;
    float sum_y  = 0;
    float sum_x  = 0;
    float sum_xy = 0;
    int i;
    Point_T sta = pData[0];
    Point_T end = pData[num - 1];
    float k     = 0;
    float b     = 0;

    int is_mir = 0;
    if (fabsf(sta.x - end.x) < fabsf(sta.y - end.y))
    {
        is_mir = 1;
    }

    if (is_mir == 1)
    {
        // X Y flip
        for (i = 0; i < num; ++i)
        {
            sum_x2 += ((pData[i].y - sta.y) * (pData[i].y - sta.y));
            sum_y += (pData[i].x - sta.x);
            sum_x += ((pData[i].y - sta.y));
            sum_xy += ((pData[i].y - sta.y) * (pData[i].x - sta.x));
        }

        sum_x2 /= num;
        sum_y /= num;
        sum_x /= num;
        sum_xy /= num;

        if (fabsf(sum_x2 - sum_x * sum_x) > 1e-20f)
        {
            k = (sum_xy - sum_x * sum_y) / (sum_x2 - sum_x * sum_x);
            b = sum_y + sta.x - k * (sum_x + sta.y);
        }

        // output
        fit_point->pt1.x = k * sta.y + b;
        fit_point->pt1.y = sta.y;
        fit_point->pt2.x = k * end.y + b;
        fit_point->pt2.y = end.y;
    }
    else
    {
        // X Y not flip
        for (i = 0; i < num; ++i)
        {
            sum_x2 += ((pData[i].x - sta.x) * (pData[i].x - sta.x));
            sum_y += (pData[i].y - sta.y);
            sum_x += ((pData[i].x - sta.x));
            sum_xy += ((pData[i].x - sta.x) * (pData[i].y - sta.y));
        }

        sum_x2 /= num;
        sum_y /= num;
        sum_x /= num;
        sum_xy /= num;

        if (fabsf(sum_x2 - sum_x * sum_x) > 1e-20f)
        {
            k = (sum_xy - sum_x * sum_y) / (sum_x2 - sum_x * sum_x);
            b = sum_y + sta.y - k * (sum_x + sta.x);
        }
        // else {
        //   printf("error!");
        // }
        //  output

        fit_point->pt1.x = sta.x;
        fit_point->pt1.y = k * sta.x + b;
        fit_point->pt2.x = end.x;
        fit_point->pt2.y = k * end.x + b;
    }
}

// sort array and return the middle value
float Get_Median_Value(float *data, const int num)
{
    int i, j;
    float val;
    for (i = 0; i < num; ++i)
    {
        for (j = i + 1; j < num; ++j)
        {
            if (data[i] > data[j])
            {
                val     = data[i];
                data[i] = data[j];
                data[j] = val;
            }
        }
    }

    return data[(num - 1) / 2];
}

// median filter,just filter leaf value
static float Leaf_Median_Filter(const float *in, const int num)
{
    static float g_kernel_area[2 * KERNEL_RADIUS + 1]; // Median filter array
    int ker_size;
    int j;

    if (num <= KERNEL_RADIUS)
    { // array num is so little
        ker_size = num + 1;
        for (j = 0; j < ker_size; j++)
        {
            g_kernel_area[j] = in[j];
        }
    }
    else
    {
        ker_size = KERNEL_RADIUS + 1;
        for (j = 0; j < ker_size; j++)
        {
            g_kernel_area[j] = in[num - j - 1];
        }
    }

    g_kernel_area[j] = in[num - 1]; // current value have important weight
    Get_Median_Value(g_kernel_area, ker_size);
    return g_kernel_area[ker_size / 2];
}

// calculate point and line length
static float Point_Line_Distance(const Point_T &pt, const LineSeg_T &line)
{
    Point_T pt1;
    float len;
    pt1 = Point_Project_Line(pt, line);

    len = Cal_Dis_Pt2Pt(pt1, pt);
    return len;
}

// calculate point project to line,output point
// Algorithm [p: (px,py), p1: (x1,y1), p2: (x2,y2)]
// VECTOR v(x2 - x1, y2 - y1)
// VECTOR w(px - x1, py - y1)
// c1 = w . v
// c2 = v . v
// b = c1 / c2
// RETURN POINT(x1 + b * v.x, y1 + b * v.y)
static Point_T Point_Project_Line(const Point_T &pt, const LineSeg_T &ln)
{
    Point_T rst;
    Point_T v, w;
    float c1, c2;
    float b;
    v.x = ln.pt2.x - ln.pt1.x;
    v.y = ln.pt2.y - ln.pt1.y;
    if (v.x * v.x + v.y * v.y < 1e-20f)
    {
        rst.x = ln.pt1.x;
        rst.y = ln.pt1.y;
        return rst;
    }

    w.x = pt.x - ln.pt1.x;
    w.y = pt.y - ln.pt1.y;

    c1 = w.x * v.x + w.y * v.y;
    c2 = v.x * v.x + v.y * v.y;

    if (c2 < 1e-20f)
    {
        return v;
    }
    b = c1 / c2;

    rst.x = ln.pt1.x + b * v.x;
    rst.y = ln.pt1.y + b * v.y;
    return rst;
}

// calculate line1 project line2 length
static float Line_Project_length(const LineSeg_T &ln1, const LineSeg_T &ln2)
{
    Point_T pt1, pt2;
    pt1       = Point_Project_Line(ln1.pt1, ln2);
    pt2       = Point_Project_Line(ln1.pt2, ln2);
    float len = Cal_Dis_Pt2Pt(pt1, pt2);
    return len;
}

// point and quadrangle relationship, In or Not?
// return > 0 in; return <=0 not in
// http://alienryderflex.com/polygon/
// https://blog.csdn.net/z104207/article/details/44997275
// The function will return YES if the point x,y is inside the polygon, or
// NO if it is not.  If the point is exactly on the edge of the polygon,
// then the function may return YES or NO.
static int In_or_Not(const Point_T *poly, Point_T pt)
{
    int i, j;
    int res = 0;
    for (i = 0, j = 4 - 1; i < 4; j = i++)
    {
        if ((((poly[i].y <= pt.y) && (pt.y < poly[j].y)) ||
             ((poly[j].y <= pt.y) && (pt.y < poly[i].y))) &&
            (pt.x <
             (poly[j].x - poly[i].x) * (pt.y - poly[i].y) / (poly[j].y - poly[i].y) +
                 poly[i].x))
        {
            res = !res;
        }
    }
    return res;
}

// Get all A function line which in quadrangle
// such as A process line, vehicle pos, park slot(ABCDEF),tarpos, avm park corner, slot
// shape
void Get_A_Para(const VehPos_T &vel_pos, const SlotObj_T &pk_area,
                const VehPos_T &tar_vel, const Avm_Pot_T &avm_pt)
{
    static FusionObj_T lfus, rfus;

    float park_wid;

    g_avm_a_pt   = avm_pt;
    g_tar_vel    = tar_vel;
    g_opark_area = pk_area; // original park

    const int &slot_shape = RTE_PK_SlotDetect_Get_SlotShape();
    Convert_Park_Area(slot_shape, &pk_area, &g_npark_area);

    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Left(&lfus);
    RTE_PK_SensorFusion_Get_Fusion_ObsInfo_A_Right(&rfus);

    g_new_bc_length = Cal_Dis_Pt2Pt(g_npark_area.ptB, g_npark_area.ptC);
    g_new_ed_length = Cal_Dis_Pt2Pt(g_npark_area.ptD, g_npark_area.ptE);
    g_norm_bc       = Norm_Lineseq(g_npark_area.ptB, g_npark_area.ptC);
    g_norm_cd       = Norm_Lineseq(g_npark_area.ptC, g_npark_area.ptD);

    // for B2 process
    if (slot_shape == PK_SLOT_LEFT_VERT || slot_shape == PK_SLOT_LEFT_ANG_REVERSE)
    {
        g_ori_norm_cd.vx = g_norm_cd.vx;
        g_ori_norm_cd.vy = g_norm_cd.vy;
    }
    else if (slot_shape == PK_SLOT_RIGHT_VERT || slot_shape == PK_SLOT_RIGHT_ANG_REVERSE)
    {
        g_ori_norm_cd.vx = -g_norm_cd.vx;
        g_ori_norm_cd.vy = -g_norm_cd.vy;
    }
    else
    {
        g_ori_norm_cd.vx = g_norm_cd.vx;
        g_ori_norm_cd.vy = g_norm_cd.vy;
    }

    g_ori_mov_bc.pt1.x = g_opark_area.ptB.x + MAX_REAR_DIST * g_ori_norm_cd.vx;
    g_ori_mov_bc.pt1.y = g_opark_area.ptB.y + MAX_REAR_DIST * g_ori_norm_cd.vy;

    g_ori_mov_bc.pt2.x = g_ori_mov_bc.pt1.x + g_norm_bc.vx;
    g_ori_mov_bc.pt2.y = g_ori_mov_bc.pt1.y + g_norm_bc.vy;

    g_ori_mov_ed.pt1.x = g_npark_area.ptE.x + MAX_REAR_DIST * g_norm_cd.vx;
    g_ori_mov_ed.pt1.y = g_npark_area.ptE.y + MAX_REAR_DIST * g_norm_cd.vy;

    g_ori_mov_ed.pt2.x = g_npark_area.ptD.x + MAX_REAR_DIST * g_norm_cd.vx;
    g_ori_mov_ed.pt2.y = g_npark_area.ptD.y + MAX_REAR_DIST * g_norm_cd.vy;

    g_front_val = 0.0f;

    g_last_tar  = tar_vel;
    g_last_park = pk_area;

    // bc angle
    g_bc_theta = atan2f(g_npark_area.ptC.y - g_npark_area.ptB.y,
                        g_npark_area.ptC.x - g_npark_area.ptB.x);
    if (avm_pt.side_dist < AVM_REACHABLE_DIST && avm_pt.side_dist > 0.0f)
    {
        // use dynamic valse as thresh [0.3f 0.7f]
        g_b_update_dist_thr = 0.3f + (avm_pt.side_dist / AVM_REACHABLE_DIST) * 0.7f;
    }
    else
    {
        g_b_update_dist_thr = 0.7f;
    }

    g_tar_axls.pt1.x = tar_vel.x + (VEHICLE_LEN - REAR_SUSPENSION) * cosf(tar_vel.theta);
    g_tar_axls.pt1.y = tar_vel.y + (VEHICLE_LEN - REAR_SUSPENSION) * sinf(tar_vel.theta);
    g_tar_axls.pt2.x = tar_vel.x;
    g_tar_axls.pt2.y = tar_vel.y;
    g_lef_dist       = Point_Line_Distance(g_npark_area.ptB, g_tar_axls);
    g_rig_dist       = Point_Line_Distance(g_npark_area.ptE, g_tar_axls);

    // judge is big park
    park_wid = Cal_Dis_Pt2Pt(g_npark_area.ptC, g_npark_area.ptD);
    if (park_wid >= (VEHICLE_LEN + BIG_PARA_SLOT_LEN_DIFF))
    {
        g_is_big_park = 1;
    }

    // for parallel park, Slot C D external CHECK_FR_RANGE meter
    g_front_start.x = g_npark_area.ptD.x + CHECK_FR_RANGE * g_norm_cd.vx;
    g_front_start.y = g_npark_area.ptD.y + CHECK_FR_RANGE * g_norm_cd.vy;
    g_rear_start.x  = g_npark_area.ptC.x - CHECK_FR_RANGE * g_norm_cd.vx;
    g_rear_start.y  = g_npark_area.ptC.y - CHECK_FR_RANGE * g_norm_cd.vy;

    g_park_cd.pt1 = g_npark_area.ptC;
    g_park_cd.pt2 = g_npark_area.ptD;

    Point_T pt1   = {tar_vel.x, tar_vel.y};
    g_tar_cd_dist = Point_Line_Distance(pt1, g_park_cd);

#ifdef PK_SlotDetect_OffLineTest
    g_IsBSlotUpata   = 1;
    g_SlotObj_before = pk_area;
    g_TargPos_before = tar_vel;
#endif
}

// return two Line_Seq angle cos value
static float Get_Line_Seg_Angle(const LineSeg_T &seq1, const LineSeg_T &seq2)
{
    Point_T vec1, vec2;

    float len;
    float rst_dot;
    float simi;
    vec1.x = seq1.pt2.x - seq1.pt1.x;
    vec1.y = seq1.pt2.y - seq1.pt1.y;

    vec2.x = seq2.pt2.x - seq2.pt1.x;
    vec2.y = seq2.pt2.y - seq2.pt1.y;

    rst_dot = (vec1.x * vec2.x + vec1.y * vec2.y);
    len     = sqrtf(vec1.x * vec1.x + vec1.y * vec1.y);
    len *= sqrtf(vec2.x * vec2.x + vec2.y * vec2.y);
    if (len < 1e-20f)
        return 0;

    simi = rst_dot / len;
    return simi;
}

float mod(float a, float b)
{
    int rst = (int)(a / b);
    return (a - rst * b);
}

// check point is in slot park area, != 0 , is inside; == 0,outside
static int Point_Check(const SlotObj_T park, const Point_T obs, const float side_thr,
                       const float be_thr, const float cd_thr)
{
    // update new slot area
    Point_T quad[4];
    quad[0].x = park.ptB.x + be_thr * g_norm_bc.vx - side_thr * g_norm_cd.vx;
    quad[0].y = park.ptB.y + be_thr * g_norm_bc.vy - side_thr * g_norm_cd.vy;
    quad[1].x = park.ptC.x - cd_thr * g_norm_bc.vx - side_thr * g_norm_cd.vx;
    quad[1].y = park.ptC.y - cd_thr * g_norm_bc.vy - side_thr * g_norm_cd.vy;
    quad[2].x = park.ptD.x - cd_thr * g_norm_bc.vx + side_thr * g_norm_cd.vx;
    quad[2].y = park.ptD.y - cd_thr * g_norm_bc.vy + side_thr * g_norm_cd.vy;
    quad[3].x = park.ptE.x + be_thr * g_norm_bc.vx + side_thr * g_norm_cd.vx;
    quad[3].y = park.ptE.y + be_thr * g_norm_bc.vy + side_thr * g_norm_cd.vy;

    int res = In_or_Not(quad, obs);
    return (res != 0) ? 1 : 0;
}

// Convert A process slot park,just slot_shape == 3
static void Convert_Park_Area(const int slot_shape, const SlotObj_T *src, SlotObj_T *dst)
{
    *dst = *src;
    if (slot_shape == PK_SLOT_LEFT_VERT || slot_shape == PK_SLOT_LEFT_ANG_REVERSE)
    {
        dst->ptB = src->ptE;
        dst->ptE = src->ptB;

        dst->ptC = src->ptD;
        dst->ptD = src->ptC;
    }
}

// is_rig != 0 means right￡?is_rig == 0 means left￡?ab is direction
// triangle calculate radar location
static int Calc_Triangle_Third(Point_T pa, Point_T pb, Point_T *pc, float a, float b,
                               float c, int is_rig)
{
    float cosa, sina;
    float anga;
    Point_T vab, vac;
    float len, mov_x, mov_y;

    if (a * b * c < 1e-20f || (a + b <= c) || (a + c <= b) || (b + c <= a))
        return 0;

    cosa = (b * b + c * c - a * a) / (2 * b * c);
    if (cosa > RADAR_338_ANGLE_COS || cosa < -RADAR_338_ANGLE_COS)
        return 0; // radar field angle

    anga = acosf(cosa);
    sina = sinf(anga);
    if (is_rig != 0)
        sina = -sina;

    vab.x = pb.x - pa.x;
    vab.y = pb.y - pa.y;

    vac.x = cosa * vab.x + sina * vab.y;
    vac.y = -sina * vab.x + cosa * vab.y;

    len   = sqrtf(vac.x * vac.x + vac.y * vac.y);
    mov_x = vac.x / len;
    mov_y = vac.y / len;

    pc->x = pa.x + mov_x * b;
    pc->y = pa.y + mov_y * b;

    return 1;
}

// calculation rear radar Triangle, just for B2 process
static void Calc_Rear_Indi(VehPos_T vel_pos, const U_RadarType *radar)
{
    Point_Val_T indir[3];
    Point_T pta, ptb, ptc;
    float lena, lenb, lenc;
    int out, idx, i;
    float val;
    Point_T pt_rol, pt_rcl, pt_rcr, pt_ror;

    Get_Object_Location(RCL_AERFA, RCL_DELTAX, RCL_DELTAY, 0, vel_pos, &pt_rcl);
    Get_Object_Location(ROL_AERFA, ROL_DELTAX, ROL_DELTAY, 0, vel_pos, &pt_rol);
    Get_Object_Location(RCR_AERFA, RCR_DELTAX, RCR_DELTAY, 0, vel_pos, &pt_rcr);
    Get_Object_Location(ROR_AERFA, ROR_DELTAX, ROR_DELTAY, 0, vel_pos, &pt_ror);

    memset(indir, 0, sizeof(indir));
    if (radar->ROL > RADAR_338_DIST_THR || radar->RCL > RADAR_338_DIST_THR)
    {
        indir[0].val = 0;
    }
    else
    {
        // use triangle calculate point
        pta  = pt_rcl;
        ptb  = pt_rol;
        lenc = Cal_Dis_Pt2Pt(pta, ptb);
        lena = UNIT_MM_TO_M * (2 * radar->ROL - radar->RCL);
        lenb = UNIT_MM_TO_M * radar->RCL;
        out  = Calc_Triangle_Third(pta, ptb, &ptc, lena, lenb, lenc, 1);
        val  = Cal_Dis_Pt2Pt(g_opark_area.ptB, ptc);
        if (out > 0 && val < MAX_REAR_DIST)
        {
            indir[0].pt  = ptc;
            indir[0].val = 1;
        }
        else
        {
            indir[0].val = 0;
        }
    }

    if (radar->RCR > RADAR_338_DIST_THR || radar->RCL > RADAR_338_DIST_THR)
    {
        indir[1].val = 0;
    }
    else
    {
        // use triangle calculate point
        pta  = pt_rcl;
        ptb  = pt_rcr;
        lenc = Cal_Dis_Pt2Pt(pta, ptb);
        lena = UNIT_MM_TO_M * (2 * radar->RCR - radar->RCL);
        lenb = UNIT_MM_TO_M * radar->RCL;
        out  = Calc_Triangle_Third(pta, ptb, &ptc, lena, lenb, lenc, 0);
        val  = Cal_Dis_Pt2Pt(g_opark_area.ptB, ptc);
        if (out > 0 && val < MAX_REAR_DIST)
        {
            indir[1].pt  = ptc;
            indir[1].val = 1;
        }
        else
        {
            indir[1].val = 0;
        }
    }

    if (radar->ROR > RADAR_338_DIST_THR || radar->RCR > RADAR_338_DIST_THR)
    {
        indir[2].val = 0;
    }
    else
    {
        // use triangle calculate point
        pta  = pt_rcr;
        ptb  = pt_ror;
        lenc = Cal_Dis_Pt2Pt(pta, ptb);
        lena = UNIT_MM_TO_M * (2 * radar->ROR - radar->RCR);
        lenb = UNIT_MM_TO_M * radar->RCR;
        out  = Calc_Triangle_Third(pta, ptb, &ptc, lena, lenb, lenc, 0);
        val  = Cal_Dis_Pt2Pt(g_opark_area.ptB, ptc);
        if (out > 0 && val < MAX_REAR_DIST)
        {
            indir[2].pt  = ptc;
            indir[2].val = 1;
        }
        else
        {
            indir[2].val = 0;
        }
    }

    // rear radar statistic
    for (i = 0; i < 3; ++i)
    {
        if (indir[i].val > 0)
        {
            val = Point_Line_Distance(indir[i].pt, g_ori_mov_bc);
            idx = (int)(STAT_SIZE * val / (2 * MAX_REAR_DIST));
            if (idx >= 0 && idx < STAT_SIZE)
            {
                if (i == 1)
                {
                    // middle radar have big weight
                    g_mov_stat += 3;
                    g_rear_stat[idx] += 3;
                }
                else
                {
                    // side radar have small weight
                    g_mov_stat += 1;
                    g_rear_stat[idx] += 1;
                }
            }
        }
    }
}

// calculate line segment(pt1-pt2) and park slot relationship,1 means cross; 0 means not
// cross
static int Is_Lineseg_Cross_Slot(SlotObj_T slot, Point_T pt1, Point_T pt2)
{
    Relation_T rt1, rt2, rt3, rt4;
    Point_T ptB, ptC, ptD, ptE;
    ptB = slot.ptB;
    ptC = slot.ptC;
    ptD = slot.ptD;
    ptE = slot.ptE;

    Get_Dist_Dir_Pt2PointLine(ptB, ptC, pt1, NULL, &rt1);
    Get_Dist_Dir_Pt2PointLine(ptB, ptC, pt1, NULL, &rt2);

    Get_Dist_Dir_Pt2PointLine(ptE, ptD, pt1, NULL, &rt3);
    Get_Dist_Dir_Pt2PointLine(ptE, ptD, pt1, NULL, &rt4);

    if (rt1 == rt2 && rt3 == rt4)
        return 0;
    else
        return 1;
}
// judge tarpos is cross park lever
static void Judge_Obj_Cross(VehPos_T *veh, const Avm_Obj_T *avm_obj)
{
    float dist, simi, pj_len, len;
    LineSeg_T ls;
    Point_T mid_pt, pj_pt;

    dist = Cal_Dis_Pt2Pt(avm_obj->ls.pt1, avm_obj->ls.pt2);

    if (dist > AVM_OBJ_MIN_DIST)
    { // park lever length should this length
        ls.pt1.x = veh->x + (VEHICLE_LEN - REAR_SUSPENSION) * cosf(veh->theta);
        ls.pt1.y = veh->y + (VEHICLE_LEN - REAR_SUSPENSION) * sinf(veh->theta);
        ls.pt2.x = veh->x;
        ls.pt2.y = veh->y;
        simi     = Get_Line_Seg_Angle(ls, avm_obj->ls);
        if (fabsf(simi) < 0.342f)
        { // park lever angle shoud in this range
            mid_pt = Get_Mid_Point(avm_obj->ls.pt1, avm_obj->ls.pt2);
            pj_pt  = Point_Project_Line(mid_pt, ls);
            pj_len = Cal_Dis_Pt2Pt(pj_pt, ls.pt1);
            // printf("%f\n\n",pj_len);
            len = pj_len - (VEHICLE_LEN - REAR_SUSPENSION + VEHICLE_REAR_SAFE_DIST);
            if (len > -1.5f && len < 0)
            { // if cross,tarpos and park lever shoud keep a safe distance
                veh->x = pj_pt.x + VEHICLE_REAR_SAFE_DIST * cosf(veh->theta);
                veh->y = pj_pt.y + VEHICLE_REAR_SAFE_DIST * sinf(veh->theta);
            }
        }
    }
}

// is update calculate result to RTE flag
// in the CCP variables, can see Update_B_Flag_ccp
static void Update_Flag(void)
{
    int i;
    int num;

    if (g_update_B_flag == 0)
    {
        num = 0;

        // in the park area, fitline's num > 0
        for (i = 0; i < SF_OBJ_NUM; ++i)
        {
            if (g_lf_fus.attr[i] == B_CHECK_LEFT || g_lf_fus.attr[i] == B_LINE_FRONT)
            {
                ++num;
            }
        }

        for (i = 0; i < SF_OBJ_NUM; ++i)
        {
            if (g_rr_fus.attr[i] == B_CHECK_RIHGT || g_rr_fus.attr[i] == B_LINE_REAR)
            {
                ++num;
            }
        }
        // 2 situation
        if (g_avm_b_flag == 1 || num > 0 || g_bfirst_rev == 2)
        {
            g_update_B_flag   = 1;
            Update_B_Flag_ccp = 1;
        }
    }
}
// calculate distance 2 tarpos
static float Calc_Tarpos_Dist(VehPos_T tar1, VehPos_T tar2)
{
    float x2, y2;
    x2 = (tar1.x - tar2.x) * (tar1.x - tar2.x);
    y2 = (tar1.y - tar2.y) * (tar1.y - tar2.y);
    return sqrtf(x2 + y2);
}

// Update radar parallel park
void Calc_B1_Update(SlotObj_T *slo_obj, VehPos_T *ud_vp)
{
    float rear_dist, front_dist, dist;
    LineSeg_T ls;
    static Point_T pk_qua[4];
    int i, res1, res2;
    static LineSeg_T pk_ls[2];
    float val1, val2;

    // get original park as initial park slot
    const SlotObj_T &npark_area = GetNParkArea();
    SlotObj_T tmp_slot;
    memcpy(&tmp_slot, &npark_area, sizeof(_SlotObj_T));

    front_dist = sqrtf(g_front_mov.vc.vx * g_front_mov.vc.vx +
                       g_front_mov.vc.vy * g_front_mov.vc.vy);
    rear_dist =
        sqrtf(g_rear_mov.vc.vx * g_rear_mov.vc.vx + g_rear_mov.vc.vy * g_rear_mov.vc.vy);
    // add distance as update weight
    if (g_rear_mov.val < 0.4f)
    {
        g_rear_mov.val *= (CHECK_SIDE_RANGE - rear_dist);
    }

    if (g_front_mov.val < 0.4f)
    {
        g_front_mov.val *= (CHECK_SIDE_RANGE - front_dist);
    }

    // calculate BCDE
    tmp_slot.ptB.x = npark_area.ptB.x + g_rear_mov.val * g_rear_mov.vc.vx;
    tmp_slot.ptB.y = npark_area.ptB.y + g_rear_mov.val * g_rear_mov.vc.vy;
    tmp_slot.ptC.x = npark_area.ptC.x + g_rear_mov.val * g_rear_mov.vc.vx;
    tmp_slot.ptC.y = npark_area.ptC.y + g_rear_mov.val * g_rear_mov.vc.vy;

    tmp_slot.ptD.x = npark_area.ptD.x + g_front_mov.val * g_front_mov.vc.vx;
    tmp_slot.ptD.y = npark_area.ptD.y + g_front_mov.val * g_front_mov.vc.vy;
    tmp_slot.ptE.x = npark_area.ptE.x + g_front_mov.val * g_front_mov.vc.vx;
    tmp_slot.ptE.y = npark_area.ptE.y + g_front_mov.val * g_front_mov.vc.vy;

    tmp_slot.ptA = g_opark_area.ptA;
    tmp_slot.ptF = g_opark_area.ptF;

    val1 = Cal_Dis_Pt2Pt(tmp_slot.ptB, npark_area.ptB);
    val2 = Cal_Dis_Pt2Pt(tmp_slot.ptE, npark_area.ptE);
    // tarpos update when BCDE move distance > this value
    if (val1 + val2 > 1e-2f)
    {
        if (g_is_big_park)
        {
            // big park
            LOGW << "Radar big park(>7.7m), move front!";
            ud_vp->x     = g_tar_vel.x + g_front_mov.val * g_front_mov.vc.vx;
            ud_vp->y     = g_tar_vel.y + g_front_mov.val * g_front_mov.vc.vy;
            ud_vp->theta = g_tar_vel.theta;
        }
        else
        {
            // small park, vehicle in the middle
            LOGW << "Radar small park(<7.7m), keep targpos in the middle!";
            ud_vp->x = g_tar_vel.x + (g_front_mov.val * g_front_mov.vc.vx +
                                      g_rear_mov.val * g_rear_mov.vc.vx) *
                                         0.5f;
            ud_vp->y = g_tar_vel.y + (g_front_mov.val * g_front_mov.vc.vy +
                                      g_rear_mov.val * g_rear_mov.vc.vy) *
                                         0.5f;
            ud_vp->theta = g_tar_vel.theta;
        }

        // set new line to global array
        ls.pt1                        = tmp_slot.ptB;
        ls.pt2                        = tmp_slot.ptC;
        g_lf_fus.obj[SF_OBJ_NUM - 1]  = ls;
        g_lf_fus.attr[SF_OBJ_NUM - 1] = B_LINE_FRONT;

        ls.pt1                        = tmp_slot.ptE;
        ls.pt2                        = tmp_slot.ptD;
        g_rr_fus.obj[SF_OBJ_NUM - 1]  = ls;
        g_rr_fus.attr[SF_OBJ_NUM - 1] = B_LINE_REAR;
    }

    // set result to output
    *slo_obj = tmp_slot;

    // avoid the slot park big shake, the program dead
    pk_ls[0].pt1 = slo_obj->ptB;
    pk_ls[0].pt2 = slo_obj->ptC;
    pk_ls[1].pt1 = slo_obj->ptD;
    pk_ls[1].pt2 = slo_obj->ptE;
    res1         = PK_PosLineSegCross_Check(g_cur_veh, 2, pk_ls, 0);
    dist         = Calc_Tarpos_Dist(g_tar_vel, *ud_vp);
    if (res1 >= 1)
    {
        *ud_vp   = g_last_tar;
        *slo_obj = g_last_park;
    }
    else
    {
        if (dist >= Parallel_Target_Max_mov_ccp)
        {
            *ud_vp = g_last_tar;
        }
        g_last_tar  = *ud_vp;
        g_last_park = *slo_obj;
    }

    // remove all line segment in A process
    pk_qua[0]   = slo_obj->ptB;
    pk_qua[1].x = slo_obj->ptB.x + PARA_PARK_EXTEND * g_norm_bc.vx;
    pk_qua[1].y = slo_obj->ptB.y + PARA_PARK_EXTEND * g_norm_bc.vy;
    pk_qua[2].x = slo_obj->ptE.x + PARA_PARK_EXTEND * g_norm_bc.vx;
    pk_qua[2].y = slo_obj->ptE.y + PARA_PARK_EXTEND * g_norm_bc.vy;
    pk_qua[3]   = slo_obj->ptE;

    for (i = 0; i < SF_OBJ_NUM - 1; ++i)
    {
        if (g_lf_fus.attr[i] != 0)
        {
            res1 = In_or_Not(pk_qua, g_lf_fus.obj[i].pt1);
            res2 = In_or_Not(pk_qua, g_lf_fus.obj[i].pt2);

            if (res1 != 0 || res2 != 0)
            {
                g_lf_fus.attr[i] = 0;
                memset(&g_lf_fus.obj[i], 0, sizeof(LineSeg_T));
            }
        }
    }

    for (i = 0; i < SF_OBJ_NUM - 1; ++i)
    {
        if (g_rr_fus.attr[i] != 0)
        {
            res1 = In_or_Not(pk_qua, g_rr_fus.obj[i].pt1);
            res2 = In_or_Not(pk_qua, g_rr_fus.obj[i].pt2);

            if (res1 != 0 || res2 != 0)
            {
                g_rr_fus.attr[i] = 0;
                memset(&g_rr_fus.obj[i], 0, sizeof(LineSeg_T));
            }
        }
    }

#ifdef PK_SlotDetect_OffLineTest
    g_IsBSlotUpata  = 1;
    g_SlotObj_After = *slo_obj;
    g_TargPos_After = *ud_vp;
#endif
}

// update global array line
void Get_Plan_Line(FusionObj_T *lfus_obj, FusionObj_T *rfus_obj)
{
    *lfus_obj = g_lf_fus;
    *rfus_obj = g_rr_fus;
}

static void Fit_B2_LeftLine(const VehPos_T &vel_pos, const SlotObj_T &npark_area,
                            const LineSeg_T &tar_axls, const U_RadarType *radar)
{
    Point_T pt;
    Point_T *p_set;
    LineSeg_T fit_point;

    // for L
    if (radar->RSL >= RADAR_338_DIST_THR)
    {
        return;
    }

    // add original value
    Add_Value(&g_radarl, radar->RSL);

    // median filter and get object location
    float val = Leaf_Median_Filter(g_radarl.data, g_radarl.num);
    Get_Object_Location(RSL_AERFA, RSL_DELTAX, RSL_DELTAY, val, vel_pos, &pt);

#ifdef PK_B_DEBUG
    all_radar[all_num] = pt;
    ++all_num;
#endif
    // add location set
    Add_Point(&g_setl, pt);
    // every 2 point, start calculate
    if (g_setl.stat_num % MOVE_STEP == 0 && g_setl.real_num >= MOVE_STEP)
    {
        int num = 0;
        if (g_setl.real_num >= FIT_STEP)
        {
            p_set = &g_setl.set[g_setl.real_num - FIT_STEP];
            num   = FIT_STEP;
        }
        else
        {
            p_set = &g_setl.set[0];
            num   = g_setl.real_num;
        }

        // last 2 point distance
        float len  = 0.0;
        float simi = 0.0;
        if (g_cur_timl.sidx >= 0)
        {
            // get current point and last 3 point length and similar
            len = Cal_Dis_Pt2Pt(g_setl.set[g_setl.real_num - 1], g_cur_timl.ls.pt2);
            // polyfit last FIT_STEP point for new line segment
            Polyfit_Line(p_set, num, &fit_point);
            simi = Get_Line_Seg_Angle(g_cur_timl.ls, fit_point);
        }

        // suit for all situation, add in current line
        float var = Cal_Dis_Pt2Pt(g_setl.set[g_setl.real_num - 1],
                                  g_setl.set[g_setl.real_num - 2]);
        if (len < POINT2_DIST_THR && var < POINT2_DIST_THR && fabsf(simi) > SIMILAR)
        {
            p_set = &g_setl.set[g_cur_timl.sidx];
            num   = g_setl.real_num - g_cur_timl.sidx;

            Polyfit_Line(p_set, num, &fit_point);

            // start idx not change, edit point num and new fit line
            g_cur_timl.eidx = g_setl.real_num - 1;
            g_cur_timl.ls   = fit_point;

            // if current line length > 0.07f, add in RTE array, or ignore it
            if (Cal_Dis_Pt2Pt(g_cur_timl.ls.pt1, g_cur_timl.ls.pt2) >= IGNORE_LINE_THR)
            {
                // IGNORE_LINE_THR
                //  check line is update the park slot, and set line a new attribute
                int flag1 =
                    Point_Check(npark_area, g_cur_timl.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
                int flag2 =
                    Point_Check(npark_area, g_cur_timl.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
                val = Get_Line_Seg_Angle(g_cur_timl.ls, tar_axls);

                if ((flag1 != 0 || flag2 != 0) && val > FIT_LINE_COS_THR)
                {
                    // cos(angle)
                    g_lf_fus.attr[g_lf_fus.num - 1] = B_CHECK_LEFT;

                    // get left nearest point
                    float val1 = Point_Line_Distance(g_cur_timl.ls.pt1, tar_axls);
                    float val2 = Point_Line_Distance(g_cur_timl.ls.pt2, tar_axls);
                    if (val1 < val2)
                    {
                        if (g_min_lenl.val > val1)
                        {
                            g_min_lenl.pt   = g_cur_timl.ls.pt1;
                            g_min_lenl.val  = val1;
                            g_min_lenl.simi = val;
                        }
                    }
                    else
                    {
                        if (g_min_lenl.val > val2)
                        {
                            g_min_lenl.pt   = g_cur_timl.ls.pt2;
                            g_min_lenl.val  = val2;
                            g_min_lenl.simi = val;
                        }
                    }
                }
                else
                {
                    g_lf_fus.attr[g_lf_fus.num - 1] = B_OUT_LEFT;
                }

                // update global array line
                g_lf_fus.obj[g_lf_fus.num - 1] = g_cur_timl.ls;
            }
        }
        else if (var < POINT2_DIST_THR)
        {
            // create new line segment, create new current line
            // clear setl array
            for (int k = 0; k < MOVE_STEP; ++k)
            {
                g_setl.set[k] = g_setl.set[g_setl.real_num - MOVE_STEP + k];
            }
            g_setl.real_num = MOVE_STEP;

            p_set = &g_setl.set[0];
            Polyfit_Line(p_set, MOVE_STEP, &fit_point);

            // get new set idx and new fit line
            g_cur_timl.sidx = 0;
            g_cur_timl.eidx = g_setl.real_num - 1;
            g_cur_timl.ls   = fit_point;

            // check fitline attribute
            int flag1 =
                Point_Check(npark_area, g_cur_timl.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
            int flag2 =
                Point_Check(npark_area, g_cur_timl.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
            // ignore so small line segment
            if (g_lf_fus.num >= 1 &&
                Cal_Dis_Pt2Pt(g_lf_fus.obj[g_lf_fus.num - 1].pt1,
                              g_lf_fus.obj[g_lf_fus.num - 1].pt2) < IGNORE_LINE_THR)
            {
                g_lf_fus.attr[g_lf_fus.num - 1] =
                    (flag1 + flag2 != 0) ? B_CHECK_LEFT : B_OUT_LEFT;
                // add to global array
                g_lf_fus.obj[g_lf_fus.num - 1] = g_cur_timl.ls;
            }
            else if (Cal_Dis_Pt2Pt(g_cur_timl.ls.pt1, g_cur_timl.ls.pt2) >=
                     LINE_LEN_NO_ZERO)
            {
                g_lf_fus.attr[g_lf_fus.num] =
                    (flag1 + flag2 != 0) ? B_CHECK_LEFT : B_OUT_LEFT;
                // add to global array
                g_lf_fus.obj[g_lf_fus.num] = g_cur_timl.ls;
                ++g_lf_fus.num;
                Mov_FusObj(&g_lf_fus);
            }
        }
        else
        {
            // other situation, clear global array
            g_setl.real_num = 0;
        }
    }
}

static void Fit_B2_RightLine(const VehPos_T &vel_pos, const SlotObj_T &npark_area,
                             const LineSeg_T &tar_axls, const U_RadarType *radar)
{
    Point_T pt;
    Point_T *p_set;
    LineSeg_T fit_point;

    if (radar->RSR >= RADAR_338_DIST_THR)
    {
        return;
    }

    // add original value
    Add_Value(&g_radarr, radar->RSR);

    // median filter and get object location
    float val = Leaf_Median_Filter(g_radarr.data, g_radarr.num);
    Get_Object_Location(RSR_AERFA, RSR_DELTAX, RSR_DELTAY, val, vel_pos, &pt);

#ifdef PK_B_DEBUG
    all_radar[all_num] = pt;
    ++all_num;
#endif
    // add location set
    Add_Point(&g_setr, pt);

    // every 2 point, start calculate
    if (g_setr.stat_num % MOVE_STEP == 0 && g_setr.real_num >= MOVE_STEP)
    {
        int num = 0;
        if (g_setr.real_num - FIT_STEP >= 0)
        {
            p_set = &g_setr.set[g_setr.real_num - FIT_STEP];
            num   = FIT_STEP;
        }
        else
        {
            p_set = &g_setr.set[0];
            num   = g_setr.real_num;
        }

        // polyfit last FIT_STEP point for new line segment
        Polyfit_Line(p_set, num, &fit_point);

        // last 2 point distance
        float len  = 0.0;
        float simi = 0.0;
        float var  = Cal_Dis_Pt2Pt(g_setr.set[g_setr.real_num - 1],
                                   g_setr.set[g_setr.real_num - 2]);
        if (g_cur_timr.sidx < 0)
        {
            // when first fit, set varible to 0
            len  = 0;
            simi = 0;
        }
        else
        {
            // get current point and last 3 point length and similar
            len  = Cal_Dis_Pt2Pt(g_setr.set[g_setr.real_num - 1], g_cur_timr.ls.pt2);
            simi = Get_Line_Seg_Angle(g_cur_timr.ls, fit_point);
        }

        // suit for all situation, add in current line
        if (len < POINT2_DIST_THR && var < POINT2_DIST_THR && fabsf(simi) > SIMILAR)
        {
            p_set = &g_setr.set[g_cur_timr.sidx];
            num   = g_setr.real_num - g_cur_timr.sidx;
            Polyfit_Line(p_set, num, &fit_point);

            // start idx not change, edit point num and new fit line
            g_cur_timr.eidx = g_setr.real_num - 1;
            g_cur_timr.ls   = fit_point;

            // if current line length > 0.07f, add in RTE array, or ignore it
            if (Cal_Dis_Pt2Pt(g_cur_timr.ls.pt1, g_cur_timr.ls.pt2) >= IGNORE_LINE_THR)
            {
                // IGNORE_LINE_THR
                //  check line is update the park slot, and set line a new attribute
                int flag1 =
                    Point_Check(npark_area, g_cur_timr.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
                int flag2 =
                    Point_Check(npark_area, g_cur_timr.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
                val = Get_Line_Seg_Angle(g_cur_timr.ls, tar_axls);
                if ((flag1 != 0 || flag2 != 0) && val > FIT_LINE_COS_THR)
                {
                    // cos(angle)
                    g_rr_fus.attr[g_rr_fus.num - 1] = B_CHECK_RIHGT;

                    // get right nearest point
                    float val1 = Point_Line_Distance(g_cur_timr.ls.pt1, tar_axls);
                    float val2 = Point_Line_Distance(g_cur_timr.ls.pt2, tar_axls);
                    if (val1 < val2)
                    {
                        if (g_min_lenr.val > val1)
                        {
                            g_min_lenr.pt   = g_cur_timr.ls.pt1;
                            g_min_lenr.val  = val1;
                            g_min_lenr.simi = val;
                        }
                    }
                    else
                    {
                        if (g_min_lenr.val > val2)
                        {
                            g_min_lenr.pt   = g_cur_timr.ls.pt2;
                            g_min_lenr.val  = val2;
                            g_min_lenr.simi = val;
                        }
                    }
                }
                else
                {
                    g_rr_fus.attr[g_rr_fus.num - 1] = B_OUT_RIHGT;
                }

                // update global array line
                g_rr_fus.obj[g_rr_fus.num - 1] = g_cur_timr.ls;
            }
        }
        else if (var < POINT2_DIST_THR)
        {
            // create new line segment, create new current line
            // clear setl array
            for (int k = 0; k < MOVE_STEP; ++k)
            {
                g_setr.set[k] = g_setr.set[g_setr.real_num - MOVE_STEP + k];
            }
            g_setr.real_num = MOVE_STEP;

            p_set = &g_setr.set[0];
            Polyfit_Line(p_set, MOVE_STEP, &fit_point);

            // get new set idx and new fit line
            g_cur_timr.sidx = 0;
            g_cur_timr.eidx = g_setr.real_num - 1;
            g_cur_timr.ls   = fit_point;

            // check fitline attribute
            int flag1 =
                Point_Check(npark_area, g_cur_timr.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
            int flag2 =
                Point_Check(npark_area, g_cur_timr.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
            // ignore so small line segment
            if (g_rr_fus.num >= 1 &&
                Cal_Dis_Pt2Pt(g_rr_fus.obj[g_rr_fus.num - 1].pt1,
                              g_rr_fus.obj[g_rr_fus.num - 1].pt2) < IGNORE_LINE_THR)
            {
                g_rr_fus.attr[g_rr_fus.num - 1] =
                    (flag1 + flag2 != 0) ? B_CHECK_RIHGT : B_OUT_RIHGT;
                // add to global array
                g_rr_fus.obj[g_rr_fus.num - 1] = g_cur_timr.ls;
            }
            else if (Cal_Dis_Pt2Pt(g_cur_timr.ls.pt1, g_cur_timr.ls.pt2) >=
                     LINE_LEN_NO_ZERO)
            {
                g_rr_fus.attr[g_rr_fus.num] =
                    (flag1 + flag2 != 0) ? B_CHECK_RIHGT : B_OUT_RIHGT;
                // add to global array
                g_rr_fus.obj[g_rr_fus.num] = g_cur_timr.ls;
                ++g_rr_fus.num;
                Mov_FusObj(&g_rr_fus);
            }
        }
        else
        {
            // other situation, clear global array
            g_setr.real_num = 0;
        }
    }
}

static void Fit_B2_LeftLineF(const VehPos_T &vel_pos, const SlotObj_T &npark_area,
                             const LineSeg_T &tar_axls, const U_RadarType *radar)
{
    Point_T pt;
    Point_T *p_set;
    LineSeg_T fit_point;
    static Idx_Val_T cur_tim;
    static Array_T radarDatas; // rsl radar scan value

    // for L
    if (radar->FSL >= 3000)
    {
        return;
    }

    if (g_setfl.stat_num == 0)
    {
        memset(&radarDatas, 0, sizeof(radarDatas));
    }

    if (radarDatas.num == 0 && g_setfl.stat_num == 0)
    {
        cur_tim.sidx = -1;
        cur_tim.eidx = -1;
    }

    // add original value
    Add_Value(&radarDatas, radar->FSL);

    // median filter and get object location
    float val = Leaf_Median_Filter(radarDatas.data, radarDatas.num);
    Get_Object_Location(FSL_AERFA, FSL_DELTAX, FSL_DELTAY, val, vel_pos, &pt);

    // add location set
    Add_Point(&g_setfl, pt);
    // every 2 point, start calculate
    if (g_setfl.stat_num % MOVE_STEP == 0 && g_setfl.real_num >= MOVE_STEP)
    {
        int num = 0;
        if (g_setfl.real_num >= FIT_STEP)
        {
            p_set = &g_setfl.set[g_setfl.real_num - FIT_STEP];
            num   = FIT_STEP;
        }
        else
        {
            p_set = &g_setfl.set[0];
            num   = g_setfl.real_num;
        }

        // last 2 point distance
        float len  = 0.0;
        float simi = 0.0;
        if (cur_tim.sidx >= 0)
        {
            // get current point and last 3 point length and similar
            len = Cal_Dis_Pt2Pt(g_setfl.set[g_setfl.real_num - 1], cur_tim.ls.pt2);
            // polyfit last FIT_STEP point for new line segment
            Polyfit_Line(p_set, num, &fit_point);
            simi = Get_Line_Seg_Angle(cur_tim.ls, fit_point);
        }

        // suit for all situation, add in current line
        float var = Cal_Dis_Pt2Pt(g_setfl.set[g_setfl.real_num - 1],
                                  g_setfl.set[g_setfl.real_num - 2]);
        // 最后两个点之间距离小于20cm,
        if (len < POINT2_DIST_THR && var < POINT2_DIST_THR && fabsf(simi) > SIMILAR)
        {
            p_set = &g_setfl.set[cur_tim.sidx];
            num   = g_setfl.real_num - cur_tim.sidx;

            Polyfit_Line(p_set, num, &fit_point);

            // start idx not change, edit point num and new fit line
            cur_tim.eidx = g_setfl.real_num - 1;
            cur_tim.ls   = fit_point;

            // if current line length > 0.07f, add in RTE array, or ignore it
            if (Cal_Dis_Pt2Pt(cur_tim.ls.pt1, cur_tim.ls.pt2) >= IGNORE_LINE_THR)
            {
                // IGNORE_LINE_THR
                //  check line is update the park slot, and set line a new attribute
                int flag1 =
                    Point_Check(npark_area, cur_tim.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
                int flag2 =
                    Point_Check(npark_area, cur_tim.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
                val = Get_Line_Seg_Angle(cur_tim.ls, tar_axls);

                if ((flag1 != 0 || flag2 != 0) && val > FIT_LINE_COS_THR)
                {
                    g_lf_fus.attr[g_lf_fus.num - 1] = B_CHECK_LEFT;
                }
                else
                {
                    g_lf_fus.attr[g_lf_fus.num - 1] = B_OUT_LEFT;
                }

                // update global array line
                g_lf_fus.obj[g_lf_fus.num - 1] = cur_tim.ls;
            }
        }
        else if (var < POINT2_DIST_THR)
        {
            // create new line segment, create new current line
            // clear setl array
            for (int k = 0; k < MOVE_STEP; ++k)
            {
                g_setfl.set[k] = g_setfl.set[g_setfl.real_num - MOVE_STEP + k];
            }
            g_setfl.real_num = MOVE_STEP;

            p_set = &g_setfl.set[0];
            Polyfit_Line(p_set, MOVE_STEP, &fit_point);

            // get new set idx and new fit line
            cur_tim.sidx = 0;
            cur_tim.eidx = g_setfl.real_num - 1;
            cur_tim.ls   = fit_point;

            // check fitline attribute
            int flag1 = Point_Check(npark_area, cur_tim.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
            int flag2 = Point_Check(npark_area, cur_tim.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
            // ignore so small line segment
            if (g_lf_fus.num >= 1 &&
                Cal_Dis_Pt2Pt(g_lf_fus.obj[g_lf_fus.num - 1].pt1,
                              g_lf_fus.obj[g_lf_fus.num - 1].pt2) < IGNORE_LINE_THR)
            {
                g_lf_fus.attr[g_lf_fus.num - 1] =
                    (flag1 + flag2 != 0) ? B_CHECK_LEFT : B_OUT_LEFT;
                // add to global array
                g_lf_fus.obj[g_lf_fus.num - 1] = cur_tim.ls;
            }
            else if (Cal_Dis_Pt2Pt(cur_tim.ls.pt1, cur_tim.ls.pt2) >= LINE_LEN_NO_ZERO)
            {
                g_lf_fus.attr[g_lf_fus.num] =
                    (flag1 + flag2 != 0) ? B_CHECK_LEFT : B_OUT_LEFT;
                // add to global array
                g_lf_fus.obj[g_lf_fus.num] = cur_tim.ls;
                ++g_lf_fus.num;
                Mov_FusObj(&g_lf_fus);
            }
        }
        else
        {
            // other situation, clear global array
            g_setfl.real_num = 0;
        }
    }
}

static void Fit_B2_RightLineF(const VehPos_T &vel_pos, const SlotObj_T &npark_area,
                              const LineSeg_T &tar_axls, const U_RadarType *radar)
{
    Point_T pt;
    Point_T *p_set;
    LineSeg_T fit_point;
    static Idx_Val_T cur_tim;
    static Array_T radarDatas; // rsl radar scan value

    // for L
    if (radar->FSR >= 3000)
    {
        return;
    }

    if (g_setfr.stat_num == 0)
    {
        memset(&radarDatas, 0, sizeof(radarDatas));
    }

    if (radarDatas.num == 0 && g_setfr.stat_num == 0)
    {
        cur_tim.sidx = -1;
        cur_tim.eidx = -1;
    }

    // add original value
    Add_Value(&radarDatas, radar->FSR);

    // median filter and get object location
    float val = Leaf_Median_Filter(radarDatas.data, radarDatas.num);
    Get_Object_Location(FSR_AERFA, FSR_DELTAX, FSR_DELTAY, val, vel_pos, &pt);

    // add location set
    Add_Point(&g_setfr, pt);
    // every 2 point, start calculate
    if (g_setfr.stat_num % MOVE_STEP == 0 && g_setfr.real_num >= MOVE_STEP)
    {
        int num = 0;
        if (g_setfr.real_num >= FIT_STEP)
        {
            p_set = &g_setfr.set[g_setfr.real_num - FIT_STEP];
            num   = FIT_STEP;
        }
        else
        {
            p_set = &g_setfr.set[0];
            num   = g_setfr.real_num;
        }

        // last 2 point distance
        float len  = 0.0;
        float simi = 0.0;
        if (cur_tim.sidx >= 0)
        {
            // get current point and last 3 point length and similar
            len = Cal_Dis_Pt2Pt(g_setfr.set[g_setfr.real_num - 1], cur_tim.ls.pt2);
            // polyfit last FIT_STEP point for new line segment
            Polyfit_Line(p_set, num, &fit_point);
            simi = Get_Line_Seg_Angle(cur_tim.ls, fit_point);
        }

        // suit for all situation, add in current line
        float var = Cal_Dis_Pt2Pt(g_setfr.set[g_setfr.real_num - 1],
                                  g_setfr.set[g_setfr.real_num - 2]);
        // 最后两个点之间距离小于20cm,
        if (len < POINT2_DIST_THR && var < POINT2_DIST_THR && fabsf(simi) > SIMILAR)
        {
            p_set = &g_setfr.set[cur_tim.sidx];
            num   = g_setfr.real_num - cur_tim.sidx;

            Polyfit_Line(p_set, num, &fit_point);

            // start idx not change, edit point num and new fit line
            cur_tim.eidx = g_setfr.real_num - 1;
            cur_tim.ls   = fit_point;

            // if current line length > 0.07f, add in RTE array, or ignore it
            if (Cal_Dis_Pt2Pt(cur_tim.ls.pt1, cur_tim.ls.pt2) >= IGNORE_LINE_THR)
            {
                // IGNORE_LINE_THR
                //  check line is update the park slot, and set line a new attribute
                int flag1 =
                    Point_Check(npark_area, cur_tim.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
                int flag2 =
                    Point_Check(npark_area, cur_tim.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
                val = Get_Line_Seg_Angle(cur_tim.ls, tar_axls);

                if ((flag1 != 0 || flag2 != 0) && val > FIT_LINE_COS_THR)
                {
                    g_rr_fus.attr[g_rr_fus.num - 1] = B_CHECK_LEFT;
                }
                else
                {
                    g_rr_fus.attr[g_rr_fus.num - 1] = B_OUT_LEFT;
                }

                // update global array line
                g_rr_fus.obj[g_rr_fus.num - 1] = cur_tim.ls;
            }
        }
        else if (var < POINT2_DIST_THR)
        {
            // create new line segment, create new current line
            // clear setl array
            for (int k = 0; k < MOVE_STEP; ++k)
            {
                g_setfr.set[k] = g_setfr.set[g_setfr.real_num - MOVE_STEP + k];
            }
            g_setfr.real_num = MOVE_STEP;

            p_set = &g_setfr.set[0];
            Polyfit_Line(p_set, MOVE_STEP, &fit_point);

            // get new set idx and new fit line
            cur_tim.sidx = 0;
            cur_tim.eidx = g_setfr.real_num - 1;
            cur_tim.ls   = fit_point;

            // check fitline attribute
            int flag1 = Point_Check(npark_area, cur_tim.ls.pt1, CHECK_SIDE_RANGE, 0, 0);
            int flag2 = Point_Check(npark_area, cur_tim.ls.pt2, CHECK_SIDE_RANGE, 0, 0);
            // ignore so small line segment
            if (g_rr_fus.num >= 1 &&
                Cal_Dis_Pt2Pt(g_rr_fus.obj[g_rr_fus.num - 1].pt1,
                              g_rr_fus.obj[g_rr_fus.num - 1].pt2) < IGNORE_LINE_THR)
            {
                g_rr_fus.attr[g_rr_fus.num - 1] =
                    (flag1 + flag2 != 0) ? B_CHECK_LEFT : B_OUT_LEFT;
                // add to global array
                g_rr_fus.obj[g_rr_fus.num - 1] = cur_tim.ls;
            }
            else if (Cal_Dis_Pt2Pt(cur_tim.ls.pt1, cur_tim.ls.pt2) >= LINE_LEN_NO_ZERO)
            {
                g_rr_fus.attr[g_rr_fus.num] =
                    (flag1 + flag2 != 0) ? B_CHECK_LEFT : B_OUT_LEFT;
                // add to global array
                g_rr_fus.obj[g_rr_fus.num] = cur_tim.ls;
                ++g_rr_fus.num;
                Mov_FusObj(&g_rr_fus);
            }
        }
        else
        {
            // other situation, clear global array
            g_setfr.real_num = 0;
        }
    }
}

// Fit vertical park reachable space
// vel_pos: current vehicle pos
// odo: milemeter
// cvel: target speed
// radar: 12 radar value
void Fit_B2_Line(VehPos_T vel_pos, float odo, float cvel, int bfirst_rev,
                 const U_RadarType *radar)
{
    Point_T pt, cal;
    LineSeg_T lineSeg;

    const SlotObj_T &npark_area = GetNParkArea();
    const LineSeg_T &tar_axls   = GetInitTarg();
    const int &slot_shape       = RTE_PK_SlotDetect_Get_SlotShape();
    const int &is_avm_type      = RTE_PK_DataConvt_Get_IsAVM_Slot();

    cal.x       = vel_pos.x;
    cal.y       = vel_pos.y;
    pt.x        = g_tar_vel.x;
    pt.y        = g_tar_vel.y;
    lineSeg.pt1 = pt;
    lineSeg.pt2 = cal;

    // get current vehicle and tarpos project distance
    float dist = Line_Project_length(lineSeg, tar_axls);
    if (dist < g_veh_tar_dist)
    {
        g_veh_tar_dist = dist;
    }

    // calculate when vehicle move > VEHICLE_MOVE_DIST and vehicle backward move
    if (odo - g_last_path > VEHICLE_MOVE_DIST)
    {
        g_last_path       = odo;
        VehPos_T lvel_pos = vel_pos;
        VehPos_T rvel_pos = vel_pos;

        if (cvel < 0)
        {
            // add for adjust for slot park corner update area
            float ang_diff =
                fabsf(mod(vel_pos.theta, PI2) - mod((g_tar_vel.theta + PI), PI2));
            ang_diff = mod(fabsf(ang_diff - PI), PI_2) * PI_DEG;

            if (fabsf(ang_diff - 45) < ADJUSTANGLE)
            {
                if (slot_shape == PK_SLOT_LEFT_VERT ||
                    slot_shape == PK_SLOT_LEFT_ANG_REVERSE)
                {
                    lvel_pos.theta = vel_pos.theta - 9 * PI_RAD; // 9 to 10
                }
                else if (slot_shape == PK_SLOT_RIGHT_VERT ||
                         slot_shape == PK_SLOT_RIGHT_ANG_REVERSE)
                {
                    rvel_pos.theta = vel_pos.theta + 9 * PI_RAD;
                }
                else
                {
                }
            }

            Fit_B2_LeftLine(lvel_pos, npark_area, tar_axls, radar);
            Fit_B2_RightLine(rvel_pos, npark_area, tar_axls, radar);
        }
        else if (cvel > 0)
        {
            Fit_B2_LeftLineF(lvel_pos, npark_area, tar_axls, radar);
            Fit_B2_RightLineF(rvel_pos, npark_area, tar_axls, radar);
        }
    }

    pt.x     = vel_pos.x;
    pt.y     = vel_pos.y;
    int flag = Point_Check(npark_area, pt, CHECK_SIDE_RANGE, -REAR_SUSPENSION / 2, 0);
    if (g_bfirst_rev == 0)
    {
        // when vehicle revert or vehicle move to a deep  or recive CCP value
        // use rear radar update park slot
        Calc_Rear_Indi(vel_pos, radar);
        if ((cvel > 0 || flag != 0 || bfirst_rev > 0))
        {
            g_bfirst_rev = 1;
        }
    }

    // statistics num >= 5 && vehicle revert
    if (g_bfirst_rev == 1)
    {
        // g_mov_stat 5
        g_bfirst_rev = 2;
        if (g_mov_stat >= REAR_STAT_THR)
        {
            // get 70% percent value as update idx
            int num = 0;
            int k   = 0;
            for (k = 0; k < STAT_SIZE; ++k)
            {
                num += g_rear_stat[k];
                if (num >= REAR_STAT_PERCENT * g_mov_stat)
                {
                    break;
                } // 0.7f
            }

            if (k < STAT_SIZE)
            {
                float val = (2 * MAX_REAR_DIST) * k / STAT_SIZE;

                // std::cout << "slot shape :" << slot_shape << " k: " << k << " num: " <<
                // num << std::endl;
                pt.x      = g_ori_mov_bc.pt1.x - val * g_ori_norm_cd.vx;
                pt.y      = g_ori_mov_bc.pt1.y - val * g_ori_norm_cd.vy;
                float len = Point_Line_Distance(pt, tar_axls);
                // get rear radar calculate value
                if ((slot_shape == PK_SLOT_RIGHT_VERT ||
                     slot_shape == PK_SLOT_RIGHT_ANG_REVERSE))
                {
                    //&& len < g_min_lenl.val
                    g_rear_radar.pt   = pt;
                    g_rear_radar.val  = len;
                    g_rear_radar.simi = 1;
                    g_min_lenl        = g_rear_radar;
                }
                else if ((slot_shape == PK_SLOT_LEFT_VERT ||
                          slot_shape == PK_SLOT_LEFT_ANG_REVERSE))
                {
                    //&& len < g_min_lenr.val
                    g_rear_radar.pt   = pt;
                    g_rear_radar.val  = len;
                    g_rear_radar.simi = 1;
                    g_min_lenr        = g_rear_radar;
                }
            }
        }
        else if (is_avm_type == 1)
        {
            // if have no obstacle,expand slot ABCDEF, should be radar park
            float len = Point_Line_Distance(g_opark_area.ptB, tar_axls) + 0.5f;
            pt.x      = g_opark_area.ptB.x + 0.5f * g_ori_norm_cd.vx;
            pt.y      = g_opark_area.ptB.y + 0.5f * g_ori_norm_cd.vy;
            if ((slot_shape == PK_SLOT_RIGHT_VERT ||
                 slot_shape == PK_SLOT_RIGHT_ANG_REVERSE))
            {
                //&& len < g_min_lenl.val
                g_rear_radar.pt   = pt;
                g_rear_radar.val  = len;
                g_rear_radar.simi = 1;
                g_min_lenl        = g_rear_radar;
            }
            else if ((slot_shape == PK_SLOT_LEFT_VERT ||
                      slot_shape == PK_SLOT_LEFT_ANG_REVERSE))
            {
                //&& len < g_min_lenr.val
                g_rear_radar.pt   = pt;
                g_rear_radar.val  = len;
                g_rear_radar.simi = 1;
                g_min_lenr        = g_rear_radar;
            }
        }
    }

    // update if write RTE
    Update_Flag();
}

// Fit parallel park reachable space
// vel_pos: current vehicle pos
// odo: milemeter
// cvel: target speed
// radar: 12 radar value
void Fit_B1_Line(VehPos_T vel_pos, float odo, float cvel, const U_RadarType *radar)
{
    int i, max_stat;
    Point_T pt, xy, cal;
    int flag, idx;
    float front_conf, rear_conf;
    LineSeg_T ls1;
    float dist;
    float val;
    int flag1;

    float ang_diff;
    VehPos_T lvel_pos, rvel_pos;

    cal.x   = vel_pos.x;
    cal.y   = vel_pos.y;
    pt.x    = g_tar_vel.x;
    pt.y    = g_tar_vel.y;
    ls1.pt1 = pt;
    ls1.pt2 = cal;

    const LineSeg_T &tar_axls   = GetInitTarg();
    const SlotObj_T &npark_area = GetNParkArea();
    const int &slot_shape       = RTE_PK_SlotDetect_Get_SlotShape();
    dist                        = Line_Project_length(ls1, tar_axls);
    if (dist < g_veh_tar_dist)
    {
        g_veh_tar_dist = dist;
    }

    // calculate when vehicle move > VEHICLE_MOVE_DIST
    if (odo - g_last_path > VEHICLE_MOVE_DIST)
    {
        g_last_path = odo;

        // use side radar to adjust BC,ED
        ang_diff = fabsf(mod(vel_pos.theta, PI2) - mod((g_tar_vel.theta + PI), PI2));
        ang_diff = mod(fabsf(ang_diff - PI), PI_2) * PI_DEG;
        lvel_pos = vel_pos;
        rvel_pos = vel_pos;
        if (fabsf(ang_diff) > 10)
        {
            // vehicle rotate a big angle
            if (slot_shape == PK_SLOT_LEFT_PARA)
            {
                // rotate radar as a angle
                lvel_pos.theta = vel_pos.theta - 15 * PI_RAD;
            }
            else
            {
                rvel_pos.theta = vel_pos.theta + 15 * PI_RAD;
            }
        }

        // use side radar update parallel park as early as possible
        if (slot_shape == PK_SLOT_LEFT_PARA)
        {
            if (radar->RSL < RADAR_313_DIST_THR)
            {
                Add_Value(&g_radarl, radar->RSL);
                val = Leaf_Median_Filter(g_radarl.data, g_radarl.num);
                Get_Object_Location(RSL_AERFA, RSL_DELTAX, RSL_DELTAY, val, lvel_pos,
                                    &pt);
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                flag1 = Point_Check(npark_area, pt, MAX_REAR_DIST, PARA_CHECK_BC_RANGE,
                                    0.7f * g_new_ed_length);

                if (flag1 != 0)
                {
                    val = Point_Line_Distance(pt, g_ori_mov_ed) - MAX_REAR_DIST;
                    if (val > 0)
                    {
                        // only shrink park,not enlarge park
                        g_front_mov.vc.vx = -val * g_ori_norm_cd.vx;
                        g_front_mov.vc.vy = -val * g_ori_norm_cd.vy;
                        g_front_mov.val   = 0.7f;
                        g_front_val       = -val;
                    }
                }
            } // end L
        }
        else
        {
            // for R
            if (radar->RSR < RADAR_313_DIST_THR)
            {
                Add_Value(&g_radarr, radar->RSR);
                val = Leaf_Median_Filter(g_radarr.data, g_radarr.num);
                Get_Object_Location(RSR_AERFA, RSR_DELTAX, RSR_DELTAY, val, rvel_pos,
                                    &pt);
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                flag1 = Point_Check(npark_area, pt, MAX_REAR_DIST, PARA_CHECK_BC_RANGE,
                                    0.7f * g_new_ed_length);

                if (flag1 != 0)
                {
                    val = Point_Line_Distance(pt, g_ori_mov_ed) - MAX_REAR_DIST;
                    if (val > 0)
                    {
                        // only shrink park,not enlarge park
                        g_front_mov.vc.vx = -val * g_ori_norm_cd.vx;
                        g_front_mov.vc.vy = -val * g_ori_norm_cd.vy;
                        g_front_mov.val   = 0.7f;
                        g_front_val       = -val;
                    }
                }
            } // end R
        }

        // for FOL
        if (radar->FOL < RADAR_338_DIST_THR)
        {
            Get_Object_Location(FOL_AERFA, FOL_DELTAX, FOL_DELTAY, radar->FOL, vel_pos,
                                &pt);
            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            // check point location is in park slot area
            if (flag != 0)
            {
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                // statistics front out radar, every radar have different weight to
                // statistics
                if (g_front_conf[0] < OUT_POINT_PERCENT)
                {
                    g_front_conf[0] += (OUT_POINT_PERCENT / OUT_POINT_THR);
                }

                xy  = Point_Project_Line(pt, g_park_cd);
                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_front_start) /
                            (2 * CHECK_FR_RANGE));

                // point project statistics
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_front_stat[idx];
                }
            }
        } // end FOL

        // for FCL
        if (radar->FCL < RADAR_338_DIST_THR)
        {
            Get_Object_Location(FCL_AERFA, FCL_DELTAX, FCL_DELTAY, radar->FCL, vel_pos,
                                &pt);
            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                // statistics fcl
                if (g_front_conf[1] < CENT_POINT_PERCENT)
                {
                    g_front_conf[1] += (CENT_POINT_PERCENT / CENT_POINT_THR);
                }

                xy  = Point_Project_Line(pt, g_park_cd);
                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_front_start) /
                            (2 * CHECK_FR_RANGE));
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    // point project statistics
                    ++g_front_stat[idx];
                }
            }
        } // end FCL

        // for FCR
        if (radar->FCR < RADAR_338_DIST_THR)
        {
            Get_Object_Location(FCR_AERFA, FCR_DELTAX, FCR_DELTAY, radar->FCR, vel_pos,
                                &pt);

            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                // statistics fcr
                if (g_front_conf[2] < CENT_POINT_PERCENT)
                {
                    g_front_conf[2] += (CENT_POINT_PERCENT / CENT_POINT_THR);
                }
                xy = Point_Project_Line(pt, g_park_cd);

                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_front_start) /
                            (2 * CHECK_FR_RANGE));

                // point project statistics
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_front_stat[idx];
                }
            }
        } // end FCR

        // for FOR
        if (radar->FOR < RADAR_338_DIST_THR)
        {
            Get_Object_Location(FOR_AERFA, FOR_DELTAX, FOR_DELTAY, radar->FOR, vel_pos,
                                &pt);
            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                // statistics for
                if (g_front_conf[3] < OUT_POINT_PERCENT)
                {
                    g_front_conf[3] += (OUT_POINT_PERCENT / OUT_POINT_THR);
                }
                xy  = Point_Project_Line(pt, g_park_cd);
                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_front_start) /
                            (2 * CHECK_FR_RANGE));

                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_front_stat[idx];
                } // point project statistics
            }
        } // end FOR

        // get max statistics idx and value
        max_stat = 0;
        idx      = 0;
        for (i = 0; i < STAT_SIZE; ++i)
        {
            if (g_front_stat[i] >= max_stat)
            {
                max_stat = g_front_stat[i];
                idx      = i;
            }
        }

        // calculate front move vector and weight
        if (max_stat > 4)
        {
            front_conf = 0.0f;
            for (i = 0; i < 4; ++i)
            {
                front_conf += g_front_conf[i];
            }

            val = -(2 * CHECK_FR_RANGE) * (-0.5f + 1.0f * idx / STAT_SIZE);
            if (front_conf > 0.7f)
            {
                g_front_mov.vc.vx =
                    (front_conf * val + (1 - front_conf) * g_front_val) * g_norm_cd.vx;
                g_front_mov.vc.vy =
                    (front_conf * val + (1 - front_conf) * g_front_val) * g_norm_cd.vy;
                g_front_mov.val = 1;
            }
            else
            {
                g_front_mov.vc.vx = g_front_val * g_ori_norm_cd.vx;
                g_front_mov.vc.vy = g_front_val * g_ori_norm_cd.vy;
                g_front_mov.val   = 0.7f;
            }
        }

        // for ROL
        if (radar->ROL < RADAR_338_DIST_THR)
        {
            Get_Object_Location(ROL_AERFA, ROL_DELTAX, ROL_DELTAY, radar->ROL, vel_pos,
                                &pt);
            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
                LOGW << "ROL detected obs on the back, radar->ROL:" << radar->ROL;
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                if (g_rear_conf[0] < OUT_POINT_PERCENT)
                {
                    g_rear_conf[0] += (OUT_POINT_PERCENT / OUT_POINT_THR);
                }
                xy = Point_Project_Line(pt, g_park_cd);

                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_rear_start) /
                            (2 * CHECK_FR_RANGE));
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_rear_stat[idx];
                }
            }
        } // end ROL

        // for RCL
        if (radar->RCL < RADAR_338_DIST_THR)
        {
            Get_Object_Location(RCL_AERFA, RCL_DELTAX, RCL_DELTAY, radar->RCL, vel_pos,
                                &pt);
            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
                LOGW << "RCL detected obs on the back, radar->RCL:" << radar->RCL;
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                if (g_rear_conf[1] < CENT_POINT_PERCENT)
                {
                    g_rear_conf[1] += (CENT_POINT_PERCENT / CENT_POINT_THR);
                }
                xy = Point_Project_Line(pt, g_park_cd);

                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_rear_start) /
                            (2 * CHECK_FR_RANGE));
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_rear_stat[idx];
                }
            }
        } // end RCL

        // for RCR
        if (radar->RCR < RADAR_338_DIST_THR)
        {
            Get_Object_Location(RCR_AERFA, RCR_DELTAX, RCR_DELTAY, radar->RCR, vel_pos,
                                &pt);

            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
                LOGW << "RCR detected obs on the back, radar->RCR:" << radar->RCR;
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                if (g_rear_conf[2] < CENT_POINT_PERCENT)
                {
                    g_rear_conf[2] += (CENT_POINT_PERCENT / CENT_POINT_THR);
                }
                xy = Point_Project_Line(pt, g_park_cd);

                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_rear_start) /
                            (2 * CHECK_FR_RANGE));
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_rear_stat[idx];
                }
            }
        } // end RCR

        // for ROR
        if (radar->ROR < RADAR_338_DIST_THR)
        {
            Get_Object_Location(ROR_AERFA, ROR_DELTAX, ROR_DELTAY, radar->ROR, vel_pos,
                                &pt);

            flag = Point_Check(npark_area, pt, CHECK_FR_RANGE, 0, CHECK_CD_RANGE);
            if (flag != 0)
            {
                LOGW << "ROR detected obs on the back, radar->ROR:" << radar->ROR;
#ifdef PK_B_DEBUG
                all_radar[all_num] = pt;
                ++all_num;
#endif
                if (g_rear_conf[3] < OUT_POINT_PERCENT)
                {
                    g_rear_conf[3] += (OUT_POINT_PERCENT / OUT_POINT_THR);
                }

                xy  = Point_Project_Line(pt, g_park_cd);
                idx = (int)(STAT_SIZE * Cal_Dis_Pt2Pt(xy, g_rear_start) /
                            (2 * CHECK_FR_RANGE));
                if (idx >= 0 && idx < STAT_SIZE)
                {
                    ++g_rear_stat[idx];
                }
            }
        } // end ROR

        // get max statistics idx and value
        max_stat = 0;
        idx      = 0;
        for (i = 0; i < STAT_SIZE; ++i)
        {
            if (g_rear_stat[i] >= max_stat)
            {
                max_stat = g_rear_stat[i];
                idx      = i;
            }
        }

        // calculate front move vector and weight
        if (max_stat > 4)
        {
            rear_conf = 0.0f;
            for (i = 0; i < 4; ++i)
            {
                rear_conf += g_rear_conf[i];
            }
            g_rear_mov.vc.vx =
                (2 * CHECK_FR_RANGE) * (-0.5f + 1.0f * idx / STAT_SIZE) * g_norm_cd.vx;
            g_rear_mov.vc.vy =
                (2 * CHECK_FR_RANGE) * (-0.5f + 1.0f * idx / STAT_SIZE) * g_norm_cd.vy;
            g_rear_mov.val = rear_conf;
            LOGW << "g_rear_mov.val:" << g_rear_mov.val;
        }
    } // end if (odo - g_last_path > VEHICLE_MOVE_DIST && cvel < 0)

    // update if write RTE
    Update_Flag();
}

static void *sflog = NULL;
static void SFBLogInit()
{
    if (sflog)
    {
        return;
    }

    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);
    char pathName[64];
    Rte_BSW_Get_LogPath(pathName);
    snprintf(fileName, sizeof(fileName) - 1,
             "%s/%d_%02d_%02d_%02d_%02d_%02d_objvoiddbg.txt", pathName, absStamp.tm_year,
             absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour, absStamp.tm_min,
             absStamp.tm_sec);

    auto logFid = fopen(fileName, "w");
    if (logFid == NULL)
    {
        return;
    }

    static uint8_t logbuff[40 * 1024] = {0};
    sflog                             = Buff_Init(logbuff, sizeof(logbuff), logFid);
    if (sflog == NULL)
    {
        fclose(logFid);
    }
}

static void SFBLogWrite(const U_RadarType &udata)
{
    if (sflog == NULL)
    {
        return;
    }

    static char sfbuff[1024] = {0};
    const int &is_avm_type   = RTE_PK_DataConvt_Get_IsAVM_Slot();
    float cvel               = RTE_PK_PathExecute_Get_TargVelspd();

    float curPos[4];
    RTE_PK_Location_Get_CurPos(curPos);

    struct timeval stamp;
    gettimeofday(&stamp, NULL);

    int writeLen = 0;
    writeLen +=
        snprintf(&sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1,
                 "%06ld:%06ld flag:%01d type:%01d bfirst_rev: %01d vel:%06lf "
                 "g_mov_stat:%03d curpos:%06lf,%06lf,%06lf,%06lf ",
                 stamp.tv_sec, stamp.tv_usec, g_update_B_flag, is_avm_type, g_bfirst_rev,
                 cvel, g_mov_stat, curPos[0], curPos[1], curPos[2], curPos[3]);

    writeLen += snprintf(
        &sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1,
        "radar:%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d,%04d ",
        (int)udata.ROL, (int)udata.RCL, (int)udata.RCR, (int)udata.ROR, (int)udata.RSL,
        (int)udata.RSR, (int)udata.FSL, (int)udata.FSR, (int)udata.RCL_ROL,
        (int)udata.RCL_RCR, (int)udata.RCR_RCL, (int)udata.RCR_ROR);

    writeLen += snprintf(&sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1, "left:");
    for (int i = 0; i < SF_OBJ_NUM; i++)
    {
        if (Get_Segment_Len(FS_ObsInfo_Left_B.obj[i]) < 0.10 ||
            FS_ObsInfo_Left_B.attr[i] == 0)
        {
            continue;
        }
        writeLen += snprintf(
            &sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1, "%04lf,%04lf,%04lf,%04lf,",
            FS_ObsInfo_Left_B.obj[i].pt1.x, FS_ObsInfo_Left_B.obj[i].pt1.y,
            FS_ObsInfo_Left_B.obj[i].pt2.x, FS_ObsInfo_Left_B.obj[i].pt2.y);
    }

    writeLen += snprintf(&sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1, "right:");
    for (int i = 0; i < SF_OBJ_NUM; i++)
    {
        if (Get_Segment_Len(FS_ObsInfo_Right_B.obj[i]) < 0.10 ||
            FS_ObsInfo_Right_B.attr[i] == 0)
        {
            continue;
        }
        writeLen += snprintf(
            &sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1, "%04lf,%04lf,%04lf,%04lf,",
            FS_ObsInfo_Right_B.obj[i].pt1.x, FS_ObsInfo_Right_B.obj[i].pt1.y,
            FS_ObsInfo_Right_B.obj[i].pt2.x, FS_ObsInfo_Right_B.obj[i].pt2.y);
    }

    writeLen += snprintf(&sfbuff[writeLen], sizeof(sfbuff) - writeLen - 1, "\n");
    Buff_Put(sflog, sfbuff, writeLen);
}

void PK_SF_B_Init(void)
{
    VehPos_T cur_vel, targpos;
    SlotObj_T slotobj;
    Avm_Pot_T avm;
    float temp[4];
    RTE_PK_Location_Get_CurPos(temp);
    memcpy(&cur_vel, temp, sizeof(cur_vel));

    g_index_obs_flag = -1; // B park lever

    // clear all global variable
    PK_SF_B_Clear();

    RTE_PK_SlotDetect_Get_SlotObj(&slotobj);
    RTE_PK_DataConvt_Get_TargAVM_SlotInfo(&avm);
    RTE_PK_SlotDetect_Get_TargPos((float *)&targpos);
    Get_A_Para(cur_vel, slotobj, targpos, avm);

    SFBLogInit();

    FS_ObsInfo_Left_B.num  = 0;
    FS_ObsInfo_Right_B.num = 0;
}

void PK_SF_B(void)
{
    VehPos_T vel_pos, targpos;
    SlotObj_T slotobj;
    float CurPos_B[4];
    U_RadarType udata_B;
    RTE_PD_Get_U_Radar(&udata_B);
    int bfirst_rev;

    // get B process some parameter
    RTE_PK_Location_Get_CurPos(CurPos_B);
    const int &SlotShap    = RTE_PK_SlotDetect_Get_SlotShape();
    const int &is_avm_type = RTE_PK_DataConvt_Get_IsAVM_Slot();

    memcpy(&vel_pos, CurPos_B, sizeof(vel_pos));
    // set current vehicle to g_cur_veh
    memcpy(&g_cur_veh, CurPos_B, sizeof(g_cur_veh));
    float cvel = RTE_PK_PathExecute_Get_TargVelspd();
    bfirst_rev = 0;

    if (is_avm_type == 0)
    {
        // radar type park
        if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
        {
            // radar parallel park
            Fit_B1_Line(vel_pos, CurPos_B[3], cvel, &udata_B);
            Calc_B1_Update(&slotobj, &targpos);
            Get_Plan_Line(&FS_ObsInfo_Left_B, &FS_ObsInfo_Right_B);
        }
        else if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT)
        {
            // radar vertical park
            Fit_B2_Line(vel_pos, CurPos_B[3], cvel, bfirst_rev, &udata_B);
            Get_Plan_Line(&FS_ObsInfo_Left_B, &FS_ObsInfo_Right_B);
        }
        else
        {
        }
    }
    else if (is_avm_type == 1 || is_avm_type == 3)
    {
        // avm type park
        if (SlotShap == PK_SLOT_LEFT_PARA || SlotShap == PK_SLOT_RIGHT_PARA)
        {
            // avm parallel park
            Fit_B1_Line(vel_pos, CurPos_B[3], cvel, &udata_B);
            Get_Plan_Line(&FS_ObsInfo_Left_B, &FS_ObsInfo_Right_B);
        }
        else if (SlotShap == PK_SLOT_LEFT_VERT || SlotShap == PK_SLOT_RIGHT_VERT ||
                 SlotShap == PK_SLOT_LEFT_ANG_REVERSE ||
                 SlotShap == PK_SLOT_RIGHT_ANG_REVERSE)
        {
            // avm vertical park
            Fit_B2_Line(vel_pos, CurPos_B[3], cvel, bfirst_rev, &udata_B);
            Get_Plan_Line(&FS_ObsInfo_Left_B, &FS_ObsInfo_Right_B);
        }
        else
        {
        }
    }

    SFBLogWrite(udata_B);

    // set B sensor fusion line segment
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Left(&FS_ObsInfo_Left_B);
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Right(&FS_ObsInfo_Right_B);
}

// Get update_B_flag,just for debug
int PK_Get_Update_B_Flag() { return g_update_B_flag; }

void PK_SF_B_End(void)
{
    if (sflog != NULL)
    {
        Buff_UnInit(sflog);
        sflog = NULL;
    }
}

#ifdef PK_B_DEBUG
// set current vehicle,just for debug
void Set_veh_Pos(VehPos_T cur) { g_cur_veh = cur; }
#endif
