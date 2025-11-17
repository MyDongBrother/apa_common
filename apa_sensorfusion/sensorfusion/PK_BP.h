/**
 * @file PK_BP.h
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

#ifndef PK_B_PRIVATE_H
#define PK_B_PRIVATE_H
#include "PK_B.h"

#ifdef __cplusplus
extern "C"
{
#endif

// #define PK_B_DEBUG
#ifdef PK_B_DEBUG
    // For Test
    Point_T all_radar[5000];
    int all_num;

    Point_T indir_mid[2000];
    int mid_num;
#endif

#ifdef PK_SlotDetect_OffLineTest
    extern int g_IsBSlotUpata;
    extern SlotObj_T g_SlotObj_After;
    extern VehPos_T g_TargPos_After;
    extern SlotObj_T g_SlotObj_before;
    extern VehPos_T g_TargPos_before;
#endif

    static Vec2_T Norm_Lineseq(Point_T spt, Point_T ept);
    static Point_T Get_Mid_Point(Point_T spt, Point_T ept);
    static void Get_Object_Location(float aerfa, float deltax, float deltay, float radar,
                                    VehPos_T vel, Point_T *obs);
    static void Polyfit_Line(const Point_T *pData, const int num, LineSeg_T *fit_point);
    static float Leaf_Median_Filter(const float *in, const int num);
    static float Point_Line_Distance(const Point_T &pt, const LineSeg_T &line);
    static Point_T Point_Project_Line(const Point_T &pt, const LineSeg_T &ln);
    static float Line_Project_length(const LineSeg_T &ln1, const LineSeg_T &ln2);
    static int In_or_Not(const Point_T *poly, Point_T pt);
    static float Get_Line_Seg_Angle(const LineSeg_T &seq1, const LineSeg_T &seq2);
    static void Convert_Park_Area(const int slot_shape, const SlotObj_T *src,
                                  SlotObj_T *dst);
    static int Calc_Triangle_Third(Point_T pa, Point_T pb, Point_T *pc, float a, float b,
                                   float c, int is_left);
    static void Calc_Rear_Indi(VehPos_T vel_pos, const U_RadarType *radar);
    static int Is_Lineseg_Cross_Slot(SlotObj_T slot, Point_T pt1, Point_T pt2);
    static void Update_Flag(void);
    static void Judge_Obj_Cross(VehPos_T *veh, const Avm_Obj_T *avm_obj);

    void Get_A_Para(const FusionObj_T *lfus, const FusionObj_T *rfus, VehPos_T vel_pos,
                    SlotObj_T pk_area, VehPos_T tar_vel, Avm_Pot_T avm_pt, int slot_shape,
                    int is_avm);

    void Fit_B2_Line(VehPos_T vel_pos, float odo, float cvel, int b_first_rev,
                     const U_RadarType *radar);
    void Calc_B2_Update(SlotObj_T *slo_obj, VehPos_T *ud_vp);
    void Calc_AVM_B2_Update(SlotObj_T *slo_obj, VehPos_T *ud_vp, const Avm_Pot_T *avm_pt,
                            const Avm_Obj_T *avm_obj);

    void Fit_B1_Line(VehPos_T vel_pos, float odo, float cvel, const U_RadarType *radar);
    void Calc_B1_Update(SlotObj_T *slo_obj, VehPos_T *ud_vp);
    void Calc_AVM_B1_Update(SlotObj_T *slo_obj, VehPos_T *ud_vp);

    void Get_Plan_Line(FusionObj_T *lfus_obj, FusionObj_T *rfus_obj);

#ifdef PK_B_DEBUG
    void Set_veh_Pos(VehPos_T cur);
#endif

#ifdef __cplusplus
}
#endif
#endif // PK_B_H
