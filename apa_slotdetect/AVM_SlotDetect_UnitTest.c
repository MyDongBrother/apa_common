#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "URadar_SlotDetect.h"
#include "MathFunc.h"
#include "PK_SD_Common.h"
#include "Module_API.h"
#include "MathFunc.h"
#include "AVM_SlotDetect.h"

#if defined(SIMULATE)
extern int CalAVMSlotInfo(Point_T NearFrontPt, Point_T NearRearPt, Point_T FarFrontPt,
                          Point_T FarRearPt, PK_SlotShapeType SlotShap,
                          SlotObj_T *SlotObj, VehPos_T *TargPos, int SlotObjAttr[5]);

void CalAVMSlotInfo_UnitTest()
{
    // targpos = {x = 0.883707047, y = -58.5503769,theta = -3.10621047}, slotobj_attr =
    // {0, 0, 0, 0, 0},
    //  stpoint = {-2.52314973, -67.1239014, -3.09068346
    Point_T near_rear;
    Point_T far_rear;
    Point_T far_front;
    Point_T near_front;

    float avpoint[] = {-6.679276,  -55.698040, -11.772776, -56.540222,
                       -12.193355, -53.996578, -7.099855,  -53.154396};
    memcpy(&near_rear, &avpoint[0], sizeof(float) * 2);
    memcpy(&far_rear, &avpoint[2], sizeof(float) * 2);
    memcpy(&far_front, &avpoint[4], sizeof(float) * 2);
    memcpy(&near_front, &avpoint[6], sizeof(float) * 2);

    PK_SlotShapeType SlotShap = PK_SLOT_RIGHT_VERT;

    SlotObj_T SlotObj;
    VehPos_T TargPos;
    int SlotObjAttr[5];

    CalAVMSlotInfo(near_front, near_rear, far_front, far_rear, SlotShap, &SlotObj,
                   &TargPos, SlotObjAttr);
}

#endif
