#include "DistCtrl.h"

void PK_DistanceCtrl(void)
{
    if (PK_VehConfigType() == CAR_TYPE_YUAN)
    {
        PK_DistanceCtrl_yuan();
    }
    else if (PK_VehConfigType() == CAR_TYPE_HAN)
    {
        PK_DistanceCtrl_han();
    }
    else
    {
        printf("------------ Error Car Type --------------------");
        return;
    }
}
