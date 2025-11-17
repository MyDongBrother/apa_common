//
// Copyright 2020 Horizon
//
/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "ApaCtrlCarBody.h"
#include "ApaRecvInfoPro.h"
#include "Rte_Func.h"
#include "SpiIf.h"

int Apa_CanInit(const char *logName)
{
    Can_InitRecvInfo(logName);
    Can_InitSendInfo();
    return 0;
}
