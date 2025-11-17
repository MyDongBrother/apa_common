#include "Ethenet_Vision.h"
#include "Rte_Func.h"

FILE *OpenLogFile(const char *name)
{
    char fileName[128] = {0};
    struct tm absStamp;
    RTE_BSW_Get_AbsStamp(&absStamp);
#ifdef ANDROID_PLATFORM
    snprintf(fileName, sizeof(fileName) - 1, "./log/%d_%02d_%02d_%02d_%02d_%02d_%s",
             absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
             absStamp.tm_min, absStamp.tm_sec, name);
#else
    snprintf(fileName, sizeof(fileName) - 1, "/tmp/apalog/%d_%02d_%02d_%02d_%02d_%02d_%s",
             absStamp.tm_year, absStamp.tm_mon, absStamp.tm_mday, absStamp.tm_hour,
             absStamp.tm_min, absStamp.tm_sec, name);
#endif
    FILE *logFile = fopen(fileName, "w");
    return logFile;
}

void Eth_InitVisionSlot()
{
#ifndef SIMULATE
    VisionMsgRecvPtr VisionLoopFunc = GetVisionLoopFunc();
    if (VisionLoopFunc != NULL)
    {
        ASyncRun(VisionLoopFunc, (void *)"18803", "apa_dds_perc");
    }
#endif
}
