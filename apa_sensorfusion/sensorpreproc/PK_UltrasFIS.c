/**
 * @file PK_UltrasFIS.c
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
** Include headers of the component
*********************************************************************************************************/
#include "PK_UltrasFIS.h"
#include "Rte_Types.h"
#include "Module_API.h"
#include "Rte_BSW.h"
#include "PK_Utility.h"
#include "PK_Calibration.h"
#include "Rte_ComIF.h"
#include <stdlib.h>
#include <string.h>

/*********************************************************************************************************
** Include other headers
*********************************************************************************************************/

/*********************************************************************************************************
** Check component version
*********************************************************************************************************/
/* Check if source file and COMMON header file are of the same software version */
#if ((PK_UltrasFIS_SW_MAJOR_VERSION != (0u)) || \
     (PK_UltrasFIS_SW_MINOR_VERSION != (0u)) || (PK_UltrasFIS_SW_PATCH_VERSION != (4U)))
#error "Versions of PK_UltrasFIS.c and PK_UltrasFIS.h are inconsistent!"
#endif

/*********************************************************************************************************
**Definition of exported symbolic constants
*********************************************************************************************************/
#pragma section all "CPU1.Private"

/*********************************************************************************************************
**Definition of exported variables
*********************************************************************************************************/
static uint16 FIS_uRadarTarget[12] = {255, 255, 255, 255, 255, 255,
                                      255, 255, 510, 510, 510, 510};

/*********************************************************************************************************
**Definition of static variables
*********************************************************************************************************/
#define MAX_RANGE_UPA 160
#define MAX_RANGE_APA 500

#define InterCycleDelta    30
#define InterCycleDeltaMin 20
#define InterCycleDeltaMax 70

static boolean bInited = FALSE;

/* front\rear radar */
#define NR_OBSV_NUM 7

#define FKR_UPA 3
static uint8 uRadarTargets[2][4][FKR_UPA];
#define findNextIdx(i) ((i == FKR_UPA - 1) ? 0 : (i + 1))

/* side radar */
#define FKR_APA 7
static uint16 uRadarTargets_APA[2][2][FKR_APA];

#define findNextIdx_APA(i) ((i == FKR_APA - 1) ? 0 : (i + 1))

/* Noise percent calculation */
#define WIN_WIDTH 30
static uint8 WinNoiseStatL[WIN_WIDTH] = {0};
static uint8 WinNoiseStatR[WIN_WIDTH] = {0};
#define findNextIdx_NS(i) ((i == WIN_WIDTH - 1) ? 0 : (i + 1))

static void FIS_updateCCP(U_RadarInfoType uri[2][6]);

void PK_UltrasFIS(void)
{
    uint8 i, j, jj;

    if (!bInited)
    {
        memset(uRadarTargets, 255, sizeof(uRadarTargets));
        for (i = 0; i < 2; i++)
            for (j = 0; j < 2; j++)
                for (jj = 0; jj < FKR_APA; jj++) uRadarTargets_APA[i][j][jj] = 510;

        bInited = TRUE;
    }

    // FIS_updateCCP();
    return;
}

void insertionSort_u8(uint8 arr[], int n)
{
    int i, j;
    uint8 key;
    for (i = 1; i < n; i++)
    {
        key = arr[i];
        j   = i - 1;

        /* move elements of arr[0..i-1], that are
        greater than key, to one position ahead
        of their current position */
        while (j >= 0 && arr[j] > key)
        {
            arr[j + 1] = arr[j];
            j          = j - 1;
        }
        arr[j + 1] = key;
    }
    return;
}

void insertionSort_u16(uint16 arr[], int n)
{
    int i, j;
    uint16 key;
    for (i = 1; i < n; i++)
    {
        key = arr[i];
        j   = i - 1;

        /* move elements of arr[0..i-1], that are
        greater than key, to one position ahead
        of their current position */
        while (j >= 0 && arr[j] > key)
        {
            arr[j + 1] = arr[j];
            j          = j - 1;
        }
        arr[j + 1] = key;
    }
    return;
}

/* Method 0: noise_val = 0.99f * noise_val + 0.01f * NoiseErr;
 *           Increase quick, decrease slow;
 * Method 1: #define WIN_WIDTH 350
 *           #define WIN_STAT_SLOT_WIDTH 35
 *           Window method, but focus on distance,
 *           may lead to statistic of small sample set under fast speed
 * Method 2: statistic of fixed number of samples in the past
 *           balanced solution
 */
static uint8 calcNoisePercentByWindow(uint8 NoiseL, uint8 NoiseR)
{
    static uint8 CurrIdx = 0;
    uint8 NoiseCntL      = 0;
    uint8 NoiseCntR      = 0;
    uint8 i;

    // if (LinApp_GetStatus() < 0)
    {
        memset(WinNoiseStatL, 0, WIN_WIDTH);
        memset(WinNoiseStatR, 0, WIN_WIDTH);
        CurrIdx = 0;
        return 0;
    }

    /* Cache the current noise status */
    WinNoiseStatL[CurrIdx] = NoiseL;
    WinNoiseStatR[CurrIdx] = NoiseR;
    CurrIdx                = findNextIdx_NS(CurrIdx);

    /* Caculate thte noise percent */
    for (i = 0; i < WIN_WIDTH; i++)
    {
        NoiseCntL += WinNoiseStatL[i];
        NoiseCntR += WinNoiseStatR[i];
    }
    return ((NoiseCntL > NoiseCntR) ? ((uint32)NoiseCntL * 100 / 30)
                                    : ((uint32)NoiseCntR * 100 / 30));
}

void FIS_getURadarFiltData(U_RadarType *urd)
{
    urd->FOL     = FIS_uRadarTarget[0];
    urd->FCL     = FIS_uRadarTarget[1];
    urd->FCR     = FIS_uRadarTarget[2];
    urd->FOR     = FIS_uRadarTarget[3];
    urd->ROL     = FIS_uRadarTarget[4];
    urd->RCL     = FIS_uRadarTarget[5];
    urd->RCR     = FIS_uRadarTarget[6];
    urd->ROR     = FIS_uRadarTarget[7];
    urd->FSL     = FIS_uRadarTarget[8];
    urd->RSL     = FIS_uRadarTarget[9];
    urd->FSR     = FIS_uRadarTarget[10];
    urd->RSR     = FIS_uRadarTarget[11];
    urd->FCL_FOL = 255;
    urd->FCL_FCR = 255;
    urd->FCR_FCL = 255;
    urd->FCR_FOR = 255;
    urd->RCL_ROL = 255;
    urd->RCL_RCR = 255;
    urd->RCR_RCL = 255;
    urd->RCR_ROR = 255;
    urd->FSL_2   = 510;
    urd->RSL_2   = 510;
    urd->FSR_2   = 510;
    urd->RSR_2   = 510;
    return;
}

static void FIS_updateCCP(U_RadarInfoType uri[2][6])
{
    for (uint8_t linkId = 0; linkId < 2; linkId++)
    {
        for (uint8_t index = 0; index < 6; index++)
        {
            if (uri[linkId][index].Status != 0)
            {
                printf("Ultronic %d %d Failed! error status %d, distance %d\r\n", linkId,
                       index, uri[linkId][index].Status, uri[linkId][index].Distance);
            }
        }
    }
}
