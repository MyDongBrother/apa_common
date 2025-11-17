#include "Uradar_ObjAvoid.h"
#include "Uradar_AvoidState.h"
#include "Record_Log.h"

enum
{
    brake_idx = 1,
    stop_idx  = 2
};
const int radarState_len                   = 5;
static uint32_t radarState[radarState_len] = {0xa5a5a5a5, 0, 0, 0x5a5a5a5a, 0xffffffff};

static bool CheckRadarState()
{
    uint32_t retSum = 0;
    for (int i = 0; i < radarState_len - 1; i++)
    {
        retSum += radarState[i];
    }
    return (retSum == radarState[radarState_len - 1] && radarState[0] == 0xa5a5a5a5 &&
            radarState[radarState_len - 2] == 0x5a5a5a5a);
}

static void UpdateRadarState()
{
    uint32_t retSum = 0;
    for (int i = 0; i < radarState_len - 1; i++)
    {
        retSum += radarState[i];
    }
    radarState[radarState_len - 1] = retSum;
}

static void SetStateBit(const int index, const int flag)
{
    ASSERT(CheckRadarState());

    if (index >= Radar_ID_MAX)
    {
        log_warn("uadar index error! %d", index);
        return;
    }

    uint32_t mask = (1 << index);
    if (flag == SET_BRAKE)
    {
        radarState[brake_idx] = radarState[brake_idx] | mask;
    }
    else if (flag == SET_STOP)
    {
        radarState[stop_idx] = radarState[stop_idx] | mask;
    }
    else if (flag == CANCEL_BRAKE)
    {
        mask                  = ~mask;
        radarState[brake_idx] = radarState[brake_idx] & mask;
    }
    else if (flag == CANCEL_STOP)
    {
        mask                 = ~mask;
        radarState[stop_idx] = radarState[stop_idx] & mask;
    }

    UpdateRadarState();
}

static bool IsSetStateBit(const int index, const int flag)
{
    ASSERT(CheckRadarState());

    if (index >= Radar_ID_MAX)
    {
        log_warn("uadar index error! %d", index);
        return false;
    }

    uint32_t mask = (1 << index);
    if (flag == SET_BRAKE)
    {
        return ((radarState[brake_idx] & mask) == 0x0) ? false : true;
    }
    else if (flag == SET_STOP)
    {
        return ((radarState[stop_idx] & mask) == 0x0) ? false : true;
    }
    else
    {
        return false;
    }
}

static void SetDangerBit(const int index) { return SetStateBit(index, SET_BRAKE); }

static void SetFatalBit(const int index)
{
    SetStateBit(index, SET_STOP);
    SetStateBit(index, SET_BRAKE);
}

static void ClearDangerBit(const int index)
{
    SetStateBit(index, CANCEL_BRAKE);
    SetStateBit(index, CANCEL_STOP);
}

static void ClearFatalBit(const int index) { SetStateBit(index, CANCEL_STOP); }

static uint16_t GetUradarState()
{
#define ISSETDANGER(ID) IsSetStateBit(ID, SET_BRAKE)
#define ISSETSTOP(ID)   IsSetStateBit(ID, SET_STOP)

    uint16_t result = 0;
    if (ISSETDANGER(Radar_ID_FSR))
    {
        result = result | 0x0002;
    } // FSR
    if (ISSETDANGER(Radar_ID_FOR) && !ISSETDANGER(Radar_ID_FCR))
    {
        result = result | 0x0002;
    } // FOR !FCR
    if (ISSETDANGER(Radar_ID_FSL))
    {
        result = result | 0x0001;
    } // FSL
    if (ISSETDANGER(Radar_ID_FOL) && !ISSETDANGER(Radar_ID_FCL))
    {
        result = result | 0x0001;
    } // FOL !FCL
    if (ISSETDANGER(Radar_ID_FCR))
    {
        result = result | 0x0008;
    } // FCR
    if (ISSETDANGER(Radar_ID_FCL))
    {
        result = result | 0x0004;
    } // FCL

    if (ISSETDANGER(Radar_ID_RSR))
    {
        result = result | 0x0020;
    } // RSR
    if (ISSETDANGER(Radar_ID_ROR) && !ISSETDANGER(Radar_ID_RCR))
    {
        result = result | 0x0020;
    } // ROR !RCR
    if (ISSETDANGER(Radar_ID_RSL))
    {
        result = result | 0x0010;
    } // RSL
    if (ISSETDANGER(Radar_ID_ROL) && !ISSETDANGER(Radar_ID_RCL))
    {
        result = result | 0x0010;
    } // ROL !RCL
    if (ISSETDANGER(Radar_ID_RCR))
    {
        result = result | 0x0080;
    } // RCR
    if (ISSETDANGER(Radar_ID_RCL))
    {
        result = result | 0x0040;
    } // RCL ROL

    if (ISSETSTOP(Radar_ID_FSR))
    {
        result = result | 0x0200;
    } // FSR
    if (ISSETSTOP(Radar_ID_FOR) && !ISSETSTOP(Radar_ID_FCR))
    {
        result = result | 0x0200;
    } // FOR !FCR
    if (ISSETSTOP(Radar_ID_FSL))
    {
        result = result | 0x0100;
    } // FSL
    if (ISSETSTOP(Radar_ID_FOL) && !ISSETSTOP(Radar_ID_FCL))
    {
        result = result | 0x0100;
    } // FOL !FCL
    if (ISSETSTOP(Radar_ID_FCR))
    {
        result = result | 0x0800;
    } // FCR
    if (ISSETSTOP(Radar_ID_FCL))
    {
        result = result | 0x0400;
    } // FCL

    if (ISSETSTOP(Radar_ID_RSR))
    {
        result = result | 0x2000;
    } // FSR
    if (ISSETSTOP(Radar_ID_ROR) && !ISSETSTOP(Radar_ID_RCR))
    {
        result = result | 0x2000;
    } // ROR !RCR
    if (ISSETSTOP(Radar_ID_RSL))
    {
        result = result | 0x1000;
    } // RSL
    if (ISSETSTOP(Radar_ID_ROL) && !ISSETSTOP(Radar_ID_RCL))
    {
        result = result | 0x1000;
    } // ROL !RCL

    if (ISSETSTOP(Radar_ID_RCR))
    {
        result = result | 0x8000;
    } // RCR
    if (ISSETSTOP(Radar_ID_RCL))
    {
        result = result | 0x4000;
    } // RCL ROL

    return result;
}

uint16_t SetUradarStateBit(const int index, const uint16_t dangerType, const int flag)
{
    if (flag == SET_BRAKE)
    {
        SetDangerBit(index);
    }

    if (flag == SET_STOP)
    {
        SetFatalBit(index);
    }

    if (flag == CANCEL_BRAKE)
    {
        ClearDangerBit(index);
    }

    if (flag == CANCEL_STOP)
    {
        ClearFatalBit(index);
    }

    uint16_t result = GetUradarState();
    return result;
}

uint8_t PK_ObjAvoid_Dir()
{
    uint16_t dangerSt = RTE_PK_ObjAvoid_Get_DangerSt();
    if (dangerSt == 0)
    {
        return 0;
    }

    return ((dangerSt & 0x0f) != 0x00) ? 1 : 2;
}

int PK_RadarDangerSt(const int idx)
{
    if (idx >= Radar_ID_MAX)
    {
        return STATE_SAFE;
    }

    if (IsSetStateBit(idx, SET_STOP))
    {
        return STATE_STOP;
    }

    if (IsSetStateBit(idx, SET_BRAKE))
    {
        return STATE_DANGER;
    }

    return STATE_SAFE;
}
