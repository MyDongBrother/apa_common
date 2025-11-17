#ifndef _URADAR_AVOIDSTATE_H
#define _URADAR_AVOIDSTATE_H
#include "Rte.h"
#include "PK_ObjAvoid.h"

enum
{
    STATE_SAFE = 0,
    STATE_DANGER,
    STATE_STOP
};

enum
{
    DIR_NODANGER = 0,
    DIR_FRONTDANGER,
    DIR_BACKDANGER
};

inline bool ObjAvoid_IsStopState(uint32_t dangerSt) { return ((dangerSt & 0xff00) != 0); }

inline bool ObjAvoid_IsDangerState(uint32_t dangerSt)
{
    return ((dangerSt & 0x00ff) != 0);
}

inline bool ObjAvoid_IsStopIdx(int idx) { return (PK_RadarDangerSt(idx) == STATE_STOP); }

inline bool ObjAvoid_IsDangerIdx(int idx)
{
    return (PK_RadarDangerSt(idx) == STATE_DANGER);
}

#endif
