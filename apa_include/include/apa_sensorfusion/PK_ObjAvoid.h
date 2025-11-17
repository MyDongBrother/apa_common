#ifndef _PK_OBJAVOID_H_
#define _PK_OBJAVOID_H_
#include "Rte_Types.h"
#include "Rte_ComIF.h"

/** Version and module identification */
#define PK_ObjAvoid_VENDOR_ID (1u)  // 运行于SCU,  u代表无符号数
#define PK_ObjAvoid_MODULE_ID (17u) // 指定模块 PK_ObjAvoid

/** Component Version Information */
#define PK_ObjAvoid_SW_MAJOR_VERSION (2u)  // 版本2
#define PK_ObjAvoid_SW_MINOR_VERSION (5u)  // 开发版
#define PK_ObjAvoid_SW_PATCH_VERSION (58u) // 第一次修改

void PK_ObjAvoid();

uint8_t PK_ObjAvoid_Dir();

int PK_RadarDangerSt(const int idx);

#endif