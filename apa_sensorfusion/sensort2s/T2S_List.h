#ifndef PK_SENSORT2S_LIST_H
#define PK_SENSORT2S_LIST_H

#include "Rte_Types.h"

/** Component Version Information */
#define PK_SensorT2S_LIST_SW_MAJOR_VERSION (0u)
#define PK_SensorT2S_LIST_SW_MINOR_VERSION (1u)
#define PK_SensorT2S_LIST_SW_PATCH_VERSION (3u)

/* To implement a circular list
 * The capacity of the list is fixed. */

void *T2S_CList_Init(void);
void T2S_CList_NewIn(void *handle, const RD_PosStamp *pData);
int T2S_CList_Search(const void *handle, const uint32 time, RD_PosStamp **pDataL,
                     RD_PosStamp **pDataH);
int T2S_Clist_GetLength(const void *handle);
void T2S_Clist_Clear(void *handle);
void T2S_Clist_Delete(void *handle);
int T2S_CList_GetAll(const void *handle, RD_PosStamp Data[50]);
#endif