#include "T2S_List.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#if ((PK_SensorT2S_LIST_SW_MAJOR_VERSION != (0u)) || \
     (PK_SensorT2S_LIST_SW_MINOR_VERSION != (1u)) || \
     (PK_SensorT2S_LIST_SW_PATCH_VERSION != (3U)))
#error "Versions of T2S_List.c and T2S_List.h are inconsistent!"
#endif

typedef struct _node_t
{
    struct _node_t *prev;
    struct _node_t *next;
    RD_PosStamp data;
} node_t;

typedef struct
{
    node_t *buff;
    node_t *head;
    node_t *tail;
    int cap;
    int len;
} clist_t;

#pragma section all "CPU1.Private"

#define CLIST_CAP 20
static clist_t cList;
static node_t NodeArray[CLIST_CAP];

void *T2S_CList_Init(void)
{
    int i;
    for (i = CLIST_CAP - 2; i > 0; i--)
    {
        NodeArray[i].prev = &(NodeArray[i - 1]);
        NodeArray[i].next = &(NodeArray[i + 1]);
    }
    NodeArray[0].prev             = &(NodeArray[CLIST_CAP - 1]);
    NodeArray[0].next             = &(NodeArray[1]);
    NodeArray[CLIST_CAP - 1].prev = &(NodeArray[CLIST_CAP - 2]);
    NodeArray[CLIST_CAP - 1].next = &(NodeArray[0]);

    cList.cap  = CLIST_CAP;
    cList.len  = 0;
    cList.head = &(NodeArray[0]);
    cList.tail = &(NodeArray[0]);

    return &cList;
}

void T2S_CList_NewIn(void *handle, const RD_PosStamp *pData)
{
    clist_t *pList = (clist_t *)handle;
    if (pList->len == pList->cap)
    {
        pList->tail = pList->head;
        pList->head = pList->head->next;
        memcpy(&pList->tail->data, pData, sizeof(RD_PosStamp));
    }
    else if (pList->len > 0)
    {
        pList->tail = pList->tail->next;
        memcpy(&pList->tail->data, pData, sizeof(RD_PosStamp));
        pList->len = pList->len + 1;
    }
    else
    {
        memcpy(&pList->head->data, pData, sizeof(RD_PosStamp));
        pList->len = pList->len + 1;
    }

    return;
}

int T2S_CList_Search(const void *handle, const uint32 t, RD_PosStamp **pDataL,
                     RD_PosStamp **pDataH)
{
    clist_t *pList = (clist_t *)handle;
    node_t *curr;

    if (0 == pList->len)
    {
        return -1;
    }

    /* Clamp */
    if (t >= pList->tail->data.t)
    {
        *pDataL = &pList->tail->data;
        *pDataH = &pList->tail->data;
        return -2;
    }

    if (t <= pList->head->data.t)
    {
        *pDataL = &pList->head->data;
        *pDataH = &pList->head->data;
        return -2;
    }

    /* Start from the latest */
    curr = pList->tail->prev;
    while (1)
    { /* Never will become a dead loop */
        if (t >= curr->data.t)
            break;
        curr = curr->prev;
    }
    *pDataL = &curr->data;
    *pDataH = &curr->next->data;

    return 0;
}

int T2S_CList_GetAll(const void *handle, RD_PosStamp Data[50])
{
    clist_t *pList = (clist_t *)handle;

    if (0 == pList->len)
    {
        return -1;
    }

    node_t *head = pList->head;
    for (int i = 0; i < pList->len; i++)
    {
        memcpy(&Data[i], &head->data, sizeof(RD_PosStamp));
        head = head->next;
    }

    return pList->len;
}

int T2S_Clist_GetLength(const void *handle)
{
    clist_t *pList = (clist_t *)handle;
    return pList->len;
}

void T2S_Clist_Clear(void *handle)
{
    clist_t *pList = (clist_t *)handle;
    pList->len     = 0;
    pList->head    = &(NodeArray[0]);
    pList->tail    = &(NodeArray[0]);
    return;
}

// void T2S_Clist_Delete(void *handle)
// {
//     clist_t *pList = (clist_t *)handle;
//     return;
// }
