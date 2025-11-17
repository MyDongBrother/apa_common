#include "Record_Log.h"
#include "Record_Buff.h"
#include "MathFunc.h"
#include "Rte_Func.h"

typedef struct
{
    uint8_t *data;
    int buffLen;
    int head;
    int validLen;
    FILE *logFd;
    int counter;
    pthread_mutex_t mutex;
} Buff_Inf;

static void TrySaveLog(void *bufFd);
static void *CloseLog(void *bufFd);
static void *SaveBuffToLog(void *args);

void *Buff_Init(uint8_t *buff, const int buffLen, FILE *logFd)
{
    memset(buff, 0, sizeof(Buff_Inf));
    Buff_Inf *buffInf = (Buff_Inf *)buff;
    if (buffInf == nullptr)
    {
        return NULL;
    }

    if (logFd == NULL)
    {
        return NULL;
    }

    if ((uint32_t)buffLen <= sizeof(Buff_Inf) + 256)
    {
        return NULL;
    }

    if (pthread_mutex_init(&buffInf->mutex, NULL) < 0)
    {
        printf("pthread_mutex_init %d %s\n", errno, strerror(errno));
        return NULL;
    }

    buffInf->data     = buff + sizeof(Buff_Inf);
    buffInf->buffLen  = buffLen - sizeof(Buff_Inf);
    buffInf->logFd    = logFd;
    buffInf->head     = 0;
    buffInf->validLen = 0;
    buffInf->counter  = 0;

    return (void *)buffInf;
}

void Buff_UnInit(void *bufFd)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;

    if (buffInf == nullptr)
    {
        return;
    }
    if (buffInf->logFd == NULL)
    {
        pthread_mutex_destroy(&buffInf->mutex);
        return;
    }

    if (buffInf->validLen <= 0)
    {
        fclose(buffInf->logFd);
        buffInf->logFd = NULL;
        pthread_mutex_destroy(&buffInf->mutex);
        return;
    }

    // cache old buff
    int save_buff_len = Buff_Len(bufFd);
    int offset        = sizeof(Buff_Inf);
    uint8_t *data     = (uint8_t *)malloc(save_buff_len + offset);
    int readLen       = Buff_Get(bufFd, data + offset, save_buff_len);
    Buff_Inf *tempInf = (Buff_Inf *)data;
    tempInf->data     = data + offset;
    tempInf->logFd    = buffInf->logFd;
    tempInf->validLen = -1;
    tempInf->buffLen  = readLen;

    // async save.
    int result = ASyncPool(SaveBuffToLog, tempInf);
    if (result != 0)
    {
        free((void *)data);
        fclose(buffInf->logFd);
    }

    // free old cache
    buffInf->logFd    = NULL;
    buffInf->validLen = 0;
    pthread_mutex_destroy(&buffInf->mutex);
}

int Buff_Put(void *bufFd, const char *data, const int dataLen)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;
    if (buffInf == nullptr || dataLen <= 0)
    {
        return 0;
    }

    pthread_mutex_lock(&buffInf->mutex);
    if (dataLen > buffInf->buffLen - buffInf->validLen)
    {
        pthread_mutex_unlock(&buffInf->mutex);
        return 0;
    }

    auto start = (buffInf->head + buffInf->validLen) % buffInf->buffLen;
    if (start + dataLen <= buffInf->buffLen)
    {
        memcpy(buffInf->data + start, data, dataLen);
    }
    else
    {
        auto left = buffInf->buffLen - start;
        memcpy(buffInf->data + start, data, left);
        memcpy(buffInf->data, &data[left], dataLen - left);
    }

    buffInf->validLen = buffInf->validLen + dataLen;
    buffInf->counter++;
    pthread_mutex_unlock(&buffInf->mutex);

    TrySaveLog(bufFd);

    return dataLen;
}

int Buff_Get(void *bufFd, uint8_t *data, const int dataLen)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;

    if (buffInf == nullptr || buffInf->validLen <= 0)
    {
        return 0;
    }

    pthread_mutex_lock(&buffInf->mutex);
    int readLen = rte_min(dataLen, buffInf->validLen);
    if (buffInf->head + readLen <= buffInf->buffLen)
    {
        memcpy(data, buffInf->data + buffInf->head, readLen);
        buffInf->head     = buffInf->head + readLen;
        buffInf->validLen = buffInf->validLen - readLen;
    }
    else
    {
        auto left = buffInf->buffLen - buffInf->head;
        memcpy(data, buffInf->data + buffInf->head, left);
        memcpy(&data[left], buffInf->data, readLen - left);
        buffInf->head     = readLen - left;
        buffInf->validLen = buffInf->validLen - readLen;
    }
    pthread_mutex_unlock(&buffInf->mutex);

    return readLen;
}

int Buff_Len(void *bufFd)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;
    if (buffInf == nullptr)
    {
        return 0;
    }
    pthread_mutex_lock(&buffInf->mutex);
    int result = buffInf->validLen;
    pthread_mutex_unlock(&buffInf->mutex);
    return result;
}

static void *SaveBuffToLog(void *args)
{
    Buff_Inf *buffInf = (Buff_Inf *)args;
    if (buffInf == nullptr)
    {
        return NULL;
    }

    if (buffInf->logFd == NULL)
    {
        free((void *)buffInf);
        return 0;
    }

    int buffLen   = buffInf->buffLen;
    uint8_t *data = buffInf->data;
    int writelen  = 0;
    while (buffLen > 0)
    {
        int result = fwrite(data + writelen, 1, buffLen, buffInf->logFd);
        if (result > 0)
        {
            writelen = writelen + result;
            buffLen  = buffLen - result;
        }
    }

    if (buffInf->validLen < 0)
    {
        fclose(buffInf->logFd);
    }

    free((void *)buffInf);
    return 0;
}

static void TrySaveLog(void *bufFd)
{
    Buff_Inf *buffInf = (Buff_Inf *)bufFd;
    if (buffInf == nullptr || buffInf->logFd == NULL)
    {
        return;
    }

    pthread_mutex_lock(&buffInf->mutex);
    bool savelog = (4 * buffInf->validLen > 3 * buffInf->buffLen) ||
                   (buffInf->validLen > 0 && buffInf->counter > 300);
    if (savelog)
    {
        buffInf->counter = 0;
    }
    auto logfd = buffInf->logFd;
    pthread_mutex_unlock(&buffInf->mutex);

    if (savelog)
    {
        // cache old buff
        int save_buff_len = Buff_Len(bufFd);
        int offset        = sizeof(Buff_Inf);
        uint8_t *data     = (uint8_t *)malloc(save_buff_len + offset);
        int readLen       = Buff_Get(bufFd, data + offset, save_buff_len);

        if (readLen <= 0)
        {
            free(data);
            return;
        }

        Buff_Inf *tempInf = (Buff_Inf *)data;
        tempInf->data     = data + offset;
        tempInf->logFd    = logfd;
        tempInf->buffLen  = readLen;
        tempInf->validLen = readLen;

        // async save
        int result = ASyncPool(SaveBuffToLog, tempInf);
        if (result != 0)
        {
            free((void *)data);
        }
    }
}
