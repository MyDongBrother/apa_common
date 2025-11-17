#ifndef RECORD_LOG_HEADER__
#define RECORD_LOG_HEADER__
#include <signal.h>
#include <math.h>

enum LOG_LEVEL
{
    LOG_INFO = 0,
    LOG_WARNING,
    LOG_ERROR,
    LOG_FATAL
};

enum
{
    PK_DETECTSLOT = 0,
    PK_LOCATION,
    PK_PATHPLAN,
};

void LOG_Output(int type, LOG_LEVEL level, const char *file, int line, const char *fmt,
                ...);

#define log_info(_fmt, _args...)                                    \
    {                                                               \
        LOG_Output(1, LOG_INFO, __FILE__, __LINE__, _fmt, ##_args); \
    }

#define log_warn(_fmt, _args...)                                       \
    {                                                                  \
        LOG_Output(1, LOG_WARNING, __FILE__, __LINE__, _fmt, ##_args); \
    }

#define log_err(_fmt, _args...)                                      \
    {                                                                \
        LOG_Output(1, LOG_ERROR, __FILE__, __LINE__, _fmt, ##_args); \
    }

#define log_fatal(_fmt, _args...)                                    \
    {                                                                \
        LOG_Output(1, LOG_FATAL, __FILE__, __LINE__, _fmt, ##_args); \
    }

void LOG_SetLevel(int type, LOG_LEVEL level);

void LOG_ASSERT(bool check, const char *file, int line, const char *info = NULL);

#define ASSERT(check)                            \
    {                                            \
        LOG_ASSERT((check), __FILE__, __LINE__); \
    }

#define ASSERT_INFO(check, info)                       \
    {                                                  \
        LOG_ASSERT((check), __FILE__, __LINE__, info); \
    }
#define VERIFY_FLOAT(value)                                                         \
    {                                                                               \
        if (fpclassify((value)) == FP_NAN || fpclassify((value)) == FP_INFINITE)    \
            LOG_ASSERT(false, __FILE__, __LINE__,                                   \
                       (fpclassify((value)) == FP_NAN) ? "FP_NAN" : "FP_INFINITE"); \
    }

#endif
