#ifndef RAINCOM_COM_LOG_H_
#define RAINCOM_COM_LOG_H_

#include <glog/logging.h>

#define RINFO google::LogMessage(__FILE__, __LINE__, google::INFO).stream()

#define RWARN google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()

#define RERROR google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()

#define RFATAL google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()

#define CHECK_RINFO(condition) \
    if (!(condition))          \
    google::LogMessage(__FILE__, __LINE__, google::INFO).stream()

#define CHECK_RWARN(condition) \
    if (!(condition))          \
    google::LogMessage(__FILE__, __LINE__, google::WARNING).stream()

#define CHECK_RERROR(condition) \
    if (!(condition))           \
    google::LogMessage(__FILE__, __LINE__, google::ERROR).stream()

#define CHECK_RFATAL(condition) \
    if (!(condition))           \
    google::LogMessage(__FILE__, __LINE__, google::FATAL).stream()

#ifdef NDEBUG
#define RDEBUG \
    if (0)     \
    google::LogMessage(__FILE__, __LINE__, google::INFO).stream()
#define CHECK_RDEBUG(condition) \
    if (0)                      \
        if (!(condition))       \
    google::LogMessage(__FILE__, __LINE__, google::INFO).stream()
#else
#define RDEBUG google::LogMessage(__FILE__, __LINE__, google::INFO).stream()

#define CHECK_RDEBUG(condition) \
    if (!(condition))           \
    google::LogMessage(__FILE__, __LINE__, google::INFO).stream()
#endif

#endif // RAINCOM_COM_LOG_H_
