#ifndef RAINCOM_COM_UTILS_H_
#define RAINCOM_COM_UTILS_H_

// #include "raincom_com/log.hpp"
#include "raincom_com/log.hpp"
#include <cstddef>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#include <string>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <dirent.h>
#include <cmath>

namespace raincom
{
namespace com
{

// 1 sec = 1000000000 nsec
#define SEC_IN_NSEC_L   1000000000L
#define SEC_IN_NSEC_ULL 1000000000ULL
#define SEC_IN_MSEC_L   1000L
#define SEC_IN_MSEC_ULL 1000ULL

typedef union {
    uint8_t byte;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b1 : 1;
        uint8_t b2 : 1;
        uint8_t b3 : 1;
        uint8_t b4 : 1;
        uint8_t b5 : 1;
        uint8_t b6 : 1;
        uint8_t b7 : 1;
    } DataL1Bit1;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b23 : 2;
        uint8_t b45 : 2;
        uint8_t b67 : 2;
    } DataL2Bit01;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b12 : 2;
        uint8_t b34 : 2;
        uint8_t b56 : 2;
        uint8_t b7 : 1;
    } DataL2Bit23;
    struct
    {
        uint8_t b02 : 3;
        uint8_t b35 : 3;
        uint8_t b67 : 2;
    } DataL3Bit02;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b13 : 3;
        uint8_t b46 : 3;
        uint8_t b7 : 1;
    } DataL3Bit13;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b24 : 3;
        uint8_t b57 : 3;
    } DataL3Bit24;
    struct
    {
        uint8_t b03 : 4;
        uint8_t b47 : 4;
    } DataL4Bit03;
    struct
    {
        uint8_t b04 : 5;
        uint8_t b57 : 3;
    } DataL5Bit04;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b15 : 5;
        uint8_t b67 : 2;
    } DataL5Bit15;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b26 : 5;
        uint8_t b7 : 1;
    } DataL5Bit26;
    struct
    {
        uint8_t b02 : 3;
        uint8_t b37 : 5;
    } DataL5Bit37;
    struct
    {
        uint8_t b05 : 6;
        uint8_t b67 : 2;
    } DataL6Bit05;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b16 : 6;
        uint8_t b7 : 1;
    } DataL6Bit16;
    struct
    {
        uint8_t b01 : 2;
        uint8_t b27 : 6;
    } DataL6Bit27;
    struct
    {
        uint8_t b06 : 7;
        uint8_t b7 : 1;
    } DataL7Bit06;
    struct
    {
        uint8_t b0 : 1;
        uint8_t b17 : 7;
    } DataL7Bit17;
} DataOneByte;

void list_dir(const std::string &path, std::vector<std::string> &files,
              const std::string &ext = "");

/**
 * @brief Normalize time
 *
 * @param sec
 * @param nsec
 */
inline void time_normalize_sec_nsec(int64_t &sec, int64_t &nsec)
{
    int64_t sec_part  = sec;
    int64_t nsec_part = nsec;

    while (nsec_part >= SEC_IN_NSEC_L)
    {
        nsec_part -= SEC_IN_NSEC_L;
        ++sec_part;
    }

    while (nsec_part < 0)
    {
        nsec_part += SEC_IN_NSEC_L;
        --sec_part;
    }

    if (sec_part < 0 || sec_part > INT32_MAX)
    {
        // RFATAL << "Time out of range of int32";
        return;
    }

    sec  = sec_part;
    nsec = nsec_part;
}

inline bool file_exists(const std::string &path)
{
    struct stat buf;
    return stat(path.c_str(), &buf) == 0;
}

inline bool create_dir(const std::string &path, mode_t mode)
{
    if (path.empty())
    {
        return false;
    }

    size_t pos;
    std::string path_{path};
    std::vector<std::string> sub_dirs;
    while ((pos = path_.find("/")) != std::string::npos)
    {
        sub_dirs.push_back(path_.substr(0, pos));
        path_.erase(0, pos + 1);
    }
    sub_dirs.push_back(path_);

    path_.clear();
    for (const auto &it : sub_dirs)
    {
        path_ += it + "/";
        if (file_exists(path_))
        {
            // return true or false?
            continue;
        }

        if (mkdir(path_.c_str(), mode) != 0)
        {
            RERROR << strerror(errno);
            return false;
        }
    }
    return true;
}

static inline bool is_private_topic(const char *const topic)
{
    if (topic != nullptr && topic[0] == '~')
        return true;
    else
        return false;
}

inline std::string unix_sec_to_string(uint64_t unix_seconds,
                                      const std::string &format_str = "%Y-%m-%d-%H:%M:%S")
{
    time_t t = unix_seconds;
    struct tm ptm;
    struct tm *ret = localtime_r(&t, &ptm);
    if (ret == nullptr)
    {
        return std::string("");
    }
    uint32_t length = 64;
    std::vector<char> buff(length, '\0');
    strftime(buff.data(), length, format_str.c_str(), ret);
    return std::string(buff.data());
}

inline float kmph_to_mps(const float kmph) { return kmph / 3.6f; }

inline float mph_to_mps(const float mph) { return mph * 0.44704f; }

inline float mps_to_kmph(const float mps) { return mps * 3.6f; }

inline float rad_to_deg(const float rad) { return rad / M_PI * 180.0f; }

inline float deg_to_rad(const float deg) { return deg / 180.0f * M_PI; }

/**
 * @brief return current executable file absolute path
 *
 * @return std::string
 */
std::string this_exe_path();

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_UTILS_H_
