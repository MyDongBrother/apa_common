#ifndef RAINCOM_COM_TIME_H_
#define RAINCOM_COM_TIME_H_

#include "raincom_com/duration.hpp"
#include <cstdint>
#include <ostream>

namespace raincom
{
namespace com
{

class Time
{
  public:
    Time();
    explicit Time(const double sec);
    Time(const Time &other);
    Time(const Time &&other);
    ~Time();
    Time &operator=(const Time &other);
    Time &operator=(const Time &&other);
    bool operator==(const Time &other) const;
    bool operator!=(const Time &other) const;
    bool operator>(const Time &other) const;
    bool operator<(const Time &other) const;
    bool operator>=(const Time &other) const;
    bool operator<=(const Time &other) const;
    Time operator+(const Duration &other) const;
    Time operator-(const Duration &other) const;
    Duration operator-(const Time &other) const;
    Time &operator+=(const Duration &other);
    Time &operator-=(const Duration &other);

    static Time now();
    double to_sec() const;
    int64_t to_msec() const;
    uint32_t to_trunc_msec() const;
    int64_t to_nsec() const;
    Time &from_sec(const double sec);

    friend std::ostream &operator<<(std::ostream &os, const Time &time);

  private:
    uint32_t sec_;
    uint32_t nsec_;
}; // class Time

std::ostream &operator<<(std::ostream &os, const Time &time);

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_TIME_H_
