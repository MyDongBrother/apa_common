#ifndef RAINCOM_COM_DURATION_H_
#define RAINCOM_COM_DURATION_H_

#include <cstdint>

namespace raincom
{
namespace com
{

class Time;

class Duration
{
  public:
    Duration();
    explicit Duration(double sec);
    // explicit Duration(int freq);
    Duration(int32_t sec, int32_t nsec);
    Duration(const Duration &other);
    Duration(const Duration &&other);
    ~Duration();

    Duration &operator=(const Duration &other);
    Duration &operator=(const Duration &&other);
    Duration operator+(const Duration &other) const;
    Duration operator-(const Duration &other) const;
    Duration operator-() const;
    Duration &operator+=(const Duration &other);
    Duration &operator-=(const Duration &other);
    bool operator==(const Duration &other) const;
    bool operator!=(const Duration &other) const;
    bool operator>(const Duration &other) const;
    bool operator<(const Duration &other) const;
    bool operator>=(const Duration &other) const;
    bool operator<=(const Duration &other) const;

    double to_sec() const;
    int64_t to_msec() const;
    int64_t to_nsec() const;
    Duration &from_sec(double duration);

    friend class Time;

  protected:
    int32_t sec_;
    int32_t nsec_;
}; // class Duration

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_DURATION_H_
