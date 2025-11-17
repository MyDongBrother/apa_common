#ifndef RAINCOM_COM_RATE_H_
#define RAINCOM_COM_RATE_H_

#include "raincom_com/duration.hpp"
#include "raincom_com/time.hpp"

namespace raincom
{
namespace com
{

class Rate : public Duration
{
  public:
    Rate(const Duration &duration);
    Rate(int freq);
    Rate(const Rate &other);
    Rate(const Rate &&other);
    ~Rate();

    Rate &operator=(const Rate &other);
    Rate &operator=(const Rate &&other);
    void reset();
    void sleep();

  private:
    Time start_;
}; // class Rate

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_RATE_H_
