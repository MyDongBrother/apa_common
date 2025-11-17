#ifndef RAINCOM_COM_BASE_PUBLISHER_
#define RAINCOM_COM_BASE_PUBLISHER_

namespace raincom
{
namespace com
{

class BasePublisher
{
  public:
    virtual ~BasePublisher()           = default;
    virtual int num_subscriber() const = 0;
    virtual void destroy()             = 0;
}; // class BasePublisher

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_BASE_PUBLISHER_
