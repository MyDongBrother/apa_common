#ifndef RAINCOM_COM_BASE_SUBSCRIBER_
#define RAINCOM_COM_BASE_SUBSCRIBER_

namespace raincom
{
namespace com
{

class BaseSubscriber
{
  public:
    virtual ~BaseSubscriber() = default;
    virtual void destroy()    = 0;
}; // class BaseSubscriber

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_BASE_SUBSCRIBER_
