#ifndef RAINCOM_COM_SHARED_CHANNEL_SUBSCRIBER_H_
#define RAINCOM_COM_SHARED_CHANNEL_SUBSCRIBER_H_

#include "raincom_com/communication/shared_channel/channel_manager.hpp"
#include "raincom_com/communication/base_subscriber.hpp"

namespace raincom
{
namespace com
{
namespace shared_channel
{

template <typename T>
class SubscriberImpl : public BaseSubscriber
{
  public:
    using CallbackT = std::function<void(const std::shared_ptr<const T>)>;

    /**
     * @brief Construct a new Subscriber object
     *
     * @param topic_name
     * @param queue_size
     * @param callback
     */
    SubscriberImpl(const std::string &topic_name, int queue_size, CallbackT callback);
    ~SubscriberImpl();
    virtual void destroy() override {}

  private:
    const std::string topic_name_;
    const int queue_size_;

    SubscriberImpl(const SubscriberImpl &)             = delete;
    SubscriberImpl(const SubscriberImpl &&)            = delete;
    SubscriberImpl &operator=(const SubscriberImpl &)  = delete;
    SubscriberImpl &operator=(const SubscriberImpl &&) = delete;
}; // class SubscriberImpl

template <typename T>
SubscriberImpl<T>::SubscriberImpl(const std::string &topic_name, int queue_size,
                                  CallbackT callback)
    : topic_name_(topic_name), queue_size_(queue_size)
{
    ChannelManager *channel_manager = ChannelManager::instance();
    channel_manager->subscribe(topic_name_, queue_size_, callback);
}

template <typename T>
SubscriberImpl<T>::~SubscriberImpl()
{
}

} // namespace shared_channel
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_SHARED_CHANNEL_SUBSCRIBER_H_
