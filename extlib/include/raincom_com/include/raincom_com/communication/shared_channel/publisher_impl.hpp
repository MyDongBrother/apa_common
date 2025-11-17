#ifndef RAINCOM_COM_SHARED_CHANNEL_PUBLISHER_H_
#define RAINCOM_COM_SHARED_CHANNEL_PUBLISHER_H_

#include "raincom_com/communication/shared_channel/channel_manager.hpp"
#include "raincom_com/communication/base_publisher.hpp"

namespace raincom
{
namespace com
{
namespace shared_channel
{

template <typename T>
class PublisherImpl : public BasePublisher
{
  public:
    /**
     * @brief Construct a new Publisher object
     *
     * @param topic_name
     * @param queue_size
     */
    PublisherImpl(const std::string &topic_name, int queue_size);
    ~PublisherImpl();

    // /**
    //  * @brief publish the `proto` into the channel
    //  *
    //  * @tparam T protobuf message type
    //  * @param proto protobuf message data, shared ptr
    //  */
    // virtual void publish(const std::shared_ptr<const T> proto) override;

    /**
     * @brief publish the `proto` into the channel
     *
     * @tparam T protobuf message type
     * @param proto protobuf message data
     */
    void publish(const T &proto);

    /**
     * @brief
     *
     * @return int
     */
    virtual int num_subscriber() const override;
    virtual void destroy() override {}

  private:
    PublisherImpl(const PublisherImpl &)             = delete;
    PublisherImpl(const PublisherImpl &&)            = delete;
    PublisherImpl &operator=(const PublisherImpl &)  = delete;
    PublisherImpl &operator=(const PublisherImpl &&) = delete;

    ChannelManager *channel_manager_;
    const std::string topic_;
}; // class PublisherImpl

template <typename T>
PublisherImpl<T>::PublisherImpl(const std::string &topic_name, int queue_size)
    : topic_(topic_name)
{
    channel_manager_ = ChannelManager::instance();
    channel_manager_->create_channel<T>(topic_name, queue_size);
}

template <typename T>
PublisherImpl<T>::~PublisherImpl()
{
}

template <typename T>
void PublisherImpl<T>::publish(const T &proto)
{
    channel_manager_->push(topic_, proto);
}

// template<typename T>
// void PublisherImpl<T>::publish(const std::shared_ptr<const T> proto)
// {
//     channel_manager_->push(topic_, proto);
// }

template <typename T>
int PublisherImpl<T>::num_subscriber() const
{
    auto chan = channel_manager_->get_channel<T>(topic_);
    return chan->callbacks.size();
}

} // namespace shared_channel
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_SHARED_CHANNEL_PUBLISHER_H_
