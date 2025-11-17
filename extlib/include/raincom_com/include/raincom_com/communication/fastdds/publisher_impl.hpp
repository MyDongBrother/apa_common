#ifndef RAINCOM_COM_FASTDDS_PUBLISHER_H_
#define RAINCOM_COM_FASTDDS_PUBLISHER_H_

#include "raincom_com/communication/fastdds/message/underlay_message.h"
#include "raincom_com/communication/base_publisher.hpp"
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>

namespace raincom
{
namespace com
{

class RawTopic;
class SafeTaskQueueManager;

namespace fastdds
{

class PubListener;
class PublishTask;

class PublisherImpl : public BasePublisher
{
  public:
    /**
     * @brief Construct a new Publisher object
     *
     * @param topic_name
     * @param queue_size
     * @param task_queue_manager
     */
    PublisherImpl(const std::string &topic_name, int queue_size,
                  SafeTaskQueueManager *task_queue_manager);
    virtual ~PublisherImpl() override;

    const std::string &topic_name() { return topic_name_; }

    /**
     * @brief return number of subscriber
     *
     * @return int
     */
    virtual int num_subscriber() const override;

    /**
     * @brief The function to porocess the message
     *
     * @param msg
     */
    void msg_processor(UnderlayMessage &msg);
    virtual void destroy() override;

  private:
    PublisherImpl(const PublisherImpl &)             = delete;
    PublisherImpl(const PublisherImpl &&)            = delete;
    PublisherImpl &operator=(const PublisherImpl &)  = delete;
    PublisherImpl &operator=(const PublisherImpl &&) = delete;

    eprosima::fastdds::dds::DomainParticipant *participant_;
    eprosima::fastdds::dds::Publisher *publisher_;
    eprosima::fastdds::dds::Topic *fastdds_topic_;
    eprosima::fastdds::dds::DataWriter *writer_;
    eprosima::fastdds::dds::TypeSupport type_;

    PubListener *listener_;
    // const int queue_id_;
    SafeTaskQueueManager *task_queue_manager_;
    RawTopic *topic_;
    bool destroyed_;
    const std::string topic_name_;

    void underlay_publish(const void *const msg, int msg_len,
                          const std::string &msg_type_name,
                          const std::string &msg_type_id, double stamp);
    void destroy_();
}; // class Publisher

class PubListener : public eprosima::fastdds::dds::DataWriterListener
{
  public:
    PubListener(PublisherImpl *pub) : matched_(0), first_connected_(false), pub_(pub) {}
    virtual ~PubListener() override = default;

    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter *writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus &info) override;

  private:
    int matched_;
    bool first_connected_;
    PublisherImpl *pub_;
}; // class PubListener

} // namespace fastdds
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_FASTDDS_PUBLISHER_H_
