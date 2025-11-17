#ifndef RAINCOM_COM_FASTDDS_TOPIC_HELPER_H_
#define RAINCOM_COM_FASTDDS_TOPIC_HELPER_H_

#include "raincom_com/macros.hpp"
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <mutex>
#include <unordered_map>

namespace raincom
{
namespace com
{
namespace fastdds
{

class TopicHelper
{
    DEF_SINGLETON(TopicHelper)
  public:
    eprosima::fastdds::dds::Topic *get(
        eprosima::fastdds::dds::DomainParticipant *participant, const std::string &name,
        const eprosima::fastdds::dds::TypeSupport &type);
    bool release(eprosima::fastdds::dds::DomainParticipant *participant,
                 const std::string &name);

  private:
    struct TopicWatcher
    {
        eprosima::fastdds::dds::Topic *topic;
        int cnt;
    }; // struct TopicWatcher

    std::mutex locker_;
    std::unordered_map<std::string, TopicWatcher> topics_;
    TopicHelper();
}; // class TopicHelper

} // namespace fastdds
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_FASTDDS_TOPIC_HELPER_H_
