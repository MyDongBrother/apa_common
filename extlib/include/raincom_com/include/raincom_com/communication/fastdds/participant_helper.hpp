#ifndef RAINCOM_COM_FASTDDS_PARTICIPANT_HELPER_H_
#define RAINCOM_COM_FASTDDS_PARTICIPANT_HELPER_H_

#include "raincom_com/macros.hpp"
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <mutex>

namespace raincom
{
namespace com
{
namespace fastdds
{

class ParticipantHelper
{
    DEF_SINGLETON(ParticipantHelper)
  public:
    eprosima::fastdds::dds::DomainParticipant *get();
    bool release();

  private:
    std::mutex locker_;
    int cnt_;
    eprosima::fastdds::dds::DomainParticipant *participant_;
    ParticipantHelper();
}; // class ParticipantHelper

} // namespace fastdds
} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_FASTDDS_PARTICIPANT_HELPER_H_
