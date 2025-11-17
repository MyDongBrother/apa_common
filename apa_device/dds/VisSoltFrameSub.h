/**
 * @file VisSoltFrameSub.h
 * @brief
 * @author jiandong.liu (liujiandong@bm-intelligent.com)
 * @version 0.1
 * @date 2025-11-04
 *
 * @copyright Copyright (c) 2025 bm-intelligent
 * All rights reserved.
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date       <th>Version  <th>Author       <th>Description
 * <tr><td>2025-11-04 <td>1.0      <td>jiandong.liu     <td>first version
 * </table>
 */

#pragma once

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>

class VisSoltFrameSub
{
  public:
    VisSoltFrameSub();

    virtual ~VisSoltFrameSub();

    bool init();

    void run();

  private:
    eprosima::fastdds::dds::DomainParticipant *participant_;
    eprosima::fastdds::dds::Subscriber *subscriber_;
    eprosima::fastdds::dds::Topic *topic_;
    eprosima::fastdds::dds::DataReader *reader_;
    eprosima::fastdds::dds::TypeSupport type_;

    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
      public:
        SubListener() = default;

        ~SubListener() override = default;

        void on_data_available(eprosima::fastdds::dds::DataReader *reader) override;

        void on_subscription_matched(
            eprosima::fastdds::dds::DataReader *reader,
            const eprosima::fastdds::dds::SubscriptionMatchedStatus &info) override;

        int matched      = 0;
        uint32_t samples = 0;
    } listener_;
};
