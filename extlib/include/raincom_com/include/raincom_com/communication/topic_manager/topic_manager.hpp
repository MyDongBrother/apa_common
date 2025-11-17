#ifndef RAINCOM_COM_TOPIC_MANAGER_H_
#define RAINCOM_COM_TOPIC_MANAGER_H_

#include "raincom_com/communication/topic_manager/topic.hpp"
#include "raincom_com/macros.hpp"
#include "raincom_com/thread_pool/thread_pool.hpp"
#include <unordered_map>

namespace raincom
{
namespace com
{

class RawTopic;

class TopicManager
{
    DEF_SINGLETON(TopicManager)
  public:
    TopicManager();
    ~TopicManager();
    // For pub
    template <typename T>
    bool create_pub_topic(const std::string &topic_name, int que_size);
    bool create_pub_topic(const std::string &topic_name);
    template <typename T>
    Topic<T> *get_pub_topic(const std::string &topic_name);
    RawTopic *get_pub_topic(const std::string &topic_name);
    bool pub_topic_exists(const std::string &topic_name);
    // Sor sub
    template <typename T>
    bool create_sub_topic(const std::string &topic_name, int que_size);
    bool create_sub_topic(const std::string &topic_name);
    template <typename T>
    Topic<T> *get_sub_topic(const std::string &topic_name);
    RawTopic *get_sub_topic(const std::string &topic_name);
    bool sub_topic_exists(const std::string &topic_name);

  private:
    std::unordered_map<std::string, RawTopic *> sub_topics_;
    std::unordered_map<std::string, RawTopic *> pub_topics_;
}; // class TopicManager

template <typename T>
bool TopicManager::create_pub_topic(const std::string &topic_name, int que_size)
{
    if (pub_topic_exists(topic_name))
        return false;
    auto task_que_manager{ThreadPool::instance()->task_queue_manager()};
    pub_topics_.insert(
        std::make_pair(topic_name, reinterpret_cast<RawTopic *>(new Topic<T>(
                                       topic_name, que_size, task_que_manager))));
    return true;
}

template <typename T>
bool TopicManager::create_sub_topic(const std::string &topic_name, int que_size)
{
    if (sub_topic_exists(topic_name))
        return false;
    auto task_que_manager{ThreadPool::instance()->task_queue_manager()};
    sub_topics_.insert(
        std::make_pair(topic_name, reinterpret_cast<RawTopic *>(new Topic<T>(
                                       topic_name, que_size, task_que_manager))));
    return true;
}

template <typename T>
Topic<T> *TopicManager::get_pub_topic(const std::string &topic_name)
{
    if (!pub_topic_exists(topic_name))
        return nullptr;
    return reinterpret_cast<Topic<T> *>(pub_topics_.at(topic_name));
}

template <typename T>
Topic<T> *TopicManager::get_sub_topic(const std::string &topic_name)
{
    if (!sub_topic_exists(topic_name))
        return nullptr;
    return reinterpret_cast<Topic<T> *>(sub_topics_.at(topic_name));
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_TOPIC_MANAGER_H_
