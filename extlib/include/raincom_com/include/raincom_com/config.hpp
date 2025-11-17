#ifndef RAINCOM_COM_CONFIG_H_
#define RAINCOM_COM_CONFIG_H_

#include "raincom_com/log.hpp"
#include "raincom_com/macros.hpp"
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <list>
#include <vector>
#include <string>

namespace raincom
{
namespace com
{

const std::string DELIMITER{"/"};
const std::string YAML_EXT{".yaml"};
void string_spliter(const std::string &s, std::list<std::string> &strs);

class Config
{
    DEF_SINGLETON(Config)
  public:
    template <typename T>
    static inline void get(const std::string &key, T &value, const T &default_value)
    {
        return instance()->underlay_get(key, value, default_value);
    }

    template <typename T>
    static void get(const std::string &key, std::vector<T> &value,
                    const std::vector<T> &default_value)
    {
        return instance()->underlay_get(key, value, default_value);
    }

    void set_base_dir(const std::string &base_dir);

  private:
    std::string base_dir_;
    std::unordered_map<std::string, YAML::Node> yaml_data_;

    Config() {}
    ~Config() = default;
    bool load_yaml(const std::string &path, YAML::Node &node);
    bool find_node(const std::string &path, YAML::Node &node);
    bool find_base_node(const std::list<std::string> &fields, YAML::Node &node,
                        std::list<std::string> &inner_fields);

    template <typename T>
    void underlay_get(const std::string &key, T &value, const T &default_value);
    template <typename T>
    void underlay_get(const std::string &key, std::vector<T> &value,
                      const std::vector<T> &default_value);
}; // class Config

template <typename T>
void Config::underlay_get(const std::string &key, T &value, const T &default_value)
{
    YAML::Node node;
    if (!find_node(key, node))
    {
        value = default_value;
        return;
    }

    if (node.IsScalar())
    {
        value = node.as<T>();
    }
    else if (node.IsSequence())
    {
        RERROR << "This node should be a sequance, not a scalar";
    }
    else
    {
        RERROR << "This node should be a sequance, it's not";
    }
}

template <typename T>
void Config::underlay_get(const std::string &key, std::vector<T> &value,
                          const std::vector<T> &default_value)
{
    YAML::Node node;
    if (!find_node(key, node))
    {
        value = default_value;
        return;
    }

    if (node.IsScalar())
    {
        RERROR << "This node should be a scalar, not a sequence";
    }
    else if (node.IsSequence())
    {
        value.reserve(node.size());
        for (std::size_t i = 0; i < node.size(); i++)
        {
            value.push_back(node[i].as<T>());
        }
    }
    else
    {
        RERROR << "This node should be a scalar, it's not";
    }
}

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_CONFIG_H_
