#ifndef RAINCOM_COM_UDP_SERVER_H_
#define RAINCOM_COM_UDP_SERVER_H_

#include <cstddef>
#include <functional>
#include <string>
#include <arpa/inet.h>

namespace raincom
{
namespace com
{

class UDPServer
{
  public:
    using Callback = std::function<void(const char *data, size_t length)>;

  public:
    UDPServer();
    ~UDPServer();
    void run(const std::string &local_ip, int port);
    void set_callback(Callback cb) { cb_ = cb; };
    void stop() { stopped_ = true; }

  private:
    int socket_id_;
    bool stopped_;
    Callback cb_;
    struct sockaddr_in remote_addr_;
    struct sockaddr_in local_addr_;
}; // class UDPServer

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_UDP_SERVER_H_
