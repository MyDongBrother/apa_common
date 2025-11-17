#ifndef RAINCOM_COM_UDP_CLIENT_H_
#define RAINCOM_COM_UDP_CLIENT_H_

#include <arpa/inet.h>
#include <string>

namespace raincom
{
namespace com
{

class UDPClient
{
  public:
    UDPClient();
    ~UDPClient() = default;
    void init(const std::string &addr, int port);
    bool send(const void *data, int length);

  private:
    int sock_;
    sockaddr_in serv_addr_;
}; // class UDPClient

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_UDP_CLIENT_H_
