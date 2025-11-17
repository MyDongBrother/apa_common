#ifndef RAINCOM_COM_DEBUG_H_
#define RAINCOM_COM_DEBUG_H_

namespace raincom
{
namespace com
{

void segv_sig_handler(int sig);
void abrt_sig_handler(int sig);

} // namespace com
} // namespace raincom

#endif // RAINCOM_COM_DEBUG_H_
