#ifndef SOCKET_WRAPPER_H
#define SOCKET_WRAPPER_H

namespace lprobot {
namespace device {
namespace internal {

namespace linx_socket {

int Writen(int fd, const void *vptr, int n);
int Readn(int fd, void *vptr, int maxlen);
int CreatSocket(const char *ip, int port);
int StartLisen(int fd);
bool Close(int fd);

}  // namespace linx_socket

}  // namespace internal
}  // namespace device
}  // namespace lprobot

#endif