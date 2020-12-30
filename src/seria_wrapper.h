#ifndef SERIA_WRAPPER_H
#define SERIA_WRAPPER_H



namespace lprobot {
namespace device {
namespace internal {

namespace linx_seria {
int  Create(const char *port, int baut);
bool Writen(int fd, char *buf, int n);
bool Readn(int fd, const char *data, int n);
bool Close(int fd);
}  // namespace linx_seria
}  // namespace internal
}  // namespace device
}  // namespace lprobot

#endif