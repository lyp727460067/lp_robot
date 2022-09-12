#ifndef SERIA_WRAPPER_H
#define SERIA_WRAPPER_H



namespace lprobot {
namespace device {
namespace internal {

namespace linx_seria {
int Create(const char *port, int baut);
int Writen(int fd, const char *buf, int n);
int Readn(int fd, char *data, int n, const int &timeout_ms = 10000,
          int bardrate_ = 115200);
bool Close(int fd);
}  // namespace linx_seria
}  // namespace internal
}  // namespace device
}  // namespace lprobot

#endif