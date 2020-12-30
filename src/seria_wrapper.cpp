#include "seria_wrapper.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

namespace lprobot {
namespace device {
namespace internal {

namespace linx_seria {

int Create(const char *port, int baut) {}
bool Writen(int fd, const char *data, int n) {}
bool Readn(int fd, const char *data, int n) {}
bool Close(int fd) {}
}  // namespace linx_seria

}  // namespace internal
}  // namespace device
}  // namespace lprobot
