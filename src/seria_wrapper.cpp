#include "seria_wrapper.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>

namespace lprobot {
namespace device {
namespace internal {

namespace linx_seria {
int Create(const char *port, int baut) {
  char buf[1024];
  int crport_fd_ = -1;
  int bardrate_ = baut;
  int ret = snprintf(buf, sizeof(buf), "%s", port);
  if (ret < 0 || ret >= static_cast<int>(sizeof(buf)))
    printf("%s %d: Warning buffer size(%u) is not enough(%d), please check!\n",
           __FUNCTION__, __LINE__, sizeof(buf), ret);
  crport_fd_ = open(buf, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (crport_fd_ == -1) {
    printf("Open device %s, on baudrate %d failed!.", buf,baut);
    return -1;
  }

  fcntl(crport_fd_, F_SETFL, FNDELAY);

  tcgetattr(crport_fd_, &orgopt_);
  tcgetattr(crport_fd_, &curopt_);
  speed_t CR_BAUDRATE;
  switch (baudrate) {
    case 9600:
      CR_BAUDRATE = B9600;
      break;
    case 19200:
      CR_BAUDRATE = B19200;
      break;
    case 38400:
      CR_BAUDRATE = B38400;
      break;
    case 57600:
      CR_BAUDRATE = B57600;
      break;
    case 115200:
      CR_BAUDRATE = B115200;
      break;
    case 230400:
      CR_BAUDRATE = B230400;
      break;
    default:
      CR_BAUDRATE = B115200;
      break;
  }
  cfsetispeed(&curopt_, CR_BAUDRATE);
  cfsetospeed(&curopt_, CR_BAUDRATE);
  /* Mostly 8N1 */
  curopt_.c_cflag &= ~PARENB;
  curopt_.c_cflag &= ~CSTOPB;
  curopt_.c_cflag &= ~CSIZE;
  curopt_.c_cflag |= CS8;
  curopt_.c_cflag |= CREAD;
  curopt_.c_cflag |= CLOCAL;  // disable modem status check

  cfmakeraw(&curopt_);  // make raw is_max_clean_state_

  if (tcsetattr(crport_fd_, TCSANOW, &curopt_) != 0) {
    return false;
  }
  printf("\033[32mserial port %s\033[0m init as %d done...", buf, crport_fd_);
  return crport_fd_;
}

int Writen(int fd, const char *data, int len) {
  int ret;
  ret = static_cast<int>(::write(fd, data, len));
  return ret;
}
int Readn(int fd, const char *buf, int n, const uint16_t &timeout_ms = 50,
          int bardrate_ = 115200) {
  int r_ret = 0, s_ret = 0;
  int len = n;
  uint8_t *t_buf;
  t_buf = (uint8_t *)calloc(len, sizeof(uint8_t));
  // memset(t_buf,0,size_of_path);
  fd_set read_fd_set;
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  int length = 0;

  while (true) {
    FD_ZERO(&read_fd_set);
    FD_SET(fd, &read_fd_set);

    s_ret = select(FD_SETSIZE, &read_fd_set, NULL, NULL, &timeout);
    if (s_ret < 0) {
      printf("-------select error------------");
      free(t_buf);
      FD_CLR(fd, &read_fd_set);
      return -1;
    } else if (s_ret == 0) {
      printf("select function \033[33mtimeout!\033[0m");
      free(t_buf);
      FD_CLR(fd, &read_fd_set);
      return 0;
    } else if (s_ret > 0) {
      if (FD_ISSET(fd, &read_fd_set)) {
        if (ioctl(fd, FIONREAD, &length) == -1) {
          printf("ioctl return -1");
          free(t_buf);
          FD_CLR(fd, &read_fd_set);
          return -1;
        }
        if (length >= (size_t)len) {
          r_ret = ::read(fd, t_buf, len);
          if (r_ret == len)
            for (int i = 0; i < len; i++) {
              buf[i] = t_buf[i];
            }
          // memcpy(buf,t_buf,size_of_path);
          free(t_buf);
          FD_CLR(fd, &read_fd_set);
          return r_ret;
        } else {
          int time_remain = timeout.tv_sec * 1000000 + timeout.tv_usec;
          int time_expect = (len - length) * 1000000 * 8 / bardrate_;
          if (time_remain > time_expect) usleep(time_expect);
        }
      }
    }
  }
  free(t_buf);
  return s_ret;
}
bool Close(int fd) {
  char buf[128];
  int retval;
  if (fd == -1) return -1;

  ::read(fd, buf, 128);
  tcsetattr(fd, TCSANOW, &orgopt_);
  retval = ::close(fd);
  return true;
}
}  // namespace linx_seria

}  // namespace internal
}  // namespace device
}  // namespace lprobot
