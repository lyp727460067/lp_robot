
#include "socket_wrapper.h"

#include <arpa/inet.h>
#include <errno.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>

namespace lprobot {
namespace device {
namespace internal {
namespace linx_socket {
namespace {
constexpr int socket_que_size = 5;
}

int Writen(int fd, const void *vptr, int n) {
  ssize_t nleft = n;
  const char *ptr = (const char *)vptr;
  while (nleft > 0) {
    ssize_t nwriten = 0;
    if ((nwriten = write(fd, ptr, nleft)) < 0) {
      if (nwriten < 0 && errno == EINTR) {
        nwriten = 0;
      } else {
        return -1;
      }
    }
    nleft -= nwriten;
    ptr += nwriten;
  }
  std::cout<<"start write"<<std::endl;  
  return n;
}

int Readn(int fd, void *vptr, int maxlen) {
  bool ret = false;
  ssize_t nread = 0;
  while (!ret) {
    if ((nread = read(fd, vptr, maxlen)) < 0) {
      if (errno == EINTR) {
        nread = 0;
	ret = true;
      } else {
        return -1;
      }
    } else {
      ret = 1;
    }
  }
  return nread;
}
int StartLisen(int fd) {
  struct sockaddr_in c_addr;
  socklen_t c_lent = sizeof(c_addr);
  int fd_c = accept(fd, (struct sockaddr *)&c_addr, &c_lent);
  std::cout << "start accept" << std::endl;
  if (fd_c < 0) {
    perror("accept:");
    if (errno == EPROTO || errno == ECONNABORTED) {
      return -1;
    } else {
    }
  }
  return fd_c;
}

int CreatSocket(const char *ip, int port) {
  int ret = -1;
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    return -1;
  }
  int reuse = 1;

  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    perror("setsockopet error\n");
    return -1;
  }

  struct sockaddr_in s_addr;
  memset(&s_addr, 0, sizeof(s_addr));
  // inet_pton(AF_INET, ip, &s_addr.sin_port);
  s_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  s_addr.sin_port = htons(port);
  s_addr.sin_family = AF_INET;
  if (bind(fd, (struct sockaddr *)&s_addr, sizeof(s_addr)) < 0) {
    perror("fd");
    close(fd);
    return -2;
  }
  if (listen(fd, socket_que_size) < 0) {
    close(fd);
    return -3;
  }
  std::cout << "start listen" << std::endl;
  return fd;
}
int CreatSocket(const char *ip, int port, int socket_que_size) {
  int ret = -1;
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    return -1;
  }
  struct sockaddr_in s_addr;
  memset(&s_addr, 0, sizeof(s_addr));
  // inet_pton(AF_INET, ip, &s_addr.sin_port);
  s_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  s_addr.sin_port = htons(port);
  s_addr.sin_family = AF_INET;
  if (bind(fd, (struct sockaddr *)&s_addr, sizeof(s_addr)) < 0) {
    close(fd);
    return -2;
  }
  std::cout << "start listen" << std::endl;
  if (listen(fd, socket_que_size) < 0) {
    close(fd);
    return -3;
  }
  return fd;
}

bool Close(int fd) {
  printf("close fd=%d\n",fd);
  close(fd);
  return true;
}
}  // namespace linx_socket
}  // namespace internal
}  // namespace device
}  // namespace lprobot
