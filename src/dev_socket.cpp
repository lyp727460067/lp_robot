
#include "dev_socket.h"

#include <array>
#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include "socket_wrapper.h"
// #include "tcp_connect.h"
namespace lprobot {
namespace device {
namespace internal {

namespace {
constexpr int kBuffSize = 1024;
}  // namespace

class Connection {
 public:
  Connection(int fd1, DevSocket::CallBack c) : call_back_f_(c), fd(fd1) {
    fu = std::async(std::launch::async, [this]() { updta(); });
  };
  void updta() {
    while (!kill_thread_) {
      std::array<uint8_t, kBuffSize> buf;
      int len = linx_socket::Readn(fd, buf.data(), kBuffSize);
      std::cout << "start to read" << std::endl;
      if (len > 0) {
        if (call_back_f_) {
          call_back_f_({buf.begin(), buf.begin() + len});
        }
      } else if (len == 0) {
        break;
      } else {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << __FUNCTION__ << __LINE__ << "ERR" << std::endl;
      }
    }
  }
  bool Write(std::vector<uint8_t> data) {
    if (linx_socket::Writen(fd, data.data(), data.size()) < 0) {
      return false;
    }
   
  }
  ~Connection() {
    kill_thread_ = true;
    if (fd != -1) {
      linx_socket::Close(fd);
    }
    fd = -1;
    fu.get();
  }
  std::future<void> &GetFu() { return fu; }
  int GetFd() { return fd; }
 private:
  int fd = -1;
  bool kill_thread_ = false;
  DevSocket::CallBack call_back_f_ = nullptr;
  std::future<void> fu;
};

class DevSocket::Socket {
 public:
  Socket(){};
  Socket(std::pair<std::string, int> port, const CallBack& callback_)
      : call_back_f_(callback_) {
    int n;
    if ((n = linx_socket::CreatSocket(port.first.c_str(), port.second)) < 0) {
      throw std::string("CreatSocket  err ") + std::to_string(n);
    }
    fd = n;
    auto func = [this](){
      while (!kill_thread_) {
        std::future<int> f = std::async(std::launch::async, [this]() {
          return linx_socket::StartLisen(fd);
        });
        while (f.wait_for(std::chrono::seconds(0)) !=
               std::future_status::ready) {
          if (kill_thread_) break;
          for (auto it = connections_.begin(); it != connections_.end();) {
            if ((*it)->GetFu().wait_for(std::chrono::seconds(0)) ==
                std::future_status::ready) {
              std::cout << "connetion break fd=" << (*it)->GetFd() << std::endl;
              it = connections_.erase(it);
            }
            if (it != connections_.end()) {
              ++it;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        int clien_fd = f.get();
        if (clien_fd < 0) {
          std::cout << "coccept err";
        } else {
          std::cout << "coccept fd = " << clien_fd << std::endl;
          connections_.push_back(
              std::make_shared<Connection>(clien_fd, call_back_f_));
        }
      }
    };
    thread_ = std::thread(std::move(func));
  };
  bool SendData(const std::vector<uint8_t> &data) {
    for (auto con : connections_) {
      try {
        con->Write(data);
      } catch (...) {
        
      }
    }
    return true;
  }
  ~Socket() {
    kill_thread_ = true;
    if (fd != -1) {
      linx_socket::Close(fd);
      fd = -1;
    }
    std::cout<<"close fd"<<std::endl; 
    if (thread_.joinable()) {
      thread_.join();
    }
    std::cout<<"close ok"<<std::endl; 
  }

 private:
  int fd = -1;
  bool kill_thread_ = false;
  std::vector<std::shared_ptr<Connection>> connections_;
  CallBack call_back_f_ = nullptr;
  std::thread thread_;
};

DevSocket::~DevSocket() {}
DevSocket::DevSocket() {
  std::pair<std::string, int> par{"127.0.0.1", 8555};
  SocketImpl = std::unique_ptr<Socket>(new Socket(par, nullptr));
}
DevSocket::DevSocket(const CallBack& callback) {
  std::pair<std::string, int> par{"127.0.0.1", 8555};
  SocketImpl = std::unique_ptr<Socket>(new Socket(par, callback));
}

DevSocket::DevSocket(const std::pair<std::string,int> id,const CallBack& callback)
{
  SocketImpl = std::unique_ptr<Socket>(new Socket(id, callback));
}
bool DevSocket::tx(const std::vector<uint8_t> &data) const {
  SocketImpl->SendData(data);
  return true;
}

std::vector<uint8_t> DevSocket::rx() const { return {}; }

}  // namespace internal
}  // namespace device
}  // namespace lprobot
