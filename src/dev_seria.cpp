#include <array>
#include <chrono>
#include <future>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include "seria_wrapper.h"
#include "dev_seria.h"
#include  <memory>  
#include "glog/logging.h"
namespace lprobot {
namespace device {
namespace internal {



template class DevSeri<CheiryDataProcess>;
template class DevSeri<RtkDataProcess>;

template <typename DataProcess>
template <typename Data>
class DevSeri<DataProcess>::Serio {
 public:
  Serio(std::pair<std::string, int> para, CallBack callback,int lenth=1)
      : result_fun_(callback),process_lenth_(lenth) {
    int try_time = try_times_;
    int fd;
    while (try_time && !kill_thread_ ) {
      fd = linx_seria::Create(para.first.data(), para.second);
      if (fd == -1) {
        try_time--;
        usleep(200000);
        continue;
      }
      break;
    }
    LOG(INFO)<<"open tty success";
    if (try_time != 0) {
      thread_ = std::unique_ptr<std::thread>(
          new std::thread([this]() { this->update(); }));
      fd_ = fd;
    } else {
      throw std::string("CreatSerio  err  with port") +
          std::to_string(para.second);
    }
  }

  bool SendData(const std::vector<uint8_t>& data) {
    return linx_seria::Writen(fd_, (char*)data.data(), data.size());
  }

  void update() {

    std::vector<uint8_t> q_data;
    while (!kill_thread_) {
      std::vector<uint8_t> data(process_lenth_,0);  //////
      int ret = linx_seria::Readn(fd_,(char*)data.data(), process_lenth_);
      if (ret <= 0) {
        std::cout << "Serial read return " << ret << "request 1 byte"
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(loop_peri_));
        continue;
      }
      q_data.insert(q_data.end(), data.begin(), data.end());
      std::vector<uint8_t> result = data_process_(q_data);
      if (result_fun_) {
        result_fun_(std::move(result));
      }
    }
  }
  ~Serio() {
    kill_thread_ = true;
    if (fd_ != -1) {
      linx_seria::Close(fd_);
    }
    fd_ = -1;
    if (thread_->joinable()) {
      thread_->join();
    }
  }

 private:
  using CallBack = std::function<void(std::vector<uint8_t>&&)>;
  int fd_;
  std::unique_ptr<std::thread> thread_;
  constexpr static uint32_t try_times_ = 100;
  int loop_peri_ =10;
  int process_lenth_;
  bool kill_thread_ = false;
  CallBack result_fun_ = nullptr;
  Data data_process_;
  std::mutex mutex_read_;
};
template <typename DataProcess>
DevSeri<DataProcess>::DevSeri(const std::pair<std::string, int>& para,
                              CallBack callback, int pro_len, int peri) {
  serio_impl_ =
      std::make_unique<Serio<DataProcess>>(para, std::move(callback), pro_len);
}

template <typename DataProcess>
DevSeri<DataProcess>::~DevSeri() {}

template <typename DataProcess>
bool DevSeri<DataProcess>::tx(const std::vector<uint8_t>& data) const {
  serio_impl_->SendData(data);
};

template <typename DataProcess>
std::vector<uint8_t> DevSeri<DataProcess>::rx() const {};

}  // namespace internal

}  // namespace device
}  // namespace lprobot