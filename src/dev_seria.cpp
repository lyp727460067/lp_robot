#include <array>
#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <map>
#include "seria_wrapper.h"
namespace lprobot {
namespace device {
namespace internal {

template <typename DataProcess>
class DevSeri<DataProcess>::Serio {
 public:
  DevSeri(std::pair<std::string, int> para) {
    int try_time = try_times_;
    while (try_time) {
      int fd = linx_seria::Create(para.first, para.second);
      if (fd == -1) {
        try_time--;
        usleep(200000);
        continue;
      }
      break;
    }
    if (try_time 1 = 0) {
      thread_ = std::unique_ptr<std::thread>(
          new std::thread([this]() { this->update(); }));
      fd_ = fd;
    } else {
      throw std::string("CreatSerio  err  with port") +
          std::to_string(para.first);
    }
  }

  bool SendData(std::vector<uint8_t data>) {
    return linx_seria::Writen(fd_, &data.data(), data.size());
  }

  void update() {
    std::vector<uint8_t> q_data(0, process_lenth_ * 3);
    while (!kill_thread_) {
      std::vector<uint8_t> data(0, process_lenth_);  //////
      int ret = linx_seria::Readn(&data.data, process_lenth_);
      if (ret <= 0) {
        printf("Serial read return %d, request 1 byte.", ret);
        std::this_thread::sleep_for(std::chrono::seconds(loop_peri_));
        continue;
      }
      q_data.emplace_back(data);
      std::vector<uint8_t> result = data_process_(q_data);
      if (result_fun_) {
        result_fun_(result);
      }
    }
  }
  ~Serio() {
    kill_thread_ = true;
    if (thread_->joinable()) {
      thread_->join();
    }
  }

 private:
  int fd_;
  std::unique_ptr<std::thread> thread_;
  constexpr static uint32_t try_times_ = 100;
  int loop_peri_;
  int process_lenth_;
  bool kill_thread_ = false;
  CallBack  result_fun_= nullptr;
  DataProcess data_process_;
  std::mutex mutex_read_;

};
template <typename DataProcess>
DevSeri<DataProcess>::DevSeri(std::pair<std::string, int> para, int peri,
                              int pro_len)
    {
 
}
template <typename DataProce>
DevSeri<DataProce>::~DevSeri() {
 
}

}  // namespace internal

}  // namespace device
}  // namespace lprobot
}  // namespace lprobot