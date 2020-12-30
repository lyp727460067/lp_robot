#include "dev_seria.h"

#include <array>
#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <thread>

#include "seria_wrapper.h"
namespace lprobot {
namespace device {
namespace internal {

template <typename DataProcess>
class DevSeri<DataProcess>::Serio {
 public:
  DevSeri(std::pair<std::string, int>) {}
  bool ReadOneByte(uint8_t* data) { return linx_seria::Readn(fd, data, 1); }
  bool WriteOneByte(uint8_t data) { return linx_seria::Writen(fd, &data, 1); }
  bool WriteNByte(std::vector<uint8_t> datas) {
    for (auto data : datas) {
      if (!WriteOneByte(data)) return false;
    }
    return true;
  }

 private:
  int fd_;
};
template <typename DataProcess>
DevSeri<DataProcess>::DevSeri(std::pair<string, int> para, int peri,
                              int pro_len)
    : loop_peri_{peri}, process_lenth_{pro_len} {
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
  if (try_time == 0) {
    thread_ = std::unique_ptr<std::thread>(
        new std::thread([this]() { this->update(); }));
  }
}
template <typename DataProce>
DevSeri<DataProce>::~DevSeri() {
  kill_thread_ = true;
  if (thread_->joinable()) {
    thread_->join();
  }
}
template <typename DataProce>
void DevSeri<DataProce>::update() {
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
}  // namespace internal

}  // namespace device
}  // namespace lprobot
}  // namespace lprobot