#ifndef DEV_SERIA_H
#define DEV_SERIA_H



#include <functional>
#include <memory>
#include <thread>

#include "dev_interface.h"


namespace lprobot {
namespace device {
namespace internal {

class BaterryDataProcess {
 public:
  std::vector<uint8_t> operator()(std::vector<uint8_t>&) {
    auto it = std::find(d.begin(), d.end(), 'c');
    std::vector<uint8_t> resul;

    if (it == d.end()) {
      return resul;
    }
    auto cit = it;
    if (*++it != 'm') {
      return resul;
    }
    if (*++it != 0x0d) {
      return resul;
    }
    if (*++it != 0x0a) {
      return resul;
    }
    std::string string_value;
    for (auto p = d.begin(); p != cit; ++p) {
      string_value.push_back(*p);
    }
    /*
    std::istringstream s(string_value);
    float re;
    s >> re;
    resul.push_back(re);
    d.erase(d.begin(), it);*/

    return resul;
  }
};
class CheiryDataProcess {
 public:
  std::vector<uint8_t> operator()(std::vector<uint8_t>& d) {
    auto it = std::find(d.begin(), d.end(), 0xcb);
    std::vector<uint8_t> resul;
    if (it == d.end()) {
      return resul;
    }

    if (*++it != 0x55) {
      return resul;
    }
    int data_length = *++it;
    if ((it + data_length) > d.end()) {
      return resul;
    }
    std::string string_value;
    for (int i = 0; i < data_length; i++) {
      resul.push_back(*++it);
    }
    d.erase(d.begin(), it);
    return resul;
  }
};

template<typename DataProcess>
class DevSeri : public DevInterface {
 public:
  using CallBack = std::function<void(std::vector<uint8_t>&&)>;
  DevSeri() = default;
  DevSeri(std::pair<string, int> para, int peri = 10, int pro_len = 10);
  DevSeri(std::pair<string, int> para, const CallBack& callback, int peri = 10,
          int pro_len = 10) {}

  DevSeri(const CallBack& callback);
  void update();
  virtual bool tx(const std::vector<uint8_t>& data) const override;
  virtual std::vector<uint8_t> rx() const override;
  ~DevSeri();

 private:
  class Serio;
  std::unique_ptr<Serio> SerioImpl_;
  constexpr static uint32_t try_times_ = 100;
  int loop_peri_;
  int process_lenth_;
  bool kill_thread_ = false;
  CallBack  result_fun_= nullptr;
  DataProcess data_process_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_read_;
};
}  // namespace internal
}  // namespace device
}  // namespace lprobot
#endif