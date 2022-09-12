#ifndef DEV_SERIA_H
#define DEV_SERIA_H

#include <functional>
#include <memory>
#include <thread>
# include  <algorithm> 
#include "dev_interface.h"
#include "glog/logging.h"
namespace lprobot {
namespace device {
namespace internal {

class BaterryDataProcess {
 public:
  std::vector<uint8_t> operator()(std::vector<uint8_t>&){
      //   auto it = std::find(d.begin(), d.end(), 'c');
      //   std::vector<uint8_t> resul;

      //   if (it == d.end()) {
      //     return resul;
      //   }
      //   auto cit = it;
      //   if (*++it != 'm') {
      //     return resul;
      //   }
      //   if (*++it != 0x0d) {
      //     return resul;
      //   }
      //   if (*++it != 0x0a) {
      //     return resul;
      //   }
      //   std::string string_value;
      //   for (auto p = d.begin(); p != cit; ++p) {
      //     string_value.push_back(*p);
      //   }
      //   return resul;
      // }
  };
};
class CheiryDataProcess {
 public:
  std::vector<uint8_t> operator()(std::vector<uint8_t>& d) {
    auto it = std::find(d.begin(), d.end(),0x55);
    std::vector<uint8_t> resul;
    if (it == d.end()) {
      return resul;
    }
    // LOG(INFO)<<"test";
    if (*++it != 0xaa) {
      return resul;
    }
    // LOG(INFO)<<"test1";

    char test_lenth[4];
    test_lenth[0] = *++it;
    test_lenth[1] = *++it;
    test_lenth[2] = *++it;
    test_lenth[3] = *++it;
    
    int   data_lenth=    *((int*)&test_lenth);
    // if ((it + data_length) != d.end()) {
    //   return resul;
    // }
    // LOG(INFO)<<"test2";
    std::string string_value;
    for (int i = 0; i < data_lenth-6; i++) {
      resul.push_back(*++it);
    }
    d.erase(d.begin(), it);
    return resul;
  }
};
template <typename DataProcess>
class DevSeri : public DevInterface {
 public:
  using CallBack = std::function<void(std::vector<uint8_t>&&)>;
  DevSeri(const std::pair<std::string, int>& para, CallBack callback,
          int peri = 10, int pro_len = 10);

  void update();
  virtual bool tx(const std::vector<uint8_t>& data) const override;
  virtual std::vector<uint8_t> rx() const override;
  ~DevSeri();

 private:
  template <typename Data>
  class Serio;
  std::unique_ptr<Serio<DataProcess>> serio_impl_;
};


#define  DevSeriWithDataProcess  DevSeri<lprobot::device::internal::CheiryDataProcess>
}  // namespace internal
}  // namespace device
}  // namespace lprobot
#endif