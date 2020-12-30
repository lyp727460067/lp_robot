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
  int fd;
};
template <typename DataProcess>
DevSeri<DataProcess>::DevSeri(const std::pair<std::string, int> id,
                              const CallBack& callback) {}
}  // namespace internal
}  // namespace device
}  // namespace lprobot