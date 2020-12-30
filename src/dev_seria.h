#ifndef DEV_SERIA_H
#define DEV_SERIA_H



#include <functional>
#include <memory>
#include <thread>

#include "dev_interface.h"


namespace lprobot {
namespace device {
namespace internal {


class BaterryDataProcess
{
  public:
  std::vector<uint8_t> operator()(std::vector<uint8_t>&)
  {



  }


};
class CheiryDataProcess
{
  public:
  std::vector<uint8_t> operator()(std::vector<uint8_t>&)
  {
  }
};


template<typename DataProcess>
class DevSeri : public DevInterface {
 public:
  using CallBack = std::function<void(std::vector<uint8_t>&&)>;
  DevSeri()=default;
  DevSeri(const CallBack& callback);
  DevSeri(const std::pair<std::string, int> id, const CallBack& callback);
  virtual bool tx(const std::vector<uint8_t>& data) const override;
  virtual std::vector<uint8_t> rx() const override;
  ~DevSeri();

 private:
  class Serio;
  std::unique_ptr<DataProcess> data_process_;
  std::unique_ptr<Serio> SerioImpl;
};
}  // namespace internal
}  // namespace device
}  // namespace lprobot
#endif