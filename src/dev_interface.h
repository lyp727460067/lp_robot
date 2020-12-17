#ifndef _DEV_INTERFACE_H
#define _DEV_INTERFACE_H_
#include <vector>

namespace lprobot {
namespace device {
class DevInterface {
 public:
 
  virtual bool tx(const std::vector<uint8_t> &data) const = 0;
  virtual std::vector<uint8_t> rx() const = 0;
  virtual ~DevInterface(){};
};
}  // namespace device

}  // namespace lprobot

#endif