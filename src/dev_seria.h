#ifndef DEV_SERIA_H
#define DEV_SERIA_H
#include "iomanip"
#include <functional>
#include <memory>
#include <thread>
# include  <algorithm> 
#include "dev_interface.h"
#include "glog/logging.h"
namespace lprobot {
namespace device {
namespace internal {




class RtkDataProcess {
 public:
  struct GpsType{
    double lat;
    double log;
    double alt;
    int qual;
  };
  void Log(char c) {
    std::string test;
    test.push_back(c);
     LOG(INFO)<< test ;
  }
  std::vector<uint8_t> operator()(std::vector<uint8_t>& d) {
    auto it = std::find(d.begin(), d.end(), '$');
    // Log(*it);
    // LOG(INFO)<<d.size();
    std::vector<uint8_t> resul;
    if (it == d.end()) {
      return resul;
    }

    // Log(*it);
    it = std::find(it, d.end(), ',');
    if (it == d.end()) {
      return resul;
    }
    if (*std::prev(it) != 'A') {
      d.erase(d.begin(), it);
      return resul;
    }

    // Log(*it);
    //
    auto Tostring = [&](std::vector<uint8_t>::iterator& it) -> std::string {
      std::string result;
      for (; it != d.end(); ++it) {
        if (*it != ',') {
          result.push_back(*it);
        } else {
          return result;
        }
      }
      return {};
    };
    //
    auto ItPreIsValid = [&](std::vector<uint8_t>::iterator& it) {
      if (it == d.end()) return false;
      ++it;
      if (it == d.end()) return false;
      return true;
    };
    if (!ItPreIsValid(it)) return resul;//到 ,
    std::string utc = Tostring(it);
    if (!ItPreIsValid(it)) return resul;
    std::string lat = Tostring(it);
    if (!ItPreIsValid(it)) return resul;
    if (!ItPreIsValid(it)) return resul;
    if (!ItPreIsValid(it)) return resul;
    std::string lon =Tostring(it);
    if (!ItPreIsValid(it)) return resul;
    if (!ItPreIsValid(it)) return resul;
    if (!ItPreIsValid(it)) return resul;
    if (!ItPreIsValid(it)) return resul;
    int qual = (*std::prev(it))-'0';
    // LOG(INFO)<<qual;
    // Log(qual);
    if (!ItPreIsValid(it)) return resul;
    std::string stat = Tostring(it);
    if (!ItPreIsValid(it)) return resul;
    std::string factor = Tostring(it);
    // if (factor.empty()) return resul;
    if (!ItPreIsValid(it)) return resul;
    std::string alt = Tostring(it);
    if (!ItPreIsValid(it)) return resul;//m
    if (!ItPreIsValid(it)) return resul;//,
    if (!ItPreIsValid(it)) return resul;//un
    std::string undulation = Tostring(it);
    if (!ItPreIsValid(it)) return resul;//m
    if (!ItPreIsValid(it)) return resul;//,
    if (!ItPreIsValid(it)) return resul;//age
    std::string age = Tostring(it);
    if (!ItPreIsValid(it)) return resul;//*
    // std::string crc = Tostring(it);//crc
    //
    // if (!ItPreIsValid(it)) return resul;//jieshu
    // if (!ItPreIsValid(it)) return resul;//jieshu
    // LOG(INFO)<<crc;
    d.erase(d.begin(), it);
    // LOG(INFO)<<d.size();
    GpsType gps_data;
    size_t size = 24;
    gps_data.alt = std::stod(alt,&size);
    gps_data.lat = std::stold(lat,&size);
    gps_data.log = std::stold(lon,&size);
    gps_data.qual = qual;
    resul.resize(sizeof(gps_data));
    // LOG(INFO)<<gps_data.lat;
    // LOG(INFO)<<gps_data.log;
    memcpy((void*)resul.data(), (void*)&gps_data, sizeof(GpsType));
    return resul;
  }
};
// class ImuDataProcess {
//  public:
//   std::vector<uint8_t> operator()(std::vector<uint8_t>& d) {
//     auto it = std::find(d.begin(), d.end(), 0x55);
//     std::vector<uint8_t> resul;
//     if (it == d.end()) {
//       return resul;
//     }
//     // LOG(INFO)<<"test";
//     if (*++it != 0xaa) {
//       return resul;
//     }
//     // LOG(INFO)<<"test1";

//     char test_lenth[4];
//     test_lenth[0] = *++it;
//     test_lenth[1] = *++it;
//     test_lenth[2] = *++it;
//     test_lenth[3] = *++it;

//     int data_lenth = *((int*)&test_lenth);
//     // if ((it + data_length) != d.end()) {
//     //   return resul;
//     // }
//     // LOG(INFO)<<"test2";
//     std::string string_value;
//     for (int i = 0; i < data_lenth - 6; i++) {
//       resul.push_back(*++it);
//     }
//     d.erase(d.begin(), it);
//     return resul;
//   }
// };

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
          int pro_len = 1, int peri = 10);

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
#define  DevSeriWithRtkDataProcess  DevSeri<lprobot::device::internal::RtkDataProcess>
}  // namespace internal
}  // namespace device
}  // namespace lprobot
#endif