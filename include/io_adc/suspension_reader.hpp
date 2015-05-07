#ifndef IO_ADC_SUSPENSION_READER_HPP
#define IO_ADC_SUSPENSION_READER_HPP

#include <array>

struct SuspensionPortConf {
  uint16_t xAdcPort;
  uint16_t zAdcPort;
};

template <typename T> struct VecXZ {
  T x;
  T z;

  VecXZ() : x(0), z(0) {}

};

using SuspensionValue = VecXZ<float>;

class SuspensionReader {


public:
  SuspensionReader(SuspensionPortConf suspensionPortConf, const std::string& name);

  void update(int adc_card);

  SuspensionValue getValue();

private:
  std::string name;
  SuspensionPortConf portConf;
  SuspensionValue lastSuspensionValue;

  static uint16_t suspensionRangeVolt;
};

#endif // IO_ADC_SUSPENSION_READER_HPP
