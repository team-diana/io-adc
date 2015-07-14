#ifndef IO_ADC_ADC_CARD_PYTHON_HPP
#define IO_ADC_ADC_CARD_PYTHON_HPP

#include <io_adc/consts.hpp>

#include <cstdint>

class AdcCardPython {

public:

  AdcCardPython();
  ~AdcCardPython();

  int open(uint16_t mode, uint16_t trigMode);
  void close();

  int32_t readChannelRaw(uint32_t channel, io_adc::VoltageRange range);
  double readChannel(uint32_t channel, io_adc::VoltageRange range);

private:

  uint32_t adcCard;

};


#endif // IO_ADC_ADC_CARD_PYTHON_HPP
