#ifndef IO_ADC_CONSTS_HPP
#define IO_ADC_CONSTS_HPP

#define ADC_ANALOG_INPUT_PORTS_NUM 64

namespace io_adc {

    enum VoltageRange {
      B_10_V    = 1,
      B_5_V     = 2,
      B_2_5_V   = 3,
      B_1_25_V  = 4,
      U_10_V    = 15,
      U_5_V     = 16,
      U_2_5_V   = 17,
      U_1_25_V  = 18
    };
}


#endif // IO_ADC_CONSTS_HPP
