#ifndef IO_ADC_UTILS_HPP
#define IO_ADC_UTILS_HPP

#include <string>

namespace io_adc {
    enum class RangeType {
      bipolar,
      unipolar
    };

    void log_input_error_adc(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_output_error_adc(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_input_error_dio(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_output_error_dio(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_register_card_error(const std::string& msg, uint16_t error_enum_value);
//     void log_error(std::string cardName, std::string type, uint16_t port, std::string msg = "", uint16_t error_enum_value);

    // Return greater or equal voltage range supported.
    int voltageRangeToEnum(const std::string& voltageRange, RangeType rangeType);
}

#endif // IO_ADC_UTILS_HPP
