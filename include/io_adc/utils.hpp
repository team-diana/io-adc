#ifndef IO_ADC_UTILS_HPP
#define IO_ADC_UTILS_HPP

#include <string>

#include <ros/ros.h>

namespace io_adc {
    enum class RangeType {
      bipolar,
      unipolar
    };

    enum class AdcMode {
      localGND,
      userCMMD,
      differential
    };

    struct P9116Params {
      AdcMode adcMode;
      RangeType rangeType;

      uint16_t getConfigCtrlValue() const;
    };

    void log_input_error_adc(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_output_error_adc(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_input_error_dio(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_output_error_dio(uint16_t port, const std::string& msg, uint16_t error_enum_value);
    void log_register_card_error(const std::string& msg, uint16_t error_enum_value);
//     void log_error(std::string cardName, std::string type, uint16_t port, std::string msg = "", uint16_t error_enum_value);

    // Return greater or equal voltage range supported.
    int voltageRangeToEnum(const std::string& voltageRange, RangeType rangeType);
    AdcMode adcModeFromString(const std::string& mode);

    P9116Params getP9116ParamsFromRosParams(const ros::NodeHandle& nodeHandle );
    void checkParamExistenceOrExit(const ros::NodeHandle, const std::string& paramName);

}

#endif // IO_ADC_UTILS_HPP
