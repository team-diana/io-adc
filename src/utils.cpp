#include "io_adc/utils.hpp"

#include <team_diana_lib/strings/strings.h>

#include "ros/ros.h"

namespace io_adc {

  std::string getErrorString(uint16_t errorEnumValue) {
    // TODO: generate error string
    switch(errorEnumValue) {
      default:
        return Td::toString(errorEnumValue);
    }
  }

  void log_error(const std::string& cardName, const std::string& type,
                         uint16_t port, const std::string& msg, uint16_t errorEnumValue)
  {
    std::string card_error_str = getErrorString(errorEnumValue);
    ROS_ERROR("[%s] port: %i - %s error %s %s", cardName.c_str(), port, type.c_str(), card_error_str.c_str(), msg.c_str());
  }

  void log_input_error_adc(uint16_t port, const std::string& msg, uint16_t errorEnumValue)
  {
    log_error("ADC", "INPUT", port, msg, errorEnumValue);
  }

  void log_output_error_adc(uint16_t port, const std::string& msg, uint16_t errorEnumValue)
  {
    log_error("ADC", "OUTPUT", port, msg, errorEnumValue);
  }

  void log_input_error_dio(uint16_t port, const std::string& msg, uint16_t errorEnumValue)
  {
    log_error("DIO", "INPUT", port, msg, errorEnumValue);
  }

  void log_output_error_dio(uint16_t port, const std::string& msg, uint16_t errorEnumValue)
  {
    log_error("DIO", "OUTPUT", port, msg, errorEnumValue);
  }

  void log_register_card_error(const std::string& msg, uint16_t error_enum_value)
  {
    ROS_ERROR("Error while registering card: %s %s", msg.c_str(), getErrorString(error_enum_value).c_str());
  }


}
