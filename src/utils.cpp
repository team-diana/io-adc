#include "io_adc/utils.hpp"
#include "io_adc/dask.h"

#include <team_diana_lib/strings/strings.h>
#include <team_diana_lib/logging/logging.h>

#include "ros/ros.h"

#include <string>

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

  int voltageRangeToEnum(const std::string& voltageRange, RangeType rangeType)
  {
    // NOTE: Other option are available but not listed here
    if(rangeType == RangeType::bipolar) {
      if(voltageRange == "1.25") {
        return AD_B_1_25_V;
      } else if(voltageRange == "2.5") {
        return AD_B_2_5_V;
      } else if (voltageRange == "5") {
        return AD_B_5_V;
      }
    } else if (rangeType == RangeType::unipolar) {
      if(voltageRange == "1.25") {
        return AD_U_1_25_V;
      } else if(voltageRange == "2.5") {
        return AD_U_2_5_V;
      } else if (voltageRange == "5") {
        return AD_U_5_V;
      }
    }
    throw std::runtime_error("unknown voltage range");
    return -1;
  }

  AdcMode adcModeFromString(const std::string& mode) {
    std::string adcModeStr = mode;
    if(adcModeStr == "userCMMD") {
      return AdcMode::userCMMD;
    } else if(adcModeStr == "localGND") {
      return  AdcMode::localGND ;
    } else if (adcModeStr == "differential"){
      return AdcMode::differential;
    } else {
      throw std::runtime_error(Td::toString("UNKOWN adc_mode: ",  adcModeStr ));
    }
  }

  P9116Params getP9116ParamsFromRosParams(const ros::NodeHandle& nodeHandle)  {
    bool unipolar;
    nodeHandle.param(std::string("unipolar"), unipolar, false);

    std::string adcModeStr;
    nodeHandle.param(std::string("adc_mode"), adcModeStr, std::string("userCMMD"));

    std::cout << "Enabling " << adcModeStr << " Mode" << std::endl;

    P9116Params params;

    params.adcMode = adcModeFromString(adcModeStr);
    if(unipolar) {
      params.rangeType = RangeType::unipolar;
    } else {
      params.rangeType = RangeType::bipolar;
    }

    return params;
  }

  uint16_t P9116Params::getConfigCtrlValue() const {
    uint16_t mode = 0;
    switch(adcMode) {
      case AdcMode::userCMMD:
        mode = P9116_AI_UserCMMD;
        break;
      case AdcMode::localGND:
        mode = P9116_AI_LocalGND;
        break;
      case AdcMode::differential:
        mode = P9116_AI_Differential;
        break;
    }

    uint16_t polarity = 0;
    if(rangeType == RangeType::bipolar) {
      polarity |= P9116_AI_BiPolar;
    } else {
      polarity |= P9116_AI_UniPolar;
    }

    return mode|polarity;
  }

  std::string P9116Params::toString() const
  {
    return Td::toString("P9116Params{", io_adc::toString(adcMode), "-" , io_adc::toString(rangeType), "}");
  }

  void checkParamExistenceOrExit(const ros::NodeHandle nodeHandle, const std::string& paramName) {
    if(!nodeHandle.hasParam(paramName)) {
      Td::ros_error(Td::toString("Unable to find param \"", paramName, "\""));
      exit(-1);
    }
  }

  const char* toString(AdcMode mode)
  {
    switch(mode) {
      case AdcMode::localGND     : return "localGND";
      case AdcMode::userCMMD     : return "userCMMD";
      case AdcMode::differential : return "differential";
      default: return "UNKNOWN";
    }
  }

  const char* toString(RangeType rangeType)
  {
    switch (rangeType)
    {
        case RangeType::unipolar : return "unipolar";
        case RangeType::bipolar : return "bipolar";
        default: return "UNKNOWN";
    };
  }

}
