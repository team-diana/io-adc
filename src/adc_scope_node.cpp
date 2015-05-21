#include "io_adc/adc_scope_node.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"
#include "io_adc/consts.hpp"

#include <std_msgs/Float32.h>

#include <boost/range/irange.hpp>

#include <team_diana_lib/strings/strings.h>

#include <algorithm>

using namespace io_adc;

AdcScopeNode::AdcScopeNode() : nodeHandle("adc_scope")
{
  uint32_t i = 0;
  std::for_each(rawPublishers.begin(), rawPublishers.end(), [&](ros::Publisher& p) {
    p = nodeHandle.advertise<std_msgs::Float32>(Td::toString("analog_input", i), 1000);
    i++;
  });

  ros::NodeHandle private_node_handle("~");

  std::cout << "--- PORT CONFIGURATION --- "<< std::endl;
  i = 0;
  std::for_each(voltageRanges.begin(), voltageRanges.end(), [&](uint16_t& range) {
    double voltageRange;
    bool unipolar;

    private_node_handle.param(Td::toString("voltage_range_", i), voltageRange, 2.5);
    private_node_handle.param(Td::toString("voltage_range_", i, "_unipolar"), unipolar, false);

    if(unipolar) {
      range = voltageRangeDoubleToEnum(voltageRange, RangeType::unipolar);
      std::cout << "port " << i << " UNIPOLAR range " << voltageRange << std::endl;
    } else {
      range = voltageRangeDoubleToEnum(voltageRange, RangeType::bipolar);
      std::cout << "port " << i << " BIPOLAR range " << voltageRange << std::endl;
    }
    i++;
  });

  std::cout << std::endl << "-------------------------- "<< std::endl;

}

AdcScopeNode::~AdcScopeNode()
{
  ROS_INFO("Releasing cards");

  Release_Card(adcCard);
}


bool AdcScopeNode::init()
{
    uint16_t err;

    ROS_INFO("Init PCI_9116");
    if ((adcCard = Register_Card(PCI_9116, 0)) < 0) {
        log_register_card_error("PCI_9116", adcCard);
        return false;
    }

    ROS_INFO("Setup PCI_9116");
    if ((err = AI_9116_Config(adcCard, P9116_AI_SingEnded|P9116_AI_UserCMMD, P9116_AI_SoftPolling ,0, 0, 0)) != NoError) {
        log_register_card_error("PCI_9116 configure error", err);
        return false;
    }

    ROS_INFO("Init done");

    return true;
}


void AdcScopeNode::run()
{
  ROS_INFO("Run");

  while(ros::ok()) {
    std::array<uint16_t, ADC_ANALOG_INPUT_PORTS_NUM> rawIntegerValues;
    std::array<float, ADC_ANALOG_INPUT_PORTS_NUM> rawValues;

    for(auto i : boost::irange(0, ADC_ANALOG_INPUT_PORTS_NUM)) {
      uint16_t err;
      if ((err = AI_ReadChannel(adcCard, i, voltageRanges[i], &rawIntegerValues[i])) != NoError) {
        log_input_error_adc(i, "while reading raw value", err);
        rawValues[i] = rawIntegerValues[i];
      }
    }

    for(auto i : boost::irange(0, ADC_ANALOG_INPUT_PORTS_NUM)) {
      std_msgs::Float32 msg;
      msg.data = rawValues[i];
      rawPublishers[i].publish(msg);
    }

    ros::spinOnce();
  }

  ROS_INFO("Stop");
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "adc_scope");

  AdcScopeNode adcScopeNode;

  if(!adcScopeNode.init()) {
    ROS_ERROR("error in init, exiting");
  }

  adcScopeNode.run();
}
