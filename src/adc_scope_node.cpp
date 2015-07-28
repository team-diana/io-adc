#include "io_adc/adc_scope_node.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"
#include "io_adc/consts.hpp"

#include <std_msgs/Float32.h>

#include <boost/range/irange.hpp>

#include <team_diana_lib/strings/strings.h>

#include <algorithm>
#include <thread>
#include <string>

using namespace io_adc;

AdcScopeNode::AdcScopeNode() : nodeHandle("adc_scope")
{
  uint32_t i = 0;
  std::for_each(rawPublishers.begin(), rawPublishers.end(), [&](ros::Publisher& p) {
    p = nodeHandle.advertise<std_msgs::Float32>(Td::toString("analog_input", i), 1000);
    i++;
  });


  std::cout << "--- PORT CONFIGURATION --- "<< std::endl;
  i = 0;

  int pause;
  nodeHandle.param(Td::toString("pause_micro_seconds"), pause, 0);
  sleepTime = std::chrono::microseconds(pause);

  p9116Params = getP9116ParamsFromRosParams(nodeHandle);

  std::for_each(voltageRanges.begin(), voltageRanges.end(), [&](uint16_t& range) {
    std::string voltageRange;

    std::string paramName = Td::toString("voltage_range_", i);
    nodeHandle.param(paramName, voltageRange, std::string("2.5"));

    if(p9116Params.rangeType == RangeType::unipolar) {
      range = voltageRangeToEnum(voltageRange, RangeType::unipolar);
      std::cout << "port " << i << " UNIPOLAR range " << voltageRange << std::endl;
    } else {
      range = voltageRangeToEnum(voltageRange, RangeType::bipolar);
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
    if ((err = AI_9116_Config(adcCard, p9116Params.getConfigCtrlValue(), P9116_AI_SoftPolling ,0, 0, 0)) != NoError) {
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
    std::array<int16_t, ADC_ANALOG_INPUT_PORTS_NUM> rawIntegerValues;

    for(auto i : boost::irange(0, ADC_ANALOG_INPUT_PORTS_NUM)) {
      uint16_t err;
      if ((err = AI_ReadChannel(adcCard, i, voltageRanges[i], (uint16_t*)&rawIntegerValues[i]) ) != NoError) {
        log_input_error_adc(i, "while reading raw value", err);
        rawIntegerValues[i] = 0;
      }
    }

    for(auto i : boost::irange(0, ADC_ANALOG_INPUT_PORTS_NUM)) {
      std_msgs::Float32 msg;
      double converted;
      AI_VoltScale(adcCard, voltageRanges[i], rawIntegerValues[i], &converted);
      msg.data = converted;
      rawPublishers[i].publish(msg);
    }

    std::this_thread::sleep_for(sleepTime);
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
