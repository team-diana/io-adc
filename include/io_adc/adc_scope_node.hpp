#ifndef IO_ADC_ADC_SCOPE_NODE_HPP
#define IO_ADC_ADC_SCOPE_NODE_HPP

#include <ros/ros.h>

class AdcScopeNode {

public:
  AdcScopeNode();
  ~AdcScopeNode();

  bool init();

  void run();

private:
  ros::NodeHandle nodeHandle;
  std::array<ros::Publisher, 64> rawPublishers;
  std::array<uint16_t, 64> voltageRanges;
  uint16_t adcCard;

};


#endif // IO_ADC_ADC_SCOPE_NODE_HPP
