#ifndef IO_ADC_IO_SCOPE_NODE_HPP
#define IO_ADC_IO_SCOPE_NODE_HPP

#include <ros/ros.h>

class IoScopeNode {

public:
  IoScopeNode();
  ~IoScopeNode();

  bool init();

  void run();

private:
  ros::NodeHandle nodeHandle;
  ros::Publisher scopePublisher;
  uint16_t ioCard;

};

#endif // IO_ADC_IO_SCOPE_NODE_HPP
