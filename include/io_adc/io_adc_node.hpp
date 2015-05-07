#ifndef IO_ADC_IO_ADC_NODE_HPP
#define IO_ADC_IO_ADC_NODE_HPP

#include "io_adc/suspension_reader.hpp"

#include "ros/node_handle.h"


class IoAdcNode {
 
public:
  IoAdcNode();
  IoAdcNode(const IoAdcNode& oth) = delete;

  ~IoAdcNode();

  bool init();

  void run();


private:

  void setupReaders();
  void setupSuspensionReaders();

  void updateSuspensions();
  void publishSuspension(SuspensionValue suspension1Value);

private:
    ros::NodeHandle nodeHandle;
    uint16_t adcCard;
    uint16_t ioCard;


    ros::Publisher suspensionPublisher;

    // back right
    std::unique_ptr<SuspensionReader> suspenionReader1;

};

#endif // IO_ADC_IO_ADC_NODE_HPP
