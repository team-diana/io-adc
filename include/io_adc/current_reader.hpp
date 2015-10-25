#ifndef IO_ADC_CURRENT_READER_HPP
#define IO_ADC_CURRENT_READER_HPP

#include "ros/publisher.h"
#include "ros/node_handle.h"

#include <array>
#include <map>

struct CurrentReaderConf {
    std::string name;
    int port;
    int portVoltRange;
    float zeroOffset;
    float vToAmpFactor;
};

class CurrentReader {

public:
  CurrentReader();

  void addConfiguration(ros::NodeHandle& nodeHandle, CurrentReaderConf conf);

  void update(int adc_card);
  std::map<std::string, float> getValues();

private:
  std::string name;
  std::map<std::string, CurrentReaderConf> configurations;
  std::map<std::string, float> lastCurrentValues;
  std::map<std::string, ros::Publisher> currentPublishers;

};

#endif // IO_ADC_SUSPENSION_READER_HPP
