#include "io_adc/current_reader.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"

#include "std_msgs/Float32.h"

#include "team_diana_lib/strings/strings.h"

#include <algorithm>

using namespace io_adc;
using namespace std;

CurrentReader::CurrentReader()  {
}

void CurrentReader::update(int adc_card)
{
  for (std::pair<const std::string, CurrentReaderConf>& conf: configurations) {
    uint16_t err;
    uint16_t valueRaw;
    ROS_INFO("Reading %s", conf.first.c_str());
    if ((err = AI_ReadChannel(adc_card, conf.second.port, conf.second.portVoltRange, (U16*) &valueRaw)) != NoError) {
      log_input_error_adc(conf.second.port, "while reading voltage for current " + conf.first, err);
      return;
    }
    float value = (valueRaw - conf.second.zeroOffset) * conf.second.vToAmpFactor;
    lastCurrentValues[conf.first] = value;
    std_msgs::Float32 m;
    m.data = value;
    currentPublishers[conf.first].publish(m);
  }
}

void CurrentReader::addConfiguration(ros::NodeHandle& nodeHandle, CurrentReaderConf conf)
{
  configurations[conf.name] = conf;
  currentPublishers[conf.name] = nodeHandle.advertise<std_msgs::Float32>(
      Td::toString(conf.name, "_current").c_str(), 100);
}

map<std::string, float> CurrentReader::getValues()
{
  return lastCurrentValues;
}
