#ifndef IO_ADC_IO_ADC_NODE_HPP
#define IO_ADC_IO_ADC_NODE_HPP

#include "io_adc/suspension_reader.hpp"
#include "io_adc/motor_temp_sensor_reader.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/OnOffService.h"
#include "io_adc/current_reader.hpp"

#include "ros/node_handle.h"

#include <chrono>

class IoAdcNode {

public:
  IoAdcNode();
  IoAdcNode(const IoAdcNode& oth) = delete;

  ~IoAdcNode();

  bool init();

  void run();


private:

  bool setupReaders();
  void setupMotorTemperatureSensorReaders();
  bool setupSuspensionReaders();
  bool setupCurrentReaders();

  void updateMotorTemperatureSensor();
  void updateSuspensions();
  void publishSuspension(SuspensionValue suspension1Value);
  bool setEnableSuspensionPowerCallback(io_adc::OnOffService::Request& req,
                                io_adc::OnOffService::Response& res);


private:
    ros::NodeHandle nodeHandle;
    uint16_t adcCard;
    uint16_t ioCard;
    std::chrono::milliseconds sleepTime;
    io_adc::P9116Params P9116Params;

    io_adc::MotorTempSensorReader motorTempSensor;

    ros::Publisher suspensionPublisher;
    ros::Publisher suspensionCurrentPublisher;
    ros::Publisher elmoCurrentPublisher;
    ros::Publisher switchCurrentPublisher;
    ros::Publisher pantiltCurrentPublisher;
    ros::Publisher cPCICurrentPublisher;

    ros::ServiceServer enableSuspensionPower;


    // back right
    std::unique_ptr<SuspensionReader> suspenionReader1;
    CurrentReader currentReader;

};

#endif // IO_ADC_IO_ADC_NODE_HPP
