#ifndef IO_ADC_IO_ADC_NODE_HPP
#define IO_ADC_IO_ADC_NODE_HPP

#include "io_adc/suspension_reader.hpp"
#include "io_adc/motor_temp_sensor_reader.hpp"
#include "io_adc/utils.hpp"

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

  void setupReaders();
  void setupMotorTemperatureSensorReaders();
  void setupSuspensionReaders();

  void updateMotorTemperatureSensor();
  void updateSuspensions();
  void publishSuspension(SuspensionValue suspension1Value);

private:
    ros::NodeHandle nodeHandle;
    uint16_t adcCard;
    uint16_t ioCard;
    std::chrono::milliseconds sleepTime;
    io_adc::P9116Params P9116Params;

    io_adc::MotorTempSensorReader motorTempSensor;

    ros::Publisher suspensionPublisher;

    // back right
    std::unique_ptr<SuspensionReader> suspenionReader1;

};

#endif // IO_ADC_IO_ADC_NODE_HPP
