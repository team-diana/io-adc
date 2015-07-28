#ifndef IO_ADC_IO_ADC_NODE_HPP
#define IO_ADC_IO_ADC_NODE_HPP

#include "io_adc/suspension_reader.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/motor_temperature_sensor_calibration.hpp"
#include "io_adc/circular_buffer.hpp"

#include "ros/node_handle.h"

enum {
  MotorTempBufSize = 20
};

struct MotorTempSensor {
    std::array<io_adc::MotorTemperatureSensorCalibration, 4> motTempSensCalibrations;
    std::array<io_adc::CircularBuffer<float>, 4> motTempBuffers = {{
      io_adc::CircularBuffer<float>(MotorTempBufSize),
      io_adc::CircularBuffer<float>(MotorTempBufSize),
      io_adc::CircularBuffer<float>(MotorTempBufSize),
      io_adc::CircularBuffer<float>(MotorTempBufSize)
    }};

    ros::Publisher motorTemperaturePublisher;

    void setup(ros::NodeHandle nodeHandle);
    void readNewValues(uint16_t adcCard);
    void publishNewValues();

};


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
    io_adc::P9116Params P9116Params;

    MotorTempSensor motorTempSensor;

    ros::Publisher suspensionPublisher;

    // back right
    std::unique_ptr<SuspensionReader> suspenionReader1;

};

#endif // IO_ADC_IO_ADC_NODE_HPP
