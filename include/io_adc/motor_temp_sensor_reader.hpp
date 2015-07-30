#pragma once

#include "io_adc/circular_buffer.hpp"
#include "io_adc/motor_temp_sensor_calibration.hpp"

#include <ros/ros.h>

#include <boost/circular_buffer.hpp>

namespace io_adc {

enum {
  MotorTempBufSize = 20
};

struct MotorTempSensorReader {

    std::array<MotorTempSensorCalibration, 4> motTempSensCalibrations;
    std::array<CircularBuffer<float>, 4> motTempBuffers = {{
      CircularBuffer<float>(MotorTempBufSize),
      CircularBuffer<float>(MotorTempBufSize),
      CircularBuffer<float>(MotorTempBufSize),
      CircularBuffer<float>(MotorTempBufSize)
    }};

    ros::Publisher motorTemperaturePublisher;

    void setup(ros::NodeHandle nodeHandle);
    void readNewValues(uint16_t adcCard);
    void publishNewValues();
};

}
