#include "io_adc/motor_temp_sensor_reader.hpp"
#include "io_adc/dask.h"
#include "io_adc/utils.hpp"

#include "io_adc/wheel_motor_temperature.h"

#include "team_diana_lib/strings/strings.h"

enum  {
  MOTOR_TEMP_SENS_PIN_BASE = 13
};

namespace io_adc {

void MotorTempSensorReader::readNewValues(uint16_t adcCard)
{
  for(int i = 0; i < 4; i++) {
    uint16_t err;
    int16_t rawValue;
    uint16_t range = AD_U_5_V;
    double converted;

    if ((err = AI_ReadChannel(adcCard, MOTOR_TEMP_SENS_PIN_BASE+i,
          range, (uint16_t*)&rawValue) ) != NoError) {
      log_input_error_adc(i, "while reading raw value", err);
    } else {
      AI_VoltScale(adcCard, range, rawValue, &converted);
      motTempBuffers[i].append(converted);
    }
  }
}

void MotorTempSensorReader::publishNewValues()
{
  std::vector<float> temps;
  for(int i = 0; i < 4; i++) {
    float avg = motTempBuffers[i].average();
    float temp = motTempSensCalibrations[i].baseTemp +
      (motTempSensCalibrations[i].baseTempVolt - avg) * motTempSensCalibrations[i].tempVoltDiff;
    temps.push_back(temp);
  }
  io_adc::wheel_motor_temperature msg;
  msg.temp = temps;
  motorTemperaturePublisher.publish(msg);
}


void MotorTempSensorReader::setup(ros::NodeHandle nodeHandle)
{
    for(int i = 0; i<4; i++) {
      std::string base = Td::toString("temp_sensor_calibration_motor_", i, "/");

      auto baseTemp = base + "base_temp";
      auto baseTempV = base + "base_temp_v";
      auto dangerV = base + "danger_v";
      auto tempVoltDiff = base + "temp_volt_diff";

      checkParamExistenceOrExit(nodeHandle, baseTemp);
      checkParamExistenceOrExit(nodeHandle, baseTempV);
      checkParamExistenceOrExit(nodeHandle, dangerV);
      checkParamExistenceOrExit(nodeHandle, tempVoltDiff);

      nodeHandle.param(baseTemp, motTempSensCalibrations[i].baseTemp, 0.0);
      nodeHandle.param(baseTempV, motTempSensCalibrations[i].baseTempVolt, 0.0);
      nodeHandle.param(dangerV, motTempSensCalibrations[i].dangerVolt, 0.0);
      nodeHandle.param(tempVoltDiff, motTempSensCalibrations[i].tempVoltDiff, 0.0);
    }
    motorTemperaturePublisher = nodeHandle.advertise<io_adc::wheel_motor_temperature>(
      Td::toString("motor_temperature_sensor"), 100);
}

}
