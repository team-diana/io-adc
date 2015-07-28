#pragma once

namespace io_adc {

struct MotorTemperatureSensorCalibration {
  double baseTemp;
  double tempVoltDiff;
  double baseTempVolt;
  double dangerVolt;
};

}
