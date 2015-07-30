#pragma once

namespace io_adc {

struct MotorTempSensorCalibration {
  double baseTemp;
  double tempVoltDiff;
  double baseTempVolt;
  double dangerVolt;
};

}
