#include "io_adc/suspension_reader.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"

#include <algorithm>

using namespace io_adc;

uint16_t SuspensionReader::suspensionRangeVolt = AD_B_2_5_V;

SuspensionReader::SuspensionReader(SuspensionPortConf suspensionPortConf, const std::string& name) :
portConf(suspensionPortConf),
name(name)
{

}

void SuspensionReader::update(int adc_card)
{

  uint16_t err;

  uint16_t xValueRaw;
  uint16_t yValueRaw;

  if ((err = AI_ReadChannel(adc_card, portConf.xAdcPort, suspensionRangeVolt, (U16*) &xValueRaw)) != NoError) {
    log_input_error_adc(portConf.xAdcPort, "while reading x for suspension " + name, err);
    return;
  }
  if ((err = AI_ReadChannel(adc_card, portConf.zAdcPort, suspensionRangeVolt, (U16*) &yValueRaw)) != NoError) {
    log_input_error_adc(portConf.zAdcPort, "while reading x for suspension " + name, err);
    return;
  }


// chan_voltage[i] = (chan_data_raw[i] * vRef / AG_MAX_VALUE - zeroRateV) / sensitivity;
// double sosp_z = -chan_voltage[i];
// double sosp_x = chan_voltage[i + 1];

  auto transformToValue = [](uint16_t valueRaw) {
    return float(valueRaw);
  };

  lastSuspensionValue.x = transformToValue(xValueRaw);
  lastSuspensionValue.z = transformToValue(yValueRaw);

}

SuspensionValue SuspensionReader::getValue()
{
  return lastSuspensionValue;
}
