#include "io_adc/adc_card_python.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"
#include "io_adc/consts.hpp"

#include <boost/python.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

using namespace std;

AdcCardPython::AdcCardPython()
{
}

AdcCardPython::~AdcCardPython()
{
}

void AdcCardPython::open()
{
  if ((adcCard = Register_Card(PCI_9116, 0)) < 0) {
      cout << "unable to open pci_9116" << endl;
      return;
  }

  uint16_t err;
  if ((err = AI_9116_Config(adcCard, P9116_AI_SingEnded|P9116_AI_UserCMMD, P9116_AI_SoftPolling ,0, 0, 0)) != NoError) {
      cout << "unable to open pci_9116" << endl;
      return;
  }

}

void AdcCardPython::close()
{
  if(adcCard >= 0) {
    Release_Card(adcCard);
  }

  adcCard = -1;
}

int32_t AdcCardPython::readChannelRaw(uint32_t channel, io_adc::VoltageRange range)
{
  uint16_t err;
  uint16_t value;
  if ((err = AI_ReadChannel(adcCard, channel, range, &value) ) != NoError) {
    if (range >= io_adc::VoltageRange::U_10_V) {
      return value;
    } else {
      return *(unsigned int16_t*)&value;
    }
  }
}

double AdcCardPython::readChannel(uint32_t channel, io_adc::VoltageRange range)
{
  uint16_t err;
  uint16_t value;
  double res;
  if ((err = AI_ReadChannel(adcCard, channel, range, &value) ) != NoError) {
    AI_VoltScale(adcCard, range, value, &res);
    return res;
  }
  return -NAN;
}


//   if ((err = AI_ReadChannel(adcCard, i, voltageRanges[i], (uint16_t*)&rawIntegerValues[i]) ) != NoError) {

std::string initModule() {
  return "";
}

BOOST_PYTHON_MODULE(adc_card_python)
{
  using namespace boost::python;
  using namespace io_adc;

  def("init", initModule);

  class_<AdcCardPython>("io_card_python", init<>())
    .def("read_channel_raw",     &AdcCardPython::readChannelRaw)
    .def("read_channel",  &AdcCardPython::readChannel)
    .def("open",  &AdcCardPython::open)
    .def("close", &AdcCardPython::close);


//     AD_B_10_V    = 1,
//     AD_B_5_V     = 2,
//     AD_B_2_5_V   = 3,
//     AD_B_1_25_V  = 4,
//     AD_U_10_V    = 15,
//     AD_U_5_V     = 16,
//     AD_U_2_5_V   = 17,
//     AD_U_1_25_V  = 18
      // CANBAUDRATE
    enum_<io_adc::VoltageRange>("VoltageRange")
      .value("B10", VoltageRange::B_10_V)
      .value("B5", io_adc::B_5_V)
      .value("B2_5", io_adc::B_2_5_V)
      .value("B1_25", io_adc::B_1_25_V)
      .value("U10", io_adc::U_10_V)
      .value("U5", io_adc::U_5_V )
      .value("U2_5", io_adc::U_2_5_V)
      .value("U1_25", io_adc::U_1_25_V);

}
