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

int AdcCardPython::open(uint16_t mode, uint16_t trigMode)
{
    if ((adcCard = Register_Card(PCI_9116, 0)) < 0) {
        cout << "unable to open pci_9116" << endl;
        return 0;
    }

    uint16_t err;
    if ((err = AI_9116_Config(adcCard, mode, trigMode, 0, 0, 0)) != NoError) {
        cout << "unable to open pci_9116" << endl;
        return 0;
    }

    return 1;
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
    if ((err = AI_ReadChannel(adcCard, channel, range, &value) ) == NoError) {
//         if (range >= io_adc::VoltageRange::U_10_V) {
//             return value;
//         } else {
            return *(int16_t*)&value;
//         }
    } else {
      std::cerr << "Error when AI_ReadChannel: " << err << std::endl;
      return -1;
    }
}

double AdcCardPython::readChannel(uint32_t channel, io_adc::VoltageRange range)
{
    uint16_t err;
    uint16_t value;
    double res;
    if ((err = AI_ReadChannel(adcCard, channel, range, &value) ) == NoError) {
        AI_VoltScale(adcCard, range, *(int16_t*)(&value), &res);
        return res;
    } else {
      std::cerr << "Error when AI_ReadChannel: " << err << std::endl;
      return -NAN;
    }
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

    class_<AdcCardPython>("AdcCard", init<>())
    .def("read_channel_raw",     &AdcCardPython::readChannelRaw)
    .def("read_channel",  &AdcCardPython::readChannel)
    .def("open",  &AdcCardPython::open)
    .def("close", &AdcCardPython::close);


    boost::python::scope().attr("P9116_AI_LocalGND") = P9116_AI_LocalGND;
    boost::python::scope().attr("P9116_AI_UserCMMD") = P9116_AI_UserCMMD;
    boost::python::scope().attr("P9116_AI_SingEnded") = P9116_AI_SingEnded;
    boost::python::scope().attr("P9116_AI_Differential") = P9116_AI_Differential;
    boost::python::scope().attr("P9116_AI_BiPolar") = P9116_AI_BiPolar;
    boost::python::scope().attr("P9116_AI_UniPolar") = P9116_AI_UniPolar;
    boost::python::scope().attr("P9116_TRGMOD_SOFT") = P9116_TRGMOD_SOFT;
    boost::python::scope().attr("P9116_TRGMOD_POST") = P9116_TRGMOD_POST;
    boost::python::scope().attr("P9116_TRGMOD_DELAY") = P9116_TRGMOD_DELAY;
    boost::python::scope().attr("P9116_TRGMOD_PRE") = P9116_TRGMOD_PRE;
    boost::python::scope().attr("P9116_TRGMOD_MIDL") = P9116_TRGMOD_MIDL;
    boost::python::scope().attr("P9116_AI_TrgPositive") = P9116_AI_TrgPositive;
    boost::python::scope().attr("P9116_AI_TrgNegative") = P9116_AI_TrgNegative;
    boost::python::scope().attr("P9116_AI_ExtTimeBase") = P9116_AI_ExtTimeBase;
    boost::python::scope().attr("P9116_AI_IntTimeBase") = P9116_AI_IntTimeBase;
    boost::python::scope().attr("P9116_AI_DlyInSamples") = P9116_AI_DlyInSamples;
    boost::python::scope().attr("P9116_AI_DlyInTimebase") = P9116_AI_DlyInTimebase;
    boost::python::scope().attr("P9116_AI_ReTrigEn") = P9116_AI_ReTrigEn;
    boost::python::scope().attr("P9116_AI_MCounterEn") = P9116_AI_MCounterEn;
    boost::python::scope().attr("P9116_AI_SoftPolling") = P9116_AI_SoftPolling;
    boost::python::scope().attr("P9116_AI_INT") = P9116_AI_INT;
    boost::python::scope().attr("P9116_AI_DMA") = P9116_AI_DMA;

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
