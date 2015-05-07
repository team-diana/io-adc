#include "io_adc/io_card_python.hpp"
#include "io_adc/utils.hpp"
#include "io_adc/dask.h"

#include <boost/python.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

IoCardPython::IoCardPython()
{
}

IoCardPython::~IoCardPython()
{
  close();
}

void IoCardPython::open()
{
  if ((ioCard = Register_Card(PCI_7432, 0)) < 0) {
    std::cout << "ERROR while opening cpci_7432" << std::endl;
  }
}

void IoCardPython::close()
{
  if(ioCard >= 0) {
    Release_Card(ioCard);
  }
  ioCard = -1;
}


bool IoCardPython::readLine(uint32_t i)
{
  if(i >= 32 || i < 0) {
    std::cerr << "ERROR invalid line " << i << std::endl;
    return false;
  }

  uint16_t err;
  uint16_t value;
  if((err = DO_ReadLine(ioCard, 0, i, &value)) != NoError) {
    std::cerr << "ERROR while reading line  " << i << " : "<< err << std::endl;
  }
  return value;
}

uint32_t IoCardPython::readPort()
{
  uint16_t err;
  uint64_t portValue;
  if((err = DO_ReadPort(ioCard, 0, &portValue)) != NoError) {
    std::cerr << "ERROR while reading port 0: " << err << std::endl;
  }
  return (uint32_t)portValue;
}

void IoCardPython::writeLine(uint32_t line, bool value)
{
  if(line >= 32 || line < 0) {
    std::cerr << "ERROR invalid line " << line << std::endl;
    return;
  }

  uint16_t err;
  if((err = DO_WriteLine(ioCard, 0, line, value)) != NoError) {
    std::cerr << "ERROR while writing line  " << line << " : "<< err << std::endl;
  }
}

void IoCardPython::writePort(uint32_t value)
{
  uint64_t value64 = value;
  uint16_t err;

  if((err = DO_WritePort(ioCard, 0, value64)) != NoError) {
    std::cerr << "ERROR while reading port 0: " << err << std::endl;
  }
}


std::string initModule() {
  return "";
}

BOOST_PYTHON_MODULE(io_card)
{
  using namespace boost::python;

  def("init", initModule);

  class_<IoCardPython>("io_card", init<>())
    .def("read_port",     &IoCardPython::readPort)
    .def("read_channel",  &IoCardPython::readLine)
    .def("write_port",    &IoCardPython::writePort)
    .def("write_channel", &IoCardPython::writeLine)
    .def("open",  &IoCardPython::open)
    .def("close", &IoCardPython::close);

}
