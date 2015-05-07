#ifndef IO_ADC_IO_CARD_PYTHON_HPP
#define IO_ADC_IO_CARD_PYTHON_HPP

#include <cstdint>

class IoCardPython {

public:

  IoCardPython();
  ~IoCardPython();

  void open();
  void close();

  uint32_t readPort();
  void writePort(uint32_t value);

  bool readLine(uint32_t line);
  void writeLine(uint32_t line, bool value);

private:

  uint32_t ioCard;

};


#endif // IO_ADC_IO_CARD_PYTHON_HPP
