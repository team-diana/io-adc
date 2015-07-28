#pragma once

#include <boost/circular_buffer.hpp>

namespace io_adc {

// Utility class that adds value to a circular buffer and exposes method
// for statistics (average)
template <typename T> class CircularBuffer {
  boost::circular_buffer<T> buffer;

public:
  CircularBuffer(unsigned int size) : buffer(size) {
  }

  void append(const T& value) {
    buffer.push_back(value);
  }

  double average() {
    return std::accumulate(buffer.begin(), buffer.end(), 0.0, [](double a, T v) {return a+v;})/buffer.size();
  }

};

}
