#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mtrackercar
{

class SerialPort
{
public:
  SerialPort() = default;
  SerialPort(const std::string & device, int baud_rate);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  SerialPort(SerialPort &&) = delete;
  SerialPort & operator=(SerialPort &&) = delete;

  void write_all(const std::uint8_t * data, std::size_t size);
  void read_available(std::vector<std::uint8_t> & destination);
  void open_port(const std::string & device, int baud_rate);
  void close_port();
  bool is_open() const {return file_descriptor_ >= 0;}

private:
  int file_descriptor_{-1};
};

}  // namespace mtrackercar
