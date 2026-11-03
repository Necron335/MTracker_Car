#include "mtrackercar/communication/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace mtrackercar
{

SerialPort::SerialPort(const std::string & device, int baud_rate)
{
  open_port(device, baud_rate);
}

void SerialPort::open_port(const std::string & device, int baud_rate)
{
  close_port();
  file_descriptor_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (file_descriptor_ < 0) {
    throw std::runtime_error("Cannot open " + device + ": " + std::strerror(errno));
  }

  termios options{};
  if (tcgetattr(file_descriptor_, &options) != 0) {
    const std::string reason = std::strerror(errno);
    close(file_descriptor_);
    file_descriptor_ = -1;
    throw std::runtime_error("Cannot read serial settings: " + reason);
  }

  cfmakeraw(&options);
  speed_t speed{};
  switch (baud_rate) {
    case 115200: speed = B115200; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:
      close_port();
      throw std::invalid_argument("Supported baud rates: 115200, 460800, 921600");
  }
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
  options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;

  if (tcsetattr(file_descriptor_, TCSANOW, &options) != 0) {
    const std::string reason = std::strerror(errno);
    close(file_descriptor_);
    file_descriptor_ = -1;
    throw std::runtime_error("Cannot configure serial port: " + reason);
  }
  tcflush(file_descriptor_, TCIOFLUSH);
}

SerialPort::~SerialPort()
{
  close_port();
}

void SerialPort::close_port()
{
  if (file_descriptor_ >= 0) {
    close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

void SerialPort::write_all(const std::uint8_t * data, std::size_t size)
{
  std::size_t written = 0;
  while (written < size) {
    const auto result = write(file_descriptor_, data + written, size - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0 && errno == EAGAIN) {
      pollfd descriptor{file_descriptor_, POLLOUT, 0};
      if (poll(&descriptor, 1, 100) > 0) {
        continue;
      }
      throw std::runtime_error("Serial write timed out");
    }
    throw std::runtime_error("Serial write failed: " + std::string(std::strerror(errno)));
  }
}

void SerialPort::read_available(std::vector<std::uint8_t> & destination)
{
  std::array<std::uint8_t, 256> chunk{};
  while (true) {
    const auto size = read(file_descriptor_, chunk.data(), chunk.size());
    if (size > 0) {
      destination.insert(destination.end(), chunk.begin(), chunk.begin() + size);
      continue;
    }
    if (size < 0 && errno == EINTR) {
      continue;
    }
    if (size == 0 || (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
      return;
    }
    throw std::runtime_error("Serial read failed: " + std::string(std::strerror(errno)));
  }
}

}  // namespace mtrackercar
