#include "mg_drivers/base/serial_device.hpp"

#include <cstdio>

using namespace mg_drivers;

SerialDevice::SerialDevice(std::string device_name)
: device_name_(device_name), serial_(device_name)
{
  if (!serial_.is_open_serial_) {
    printf("Serial Fail: cound not open %s\n", device_name_.c_str());
  }
}

bool SerialDevice::is_alive() const { return serial_.is_open_serial_; }

bool SerialDevice::is_device_present() const { return serial_.is_device_present(); }

void SerialDevice::close_serial() { serial_.close_serial(); }

void SerialDevice::reset_serial() { serial_.reset_serial(); }
