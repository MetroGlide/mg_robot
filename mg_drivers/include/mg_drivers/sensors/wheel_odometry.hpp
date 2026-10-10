#pragma once

// #include "mg_drivers/base/serial_communicator.hpp"
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

#include "mg_drivers/base/serial_communicator2.hpp"
#include "mg_drivers/base/serial_device.hpp"

namespace mg_drivers
{
struct OdometryData
{
  double x, y, th;
  double vx, vy, vth;
  std::vector<uint8_t> raw;

  SerialError error = SerialError::NO_ERROR;
};

class WheelOdometry : public SerialDevice
{
private:
  const int sleep_usec_ = 40000;  // usec

  int get_odom_ret_size_ = 19;
  std::vector<uint8_t> zero_buf_{0x24, 0x73};
  std::vector<uint8_t> odom_buf_{0x24, 0x75};

public:
  explicit WheelOdometry() {};
  explicit WheelOdometry(std::string device_name);
  virtual ~WheelOdometry() {};

  OdometryData get_data();
  SerialError send_zero_reset(int retry = 1);
  OdometryData decode(std::vector<uint8_t> ret);

  // オドメトリを 1 回読んで、接続を確かめる
  SerialError probe() override;
};
}  // namespace mg_drivers
