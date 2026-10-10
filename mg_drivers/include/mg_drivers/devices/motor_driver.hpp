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
struct MotorDriverResponse
{
  std::vector<uint8_t> raw;
  SerialError error = SerialError::NO_ERROR;
};
struct SpeedParameter
{
  int16_t right_wheel_speed, left_wheel_speed;  // mm/s
};

class MotorDriver : public SerialDevice
{
private:
  int sleep_usec_ = 40000;  // usec

  int send_speed_command_size_ = 7;
  int receive_speed_command_size_ = 4;
  std::vector<uint8_t> send_speed_command_header_{0x96, 0x47};

public:
  MotorDriver() {};
  MotorDriver(std::string device_name);
  virtual ~MotorDriver() {};

  MotorDriverResponse send_speed_command(SpeedParameter param);
  std::vector<uint8_t> encode(SpeedParameter param);

  // 停止の指令を送って、接続を確かめる。チェックサムの誤りは、応答があったので接続できているとみなす
  SerialError probe() override;
};
}  // namespace mg_drivers
