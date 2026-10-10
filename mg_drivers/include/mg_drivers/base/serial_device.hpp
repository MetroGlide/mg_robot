#pragma once

#include <string>

#include "mg_drivers/base/serial_communicator2.hpp"

namespace mg_drivers
{
/**
 * シリアル接続の機器 (モータドライバ、ホイールオドメトリ) の共通の基底クラス。
 * シリアルポートの開閉と、接続の確認を受け持つ。再接続の判断は SerialDriverNode が行う。
 */
class SerialDevice
{
public:
  SerialDevice() = default;
  explicit SerialDevice(std::string device_name);
  virtual ~SerialDevice() = default;

  // シリアルポートを開けているか
  virtual bool is_alive() const;
  // デバイスファイル (udev のシンボリックリンクなど) が存在するか。抜かれると消える
  virtual bool is_device_present() const;
  virtual void close_serial();
  // 閉じて、開き直す。デバイスが無いと、閉じたままになる
  virtual void reset_serial();

  // 接続の確認のために、機器と通信する。結果を返す
  virtual SerialError probe() = 0;

protected:
  std::string device_name_;
  SerialCommunicator2 serial_;
};
}  // namespace mg_drivers
