#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "mg_drivers/base/serial_device.hpp"

namespace mg_drivers
{
/**
 * シリアル接続の機器を扱うノード (モータドライバ、ホイールオドメトリ) の共通の基底クラス。
 *
 * - 通信の結果 (handle_serial_error) から連続エラー数を数え、閾値 (`<param_prefix>.error_recovery_count`、
 *   既定 4) に達したら、シリアルを開き直す (1 回だけ)
 * - 1 Hz で接続を確認し、切れていれば再接続して、`~/connected` (`std_msgs/Bool`) に出す
 *   - デバイスファイルが消えたら、ポートを閉じる。戻ったら開き直す
 *   - 開けていない、または連続エラーが閾値以上なら、開き直して probe() で通信を確かめる
 *   - エラーが 1 回以上ならば、probe() で確かめる (通信が無いあいだも切断に気づくため)
 *
 * 派生クラスは、機器を生成したあとに set_device() を呼ぶ。
 */
class SerialDriverNode : public rclcpp::Node
{
public:
  virtual ~SerialDriverNode() = default;

protected:
  SerialDriverNode(
    const std::string & node_name, const std::string & param_prefix,
    const rclcpp::NodeOptions & options);

  void set_device(std::shared_ptr<SerialDevice> device);

  // 通信の結果を反映する。エラーなし (NO_ERROR) で連続エラー数を 0 に戻す
  void handle_serial_error(SerialError error);

  // 1 Hz のタイマーが呼ぶ
  void health_check();

  // 開いていて、デバイスがあり、連続エラーが 0
  bool is_connected() const;

private:
  void probe_connection();

  std::shared_ptr<SerialDevice> device_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  int error_recovery_count_;
  int consecutive_errors_ = 0;
  std::optional<bool> last_connected_;
};
}  // namespace mg_drivers
