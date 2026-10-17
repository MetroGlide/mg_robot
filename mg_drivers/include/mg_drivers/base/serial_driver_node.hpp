#pragma once

#include <memory>
#include <optional>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "mg_drivers/base/serial_device.hpp"
#include "mg_utils/event_log.hpp"

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
 * - 診断 `connection` を出す (`diagnostic_updater`。切れていれば ERROR)。派生クラスは updater() に
 *   機器固有の診断を足せる
 * - 接続と切断、開き直しの試行を、イベントログ (`driver.*`) に出す
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

  // 機器固有の診断を足すための updater。hardware_id は param_prefix
  diagnostic_updater::Updater & updater() { return updater_; }

  // イベントログ (driver.*)。派生クラスが機器固有のイベントを出すのにも使う
  const mg_utils::EventLog & events() const { return events_; }

  // 診断 `connection` の内容を作る (updater が呼ぶ)
  void produce_connection_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // 切断の理由。接続していれば空
  std::string disconnect_reason() const;

private:
  void probe_connection();

  // シリアルを開き直す。最初の試行と、以降は 10 回ごとにイベントに出す (1 Hz の再試行で埋めないため)
  void reset_device(const std::string & reason);

  std::shared_ptr<SerialDevice> device_;
  std::string device_label_;
  diagnostic_updater::Updater updater_;
  mg_utils::EventLog events_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  int error_recovery_count_;
  int consecutive_errors_ = 0;
  int reset_attempts_ = 0;    // 今回の切断のあいだに、開き直した回数
  int disconnect_count_ = 0;  // 起動してから切断した回数
  std::optional<bool> last_connected_;
};
}  // namespace mg_drivers
