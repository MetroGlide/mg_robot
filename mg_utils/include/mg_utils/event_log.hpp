// ロガーにイベントを出す。書式の規約は doc/monitoring.md。
//
//   mg_utils::EventLog log(this->get_logger());
//   log.Info("driver.connected", {{"device", "motor_driver"}, {"port", port}});
#ifndef MG_UTILS__EVENT_LOG_HPP_
#define MG_UTILS__EVENT_LOG_HPP_

#include <initializer_list>
#include <string>

#include "mg_utils/event_format.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace mg_utils
{

class EventLog
{
public:
  explicit EventLog(rclcpp::Logger logger)
  : logger_(logger) {}

  void Info(const std::string & name, std::initializer_list<EventField> fields = {}) const
  {
    RCLCPP_INFO(logger_, "%s", FormatEvent(name, fields).c_str());
  }

  void Warn(const std::string & name, std::initializer_list<EventField> fields = {}) const
  {
    RCLCPP_WARN(logger_, "%s", FormatEvent(name, fields).c_str());
  }

  void Error(const std::string & name, std::initializer_list<EventField> fields = {}) const
  {
    RCLCPP_ERROR(logger_, "%s", FormatEvent(name, fields).c_str());
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace mg_utils

#endif  // MG_UTILS__EVENT_LOG_HPP_
