// イベントログの logfmt 書式 (evt=<domain>.<event> key=value ...) を作る。ROS に依存しない。
// 書式の規約は doc/monitoring.md。Python 版 (mg_utils/event_log.py) と同じ規則にしてある。
#ifndef MG_UTILS__EVENT_FORMAT_HPP_
#define MG_UTILS__EVENT_FORMAT_HPP_

#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace mg_utils
{

namespace event_format_detail
{

inline bool IsLower(char c) {return c >= 'a' && c <= 'z';}
inline bool IsDigit(char c) {return c >= '0' && c <= '9';}

// 先頭が小文字で、残りが小文字・数字・'_' なら true
inline bool IsIdentifier(const std::string & text)
{
  if (text.empty() || !IsLower(text[0])) {
    return false;
  }
  for (char c : text) {
    if (!IsLower(c) && !IsDigit(c) && c != '_') {
      return false;
    }
  }
  return true;
}

inline bool IsEventName(const std::string & name)
{
  const auto dot = name.find('.');
  if (dot == std::string::npos) {
    return false;
  }
  return IsIdentifier(name.substr(0, dot)) && IsIdentifier(name.substr(dot + 1));
}

}  // namespace event_format_detail

// 空白・'"'・'='・'\' を含む値と空文字列は '"' で囲み、中の '"' '\' と改行・タブをエスケープする。
inline std::string QuoteIfNeeded(const std::string & text)
{
  bool needs_quote = text.empty();
  for (char c : text) {
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\v' || c == '\f' || c == '"' ||
      c == '=' || c == '\\')
    {
      needs_quote = true;
      break;
    }
  }
  if (!needs_quote) {
    return text;
  }
  std::string out = "\"";
  for (char c : text) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c;
    }
  }
  return out + "\"";
}

// イベントの 1 つのフィールド。値は bool・数値・文字列を渡せる (bool は true/false、実数は有効桁 6)。
struct EventField
{
  std::string key;
  std::string value;

  EventField(std::string k, const std::string & v)
  : key(std::move(k)), value(QuoteIfNeeded(v)) {}
  EventField(std::string k, const char * v)
  : key(std::move(k)), value(QuoteIfNeeded(v)) {}
  EventField(std::string k, bool v)
  : key(std::move(k)), value(v ? "true" : "false") {}
  template<typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value &&
    !std::is_same<T, bool>::value>>
  EventField(std::string k, T v)
  : key(std::move(k))
  {
    std::ostringstream stream;
    stream << v;
    value = stream.str();
  }
};

// "evt=<name> key=value ..." の本文を作る。name が <domain>.<event> の形でない、
// またはキーが小文字・数字・'_' だけでないときは std::invalid_argument。
inline std::string FormatEvent(
  const std::string & name, std::initializer_list<EventField> fields = {})
{
  if (!event_format_detail::IsEventName(name)) {
    throw std::invalid_argument("event name must be <domain>.<event>: " + name);
  }
  std::string out = "evt=" + name;
  for (const auto & field : fields) {
    if (!event_format_detail::IsIdentifier(field.key) || field.key == "evt") {
      throw std::invalid_argument("invalid event key: " + field.key);
    }
    out += " " + field.key + "=" + field.value;
  }
  return out;
}

}  // namespace mg_utils

#endif  // MG_UTILS__EVENT_FORMAT_HPP_
