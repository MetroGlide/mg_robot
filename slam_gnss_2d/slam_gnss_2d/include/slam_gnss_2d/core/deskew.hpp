#pragma once

#include <functional>
#include <optional>

#include "slam_gnss_2d/core/config.hpp"
#include "slam_gnss_2d/core/data_types.hpp"

namespace slam_gnss_2d {
namespace core {

// 時刻 t [s] のオドメトリ姿勢を返す関数
using OdomLookup = std::function<std::optional<OdomData>(double)>;

// スキャン内のロボット運動によるゆがみを補正する。
// 各ビームの計測時刻のオドメトリ姿勢を補間し、全ビームの終端点を header.stamp 時点の
// ロボット座標系へ変換して、scan の ranges とビーム角度 (angles) を書き換える。
// 補正しなかった場合 (無効設定、走査時間不明、オドメトリ取得失敗) は false を返し scan を変更しない。
bool deskew_scan(ScanData& scan, const OdomLookup& odom_at, const DeskewConfig& config);

}  // namespace core
}  // namespace slam_gnss_2d
