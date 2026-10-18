#pragma once

#include <cstdint>
#include <optional>

namespace slam_gnss_2d {
namespace gnss {

// u-blox NavPVT の搬送波位相の解の種類 (flags の bit6-7: carrSoln)
enum class CarrierSolution { kNone = 0, kFloat = 1, kFixed = 2 };

// 解の種類ごとに、報告された水平精度 (hAcc) から EKF に渡す位置の分散を決める設定
struct PositionQualityConfig {
  // hAcc をこの値 [m] より小さいとはみなさない (受信機の自己申告が楽観的すぎるときの下限)
  double fix_floor_m{0.02};
  double float_floor_m{0.02};
  double single_floor_m{0.02};
  // 分散にかける係数
  double fix_scale{1.0};
  double float_scale{1.0};
  double single_scale{1.0};
  // false にすると、搬送波位相の解が無い (単独測位) 測位を使わない
  bool accept_single{true};
  // 位置の分散の 2 倍がこの値を超える測位は使わない
  double max_covariance_threshold{49.0};
};

struct Point2 {
  double x{0.0};
  double y{0.0};
};

// NavPVT の flags から搬送波位相の解の種類を取り出す。
CarrierSolution CarrierFromFlags(std::uint8_t flags);

// 水平精度 h_acc_m [m] の測位から、EKF に渡す位置の分散 [m^2] を求める。使わない測位は std::nullopt。
std::optional<double> PositionVariance(
    const PositionQualityConfig& config, CarrierSolution carrier, double h_acc_m);

// アンテナ位置 antenna (map 座標) と車体の向き base_yaw から、車体中心 (base_footprint) の位置を求める。
// lever_x, lever_y は車体座標系でのアンテナの位置。
Point2 AntennaToBase(Point2 antenna, double base_yaw, double lever_x, double lever_y);

// 車体の向きが分からないときに、レバーアームの分だけ位置が不確かになる分散 [m^2] (最悪の場合の二乗)。
double LeverArmUncertaintyVariance(double lever_x, double lever_y);

// 外から指示された、位置の分散の倍率 (自己位置の監督が、GNSS と他の情報源の残差から求める)。
// 倍率は 1 以上 max_scale 以下に収め、最後に受け取ってから timeout_sec を過ぎたら 1 に戻す
// (監督が止まったときに、GNSS を弱いままにしない)。
class VarianceScale {
 public:
  VarianceScale(double timeout_sec, double max_scale);
  void Update(double scale, double now_sec);
  double Current(double now_sec) const;

 private:
  double timeout_sec_;
  double max_scale_;
  double scale_{1.0};
  double received_sec_{0.0};
  bool received_{false};
};

// 分散に倍率をかける (倍率が 1 未満なら、かけない)。
double ApplyVarianceScale(double variance, double scale);

}  // namespace gnss
}  // namespace slam_gnss_2d
