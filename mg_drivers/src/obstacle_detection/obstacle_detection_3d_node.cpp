#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "obstacle_detection/obstacle_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

namespace
{

constexpr char kBaseFrame[] = "base_link";

static_assert(sizeof(obstacle_detection::Point3) == 3 * sizeof(float), "Point3 must be packed");

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point & from, const Clock::time_point & to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

// 平均・最大を集計する
struct StatAccumulator
{
  double sum = 0.0;
  double max = 0.0;

  void add(double v)
  {
    sum += v;
    max = std::max(max, v);
  }
};

size_t findFloatFieldOffset(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        throw std::runtime_error("PointCloud2 field '" + name + "' is not FLOAT32");
      }
      return field.offset;
    }
  }
  throw std::runtime_error("PointCloud2 has no field '" + name + "'");
}

}  // namespace

class ObstacleDetection3DNode : public rclcpp::Node
{
public:
  ObstacleDetection3DNode() : Node("obstacle_detection_3d_node")
  {
    const auto params = loadParams();
    detector_ = std::make_unique<obstacle_detection::ObstacleDetector>(params);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const bool use_sensor_data_qos = this->get_parameter("use_sensor_data_qos").as_bool();
    rclcpp::QoS qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10).reliable();

    pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/points_obstacle", qos);
    if (publish_markers_) {
      pub_markers_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("~/cluster_markers", qos);
    }

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", qos, std::bind(&ObstacleDetection3DNode::pointcloudCallback, this, _1));

    stats_window_start_ = Clock::now();
  }

private:
  // yaml のキーと 1 対 1 で宣言・取得し、不正値は起動時に例外で終了する
  obstacle_detection::ObstacleDetectionParams loadParams()
  {
    this->declare_parameter<bool>("use_sensor_data_qos", false);

    obstacle_detection::ObstacleDetectionParams p;
    p.stride = this->declare_parameter<int>("stride", p.stride);
    p.cropbox_x_min = this->declare_parameter<double>("cropbox_x_min", p.cropbox_x_min);
    p.cropbox_x_max = this->declare_parameter<double>("cropbox_x_max", p.cropbox_x_max);
    p.cropbox_y_min = this->declare_parameter<double>("cropbox_y_min", p.cropbox_y_min);
    p.cropbox_y_max = this->declare_parameter<double>("cropbox_y_max", p.cropbox_y_max);
    p.cropbox_z_min = this->declare_parameter<double>("cropbox_z_min", p.cropbox_z_min);
    p.cropbox_z_max = this->declare_parameter<double>("cropbox_z_max", p.cropbox_z_max);
    p.grid_size = this->declare_parameter<double>("grid_size", p.grid_size);
    p.delta_z_threshold =
      this->declare_parameter<double>("delta_z_threshold", p.delta_z_threshold);
    p.min_points_per_cell =
      this->declare_parameter<int>("min_points_per_cell", p.min_points_per_cell);
    p.z_outlier_trim = this->declare_parameter<int>("z_outlier_trim", p.z_outlier_trim);
    p.cluster_tolerance =
      this->declare_parameter<double>("cluster_tolerance", p.cluster_tolerance);
    p.min_cluster_cells = this->declare_parameter<int>("min_cluster_cells", p.min_cluster_cells);

    publish_markers_ = this->declare_parameter<bool>("publish_markers", true);
    publish_stats_ = this->declare_parameter<bool>("publish_stats", true);
    stats_period_ = this->declare_parameter<double>("stats_period", 1.0);
    if (stats_period_ <= 0.0) {
      throw std::invalid_argument("obstacle_detection params: stats_period must be > 0");
    }

    obstacle_detection::ObstacleDetector::validate(p);

    RCLCPP_INFO(
      this->get_logger(),
      "params: stride=%d crop x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] grid_size=%.3f "
      "delta_z_threshold=%.3f min_points_per_cell=%d z_outlier_trim=%d cluster_tolerance=%.3f "
      "min_cluster_cells=%d publish_markers=%d publish_stats=%d stats_period=%.1f",
      p.stride, p.cropbox_x_min, p.cropbox_x_max, p.cropbox_y_min, p.cropbox_y_max,
      p.cropbox_z_min, p.cropbox_z_max, p.grid_size, p.delta_z_threshold, p.min_points_per_cell,
      p.z_outlier_trim, p.cluster_tolerance, p.min_cluster_cells, publish_markers_, publish_stats_,
      stats_period_);
    return p;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto t_start = Clock::now();

    // 1. センサ座標 -> base_link（待たずに最新の変換を使う）
    Eigen::Isometry3f sensor_to_base = Eigen::Isometry3f::Identity();
    try {
      const auto tf = tf_buffer_->lookupTransform(
        kBaseFrame, msg->header.frame_id, msg->header.stamp, tf2::Duration::zero());
      const auto & q = tf.transform.rotation;
      const auto & t = tf.transform.translation;
      sensor_to_base.linear() =
        Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      sensor_to_base.translation() = Eigen::Vector3f(t.x, t.y, t.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Could not transform %s to %s: %s",
        msg->header.frame_id.c_str(), kBaseFrame, ex.what());
      return;
    }
    const auto t_tf = Clock::now();

    // 2. 検出（生バッファから直接読む）
    obstacle_detection::PointCloudView view;
    view.data = msg->data.data();
    view.num_points = static_cast<size_t>(msg->width) * msg->height;
    view.point_step = msg->point_step;
    view.x_offset = findFloatFieldOffset(*msg, "x");
    view.y_offset = findFloatFieldOffset(*msg, "y");
    view.z_offset = findFloatFieldOffset(*msg, "z");
    detector_->process(view, sensor_to_base);

    // 3. 検出ゼロでも空の点群を publish し、コストマップ側のクリアリングを可能にする
    publishObstacles(msg->header.stamp);
    if (publish_markers_) {
      publishMarkers(msg->header.stamp);
    }

    if (publish_stats_) {
      const auto t_end = Clock::now();
      recordStats(
        elapsedMs(t_start, t_tf), elapsedMs(t_tf, t_end), elapsedMs(t_start, t_end),
        (this->now() - msg->header.stamp).seconds() * 1000.0);
    }
  }

  void publishObstacles(const rclcpp::Time & stamp)
  {
    const auto & points = detector_->obstacle_points();

    sensor_msgs::msg::PointCloud2 out;
    out.header.frame_id = kBaseFrame;
    out.header.stamp = stamp;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_bigendian = false;
    out.is_dense = true;
    out.point_step = sizeof(obstacle_detection::Point3);
    out.row_step = out.point_step * out.width;
    out.fields.resize(3);
    const char * names[] = {"x", "y", "z"};
    for (uint32_t i = 0; i < 3; ++i) {
      out.fields[i].name = names[i];
      out.fields[i].offset = i * sizeof(float);
      out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out.fields[i].count = 1;
    }
    out.data.resize(points.size() * sizeof(obstacle_detection::Point3));
    if (!points.empty()) {
      std::memcpy(out.data.data(), points.data(), out.data.size());
    }
    pub_obstacle_->publish(out);
  }

  void publishMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // 前フレームのマーカーを全消去
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    int id = 0;
    int cluster_num = 1;
    for (const auto & cluster : detector_->clusters()) {
      visualization_msgs::msg::Marker bbox;
      bbox.header.frame_id = kBaseFrame;
      bbox.header.stamp = stamp;
      bbox.ns = "obstacle_clusters";
      bbox.id = id++;
      bbox.type = visualization_msgs::msg::Marker::LINE_LIST;
      bbox.action = visualization_msgs::msg::Marker::ADD;
      bbox.pose.orientation.w = 1.0;
      bbox.scale.x = 0.02;  // 線幅
      bbox.color.r = 1.0;
      bbox.color.a = 1.0;
      bbox.lifetime = rclcpp::Duration::from_seconds(0.5);

      // 直方体の 8 頂点。ビット 0,1,2 が x,y,z の min/max に対応する
      geometry_msgs::msg::Point corners[8];
      for (int i = 0; i < 8; ++i) {
        corners[i].x = (i & 1) ? cluster.max_x : cluster.min_x;
        corners[i].y = (i & 2) ? cluster.max_y : cluster.min_y;
        corners[i].z = (i & 4) ? cluster.max_z : cluster.min_z;
      }
      // 1 ビットだけ異なる頂点同士が辺で結ばれる
      for (int i = 0; i < 8; ++i) {
        for (int bit = 1; bit < 8; bit <<= 1) {
          if (!(i & bit)) {
            bbox.points.push_back(corners[i]);
            bbox.points.push_back(corners[i | bit]);
          }
        }
      }
      marker_array.markers.push_back(bbox);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = kBaseFrame;
      text.header.stamp = stamp;
      text.ns = "obstacle_clusters_text";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = (cluster.min_x + cluster.max_x) / 2.0;
      text.pose.position.y = (cluster.min_y + cluster.max_y) / 2.0;
      text.pose.position.z = cluster.max_z + 0.1;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.1;  // 文字の高さ
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = "Cluster " + std::to_string(cluster_num++) + " (" +
                  std::to_string(cluster.num_points) + " pts)";
      text.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(text);
    }

    pub_markers_->publish(marker_array);
  }

  // stats_period ごとに、区間内の平均/最大をログ出力して集計をリセットする
  void recordStats(double tf_ms, double detect_publish_ms, double total_ms, double latency_ms)
  {
    const auto & s = detector_->stats();
    ++stats_frames_;
    stat_tf_.add(tf_ms);
    stat_accumulate_.add(s.accumulate_ms);
    stat_judge_.add(s.judge_ms);
    stat_cluster_.add(s.cluster_ms);
    stat_extract_.add(s.extract_ms);
    stat_publish_.add(detect_publish_ms - s.accumulate_ms - s.judge_ms - s.cluster_ms - s.extract_ms);
    stat_total_.add(total_ms);
    stat_latency_.add(latency_ms);
    stat_input_pts_.add(static_cast<double>(s.input_points));
    stat_cropped_pts_.add(static_cast<double>(s.cropped_points));
    stat_candidate_cells_.add(static_cast<double>(s.candidate_cells));
    stat_output_pts_.add(static_cast<double>(s.output_points));
    stat_clusters_.add(static_cast<double>(s.num_clusters));

    const auto now = Clock::now();
    const double window_s = elapsedMs(stats_window_start_, now) / 1000.0;
    if (window_s < stats_period_) return;

    const double n = static_cast<double>(stats_frames_);
    auto avg = [n](const StatAccumulator & a) { return a.sum / n; };
    RCLCPP_INFO(
      this->get_logger(),
      "stats [%.1f fps, %d frames] time avg/max [ms]: tf %.2f/%.2f accumulate %.2f/%.2f "
      "judge %.2f/%.2f cluster %.2f/%.2f extract %.2f/%.2f publish %.2f/%.2f total %.2f/%.2f "
      "latency %.1f/%.1f | points avg: input %.0f cropped %.0f output %.0f | "
      "candidate_cells %.0f clusters %.1f",
      n / window_s, stats_frames_, avg(stat_tf_), stat_tf_.max, avg(stat_accumulate_),
      stat_accumulate_.max, avg(stat_judge_), stat_judge_.max, avg(stat_cluster_),
      stat_cluster_.max, avg(stat_extract_), stat_extract_.max, avg(stat_publish_),
      stat_publish_.max, avg(stat_total_), stat_total_.max, avg(stat_latency_),
      stat_latency_.max, avg(stat_input_pts_), avg(stat_cropped_pts_), avg(stat_output_pts_),
      avg(stat_candidate_cells_), avg(stat_clusters_));

    stats_frames_ = 0;
    stats_window_start_ = now;
    for (auto * a :
         {&stat_tf_, &stat_accumulate_, &stat_judge_, &stat_cluster_, &stat_extract_,
          &stat_publish_, &stat_total_, &stat_latency_, &stat_input_pts_, &stat_cropped_pts_,
          &stat_candidate_cells_, &stat_output_pts_, &stat_clusters_}) {
      *a = StatAccumulator{};
    }
  }

  std::unique_ptr<obstacle_detection::ObstacleDetector> detector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  bool publish_markers_ = true;
  bool publish_stats_ = true;
  double stats_period_ = 1.0;

  Clock::time_point stats_window_start_;
  int stats_frames_ = 0;
  StatAccumulator stat_tf_;
  StatAccumulator stat_accumulate_;
  StatAccumulator stat_judge_;
  StatAccumulator stat_cluster_;
  StatAccumulator stat_extract_;
  StatAccumulator stat_publish_;
  StatAccumulator stat_total_;
  StatAccumulator stat_latency_;
  StatAccumulator stat_input_pts_;
  StatAccumulator stat_cropped_pts_;
  StatAccumulator stat_candidate_cells_;
  StatAccumulator stat_output_pts_;
  StatAccumulator stat_clusters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetection3DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
