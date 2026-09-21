#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "slam_gnss_2d/core/data_types.hpp"
#include "slam_gnss_2d/core/slam_data_saver.hpp"

namespace slam_gnss_2d {

class SlamDataSaverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = (std::filesystem::temp_directory_path() / "slam_data_saver_test").string();
    std::filesystem::remove_all(test_dir_);
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    std::filesystem::remove_all(test_dir_);
  }

  std::string test_dir_;
};

TEST_F(SlamDataSaverTest, SaveGnssTransform) {
  std::string file_path = core::SlamDataSaver::save_gnss_transform(
      test_dir_, 35.6812, 139.7671, 388123.4, 3949821.5, 54, "north", 0.123, "gtsam");

  EXPECT_TRUE(std::filesystem::exists(file_path));

  YAML::Node root = YAML::LoadFile(file_path);
  ASSERT_TRUE(root["anchor"]);
  EXPECT_NEAR(root["anchor"]["latitude"].as<double>(), 35.6812, 1e-6);
  EXPECT_NEAR(root["anchor"]["longitude"].as<double>(), 139.7671, 1e-6);

  ASSERT_TRUE(root["anchor_utm"]);
  EXPECT_NEAR(root["anchor_utm"]["easting"].as<double>(), 388123.4, 1e-4);
  EXPECT_NEAR(root["anchor_utm"]["northing"].as<double>(), 3949821.5, 1e-4);
  EXPECT_EQ(root["anchor_utm"]["zone"].as<int>(), 54);
  EXPECT_EQ(root["anchor_utm"]["hemisphere"].as<std::string>(), "north");

  EXPECT_NEAR(root["rotation_rad"].as<double>(), 0.123, 1e-6);

  ASSERT_TRUE(root["metadata"]);
  EXPECT_EQ(root["metadata"]["slam_backend"].as<std::string>(), "gtsam");
  EXPECT_FALSE(root["metadata"]["created_at"].as<std::string>().empty());
}

TEST_F(SlamDataSaverTest, SavePoseGraph) {
  std::vector<core::PoseNode> nodes;
  nodes.push_back(core::PoseNode{0, 100.0, 1.0, 2.0, 0.5, nullptr});
  nodes.push_back(core::PoseNode{1, 101.0, 2.0, 3.0, 0.6, nullptr});

  std::vector<core::PoseEdge> edges;
  // Sequential edge: 0 -> 1
  Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * 100.0;
  edges.push_back(core::PoseEdge{0, 1, 1.0, 1.0, 0.1, info, 0.95, false});
  // Loop edge: 0 -> 5
  edges.push_back(core::PoseEdge{0, 5, 5.0, 5.0, 0.0, info, 0.90, false});

  std::string file_path = core::SlamDataSaver::save_pose_graph(
      test_dir_, nodes, edges, "dummy_bag");

  EXPECT_TRUE(std::filesystem::exists(file_path));

  std::ifstream f(file_path);
  ASSERT_TRUE(f.is_open());
  nlohmann::json root = nlohmann::json::parse(f);

  ASSERT_TRUE(root.contains("metadata"));
  EXPECT_EQ(root["metadata"]["num_nodes"], 2);
  EXPECT_EQ(root["metadata"]["num_sequential_edges"], 1);
  EXPECT_EQ(root["metadata"]["num_loop_edges"], 1);
  EXPECT_EQ(root["metadata"]["bag_path"], "dummy_bag");

  ASSERT_TRUE(root.contains("nodes"));
  EXPECT_EQ(root["nodes"].size(), 2);
  EXPECT_EQ(root["nodes"][0]["index"], 0);
  EXPECT_NEAR(root["nodes"][0]["x"].get<double>(), 1.0, 1e-6);
  EXPECT_NEAR(root["nodes"][0]["y"].get<double>(), 2.0, 1e-6);
  EXPECT_NEAR(root["nodes"][0]["yaw"].get<double>(), 0.5, 1e-6);

  ASSERT_TRUE(root.contains("sequential_edges"));
  EXPECT_EQ(root["sequential_edges"].size(), 1);
  EXPECT_EQ(root["sequential_edges"][0]["from"], 0);
  EXPECT_EQ(root["sequential_edges"][0]["to"], 1);

  ASSERT_TRUE(root.contains("loop_edges"));
  EXPECT_EQ(root["loop_edges"].size(), 1);
  EXPECT_EQ(root["loop_edges"][0]["from"], 0);
  EXPECT_EQ(root["loop_edges"][0]["to"], 5);
}

}  // namespace slam_gnss_2d
