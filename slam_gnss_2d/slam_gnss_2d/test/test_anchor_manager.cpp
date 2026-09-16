#include <gtest/gtest.h>
#include "slam_gnss_2d/gnss/anchor_manager.hpp"

namespace slam_gnss_2d {
namespace gnss {

TEST(AnchorManagerTest, TrySetAnchorAndToLocal) {
  GnssAnchorManager mgr;
  EXPECT_FALSE(mgr.is_initialized());

  GnssData gnss_bad_fix;
  gnss_bad_fix.fix_status = 1;
  gnss_bad_fix.x = 1000.0;
  gnss_bad_fix.y = 2000.0;
  gnss_bad_fix.latitude = 35.0;
  gnss_bad_fix.longitude = 139.0;

  // Reject when fix_status < min_fix_status (e.g. min 2)
  EXPECT_FALSE(mgr.try_set_anchor(gnss_bad_fix, 2));
  EXPECT_FALSE(mgr.is_initialized());

  // Accept when fix_status >= min_fix_status
  GnssData gnss_good;
  gnss_good.fix_status = 2;
  gnss_good.x = 1000.0;
  gnss_good.y = 2000.0;
  gnss_good.latitude = 35.0;
  gnss_good.longitude = 139.0;

  EXPECT_TRUE(mgr.try_set_anchor(gnss_good, 2));
  EXPECT_TRUE(mgr.is_initialized());

  // Cannot set again
  EXPECT_FALSE(mgr.try_set_anchor(gnss_good, 2));

  // to_local
  GnssData query;
  query.x = 1010.5;
  query.y = 1995.0;
  auto [local_x, local_y] = mgr.to_local(query);
  EXPECT_NEAR(local_x, 10.5, 1e-6);
  EXPECT_NEAR(local_y, -5.0, 1e-6);

  // anchor_utm & anchor_latlon
  auto utm = mgr.anchor_utm();
  ASSERT_TRUE(utm.has_value());
  EXPECT_NEAR(utm->first, 1000.0, 1e-6);
  EXPECT_NEAR(utm->second, 2000.0, 1e-6);

  auto latlon = mgr.anchor_latlon();
  ASSERT_TRUE(latlon.has_value());
  EXPECT_NEAR(latlon->first, 35.0, 1e-6);
  EXPECT_NEAR(latlon->second, 139.0, 1e-6);
}

TEST(AnchorManagerTest, ManualSetAnchor) {
  GnssAnchorManager mgr;
  mgr.set_anchor(500.0, 600.0, 36.0, 140.0);
  EXPECT_TRUE(mgr.is_initialized());

  GnssData query;
  query.x = 520.0;
  query.y = 630.0;
  auto [local_x, local_y] = mgr.to_local(query);
  EXPECT_NEAR(local_x, 20.0, 1e-6);
  EXPECT_NEAR(local_y, 30.0, 1e-6);
}

}  // namespace gnss
}  // namespace slam_gnss_2d
