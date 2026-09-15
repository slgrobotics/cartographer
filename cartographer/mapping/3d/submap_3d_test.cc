/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/3d/submap_3d.h"

#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(SubmapsTest, ParallelInsertionMatchesSerialAcrossSubmapTransitions) {
  proto::SubmapsOptions3D options;
  options.set_high_resolution(.05);
  options.set_low_resolution(.25);
  options.set_high_resolution_max_range(20.);
  options.set_num_range_data(3);
  auto* inserter = options.mutable_range_data_inserter_options();
  inserter->set_hit_probability(.55);
  inserter->set_miss_probability(.49);
  inserter->set_num_free_space_voxels(2);
  inserter->set_intensity_threshold(100.);
  ActiveSubmaps3D serial(options, 1), parallel(options, 2);
  const Eigen::VectorXf histogram = Eigen::VectorXf::Ones(12);
  for (int scan = 0; scan < 16; ++scan) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    for (int i = 0; i < 300; ++i) {
      points.push_back({{float(i % 20) * .1f + scan * .01f,
                         float(i / 20) * .1f, float(i % 7) * .1f}});
      intensities.push_back(float(i % 50));
    }
    const sensor::RangeData data{
        Eigen::Vector3f(.01f * scan, 0.f, 0.f),
        sensor::PointCloud(std::move(points), std::move(intensities)), {}};
    const auto a = serial.InsertData(data, Eigen::Quaterniond::Identity(), histogram);
    const auto b = parallel.InsertData(data, Eigen::Quaterniond::Identity(), histogram);
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) {
      EXPECT_EQ(a[i]->ToProto(true).SerializeAsString(), b[i]->ToProto(true).SerializeAsString());
    }
  }
}

TEST(SubmapsTest, ToFromProto) {
  Eigen::VectorXf histogram(2);
  histogram << 1.0f, 2.0f;
  const Submap3D expected(
      0.05, 0.25,
      transform::Rigid3d(Eigen::Vector3d(1., 2., 0.),
                         Eigen::Quaterniond(0., 0., 0., 1.)),
      histogram);
  const proto::Submap proto =
      expected.ToProto(true /* include_probability_grid_data */);
  EXPECT_FALSE(proto.has_submap_2d());
  EXPECT_TRUE(proto.has_submap_3d());
  const auto actual = Submap3D(proto.submap_3d());
  EXPECT_TRUE(expected.local_pose().translation().isApprox(
      actual.local_pose().translation(), 1e-6));
  EXPECT_TRUE(expected.local_pose().rotation().isApprox(
      actual.local_pose().rotation(), 1e-6));
  EXPECT_EQ(expected.num_range_data(), actual.num_range_data());
  EXPECT_EQ(expected.insertion_finished(), actual.insertion_finished());
  EXPECT_NEAR(expected.high_resolution_hybrid_grid().resolution(), 0.05, 1e-6);
  EXPECT_NEAR(expected.low_resolution_hybrid_grid().resolution(), 0.25, 1e-6);
  EXPECT_TRUE(expected.rotational_scan_matcher_histogram().isApprox(
      actual.rotational_scan_matcher_histogram(), 1e-6));
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
