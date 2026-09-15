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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>
#include <map>
#include <random>

#include "cartographer/common/math.h"

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::IsEmpty;

TEST(VoxelFilterTest, AllocationChangesPreserveReservoirSamplingAndOrder) {
  std::vector<RangefinderPoint> points;
  std::vector<float> intensities;
  std::map<uint64_t, std::pair<int, int>> reference;
  std::minstd_rand0 generator;
  for (int i = 0; i < 20000; ++i) {
    const Eigen::Vector3f point((i % 23) * .03f - .5f,
                                (i % 37) * .02f, (i % 7) * .01f);
    points.push_back({point});
    intensities.push_back(i);
    const Eigen::Array3f index = point.array() / .1f;
    const uint64_t x = common::RoundToInt(index.x());
    const uint64_t y = common::RoundToInt(index.y());
    const uint64_t z = common::RoundToInt(index.z());
    auto& voxel = reference[(x << 42) + (y << 21) + z];
    if (++voxel.first == 1) {
      voxel.second = i;
    } else if (std::uniform_int_distribution<>(1, voxel.first)(generator) == voxel.first) {
      voxel.second = i;
    }
  }
  std::vector<bool> selected(points.size(), false);
  for (const auto& entry : reference) selected[entry.second.second] = true;
  std::vector<float> expected;
  for (size_t i = 0; i < points.size(); ++i)
    if (selected[i]) expected.push_back(i);
  const auto actual = VoxelFilter(PointCloud(points, intensities), .1f);
  EXPECT_EQ(actual.intensities(), expected);
  std::vector<TimedPointCloudOriginData::RangeMeasurement> measurements;
  for (size_t i = 0; i < points.size(); ++i) {
    measurements.push_back({{points[i].position, float(i) * .001f}, float(i), i % 4});
  }
  const auto serial = VoxelFilter(measurements, .1f, 1);
  for (int threads : {2, 4, 8, 16}) {
    const auto parallel = VoxelFilter(measurements, .1f, threads);
    ASSERT_EQ(serial.size(), parallel.size());
    for (size_t i = 0; i < serial.size(); ++i) {
      EXPECT_EQ(serial[i].point_time.position, parallel[i].point_time.position);
      EXPECT_EQ(serial[i].point_time.time, parallel[i].point_time.time);
      EXPECT_EQ(serial[i].intensity, parallel[i].intensity);
      EXPECT_EQ(serial[i].origin_index, parallel[i].origin_index);
    }
  }
}

TEST(VoxelFilterTest, ReturnsOnePointInEachVoxel) {
  const PointCloud point_cloud({{{0.f, 0.f, 0.f}},
                                {{0.1f, -0.1f, 0.1f}},
                                {{0.3f, -0.1f, 0.f}},
                                {{0.f, 0.f, 0.1f}}});
  const PointCloud result = VoxelFilter(point_cloud, 0.3f);
  ASSERT_EQ(result.size(), 2);
  EXPECT_THAT(result.intensities(), IsEmpty());
  EXPECT_THAT(point_cloud.points(), Contains(result[0]));
  EXPECT_THAT(point_cloud.points(), Contains(result[1]));
  EXPECT_THAT(result.points(), Contains(point_cloud[2]));
}

TEST(VoxelFilterTest, CorrectIntensities) {
  std::vector<RangefinderPoint> points;
  std::vector<float> intensities;
  for (int i = 0; i < 100; ++i) {
    const float value = 0.1f * i;
    // We add points with intensity equal to the z coordinate, so we can later
    // verify that the resulting intensities are corresponding to the filtered
    // points.
    points.push_back({{-100.f, 0.3f, value}});
    intensities.push_back(value);
  }
  const PointCloud point_cloud(points, intensities);
  const PointCloud result = VoxelFilter(point_cloud, 0.3f);

  ASSERT_EQ(result.intensities().size(), result.points().size());
  for (size_t i = 0; i < result.size(); ++i) {
    ASSERT_NEAR(result[i].position.z(), result.intensities()[i], 1e-6);
  }
}

TEST(VoxelFilterTest, HandlesLargeCoordinates) {
  const PointCloud point_cloud({{{100000.f, 0.f, 0.f}},
                                {{100000.001f, -0.0001f, 0.0001f}},
                                {{100000.003f, -0.0001f, 0.f}},
                                {{-200000.f, 0.f, 0.f}}});
  const PointCloud result = VoxelFilter(point_cloud, 0.01f);
  EXPECT_EQ(result.size(), 2);
  EXPECT_THAT(result.intensities(), IsEmpty());
  EXPECT_THAT(result.points(), Contains(point_cloud[3]));
}

TEST(VoxelFilterTest, IgnoresTime) {
  TimedPointCloud timed_point_cloud;
  for (int i = 0; i < 100; ++i) {
    timed_point_cloud.push_back({{-100.f, 0.3f, 0.4f}, 1.f * i});
  }
  const TimedPointCloud result = VoxelFilter(timed_point_cloud, 0.3f);
  ASSERT_EQ(result.size(), 1);
  EXPECT_THAT(timed_point_cloud, Contains(result[0]));
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
