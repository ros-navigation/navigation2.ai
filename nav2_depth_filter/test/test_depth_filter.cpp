// Copyright (c) 2026 Ocean Code AI Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// NOTE: Authored with AI assistance, reviewed and validated by the submitter.
// Disclosed per the Nav2 project's AI-generated-software policy.

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "nav2_depth_filter/depth_filter.hpp"

using nav2_depth_filter::DepthFilterParams;
using nav2_depth_filter::filterDepth;

// Out-of-range and non-finite depths become invalid (0).
TEST(DepthFilter, RangeClampInvalidates)
{
  cv::Mat d = (cv::Mat_<float>(1, 5) << 0.05f, 0.5f, 5.0f, 99.0f,
    std::numeric_limits<float>::quiet_NaN());
  DepthFilterParams p;
  p.median_ksize = 0;
  p.min_region = 0;
  cv::Mat out;
  filterDepth(d, out, p);
  EXPECT_FLOAT_EQ(out.at<float>(0, 0), 0.0f);   // below min
  EXPECT_FLOAT_EQ(out.at<float>(0, 1), 0.5f);   // in range
  EXPECT_FLOAT_EQ(out.at<float>(0, 2), 5.0f);   // in range
  EXPECT_FLOAT_EQ(out.at<float>(0, 3), 0.0f);   // above max
  EXPECT_FLOAT_EQ(out.at<float>(0, 4), 0.0f);   // NaN
}

// A small isolated speckle blob on a large flat surface is removed; the surface
// survives.
TEST(DepthFilter, SpeckleRemovedSurfaceKept)
{
  cv::Mat d(60, 60, CV_32FC1, cv::Scalar(3.0f));  // flat 3 m surface
  // a 2x2 speckle at a wildly different depth, isolated by a depth discontinuity
  d(cv::Rect(5, 5, 2, 2)).setTo(0.6f);
  DepthFilterParams p;
  p.median_ksize = 0;      // isolate the speckle-removal stage
  p.flying_thresh = 0.0f;
  p.min_region = 20;
  cv::Mat out;
  filterDepth(d, out, p);
  // speckle gone
  EXPECT_FLOAT_EQ(out.at<float>(5, 5), 0.0f);
  // surface centre intact
  EXPECT_FLOAT_EQ(out.at<float>(30, 30), 3.0f);
}

// enabled=false equivalent: every stage disabled leaves in-range data untouched.
TEST(DepthFilter, AllStagesOffIsPassThroughInRange)
{
  cv::Mat d(10, 10, CV_32FC1, cv::Scalar(2.5f));
  DepthFilterParams p;
  p.median_ksize = 0;
  p.min_region = 0;
  p.flying_thresh = 0.0f;
  cv::Mat out;
  filterDepth(d, out, p);
  EXPECT_EQ(cv::countNonZero(out != 2.5f), 0);
}

// Flying-pixel removal clears a steep occlusion-gradient column.
TEST(DepthFilter, FlyingPixelRemoval)
{
  cv::Mat d(20, 20, CV_32FC1, cv::Scalar(2.0f));
  d(cv::Rect(10, 0, 10, 20)).setTo(6.0f);   // a hard 2m|6m step down the middle
  DepthFilterParams p;
  p.median_ksize = 0;
  p.min_region = 0;
  p.flying_thresh = 1.0f;                    // invalidate the steep boundary
  cv::Mat out;
  filterDepth(d, out, p);
  // some pixels along the discontinuity are now invalid
  EXPECT_GT(cv::countNonZero(out == 0.0f), 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
