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

#ifndef NAV2_DEPTH_FILTER__DEPTH_FILTER_NODE_HPP_
#define NAV2_DEPTH_FILTER__DEPTH_FILTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "nav2_depth_filter/depth_filter.hpp"

namespace nav2_depth_filter
{

/**
 * @class DepthFilterNode
 * @brief Composable node that cleans a metric depth image before it is consumed
 *        downstream (e.g. projected to a point cloud for a Nav2 costmap).
 *
 * Reliability post-processing for learned depth: range clamping, optional
 * flying-pixel removal, median de-speckling, and small-region outlier removal.
 * Each stage is individually disableable, and `enabled: false` passes the image
 * through untouched. Filtering happens in-place in the output message buffer;
 * the input arrives as a shared pointer and the output is published by moving a
 * unique pointer, so intra-process delivery stays zero-copy on the wire.
 */
class DepthFilterNode : public rclcpp::Node
{
public:
  explicit DepthFilterNode(const rclcpp::NodeOptions & options);

private:
  void onDepth(sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  bool enabled_{true};
  DepthFilterParams params_;
};

}  // namespace nav2_depth_filter

#endif  // NAV2_DEPTH_FILTER__DEPTH_FILTER_NODE_HPP_
