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

#include "nav2_depth_filter/depth_filter_node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "rclcpp_components/register_node_macro.hpp"

namespace nav2_depth_filter
{

DepthFilterNode::DepthFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_filter", options)
{
  enabled_ = this->declare_parameter<bool>("enabled", true);
  params_.min_depth = static_cast<float>(this->declare_parameter<double>("min_depth", 0.2));
  params_.max_depth = static_cast<float>(this->declare_parameter<double>("max_depth", 20.0));
  params_.flying_thresh =
    static_cast<float>(this->declare_parameter<double>("flying_thresh", 0.0));
  params_.median_ksize = this->declare_parameter<int>("median_ksize", 5);
  params_.speckle_max_diff =
    static_cast<float>(this->declare_parameter<double>("speckle_max_diff", 0.5));
  params_.min_region = this->declare_parameter<int>("min_region", 40);

  // cv::medianBlur on CV_32F only supports apertures 3 or 5; coerce anything else.
  if (params_.median_ksize >= 3 && params_.median_ksize != 3 && params_.median_ksize != 5) {
    RCLCPP_WARN(
      get_logger(), "median_ksize=%d unsupported for 32F; using 5", params_.median_ksize);
    params_.median_ksize = 5;
  }

  // SensorDataQoS keeps the depth stream best-effort; intra-process comms (set on
  // the container) make the shared-ptr-in / move-out path zero-copy on the wire.
  pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/depth_image", rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    "~/input/depth_image", rclcpp::SensorDataQoS(),
    std::bind(&DepthFilterNode::onDepth, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "depth_filter up (enabled=%s): range[%.2f,%.2f] median=%d flying=%.2f min_region=%d",
    enabled_ ? "true" : "false", params_.min_depth, params_.max_depth,
    params_.median_ksize, params_.flying_thresh, params_.min_region);
}

void DepthFilterNode::onDepth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Pass-through when disabled or when the encoding isn't one we filter.
  if (!enabled_ || msg->encoding != "32FC1") {
    if (enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "depth_filter passes through unsupported encoding '%s' (expects 32FC1)",
        msg->encoding.c_str());
    }
    pub_->publish(std::make_unique<sensor_msgs::msg::Image>(*msg));
    return;
  }

  const int h = static_cast<int>(msg->height);
  const int w = static_cast<int>(msg->width);
  if (msg->data.size() < static_cast<size_t>(h) * msg->step) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth_filter: short image buffer");
    return;
  }

  // Zero-copy read view over the incoming (shared) buffer.
  const cv::Mat src(
    h, w, CV_32FC1,
    const_cast<void *>(static_cast<const void *>(msg->data.data())),
    msg->step);

  // Build the output message and filter directly into its buffer.
  auto out = std::make_unique<sensor_msgs::msg::Image>();
  out->header = msg->header;
  out->height = msg->height;
  out->width = msg->width;
  out->encoding = msg->encoding;
  out->is_bigendian = msg->is_bigendian;
  out->step = static_cast<uint32_t>(w) * sizeof(float);
  out->data.resize(static_cast<size_t>(out->step) * h);

  cv::Mat dst(h, w, CV_32FC1, out->data.data(), out->step);
  filterDepth(src, dst, params_);

  pub_->publish(std::move(out));
}

}  // namespace nav2_depth_filter

RCLCPP_COMPONENTS_REGISTER_NODE(nav2_depth_filter::DepthFilterNode)
