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
//
// Pure-OpenCV depth-image reliability filter. No ROS dependency, so it can be
// unit-tested on its own. The ROS 2 composable node wraps filterDepth().

#ifndef NAV2_DEPTH_FILTER__DEPTH_FILTER_HPP_
#define NAV2_DEPTH_FILTER__DEPTH_FILTER_HPP_

#include <cmath>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace nav2_depth_filter
{

struct DepthFilterParams
{
  float min_depth = 0.2f;      // metres; below -> invalid (0)
  float max_depth = 20.0f;     // metres; above -> invalid (0)
  float flying_thresh = 0.0f;   // metres/px gradient; >0 enables flying-pixel removal
  int median_ksize = 5;         // 3 or 5 enables median blur; <3 disables
  float speckle_max_diff = 0.5f;  // metres; max depth step between same-component neighbours
  int min_region = 40;          // pixels; smaller depth-connected components removed; 0 disables
};

/// Filter a CV_32FC1 metric-depth image (metres; 0 = invalid) into dst.
/// Stages, each individually disableable via params, applied in order:
///   1. range clamp         -> out-of-range / non-finite set to 0
///   2. flying-pixel removal -> steep-gradient (occlusion) pixels set to 0 (opt-in)
///   3. median blur          -> salt-and-pepper speckle, invalids preserved
///   4. speckle removal      -> small depth-connected components set to 0
inline void filterDepth(const cv::Mat & src, cv::Mat & dst, const DepthFilterParams & p)
{
  CV_Assert(src.type() == CV_32FC1);
  cv::Mat out = src.clone();

  // Stage 1: range clamp -> invalid.
  for (int y = 0; y < out.rows; ++y) {
    float * row = out.ptr<float>(y);
    for (int x = 0; x < out.cols; ++x) {
      const float d = row[x];
      if (!std::isfinite(d) || d < p.min_depth || d > p.max_depth) {
        row[x] = 0.0f;
      }
    }
  }

  // Stage 2: flying-pixel removal (opt-in; erodes true edges, so default off).
  if (p.flying_thresh > 0.0f) {
    cv::Mat gx, gy, grad;
    cv::Sobel(out, gx, CV_32F, 1, 0, 3);
    cv::Sobel(out, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, grad);
    for (int y = 0; y < out.rows; ++y) {
      float * row = out.ptr<float>(y);
      const float * g = grad.ptr<float>(y);
      for (int x = 0; x < out.cols; ++x) {
        if (row[x] > 0.0f && g[x] > p.flying_thresh) {
          row[x] = 0.0f;
        }
      }
    }
  }

  // Stage 3: median blur, keeping invalid pixels invalid.
  if (p.median_ksize >= 3) {
    cv::Mat med;
    cv::medianBlur(out, med, p.median_ksize);
    for (int y = 0; y < out.rows; ++y) {
      float * row = out.ptr<float>(y);
      const float * m = med.ptr<float>(y);
      for (int x = 0; x < out.cols; ++x) {
        row[x] = (row[x] > 0.0f) ? m[x] : 0.0f;
      }
    }
  }

  // Stage 4: speckle removal. Group valid pixels into depth-connected components
  // (4-connectivity, neighbours within speckle_max_diff metres) via union-find,
  // then drop any component smaller than min_region pixels. This removes isolated
  // outliers of any size, including single- and few-pixel speckle.
  if (p.min_region > 0 && !out.empty()) {
    CV_Assert(out.isContinuous());
    const int rows = out.rows, cols = out.cols;
    const int npix = rows * cols;
    float * data = out.ptr<float>(0);

    std::vector<int> parent(npix);
    for (int i = 0; i < npix; ++i) {
      parent[i] = i;
    }
    auto find = [&parent](int a) {
        while (parent[a] != a) {parent[a] = parent[parent[a]]; a = parent[a];}
        return a;
      };
    auto unite = [&](int a, int b) {
        a = find(a); b = find(b);
        if (a != b) {parent[a] = b;}
      };

    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        const int i = y * cols + x;
        const float d = data[i];
        if (d <= 0.0f) {continue;}
        if (x > 0) {
          const float dl = data[i - 1];
          if (dl > 0.0f && std::fabs(d - dl) < p.speckle_max_diff) {unite(i, i - 1);}
        }
        if (y > 0) {
          const float du = data[i - cols];
          if (du > 0.0f && std::fabs(d - du) < p.speckle_max_diff) {unite(i, i - cols);}
        }
      }
    }

    std::vector<int> comp_size(npix, 0);
    for (int i = 0; i < npix; ++i) {
      if (data[i] > 0.0f) {comp_size[find(i)]++;}
    }
    for (int i = 0; i < npix; ++i) {
      if (data[i] > 0.0f && comp_size[find(i)] < p.min_region) {data[i] = 0.0f;}
    }
  }

  // copyTo (not assignment) so callers can filter directly into a pre-allocated
  // buffer — e.g. the output Image message's own data — avoiding a reallocation.
  out.copyTo(dst);
}

}  // namespace nav2_depth_filter

#endif  // NAV2_DEPTH_FILTER__DEPTH_FILTER_HPP_
