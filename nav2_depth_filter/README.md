# nav2_depth_filter

A composable ROS 2 node that cleans a **metric depth image** before it is
projected to a point cloud for a Nav2 costmap. It is a depth-image
post-processing stage for the `nav2_depth_estimation_ai` pipeline: learned depth
(e.g. DepthAnything V3) is dense but noisy, and marking that noise as obstacles
hurts navigation reliability.

Each stage is individually disableable, and `enabled: false` passes the image
through untouched — so there is one pipeline, with filtering that can be turned
off when undesirable, rather than a filtered/unfiltered fork.

## Pipeline position

```
depth_anything_v3  ──/depth_image──►  depth_filter  ──/depth_image_filtered──►  depth_image_proc  ──►  Nav2 costmap
```

Wire it in by enabling the `depth_image_proc` point-cloud node (commented in the
pipeline launch) and pointing it at `/depth_image_filtered`. See
`launch/depth_filter.launch.py`, which loads the node into the pipeline
container.

## Filter stages (in order)

1. **Range clamp** — depths outside `[min_depth, max_depth]`, and non-finite
   values, become invalid (0).
2. **Flying-pixel removal** *(opt-in, default off)* — invalidates pixels on steep
   occlusion gradients. Very effective at removing edge "flying pixels" but it
   also erodes real object boundaries, so it is off by default and left to the
   integrator.
3. **Median blur** — removes salt-and-pepper speckle; invalid pixels stay invalid.
4. **Speckle removal** — groups valid pixels into depth-connected components
   (4-connectivity, neighbours within `speckle_max_diff` m) and drops components
   smaller than `min_region` px, removing isolated outliers of any size.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | `false` republishes the input untouched |
| `min_depth` | double | `0.2` | metres; below → invalid |
| `max_depth` | double | `20.0` | metres; above → invalid |
| `median_ksize` | int | `5` | `3` or `5` to enable; `<3` disables |
| `speckle_max_diff` | double | `0.5` | metres; max depth step within one component |
| `min_region` | int | `40` | px; smaller depth-connected components removed; `0` disables |
| `flying_thresh` | double | `0.0` | metres/px gradient; `>0` enables flying-pixel removal |

## Design notes (zero-copy)

Input arrives as a `sensor_msgs::msg::Image::ConstSharedPtr` (no copy on
receive); the output is filtered directly into the output message's buffer and
published with `std::move` on a `unique_ptr`, so intra-process delivery stays
zero-copy on the wire. The filter core (`depth_filter.hpp`) is pure OpenCV with
no ROS dependency, so it is unit-tested on its own.

## Validation status (honest)

- The filter core is unit-tested (`test/`) and was cross-checked against a
  synthetic noisy metric-depth scene: with defaults it removes **~74%** of false
  depth readings while retaining **~92%** of a real obstacle; with flying-pixel
  removal enabled, ~100% of false readings at ~77% obstacle retention (it trades
  edge completeness — hence off by default).
- **These numbers are on synthetic noise**, demonstrating the algorithm is
  sound. The meaningful evidence is a before/after on real DepthAnything V3
  `/depth_image` output, which requires the live pipeline (a GPU + the TensorRT
  model) and is the integrator's next step.
- Encodings: `32FC1` (metres) is filtered; other encodings pass through with a
  warning. `16UC1` (mm) support is a straightforward follow-up.

## Build & test

```bash
colcon build --packages-select nav2_depth_filter
colcon test --packages-select nav2_depth_filter
```

## License

Apache-2.0. Copyright (c) 2026 Ocean Code AI Ltd.
