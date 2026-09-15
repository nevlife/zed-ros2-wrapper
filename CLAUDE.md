# CLAUDE.md

Fork of stereolabs/zed-ros2-wrapper (upstream base v5.4.1) with a flux publisher added to the ZED One component. The rules in the parent `flux_devel/CLAUDE.md` apply here as well.

## Scope of changes

- Keep the diff against upstream minimal. Do not add exception handling for failures that have not occurred, and do not add features that were not requested. If something must be added, say why.
- Follow upstream conventions: `sl_tools::getParam` for parameters, existing log macros (`RCLCPP_INFO_STREAM`, `DEBUG_STREAM_VD`), upstream naming, indentation and include order.
- Comments only for constraints the code cannot show.
- No no emoji in code, yaml, launch files or commit messages.
- Build with `colcon build --symlink-install`.

## flux publishing

- `flux.enable true` publishes the color image on a flux channel and stops every DDS image topic. `camera_info` stays on DDS. With `flux.enable false` nothing is published on flux.
- The channel name follows the content: `rgb/color/rect/image` when rectified, `rgb/color/raw/image` when `flux.raw_nv12` is on.
- `flux.raw_nv12` needs `general.pub_resolution: NATIVE`.
- ZED X One GS cameras must be opened staggered (`multi_camera.yaml` `stagger`), otherwise `CAMERA STREAM FAILED TO START`.