# CLAUDE.md

Fork of stereolabs/zed-ros2-wrapper (upstream base v5.4.1) with a flux publisher added to the ZED One component. The rules in the parent `flux_devel/CLAUDE.md` apply here as well.

## Scope of changes

- Keep the diff against upstream minimal. Do not add exception handling for failures that have not occurred, and do not add features that were not requested. If something must be added, say why.
- Follow upstream conventions: `sl_tools::getParam` for parameters, existing log macros (`RCLCPP_INFO_STREAM`, `DEBUG_STREAM_VD`), upstream naming, indentation and include order.
- Comments only for constraints the code cannot show.
- No emoji in code, yaml, launch files or commit messages.
- Build with `colcon build --symlink-install`.

## flux publishing

- Both components publish on flux. ZED One (`zed_camera_one`, `common_mono.yaml`): the color image on one channel. Stereo (`zed_camera`, `common_stereo.yaml`): left, right and depth on three channels.
- `flux.enable true` publishes on flux and stops the DDS topics that carry the same images. `camera_info` stays on DDS. With `flux.enable false` nothing is published on flux.
- Channel names are the DDS topic names: `rgb/color/rect/image` (ZED One), `left/color/rect/image`, `right/color/rect/image`, `depth/depth_registered` (stereo). `flux.rectified false` switches the color channels to `raw/image`; `flux.raw_nv12` (ZED One) publishes the NV12 capture buffer on `rgb/color/raw/image`.
- `flux.raw_nv12` needs `general.pub_resolution: NATIVE`.
- The stereo flux publish runs in the grab thread inside `retrieveVideoDepth()`, the same place upstream retrieves the DDS images. Depth is `32FC1` in meters; `depth.openni_depth_mode` does not apply to flux.
- ZED X One GS cameras must be opened staggered (`multi_camera.yaml` `stagger`), otherwise `CAMERA STREAM FAILED TO START`.
- The stereo component starts positional tracking in the grab loop and blocks until the static TF from `robot_state_publisher` arrives, so a stereo launch must keep `publish_urdf` true.
- Two ZED 2i on one USB hub do not sustain HD720@60 (the SDK reboots the camera); `zed2i_multi_override.yaml` grabs at 30 fps.
