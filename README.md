# ROS 2 YOLO Wrapper – Documentation

## Overview
This package implements a ROS 2–based perception pipeline for 3D object localization using **YOLO** and an **Intel RealSense D455** camera.
RGB images are used for object detection, depth information is used to recover distance, and camera intrinsics are used to compute 3D coordinates of detected objects relative to the camera.

## Hardware
- **Camera**: Intel RealSense D455
  - RGB stream
  - Depth stream
  - Intrinsic parameters provided via camera calibration

## Pipeline Description

1. The Intel RealSense D455 captures **color** and **depth** images.
2. Color images are published at **5 FPS** and used as input for YOLO-based object detection.
3. YOLO performs object detection on each incoming color image and produces bounding boxes.
4. For each detected bounding box, the center pixel is computed.
5. The depth value at the center pixel is extracted from the depth image.
6. Using the camera intrinsic parameters, the pixel location and depth are converted into real-world 3D coordinates.
7. The resulting `(x, y, z)` values represent the position of the detected object **relative to the camera frame**.

## 3D Coordinate Computation

The conversion from image coordinates to 3D camera coordinates follows the pinhole camera model:
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = ZWhere:
- `(u, v)` is the center of the bounding box in pixel coordinates
- `Z` is the depth value at `(u, v)`
- `fx`, `fy`, `cx`, `cy` are the camera intrinsic parameters

All distances are expressed in meters.

## Coordinate Frames

### Camera Frame
- All object positions are computed in the **camera optical frame**.
- The output represents the position of the object relative to the camera, not the robot.

### Camera-to-Robot Relationship
- The transformation between the camera frame and the robot frame is **not estimated automatically**.
- The camera’s position and orientation relative to the robot were obtained by **manual measurement using a measuring tape**.
- This fixed transform can be applied externally to convert object positions from the camera frame to the robot frame.

## Assumptions
- The camera is rigidly mounted on the robot.
- Camera intrinsics are accurate.
- The depth value at the bounding box center is representative of the object distance.

## Limitations
- Manual measurement of camera pose introduces positional error.
- Depth is sampled at a single pixel; no filtering or averaging is applied.
- Object pose is limited to position only (no orientation).

## Future Improvements
- Automated camera-to-robot extrinsic calibration.
- Depth noise reduction using spatial or temporal filtering.
- Averaged or median depth over the bounding box area.
- Full TF integration for direct robot-frame object localization.
