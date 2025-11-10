# Camera Calibration & Visualization Guide

## Overview
This document summarizes the camera calibration setup for the RubikPi robot. The camera is calibrated and accessible via Foxglove Studio for remote visualization.

## Quick Start - Visualizing Camera

### 1. Launch Camera with Foxglove Bridge
```bash
cd /home/ubuntu/ros2_ws/rubikpi_ros2
ros2 launch robot_vision_camera robot_vision_camera.launch.py
```

This launches:
- Camera node publishing to `/camera/image_raw` and `/camera/camera_info`
- Foxglove Bridge on port 8765 (WebSocket)

### 2. Connect from Your Computer
- **URL**: `ws://192.168.90.151:8765`
- Open Foxglove Studio (https://studio.foxglove.dev or desktop app)
- **Connection Type**: Foxglove WebSocket (NOT Raw WebSocket)
- Add Image panel to view `/camera/image_raw`

---

## Camera Calibration Details

### Calibration Parameters
- **Chessboard Size**: 9x6 (interior corners)
- **Square Size**: 0.0835 meters
- **Camera Resolution**: 1280x720

### Calibration Values (from calib/ost.yaml)
```
Camera Matrix:
  fx = 641.378180
  fy = 641.210040
  cx = 627.537440
  cy = 300.791790

Distortion Coefficients (plumb_bob):
  k1 = -0.275196
  k2 = 0.052771
  p1 = 0.001451
  p2 = 0.002248
  k3 = 0.000000
```

### Key Files
- **Source Config**: `robot_vision/config/camera_parameter.yaml`
- **Installed Config**: `/home/ubuntu/ros2_ws/install/robot_vision_camera/share/robot_vision_camera/config/camera_parameter.yaml`
- **Raw Calibration**: `calib/ost.yaml`

---

## Important Notes

### Camera Info Topic
The `/camera/camera_info` topic publishes calibration matrices that are used by:
- AprilTag detection (`apriltag_ros` package)
- Any vision-based navigation
- Pose estimation algorithms

The camera publishes calibration data loaded from the YAML file automatically.

### Recalibrating the Camera

If you need to recalibrate:
1. Print a 9x6 chessboard with square size 0.0835 meters
2. SSH with X11 forwarding: `ssh -X ubuntu@192.168.90.151`
3. Run calibration tool:
   ```bash
   ros2 run camera_calibration cameracalibrator \
     --size 9x6 \
     --square 0.0835 \
     --ros-args --remap image:=/camera/image_raw \
     --ros-args --remap camera:=camera
   ```
4. Collect images by moving chessboard around
5. Click "Calibrate" and save
6. Extract the tar.gz file
7. Update `robot_vision/config/camera_parameter.yaml`
8. Rebuild: `cd /home/ubuntu/ros2_ws && colcon build --packages-select robot_vision_camera`
9. Restart camera

---

## Robot Movement & Vision Feedback

### Next Steps
1. Use camera calibration for AprilTag pose detection
2. Implement vision-based navigation
3. Feed camera data into control loops

### Available Topics
- `/camera/image_raw` - Raw camera feed
- `/camera/image_raw/compressed` - Compressed feed (lower bandwidth)
- `/camera/camera_info` - Calibration matrices

### AprilTag Setup
The `apriltag_ros` package is already configured and ready to use with this calibrated camera. It publishes:
- `/tf` - Tag poses
- `/apriltag_detections` - Detection metadata

---

## Troubleshooting

### Foxglove Connection Issues
- Ensure robot IP: `192.168.90.151` is correct
- Use connection type: **Foxglove WebSocket** (not Raw WebSocket)
- Port 8765 should be open

### Camera Not Loading Calibration
- Check logs for: "Camera info loaded successfully"
- Verify file path in logs matches installation
- After updating calibration, must rebuild AND restart

### No Display on Robot
- rviz2 won't work (robot has no display)
- Use Foxglove Studio on your computer instead
- For calibration GUI, use SSH with X11 forwarding

---

## Key Commands Cheat Sheet

```bash
# Launch camera
ros2 launch robot_vision_camera robot_vision_camera.launch.py

# Check camera topics
ros2 topic list | grep camera

# View camera info
ros2 topic echo /camera/camera_info

# Launch AprilTag detection (uses camera calibration)
ros2 launch apriltag_ros apriltag_launch.py

# View in Foxglove
# Connect to: ws://192.168.90.151:8765
```

---

## Summary
- Camera is calibrated with 9x6 chessboard
- Visualization works via Foxglove Studio (ws://192.168.90.151:8765)
- Calibration matrices are in `/camera/camera_info`
- Ready to use for vision-based robot control

