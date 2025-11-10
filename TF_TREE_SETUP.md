# TF Tree Setup for Mobile Robot with Camera

## Overview
This document explains how to set up the TF tree for your mobile robot with a mounted camera to enable AprilTag detection.

## TF Tree Structure

```
world (fixed reference)
  ↓
odom (robot's odometry reference)
  ↓
base_link (robot's center, at ground level)
  ↓
camera_frame (camera position relative to base_link)
```

## Where Should base_link Be?

**For a car robot:**
- **Location**: Geometric center of the robot at ground level
- **Position**: Center of wheelbase (front-to-back) and center of track (left-to-right)
- **Height**: At ground level (z=0)
- **Orientation**: X forward, Y left, Z up (standard ROS convention)

Example: If your robot is 20cm x 15cm (length x width):
- base_link at (0, 0, 0) - center of the rectangle

## Fill in Camera Transform Values

### Step 1: Measure Your Camera Position

Measure from the robot's center (base_link) to camera:
- **x**: Distance forward (if camera is in front of center, positive)
- **y**: Distance left (positive) or right (negative)
- **z**: Height above ground (positive)
- **roll, pitch, yaw**: Camera orientation angles (radians)

### Step 2: Edit Config File

Edit `robot_control/config/tf_tree.yaml`:

```yaml
camera_transform:
  x: 0.15     # Fill in: Distance forward from center (meters)
  y: 0.0      # Fill in: Distance left/right (meters)
  z: 0.05     # Fill in: Height above ground (meters)
  
  roll:  0.0  # Fill in: Rotation around forward axis (radians)
  pitch: 0.0  # Fill in: Looking down = negative (radians)
  yaw:   0.0  # Fill in: Rotation around vertical axis (radians)
```

### Step 3: Update Launch File

Edit `robot_control/launch/tf_tree_launch.py` with your measured values, or use launch arguments:

```bash
ros2 launch robot_control tf_tree_launch.py \
  camera_x:=0.15 \
  camera_y:=0.0 \
  camera_z:=0.05 \
  camera_roll:=0.0 \
  camera_pitch:=0.0 \
  camera_yaw:=0.0
```

## Running the TF Tree

### Launch TF Tree Only
```bash
ros2 launch robot_control tf_tree_launch.py
```

### View TF Tree
```bash
# View current TF tree
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo base_link camera_frame

# Run custom viewer
ros2 run robot_control view_tf_tree.py
```

## Complete Setup for AprilTag Detection

### 1. Launch TF Tree
```bash
ros2 launch robot_control tf_tree_launch.py
```

### 2. Launch Camera (with proper frame_id)
The camera already publishes to `camera_frame` - verify it matches.

```bash
ros2 launch robot_vision_camera robot_vision_camera.launch.py
```

### 3. Launch AprilTag Detection
```bash
ros2 launch apriltag_ros apriltag_launch.py
```

AprilTag will now:
- Subscribe to `/camera/image_raw` and `/camera/camera_info`
- Publish tag poses to `/tf` with `world` -> `tag36h11:ID`
- Use your calibrated camera for accurate pose estimation

## Testing

### Test TF Tree Structure
```bash
# In one terminal - run everything
ros2 launch robot_control tf_tree_launch.py &
ros2 launch robot_vision_camera robot_vision_camera.launch.py &

# In another terminal - view TF tree
ros2 run robot_control view_tf_tree.py
```

You should see:
```
✅ world
✅ odom
✅ base_link
✅ camera_frame

Camera Transform (base_link -> camera_frame):
  Translation: x=0.150, y=0.000, z=0.050
  ...
```

### Test AprilTag Detection
- Print an AprilTag (e.g., tag36h11:0)
- Place it in front of robot camera
- Check topics:
```bash
ros2 topic echo /tf --filter "frame_id=='tag36h11:0'"
ros2 topic echo /apriltag_detections
```

## Coordinate System Reference

### ROS Convention (for mobile robots):
```
     +y (left)
      ↑
      |
+z ← ← ← +x (forward)
      ↑
  (up)
```

- **X**: forward
- **Y**: left
- **Z**: up
- **Right-handed coordinate system**

### Angles (in radians):
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis (nose up = positive)
- **Yaw**: Rotation around Z-axis (rotate left = positive)

## Common Issues

### Camera frame not found
- Check camera is publishing with `header.frame_id = "camera_frame"`
- Launch TF tree before launching camera
- Verify transform exists: `ros2 run tf2_ros tf2_echo base_link camera_frame`

### AprilTag detection not working
- Ensure camera calibration is loaded (`/camera/camera_info`)
- Check camera can see AprilTag clearly
- Verify lighting is adequate
- Check AprilTag ID matches calibration

## Next Steps: Vision-Based Navigation

Once TF tree is working with AprilTag detection, you can:
1. Use AprilTag poses for robot localization
2. Feed tag poses into navigation stack
3. Implement visual servoing for precise positioning

