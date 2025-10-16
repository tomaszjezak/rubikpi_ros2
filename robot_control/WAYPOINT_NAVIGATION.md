# Waypoint Navigation System

This package provides autonomous waypoint navigation capabilities for the robot, building on top of the existing motor control system.

## Features

- **Single Waypoint Navigation**: Navigate to individual waypoints with automatic path planning
- **Multi-Waypoint Sequences**: Navigate through multiple waypoints in sequence
- **Trajectory Interpolation**: Smooth trajectory generation with configurable step sizes
- **CLI Interface**: Easy command-line interface for waypoint commands
- **Real-time Monitoring**: Live status updates and pose tracking

## Motion Parameters

The system uses calibrated motion primitives:

- **Forward Motion**: 3.4 cm per command at 0.3 motor speed
- **Rotation**: 21° per command at 0.5 motor speed
- **Interpolation**: 5 cm forward steps, 10° rotation steps (advanced mode)

## Usage

### Basic Waypoint Navigation

```bash
# Navigate to a single waypoint (x, y, theta in degrees)
ros2 run robot_control waypoint_navigation --x 1.0 --y 0.0 --theta 0

# Navigate to (1m forward, 0m sideways, 0° orientation)
ros2 run robot_control waypoint_navigation --x 1.0 --y 0.0 --theta 0

# Navigate to (0m forward, 1m sideways, 90° orientation)  
ros2 run robot_control waypoint_navigation --x 0.0 --y 1.0 --theta 90

# Navigate to (1m forward, 1m sideways, 45° orientation)
ros2 run robot_control waypoint_navigation --x 1.0 --y 1.0 --theta 45
```

### Advanced Waypoint Navigation with Interpolation

```bash
# Single waypoint with trajectory interpolation
ros2 run robot_control advanced_waypoint_navigation --x 1.0 --y 1.0 --theta 90

# Multi-waypoint sequence
ros2 run robot_control advanced_waypoint_navigation --waypoints "0.5,0,0;1.0,0,0;1.0,0.5,90"
```

### Utility Commands

```bash
# Check current robot pose
ros2 run robot_control waypoint_navigation --status

# Reset robot pose to origin
ros2 run robot_control waypoint_navigation --reset

# Stop current navigation
ros2 run robot_control waypoint_navigation --stop
```

### Launch Files

```bash
# Launch basic waypoint navigation system
ros2 launch robot_control waypoint_navigation_launch.py

# Launch advanced waypoint navigation system
ros2 launch robot_control advanced_waypoint_navigation_launch.py
```

## Architecture

### Nodes

1. **`waypoint_navigation`**: Basic waypoint navigation with simple path planning
2. **`advanced_waypoint_navigation`**: Enhanced navigation with trajectory interpolation
3. **`motor_control`**: Existing motor controller (required for robot communication)

### Communication Flow

```
Waypoint Command → Path Planning → Trajectory Generation → Motor Commands → Robot
```

### Path Planning Algorithm

1. **Calculate relative movement**: `dx = target_x - current_x`, `dy = target_y - current_y`
2. **Determine required heading**: `heading = atan2(dy, dx)`
3. **Plan rotation to face target**: `rotate(heading - current_theta)`
4. **Plan forward motion**: `forward(distance)`
5. **Plan final rotation**: `rotate(target_theta - heading)`

### Trajectory Interpolation (Advanced Mode)

- Breaks large movements into smaller steps
- Configurable step sizes (default: 5cm forward, 10° rotation)
- Smoother motion with better precision
- Supports multi-waypoint sequences

## Examples

### Example 1: Simple Forward Motion
```bash
# Move 1 meter forward
ros2 run robot_control waypoint_navigation --x 1.0 --y 0.0 --theta 0
```

### Example 2: Square Pattern
```bash
# Navigate in a square pattern
ros2 run robot_control advanced_waypoint_navigation --waypoints "1,0,0;1,1,90;0,1,180;0,0,270"
```

### Example 3: Complex Path
```bash
# Navigate through multiple waypoints
ros2 run robot_control advanced_waypoint_navigation --waypoints "0.5,0,0;1.0,0.5,45;1.5,1.0,90"
```

## Integration with Existing System

The waypoint navigation system integrates seamlessly with the existing robot control infrastructure:

- **Motor Control**: Uses the same `motor_commands` topic as keyboard control
- **Serial Communication**: Leverages existing serial communication to `/dev/ttyUSB0`
- **Safety Features**: Inherits timeout and safety mechanisms from motor controller
- **ROS2 Architecture**: Follows ROS2 node patterns and conventions

## Calibration

The motion parameters can be adjusted in the source code:

```python
# In waypoint_navigation.py or advanced_waypoint_navigation.py
self.FORWARD_DISTANCE_PER_COMMAND = 0.034  # 3.4 cm per forward command
self.ROTATION_ANGLE_PER_COMMAND = math.radians(21)  # 21 degrees per rotation
self.FORWARD_SPEED = 0.3  # Motor speed for forward motion
self.ROTATION_SPEED = 0.5  # Motor speed for rotation
```

## Troubleshooting

### Common Issues

1. **Robot not responding**: Check serial connection to `/dev/ttyUSB0`
2. **Inaccurate positioning**: Recalibrate motion parameters
3. **Navigation stops**: Check for obstacles or communication errors
4. **Wrong orientation**: Verify theta values are in degrees for CLI, radians internally

### Debug Commands

```bash
# Check robot status
ros2 run robot_control waypoint_navigation --status

# Reset pose if lost
ros2 run robot_control waypoint_navigation --reset

# Stop navigation if stuck
ros2 run robot_control waypoint_navigation --stop
```

## Future Enhancements

- **Obstacle Avoidance**: Integrate with vision system for obstacle detection
- **SLAM Integration**: Use AprilTag detection for localization
- **Dynamic Replanning**: Adjust paths based on real-time sensor data
- **Speed Control**: Variable speed based on path curvature
- **Path Optimization**: Minimize total distance or time
