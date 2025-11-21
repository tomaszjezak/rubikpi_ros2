# Add Safe and Distance Path Planning Modes

## Overview
Implement two path planning modes:
1. **Safe Mode**: Maximizes distance from obstacles/walls (uses larger effective robot radius)
2. **Distance Mode**: Minimizes path length (uses actual robot radius, can cut close to obstacles)

## Implementation Strategy
Use different effective robot radii for collision checking:
- **Safe mode**: `effective_radius = robot_radius + safety_margin` (keeps path further from obstacles)
- **Distance mode**: `effective_radius = robot_radius` (allows path close to obstacles)

This approach reuses all existing `prm_core.py` functions - just pass different radius values.

## Effective Radius Explanation

**Effective radius** is the clearance distance used for collision checking. It's passed to `sample_points()` and `is_collision()` functions.

### How it affects sample points:
In `sample_points()` (line 116):
```python
dist = distance_to_box(tx, ty, obstacle_bounds)
if dist >= rr:  # Only accept points that are at least 'rr' distance from obstacles
    sample_x.append(tx)
    sample_y.append(ty)
```
- **Larger `rr`** → Only points further from obstacles are accepted
- **Smaller `rr`** → Points closer to obstacles are accepted

### How it affects path planning:
In `is_collision()` (lines 159-166):
```python
D = rr  # Step size along edge
for i in range(n_step):
    dist = distance_to_box(x, y, obstacle_bounds)
    if dist <= rr:  # Reject edge if any point is too close
        return True  # collision!
```
- **Larger `rr`** → Edges must stay further from obstacles → safer but potentially longer paths
- **Smaller `rr`** → Edges can get closer → shorter paths but closer to obstacles

## Step 1: Update Service Definition
**File:** `srv/GetPRMPath.srv`

Add planning mode parameter to request:
```
# Request: start and goal positions
float64 start_x
float64 start_y
float64 goal_x
float64 goal_y
string planning_mode  # "safe" or "distance"
---
# Response: path waypoints
float64[] path_x
float64[] path_y
bool success
string message
```

## Step 2: Update Robot Radius and Add Safety Margin Parameter
**File:** `prm_planner/prm_planner_node.py`

1. **Update robot_radius parameter:**
   - Change from `0.1` to `0.11` meters (11 cm)

2. **Add ROS2 parameter for safety margin:**
   - `safety_margin` (default: 0.0011 meters = 0.11 cm) - additional clearance for safe mode

## Step 3: Update Service Callback
**File:** `prm_planner/prm_planner_node.py`

In `plan_path_callback()`:
1. Extract `planning_mode` from request (default to "distance" if not provided)
2. Calculate effective radius based on mode:
   - If "safe": `effective_radius = self.robot_radius + self.safety_margin`
   - If "distance": `effective_radius = self.robot_radius`
3. Pass `effective_radius` to `_plan_path()` instead of `self.robot_radius`
4. Validate mode (log warning if invalid, default to "distance")
5. Log which mode is being used and the effective radius for debugging

## Step 4: Update _plan_path Method
**File:** `prm_planner/prm_planner_node.py`

Change signature to accept `effective_radius`:
```python
def _plan_path(self, sx, sy, gx, gy, effective_radius):
```

Use `effective_radius` instead of `self.robot_radius` when calling:
- `sample_points(..., effective_radius, ...)`
- `generate_road_map(..., effective_radius, ...)`

## Files to Modify
1. `srv/GetPRMPath.srv` - Add `planning_mode` string field
2. `prm_planner/prm_planner_node.py` - Update robot_radius, add safety_margin parameter, update callback and _plan_path
3. Rebuild package after service change

## Expected Behavior

### Distance Mode:
- **Effective radius**: 0.11 m (robot_radius only)
- **Path behavior**: Minimizes length, can be close to obstacles
- **Use case**: Fastest/shortest path when safety is less critical

### Safe Mode:
- **Effective radius**: 0.1111 m (0.11 m + 0.0011 m = 0.1111 m)
- **Path behavior**: Stays further from obstacles, potentially longer path
- **Use case**: Maximum safety, takes "midpoint" paths between obstacles when possible

Both modes use same prm_core functions, just with different effective radii.

## Implementation Details

### Parameter Values:
- `robot_radius`: 0.11 m (11 cm)
- `safety_margin`: 0.0011 m (0.11 cm)
- Safe mode effective radius: 0.1111 m
- Distance mode effective radius: 0.11 m

### Service Usage:
```bash
# Distance mode (default)
ros2 service call /get_prm_path prm_planner/srv/GetPRMPath "{start_x: 0.5, start_y: 0.5, goal_x: 2.0, goal_y: 2.0, planning_mode: 'distance'}"

# Safe mode
ros2 service call /get_prm_path prm_planner/srv/GetPRMPath "{start_x: 0.5, start_y: 0.5, goal_x: 2.0, goal_y: 2.0, planning_mode: 'safe'}"
```

## Testing Checklist
- [ ] Update service definition
- [ ] Update robot_radius to 0.11 m
- [ ] Add safety_margin parameter (0.0011 m)
- [ ] Update service callback to handle planning_mode
- [ ] Update _plan_path to accept effective_radius
- [ ] Rebuild package
- [ ] Test distance mode (should find shorter path)
- [ ] Test safe mode (should find safer path, potentially longer)
- [ ] Verify logging shows correct mode and effective radius

