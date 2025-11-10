# EKF SLAM - Complete Guide for Homework 3

## Overview

This directory contains comprehensive guides for implementing EKF SLAM for CSE 276 Homework 3. The guides break down the EKF SLAM algorithm from high-level concepts to detailed line-by-line explanations, with special focus on the update step.

## Documentation Index

### 1. Step-by-Step Walkthrough (START HERE!)
**File**: `EKF_SLAM_STEP_BY_STEP_WALKTHROUGH.md`

**Contents**:
- Complete walkthrough of first 5 timesteps
- Detailed calculations with actual numbers
- Shows how state grows from robot-only to robot + landmarks
- Demonstrates data association, innovation, Kalman gain, and updates
- Specific to homework scenario (10ft × 10ft square, 8 landmarks)

**Best for**: Understanding the complete flow with concrete examples

---

### 2. Update Steps Quick Reference
**File**: `EKF_SLAM_UPDATE_STEPS_QUICK_REFERENCE.md`

**Contents**:
- Quick reference for the 8 update steps
- Pseudocode for complete update function
- Matrix dimensions cheat sheet
- Key equations reference
- Common issues and solutions
- Debugging checklist

**Best for**: Quick reference while implementing

---

### 3. Visual Timeline
**File**: `EKF_SLAM_VISUAL_TIMELINE.md`

**Contents**:
- Visual representation of state evolution across timesteps
- Diagrams showing robot and landmark positions
- Uncertainty evolution graphs
- Error reduction over time
- Patterns observed across timesteps
- Homework trajectory phases (square + octagon)

**Best for**: Visual understanding and presentation

---

### 4. Matrix Dimensions Reference
**File**: `EKF_SLAM_MATRIX_DIMENSIONS.md`

**Contents**:
- Complete matrix dimension reference
- Dimension tables for all variables
- Dimension checks for common operations
- Common dimension errors and solutions
- Visual dimension guides

**Best for**: Debugging dimension mismatches

---

## Quick Start Guide

### For Beginners

1. **Start with**: `EKF_SLAM_STEP_BY_STEP_WALKTHROUGH.md`
   - Read through timesteps 1-5 carefully
   - Understand how state grows
   - See how updates work with actual numbers

2. **Then read**: `EKF_SLAM_UPDATE_STEPS_QUICK_REFERENCE.md`
   - Familiarize yourself with the 8 update steps
   - Use as reference while implementing

3. **Visualize**: `EKF_SLAM_VISUAL_TIMELINE.md`
   - See the big picture
   - Understand state evolution
   - See how uncertainty decreases

4. **Implement**: Use the guides as reference
   - Start with predict step
   - Then data association
   - Then update step
   - Test incrementally

### For Implementation

1. **Reference**: `EKF_SLAM_UPDATE_STEPS_QUICK_REFERENCE.md`
   - Use pseudocode as template
   - Check matrix dimensions
   - Debug common issues

2. **Debug**: `EKF_SLAM_MATRIX_DIMENSIONS.md`
   - Check dimension mismatches
   - Verify matrix operations
   - Fix dimension errors

3. **Understand**: `EKF_SLAM_STEP_BY_STEP_WALKTHROUGH.md`
   - See expected values
   - Understand corrections
   - Verify your implementation

## Key Concepts

### The Update Step (8 Sub-Steps)

1. **Data Association**: Match observation to existing landmark or create new
2. **Handle New Landmarks**: Extend state and covariance if new landmark
3. **Calculate Predicted Observation**: What we expect to see based on current state
4. **Calculate Innovation**: Difference between observed and predicted
5. **Calculate Innovation Covariance**: Uncertainty in the innovation
6. **Calculate Kalman Gain**: How much to trust the observation
7. **Update State**: Correct state estimate (robot + landmarks)
8. **Update Covariance**: Reduce uncertainty

### State Vector Structure

```
xEst = [robot_x, robot_y, robot_yaw, lm0_x, lm0_y, lm1_x, lm1_y, ...]
        └─── Robot (3 elements) ───┘  └─── Landmarks (2 elements each) ───┘
        
Size: N = 3 + 2 × (number of landmarks)
```

### Covariance Matrix Structure

```
PEst = [P_robot      P_robot_lm0    P_robot_lm1    ...]
       [P_lm0_robot  P_lm0          P_lm0_lm1      ...]
       [P_lm1_robot  P_lm1_lm0      P_lm1          ...]
       [...          ...            ...            ...]
       
Size: [N × N] where N = state size
```

### Key Equations

**Predict Step**:
```
x_predicted = motion_model(x_old, u)
P_predicted = G.T @ P_old @ G + Fx.T @ Q @ Fx
```

**Update Step**:
```
y = z_observed - z_predicted                    (innovation)
S = H @ P @ H.T + R                             (innovation covariance)
K = P @ H.T @ S^(-1)                            (Kalman gain)
x_new = x_old + K @ y                           (state update)
P_new = (I - K @ H) @ P_old                     (covariance update)
```

## Homework Specific Notes

### Environment
- 10ft × 10ft square
- 8 landmarks (2 on each side)
- Robot starts at unknown pose
- No prior knowledge of landmarks

### Trajectory Phases
1. **Square Path**: Discover landmarks on sides
2. **Octagon Path**: Refine landmark estimates

### Key Requirements
- Must discover landmarks dynamically (no prior knowledge)
- Must estimate both robot trajectory and landmark positions
- Must handle unknown starting pose
- Must use EKF SLAM (Extended Kalman Filter)

### Evaluation
- Compare estimated landmark positions to ground truth
- Compare estimated robot trajectory to actual trajectory
- Analyze uncertainty evolution over time

## Implementation Checklist

### Setup
- [ ] Initialize state vector (robot only)
- [ ] Initialize covariance matrix (3×3)
- [ ] Set up motion model
- [ ] Set up observation model
- [ ] Set up noise parameters

### Predict Step
- [ ] Implement motion model
- [ ] Calculate motion Jacobian (G)
- [ ] Update state (robot only)
- [ ] Update covariance (propagate uncertainty)

### Data Association
- [ ] Calculate predicted observation for each landmark
- [ ] Calculate innovation for each landmark
- [ ] Calculate Mahalanobis distance
- [ ] Find minimum distance
- [ ] Compare to threshold
- [ ] Decide: match or new landmark

### Handle New Landmarks
- [ ] Calculate landmark position from observation
- [ ] Extend state vector
- [ ] Extend covariance matrix
- [ ] Initialize new landmark uncertainty

### Update Step
- [ ] Calculate predicted observation
- [ ] Calculate innovation
- [ ] Calculate observation Jacobian (H)
- [ ] Calculate innovation covariance (S)
- [ ] Calculate Kalman gain (K)
- [ ] Update state
- [ ] Update covariance
- [ ] Normalize angles

### Testing
- [ ] Test with simulation (known ground truth)
- [ ] Test with real robot
- [ ] Verify state growth
- [ ] Verify uncertainty decreases
- [ ] Verify landmark positions improve
- [ ] Compare to ground truth

## Common Issues and Solutions

### Issue 1: Dimension Mismatch
**Solution**: Check matrix dimensions at each step. Use `EKF_SLAM_MATRIX_DIMENSIONS.md` as reference.

### Issue 2: Wrong Data Association
**Solution**: Verify Mahalanobis distance calculation. Check threshold value. Ensure innovation covariance is correct.

### Issue 3: State Not Updating
**Solution**: Verify Kalman gain is non-zero. Check that innovation is non-zero. Ensure update equations are correct.

### Issue 4: Uncertainty Not Decreasing
**Solution**: Verify `(I - K @ H)` calculation. Check observation noise. Ensure H and K are correct.

### Issue 5: Angles Wrapping
**Solution**: Always normalize angles to [-π, π]. Use `pi_2_pi()` function. Normalize both innovation and state angles.

## Resources

### Original Code
- `ekf_slam.py`: Reference implementation from PythonRobotics

### Documentation
- `EKF_SLAM_STEP_BY_STEP_WALKTHROUGH.md`: Detailed walkthrough
- `EKF_SLAM_UPDATE_STEPS_QUICK_REFERENCE.md`: Quick reference
- `EKF_SLAM_VISUAL_TIMELINE.md`: Visual timeline
- `EKF_SLAM_MATRIX_DIMENSIONS.md`: Matrix dimensions

### Additional Resources
- Lecture notes on EKF SLAM
- PythonRobotics EKF SLAM implementation
- Probabilistic Robotics (Thrun et al.) - Chapter 10

## Next Steps

1. **Read the walkthrough**: Start with `EKF_SLAM_STEP_BY_STEP_WALKTHROUGH.md`
2. **Understand the concepts**: Review key concepts above
3. **Implement predict step**: Start with motion model
4. **Implement data association**: Match observations to landmarks
5. **Implement update step**: All 8 sub-steps
6. **Test with simulation**: Verify with known ground truth
7. **Test with real robot**: Use actual sensor data
8. **Evaluate results**: Compare to ground truth

## Tips for Success

1. **Start simple**: Test with one landmark first
2. **Print values**: Output intermediate values to verify calculations
3. **Check dimensions**: Verify matrix dimensions at each step
4. **Visualize**: Plot robot and landmark positions to see updates
5. **Test incrementally**: Test each step separately before combining
6. **Use ground truth**: Compare estimates to known ground truth
7. **Monitor uncertainty**: Track how uncertainty evolves over time
8. **Handle edge cases**: What happens at boundaries? With bad observations?

## Contact

If you have questions or need help, refer to the detailed guides or consult with your instructor.

Good luck with your homework! 🚀

