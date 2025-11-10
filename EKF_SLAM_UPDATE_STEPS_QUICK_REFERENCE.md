# EKF SLAM Update Steps - Quick Reference

## The 7 Steps of the Update Process

### Step 1: Data Association
```
Question: "Which landmark did I just see?"
Action: Compare observation to all existing landmarks using Mahalanobis distance
Output: min_id (landmark index, or nLM if new)
```

### Step 2: Handle New Landmarks (if needed)
```
If min_id == nLM:
  - Calculate landmark position from observation
  - Extend state vector: xEst = [xEst; new_landmark]
  - Extend covariance: PAug = [PEst, zeros; zeros, initP]
```

### Step 3: Calculate Predicted Observation
```
Input: Current state xEst, landmark position lm
Action: Calculate what we expect to observe
  - delta = lm - robot_position
  - distance_predicted = sqrt(delta_x² + delta_y²)
  - angle_predicted = atan2(delta_y, delta_x) - robot_yaw
Output: zp = [distance_predicted, angle_predicted]
```

### Step 4: Calculate Innovation (Residual)
```
Formula: y = z_observed - z_predicted
Meaning: How wrong was our prediction?
Output: y = [distance_innovation, angle_innovation]
```

### Step 5: Calculate Innovation Covariance
```
Formula: S = H @ PEst @ H.T + Q
Meaning: How uncertain is the innovation?
Components:
  - H: Observation Jacobian [2 × N]
  - PEst: State covariance [N × N]
  - Q: Observation noise [2 × 2]
Output: S [2 × 2]
```

### Step 6: Calculate Kalman Gain
```
Formula: K = PEst @ H.T @ S^(-1)
Meaning: How much should we trust the observation?
Output: K [N × 2]
  - Each row: Correction for one state variable
  - Each column: Correction from distance/angle innovation
```

### Step 7: Update State
```
Formula: xEst_new = xEst_old + K @ y
Meaning: Correct our state estimate
Output: Updated state vector [N × 1]
  - Robot position updated
  - Landmark positions updated
```

### Step 8: Update Covariance
```
Formula: PEst_new = (I - K @ H) @ PEst_old
Meaning: Reduce our uncertainty
Output: Updated covariance matrix [N × N]
  - Diagonal elements decrease (less uncertainty)
  - Off-diagonal elements change (correlations update)
```

---

## Complete Update Step Pseudocode

```python
def update_step(xEst, PEst, z_observed):
    # Step 1: Data Association
    min_id = search_correspond_landmark_id(xEst, PEst, z_observed)
    nLM = calc_n_lm(xEst)
    
    # Step 2: Handle New Landmark
    if min_id == nLM:
        new_lm = calc_landmark_position(xEst, z_observed)
        xEst = vstack(xEst, new_lm)
        PEst = extend_covariance(PEst, initP)
    
    # Step 3: Get Landmark and Calculate Predicted Observation
    lm = get_landmark_position_from_state(xEst, min_id)
    zp = calculate_predicted_observation(lm, xEst)
    
    # Step 4: Calculate Innovation
    y = z_observed - zp
    y[1] = pi_2_pi(y[1])  # Normalize angle
    
    # Step 5: Calculate Innovation Covariance
    H = jacob_h(lm, xEst, min_id)
    S = H @ PEst @ H.T + Q
    
    # Step 6: Calculate Kalman Gain
    K = PEst @ H.T @ inv(S)
    
    # Step 7: Update State
    xEst = xEst + K @ y
    xEst[2] = pi_2_pi(xEst[2])  # Normalize robot yaw
    
    # Step 8: Update Covariance
    PEst = (eye(N) - K @ H) @ PEst
    
    return xEst, PEst
```

---

## Matrix Dimensions Cheat Sheet

| Variable | Symbol | Dimensions | Notes |
|----------|--------|------------|-------|
| State vector | `xEst` | `[N × 1]` | N = 3 + 2*nLM |
| Covariance | `PEst` | `[N × N]` | |
| Observation | `z` | `[2 × 1]` | [distance, angle] |
| Predicted obs | `zp` | `[2 × 1]` | [distance, angle] |
| Innovation | `y` | `[2 × 1]` | [distance, angle] |
| Obs Jacobian | `H` | `[2 × N]` | |
| Innovation cov | `S` | `[2 × 2]` | |
| Kalman gain | `K` | `[N × 2]` | |
| Identity | `I` | `[N × N]` | |

---

## Key Equations Reference

### Innovation
```
y = z - zp
```

### Innovation Covariance
```
S = H @ P @ H.T + Q
```

### Kalman Gain
```
K = P @ H.T @ S^(-1)
```

### State Update
```
x_new = x_old + K @ y
```

### Covariance Update
```
P_new = (I - K @ H) @ P_old
```

---

## Common Issues and Solutions

### Issue 1: Dimension Mismatch
**Symptom**: Error in matrix multiplication
**Solution**: Check that:
- `H` is `[2 × N]` where N is current state size
- `K` is `[N × 2]`
- `y` is `[2 × 1]`
- `PEst` is `[N × N]`

### Issue 2: Wrong Data Association
**Symptom**: Landmarks get mixed up
**Solution**: 
- Check Mahalanobis distance calculation
- Verify threshold value (typically 2.0)
- Ensure innovation covariance S is calculated correctly

### Issue 3: State Not Updating
**Symptom**: Estimates don't change after observation
**Solution**:
- Verify Kalman gain is non-zero
- Check that innovation is non-zero
- Ensure update equations are correct

### Issue 4: Uncertainty Not Decreasing
**Symptom**: Covariance doesn't decrease after update
**Solution**:
- Verify `(I - K @ H)` calculation
- Check that observation noise Q is reasonable
- Ensure H and K are calculated correctly

### Issue 5: Angles Wrapping
**Symptom**: Angle values grow unbounded
**Solution**:
- Always normalize angles to [-π, π]
- Use `pi_2_pi()` function for all angles
- Normalize both innovation and state angles

---

## Debugging Checklist

When debugging update steps, check:

- [ ] Data association returns correct landmark ID
- [ ] New landmarks are added correctly (state and covariance extended)
- [ ] Predicted observation matches expected values
- [ ] Innovation is reasonable (not too large)
- [ ] Innovation covariance S is positive definite
- [ ] Kalman gain K has reasonable values
- [ ] State update changes values appropriately
- [ ] Covariance update reduces uncertainty
- [ ] Matrix dimensions are correct throughout
- [ ] Angles are normalized properly

---

## Visual Flow Diagram

```
Observation z
     │
     ▼
┌─────────────────┐
│ Data Association│
└─────────────────┘
     │
     ├─→ New? ──→ Add to state
     │
     └─→ Known? ──→ Continue
                │
                ▼
         ┌──────────────┐
         │ Predict Obs  │
         └──────────────┘
                │
                ▼
         ┌──────────────┐
         │ Innovation   │
         │   y = z - zp │
         └──────────────┘
                │
                ▼
         ┌──────────────┐
         │ Innovation   │
         │  Covariance  │
         │  S = HPH.T+Q │
         └──────────────┘
                │
                ▼
         ┌──────────────┐
         │ Kalman Gain  │
         │ K = PH.T/S   │
         └──────────────┘
                │
                ├─→ Update State: x = x + K@y
                │
                └─→ Update Cov: P = (I-K@H)@P
```

---

## Quick Test Values

### Test Case 1: Simple Update
```
Initial state: xEst = [5.0, 3.0, 0.0, 10.0, 8.0]
Observation: z = [8.6, 0.12]
Expected innovation: y ≈ [0.0, 0.0] (if prediction is good)
Expected correction: Small values
```

### Test Case 2: Large Innovation
```
Initial state: xEst = [5.0, 3.0, 0.0, 10.0, 8.0]
Observation: z = [10.0, 0.5]
Expected innovation: y = [1.4, 0.38] (large!)
Expected correction: Larger values
Expected uncertainty reduction: Significant
```

### Test Case 3: New Landmark
```
Initial state: xEst = [5.0, 3.0, 0.0] (robot only)
Observation: z = [7.0, 0.78]
Expected: New landmark added at approximately (10.0, 10.0)
State size: 3 → 5
Covariance size: 3×3 → 5×5
```

---

## Tips for Implementation

1. **Start Simple**: Test with one landmark first
2. **Print Values**: Output intermediate values to verify calculations
3. **Check Dimensions**: Verify matrix dimensions at each step
4. **Visualize**: Plot robot and landmark positions to see updates
5. **Test Incrementally**: Test each step separately before combining
6. **Use Ground Truth**: Compare estimates to known ground truth
7. **Monitor Uncertainty**: Track how uncertainty evolves over time
8. **Handle Edge Cases**: What happens at boundaries? With bad observations?

---

## Next Steps

1. Implement predict step
2. Implement data association
3. Implement update step (all 8 sub-steps)
4. Test with simulation
5. Test with real robot
6. Evaluate and refine

Good luck! 🚀

