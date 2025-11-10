# PEst - Code Reference and Examples

## Where PEst is Defined in the Code

### Initial Definition (Line 227)

```python
# From ekf_slam.py, line 227
PEst = np.eye(STATE_SIZE)
```

**What this does**:
- Creates an **identity matrix** of size `STATE_SIZE × STATE_SIZE`
- `STATE_SIZE = 3` (for robot: x, y, yaw)
- Result: `PEst` is a `[3 × 3]` matrix

**The identity matrix**:
```python
PEst = [[1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]
```

---

## Complete Initialization Example

### Full Initialization Code

```python
# From ekf_slam.py, lines 224-227
# State Vector [x y yaw v]'
xEst = np.zeros((STATE_SIZE, 1))      # [3 × 1] - all zeros
xTrue = np.zeros((STATE_SIZE, 1))     # [3 × 1] - ground truth
PEst = np.eye(STATE_SIZE)             # [3 × 3] - identity matrix
```

### What Each Line Does

**Line 225**: `xEst = np.zeros((STATE_SIZE, 1))`
- Creates state vector: `[0, 0, 0]`
- Robot starts at origin (0, 0) facing 0 radians
- Size: `[3 × 1]`

**Line 227**: `PEst = np.eye(STATE_SIZE)`
- Creates covariance matrix: identity matrix
- Initial uncertainty: 1.0 for each variable
- No correlations: all off-diagonal elements are 0.0
- Size: `[3 × 3]`

### Visual Representation

```
State Vector (xEst):        Covariance Matrix (PEst):
[robot_x ]   [0.0]         [P_xx  P_xy  P_xθ ]   [1.0  0.0  0.0]
[robot_y ] = [0.0]         [P_yx  P_yy  P_yθ ] = [0.0  1.0  0.0]
[robot_yaw]  [0.0]         [P_θx  P_θy  P_θθ ]   [0.0  0.0  1.0]

Size: [3 × 1]              Size: [3 × 3]
```

---

## How PEst is Used Throughout the Code

### 1. Predict Step (Line 49)

```python
# From ekf_slam.py, line 49
PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx
```

**What happens**:
- `PEst` is updated to reflect increased uncertainty after motion
- Uncertainty increases because motion adds noise
- Formula: `P_new = G^T @ P_old @ G + process_noise`

**Example**:
```python
# Before predict
PEst = [[1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.1]]

# After predict
PEst = [[1.1, 0.0, 0.0],    # Uncertainty increased
        [0.0, 1.1, 0.0],
        [0.0, 0.0, 0.12]]
```

### 2. Adding New Landmark (Lines 61-64)

```python
# From ekf_slam.py, lines 61-64
PAug = np.vstack((
    np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
    np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))
))
xEst = xAug
PEst = PAug
```

**What happens**:
- `PEst` is extended to include new landmark
- New landmark gets initial uncertainty (`initP`)
- No correlation with existing state (zeros)

**Example**:
```python
# Before (robot only, 3×3)
PEst = [[1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.1]]

# After (robot + 1 landmark, 5×5)
PEst = [[1.0, 0.0, 0.0, 0.0, 0.0],    # Robot-robot
        [0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.1, 0.0, 0.0],
        [0.0, 0.0, 0.0, 2.0, 0.0],    # New landmark
        [0.0, 0.0, 0.0, 0.0, 2.0]]    # (initP = eye(2) * 2)
```

### 3. Update Step (Line 70)

```python
# From ekf_slam.py, line 70
PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst
```

**What happens**:
- `PEst` is updated to reflect reduced uncertainty after observation
- Uncertainty decreases because observation provides information
- Formula: `P_new = (I - K @ H) @ P_old`

**Example**:
```python
# Before update
PEst = [[1.1, 0.0, 0.0, 0.2, 0.1],
        [0.0, 1.1, 0.0, 0.1, 0.2],
        [0.0, 0.0, 0.12, 0.0, 0.0],
        [0.2, 0.1, 0.0, 2.0, 0.3],
        [0.1, 0.2, 0.0, 0.3, 2.0]]

# After update
PEst = [[0.85, 0.1, 0.0, 0.15, 0.08],   # Uncertainty decreased
        [0.1, 0.85, 0.0, 0.08, 0.15],
        [0.0, 0.0, 0.08, 0.0, 0.0],
        [0.15, 0.08, 0.0, 1.45, 0.25],
        [0.08, 0.15, 0.0, 0.25, 1.45]]
```

---

## Step-by-Step: PEst Evolution

### Timestep 1: Initialization

```python
# State: robot only
xEst = [0.0, 0.0, 0.0]  # [3 × 1]

# Covariance: identity matrix
PEst = [[1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]  # [3 × 3]
```

### Timestep 2: After Predict

```python
# State: robot moved
xEst = [0.05, 0.0, 0.0]  # [3 × 1]

# Covariance: uncertainty increased
PEst = [[1.1, 0.0, 0.0],
        [0.0, 1.1, 0.0],
        [0.0, 0.0, 0.12]]  # [3 × 3]
```

### Timestep 3: After Adding Landmark

```python
# State: robot + 1 landmark
xEst = [0.05, 0.0, 0.0, 10.05, 10.0]  # [5 × 1]

# Covariance: extended to include landmark
PEst = [[1.1, 0.0, 0.0, 0.0, 0.0],    # [5 × 5]
        [0.0, 1.1, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.12, 0.0, 0.0],
        [0.0, 0.0, 0.0, 2.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 2.0]]
```

### Timestep 4: After Update

```python
# State: robot + 1 landmark (updated)
xEst = [4.83, 4.73, 0.03, 9.72, 9.67]  # [5 × 1]

# Covariance: uncertainty decreased, correlations introduced
PEst = [[0.85, 0.1, 0.0, 0.15, 0.08],   # [5 × 5]
        [0.1, 0.85, 0.0, 0.08, 0.15],
        [0.0, 0.0, 0.08, 0.0, 0.0],
        [0.15, 0.08, 0.0, 1.45, 0.25],
        [0.08, 0.15, 0.0, 0.25, 1.45]]
```

---

## Key Functions That Use PEst

### 1. Data Association (Line 54)

```python
# From ekf_slam.py, line 54
min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])
```

**What it does**:
- Uses `PEst` to calculate Mahalanobis distance
- Mahalanobis distance accounts for uncertainty
- Helps match observations to landmarks

### 2. Innovation Calculation (Line 66)

```python
# From ekf_slam.py, line 66
y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)
```

**What it does**:
- Uses `PEst` to calculate innovation covariance
- Innovation covariance: `S = H @ PEst @ H.T + Q`
- Tells us how uncertain the innovation is

### 3. Kalman Gain (Line 68)

```python
# From ekf_slam.py, line 68
K = (PEst @ H.T) @ np.linalg.inv(S)
```

**What it does**:
- Uses `PEst` to calculate Kalman gain
- Kalman gain: `K = PEst @ H.T @ S^(-1)`
- Determines how much to trust the observation

### 4. Covariance Update (Line 70)

```python
# From ekf_slam.py, line 70
PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst
```

**What it does**:
- Updates `PEst` to reflect reduced uncertainty
- Uncertainty decreases after observation
- Correlations are updated

---

## Common PEst Operations

### 1. Check PEst Size

```python
# PEst should always be [N × N] where N = len(xEst)
assert PEst.shape == (len(xEst), len(xEst)), "PEst size mismatch!"
```

### 2. Extract Robot Covariance

```python
# Robot covariance is top-left 3×3 block
P_robot = PEst[0:3, 0:3]
```

### 3. Extract Landmark Covariance

```python
# Landmark i covariance
lm_start = STATE_SIZE + LM_SIZE * i
lm_end = STATE_SIZE + LM_SIZE * (i + 1)
P_lm = PEst[lm_start:lm_end, lm_start:lm_end]
```

### 4. Extract Robot-Landmark Cross-Covariance

```python
# Robot-landmark i cross-covariance
P_robot_lm = PEst[0:3, lm_start:lm_end]
```

### 5. Check if PEst is Valid

```python
# PEst should be symmetric
assert np.allclose(PEst, PEst.T), "PEst is not symmetric!"

# PEst should be positive definite (all eigenvalues > 0)
eigenvalues = np.linalg.eigvals(PEst)
assert np.all(eigenvalues > 0), "PEst is not positive definite!"
```

---

## Debugging PEst Issues

### Issue 1: PEst Size Mismatch

**Symptom**: Error when multiplying PEst with other matrices

**Solution**:
```python
# Check sizes
print(f"xEst size: {xEst.shape}")
print(f"PEst size: {PEst.shape}")

# Ensure they match
assert PEst.shape[0] == len(xEst), "PEst rows don't match xEst!"
assert PEst.shape[1] == len(xEst), "PEst cols don't match xEst!"
```

### Issue 2: PEst Not Updating

**Symptom**: PEst stays the same after update

**Solution**:
```python
# Check if update is being called
print(f"PEst before update: {PEst}")

# Perform update
PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

# Check if it changed
print(f"PEst after update: {PEst}")
```

### Issue 3: PEst Becoming Invalid

**Symptom**: PEst has negative eigenvalues or is not symmetric

**Solution**:
```python
# Force symmetry
PEst = (PEst + PEst.T) / 2

# Add small regularization to ensure positive definite
PEst += np.eye(len(xEst)) * 1e-6
```

### Issue 4: PEst Growing Too Large

**Symptom**: PEst values become very large

**Solution**:
```python
# Check if process noise is too large
print(f"Process noise Cx: {Cx}")

# Check if observation noise is too small
print(f"Observation noise Q: {Q}")

# Adjust noise parameters if needed
```

---

## Summary

**PEst Initialization**:
```python
PEst = np.eye(STATE_SIZE)  # Identity matrix, [3 × 3] initially
```

**PEst Updates**:
1. **Predict**: `PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx` (uncertainty increases)
2. **Add Landmark**: Extend `PEst` with zeros and `initP` (size grows)
3. **Update**: `PEst = (I - K @ H) @ PEst` (uncertainty decreases)

**PEst Usage**:
- Data association (Mahalanobis distance)
- Innovation covariance calculation
- Kalman gain calculation
- Uncertainty tracking

**Key Points**:
- PEst is always `[N × N]` where N = state size
- Diagonal elements = uncertainty (variance)
- Off-diagonal elements = correlations (covariance)
- PEst grows as state grows (adds landmarks)
- PEst should be symmetric and positive definite

For more detailed explanation, see `EKF_SLAM_PEST_EXPLAINED.md`!

