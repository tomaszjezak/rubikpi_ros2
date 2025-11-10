# EKF SLAM - Matrix Dimensions Reference

## State Vector Dimensions

### Robot State
- **Size**: `STATE_SIZE = 3`
- **Elements**: `[robot_x, robot_y, robot_yaw]`
- **Dimension**: `[3 × 1]`

### Landmark State (per landmark)
- **Size**: `LM_SIZE = 2`
- **Elements**: `[landmark_x, landmark_y]`
- **Dimension**: `[2 × 1]`

### Full State Vector
- **Size**: `N = STATE_SIZE + LM_SIZE * nLM`
  - `nLM` = number of landmarks
- **Elements**: `[robot_x, robot_y, robot_yaw, lm0_x, lm0_y, lm1_x, lm1_y, ...]`
- **Dimension**: `[N × 1]`

**Example**:
- 0 landmarks: `N = 3` → `[3 × 1]`
- 1 landmark: `N = 5` → `[5 × 1]`
- 2 landmarks: `N = 7` → `[7 × 1]`
- k landmarks: `N = 3 + 2*k` → `[3 + 2*k × 1]`

---

## Covariance Matrix Dimensions

### State Covariance (PEst)
- **Size**: `[N × N]` where `N = STATE_SIZE + LM_SIZE * nLM`
- **Structure**:
  ```
  PEst = [P_robot      P_robot_lm0    P_robot_lm1    ...]
         [P_lm0_robot  P_lm0          P_lm0_lm1      ...]
         [P_lm1_robot  P_lm1_lm0      P_lm1          ...]
         [...          ...            ...            ...]
  ```
- **Blocks**:
  - `P_robot`: `[3 × 3]` - Robot state covariance
  - `P_lm_i`: `[2 × 2]` - Landmark i covariance
  - `P_robot_lm_i`: `[3 × 2]` - Robot-landmark i cross-covariance
  - `P_lm_i_lm_j`: `[2 × 2]` - Landmark i-landmark j cross-covariance

**Example** (1 landmark):
```
PEst = [P_robot      P_robot_lm0]   (3×3)  (3×2)
       [P_lm0_robot  P_lm0      ]   (2×3)  (2×2)
       Total: [5 × 5]
```

---

## Control Input Dimensions

### Control Vector (u)
- **Size**: `[2 × 1]`
- **Elements**: `[velocity, yaw_rate]`
- **Units**: `[m/s, rad/s]`

### Control Noise Covariance (R_sim)
- **Size**: `[2 × 2]`
- **Structure**: Diagonal matrix
  ```
  R_sim = [σ²_v    0    ]
          [0       σ²_ω ]
  ```

---

## Observation Dimensions

### Observation Vector (z)
- **Size**: `[2 × 1]` (per observation)
- **Elements**: `[distance, angle]`
- **Units**: `[m, rad]`

### Observation Matrix (z - multiple observations)
- **Size**: `[M × 3]` where M = number of observations
- **Elements**: `[[distance1, angle1, id1], [distance2, angle2, id2], ...]`

### Observation Noise Covariance (Q_sim or Cx[0:2, 0:2])
- **Size**: `[2 × 2]`
- **Structure**: Diagonal matrix
  ```
  Q = [σ²_d    0    ]
      [0       σ²_θ ]
  ```

---

## Motion Model Dimensions

### State Transition Matrix (F)
- **Size**: `[3 × 3]` (for robot state only)
- **Structure**: Identity matrix
  ```
  F = [1  0  0]
      [0  1  0]
      [0  0  1]
  ```

### Control Matrix (B)
- **Size**: `[3 × 2]`
- **Structure**:
  ```
  B = [DT*cos(yaw)  0  ]
      [DT*sin(yaw)  0  ]
      [0            DT]
  ```

### Motion Model Output
- **Input**: `x_robot = [3 × 1]`, `u = [2 × 1]`
- **Output**: `x_new = [3 × 1]`
- **Formula**: `x_new = F @ x_old + B @ u`

---

## Jacobian Dimensions

### Motion Jacobian (jF)
- **Size**: `[3 × 3]` (w.r.t. robot state)
- **Structure**:
  ```
  jF = [0  0  -DT*v*sin(yaw)]
       [0  0   DT*v*cos(yaw)]
       [0  0   0            ]
  ```

### Selection Matrix (Fx)
- **Size**: `[3 × N]` where N = full state size
- **Structure**:
  ```
  Fx = [I₃  0  0  ...]  (selects robot state from full state)
       └─┘ └─────────┘
      robot  landmarks
  ```
- **Example** (1 landmark, N=5):
  ```
  Fx = [1  0  0  0  0]
       [0  1  0  0  0]
       [0  0  1  0  0]
  ```

### Motion Jacobian for Full State (G)
- **Size**: `[N × N]` where N = full state size
- **Formula**: `G = I_N + Fx.T @ jF @ Fx`
- **Structure**:
  - Identity matrix with motion corrections in robot block
  - Landmark blocks unchanged

---

## Observation Model Dimensions

### Observation Function Output
- **Input**: State `[N × 1]`, Landmark `[2 × 1]`
- **Output**: `[2 × 1]` (distance, angle)

### Observation Jacobian Intermediate (G in jacob_h)
- **Size**: `[2 × 5]` (w.r.t. [robot_x, robot_y, robot_yaw, lm_x, lm_y])
- **Structure**:
  ```
  G = [∂d/∂rx  ∂d/∂ry  ∂d/∂ryaw  ∂d/∂lx  ∂d/∂ly]
      [∂θ/∂rx  ∂θ/∂ry  ∂θ/∂ryaw  ∂θ/∂lx  ∂θ/∂ly]
  ```

### Selection Matrix for Observation (F in jacob_h)
- **Size**: `[5 × N]` (selects [robot_x, robot_y, robot_yaw, lm_i_x, lm_i_y])
- **Structure**:
  ```
  F = [F1]  (selects robot: [3 × N])
      [F2]  (selects landmark i: [2 × N])
  ```
- **F1**: `[3 × N]` - `[I₃, 0, 0, ...]`
- **F2**: `[2 × N]` - `[0, 0, 0, ..., I₂, ..., 0]` (I₂ at landmark i position)

### Observation Jacobian (H)
- **Size**: `[2 × N]` where N = full state size
- **Formula**: `H = G @ F`
- **Structure**:
  - Row 1: How distance changes w.r.t. each state variable
  - Row 2: How angle changes w.r.t. each state variable
  - Only robot and observed landmark have non-zero derivatives

---

## Innovation Dimensions

### Innovation Vector (y)
- **Size**: `[2 × 1]`
- **Elements**: `[distance_innovation, angle_innovation]`
- **Formula**: `y = z_observed - z_predicted`

### Innovation Covariance (S)
- **Size**: `[2 × 2]`
- **Formula**: `S = H @ PEst @ H.T + Q`
- **Structure**:
  ```
  S = [σ²_d_innov    σ_dθ_innov]
      [σ_dθ_innov    σ²_θ_innov]
  ```

---

## Kalman Gain Dimensions

### Kalman Gain (K)
- **Size**: `[N × 2]` where N = full state size
- **Formula**: `K = PEst @ H.T @ S^(-1)`
- **Structure**:
  - Each row: Correction for one state variable
  - Each column: Correction from distance/angle innovation
  - **Example** (N=5):
    ```
    K = [K_rx_d  K_rx_θ]  (robot x corrections)
        [K_ry_d  K_ry_θ]  (robot y corrections)
        [K_ryaw_d K_ryaw_θ]  (robot yaw corrections)
        [K_lx_d  K_lx_θ]  (landmark x corrections)
        [K_ly_d  K_ly_θ]  (landmark y corrections)
    ```

---

## Update Step Dimensions

### State Update
- **Input**: `xEst = [N × 1]`, `K = [N × 2]`, `y = [2 × 1]`
- **Formula**: `xEst_new = xEst + K @ y`
- **Output**: `xEst_new = [N × 1]`
- **Dimension check**: `[N × 1] + ([N × 2] @ [2 × 1]) = [N × 1]` ✓

### Covariance Update
- **Input**: `PEst = [N × N]`, `K = [N × 2]`, `H = [2 × N]`, `I = [N × N]`
- **Formula**: `PEst_new = (I - K @ H) @ PEst`
- **Output**: `PEst_new = [N × N]`
- **Dimension check**: 
  - `K @ H = [N × 2] @ [2 × N] = [N × N]` ✓
  - `I - K @ H = [N × N] - [N × N] = [N × N]` ✓
  - `(I - K @ H) @ PEst = [N × N] @ [N × N] = [N × N]` ✓

---

## Predict Step Dimensions

### State Prediction
- **Input**: `xEst = [N × 1]`, `u = [2 × 1]`
- **Formula**: `xEst[0:3] = motion_model(xEst[0:3], u)`
- **Output**: `xEst = [N × 1]` (only first 3 elements change)

### Covariance Prediction
- **Input**: `PEst = [N × N]`, `G = [N × N]`, `Fx = [3 × N]`, `Cx = [3 × 3]`
- **Formula**: `PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx`
- **Output**: `PEst = [N × N]`
- **Dimension check**:
  - `G.T @ PEst @ G = [N × N] @ [N × N] @ [N × N] = [N × N]` ✓
  - `Fx.T @ Cx @ Fx = [N × 3] @ [3 × 3] @ [3 × N] = [N × N]` ✓
  - `[N × N] + [N × N] = [N × N]` ✓

---

## Data Association Dimensions

### Mahalanobis Distance
- **Input**: `y = [2 × 1]`, `S = [2 × 2]`
- **Formula**: `d² = y.T @ S^(-1) @ y`
- **Output**: Scalar
- **Dimension check**: `[1 × 2] @ [2 × 2] @ [2 × 1] = [1 × 1]` (scalar) ✓

---

## New Landmark Addition Dimensions

### Extended State Vector
- **Input**: `xEst = [N × 1]`, new landmark `[2 × 1]`
- **Output**: `xAug = [N+2 × 1]`
- **Formula**: `xAug = vstack(xEst, new_landmark)`

### Extended Covariance Matrix
- **Input**: `PEst = [N × N]`, `initP = [2 × 2]`
- **Output**: `PAug = [N+2 × N+2]`
- **Structure**:
  ```
  PAug = [PEst        zeros(N×2)]
         [zeros(2×N)  initP     ]
  ```

---

## Dimension Summary Table

| Matrix/Vector | Symbol | Size | Notes |
|--------------|--------|------|-------|
| Robot state | `x_robot` | `[3 × 1]` | [x, y, yaw] |
| Landmark state | `lm_i` | `[2 × 1]` | [x, y] |
| Full state | `xEst` | `[N × 1]` | N = 3 + 2*nLM |
| State covariance | `PEst` | `[N × N]` | |
| Control input | `u` | `[2 × 1]` | [v, ω] |
| Control noise | `R` | `[2 × 2]` | |
| Observation | `z` | `[2 × 1]` | [d, θ] |
| Observation noise | `Q` | `[2 × 2]` | |
| Process noise | `Cx` | `[3 × 3]` | Robot motion noise |
| State transition | `F` | `[3 × 3]` | Robot only |
| Control matrix | `B` | `[3 × 2]` | |
| Motion Jacobian | `jF` | `[3 × 3]` | w.r.t. robot |
| Selection matrix | `Fx` | `[3 × N]` | Selects robot |
| Full motion Jacobian | `G` | `[N × N]` | |
| Observation Jacobian | `H` | `[2 × N]` | |
| Innovation | `y` | `[2 × 1]` | |
| Innovation covariance | `S` | `[2 × 2]` | |
| Kalman gain | `K` | `[N × 2]` | |

---

## Dimension Checks for Common Operations

### Predict Step
```python
# State prediction
xEst[0:3] = F @ xEst[0:3] + B @ u
# [3×1] = [3×3] @ [3×1] + [3×2] @ [2×1] ✓

# Covariance prediction
PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx
# [N×N] = [N×N] @ [N×N] @ [N×N] + [N×3] @ [3×3] @ [3×N] ✓
```

### Update Step
```python
# Innovation
y = z - zp
# [2×1] = [2×1] - [2×1] ✓

# Innovation covariance
S = H @ PEst @ H.T + Q
# [2×2] = [2×N] @ [N×N] @ [N×2] + [2×2] ✓

# Kalman gain
K = PEst @ H.T @ inv(S)
# [N×2] = [N×N] @ [N×2] @ [2×2] ✓

# State update
xEst = xEst + K @ y
# [N×1] = [N×1] + [N×2] @ [2×1] ✓

# Covariance update
PEst = (I - K @ H) @ PEst
# [N×N] = ([N×N] - [N×2] @ [2×N]) @ [N×N] ✓
```

---

## Common Dimension Errors

1. **Mismatched state and covariance sizes**
   - Error: `xEst = [5×1]` but `PEst = [3×3]`
   - Fix: Ensure both have same first dimension N

2. **Wrong Jacobian dimensions**
   - Error: `H = [2×3]` when state is `[5×1]`
   - Fix: `H` must be `[2×N]` where N is state size

3. **Kalman gain dimension mismatch**
   - Error: `K = [3×2]` when state is `[5×1]`
   - Fix: `K` must be `[N×2]` where N is state size

4. **Innovation dimension error**
   - Error: `y = [3×1]` when observation is `[2×1]`
   - Fix: `y` must be `[2×1]` (matches observation size)

5. **Covariance update dimension error**
   - Error: `I - K @ H` doesn't match `PEst` dimensions
   - Fix: Ensure `K @ H` is `[N×N]` to match `PEst`

---

## Tips for Debugging Dimension Issues

1. **Print dimensions**: Add `print(shape)` statements to check dimensions
2. **Use assertions**: `assert matrix.shape == (expected_rows, expected_cols)`
3. **Check state size**: Ensure `N = STATE_SIZE + LM_SIZE * nLM` is correct
4. **Verify Jacobians**: Jacobians must match state size, not just robot size
5. **Landmark indexing**: When accessing landmarks, ensure indices are correct:
   - Landmark i: `xEst[STATE_SIZE + LM_SIZE*i : STATE_SIZE + LM_SIZE*(i+1)]`

---

## Visual Dimension Guide

```
┌─────────────────────────────────────────────────────────┐
│                   STATE VECTOR (xEst)                    │
│                    [N × 1]                               │
├─────────────────────────────────────────────────────────┤
│ [robot_x]      │  [lm0_x]  │  [lm1_x]  │  [lm2_x]  │ ...│
│ [robot_y]      │  [lm0_y]  │  [lm1_y]  │  [lm2_y]  │ ...│
│ [robot_yaw]    │           │           │           │ ...│
└─────────────────────────────────────────────────────────┘
  3 elements       2 elements  2 elements  2 elements

┌─────────────────────────────────────────────────────────┐
│              COVARIANCE MATRIX (PEst)                    │
│                    [N × N]                               │
├─────────────────────────────────────────────────────────┤
│ P_robot      │ P_robot_lm0 │ P_robot_lm1 │ P_robot_lm2 │
│ [3×3]        │ [3×2]       │ [3×2]       │ [3×2]       │
├──────────────┼─────────────┼─────────────┼─────────────┤
│ P_lm0_robot  │ P_lm0       │ P_lm0_lm1   │ P_lm0_lm2   │
│ [2×3]        │ [2×2]       │ [2×2]       │ [2×2]       │
├──────────────┼─────────────┼─────────────┼─────────────┤
│ P_lm1_robot  │ P_lm1_lm0   │ P_lm1       │ P_lm1_lm2   │
│ [2×3]        │ [2×2]       │ [2×2]       │ [2×2]       │
├──────────────┼─────────────┼─────────────┼─────────────┤
│ P_lm2_robot  │ P_lm2_lm0   │ P_lm2_lm1   │ P_lm2       │
│ [2×3]        │ [2×2]       │ [2×2]       │ [2×2]       │
└─────────────────────────────────────────────────────────┘
```

This visual guide shows how the state vector and covariance matrix are structured when you have multiple landmarks!

