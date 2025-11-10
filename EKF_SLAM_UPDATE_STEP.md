# EKF SLAM - Deep Dive into the UPDATE STEP

## Overview

The **UPDATE STEP** is where the magic happens! This is where we:
1. Compare what we **observe** with what we **predict**
2. Calculate how wrong we were (innovation)
3. Figure out how much to trust the observation (Kalman gain)
4. Correct our state estimate
5. Reduce our uncertainty

**Key Insight**: The update step corrects BOTH the robot's position AND the landmark positions simultaneously!

---

## The Update Step Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    UPDATE STEP FLOW                          │
└─────────────────────────────────────────────────────────────┘

1. DATA ASSOCIATION
   "Which landmark did I just see?"
   ┌──────────────┐
   │ Observation  │ ──→ Match to existing landmark OR create new
   │   [d, θ]     │
   └──────────────┘

2. CALCULATE PREDICTED OBSERVATION
   "What should I see based on my current belief?"
   ┌──────────────┐
   │ Current      │ ──→ Calculate expected [distance, angle]
   │ State xEst   │      to the landmark
   └──────────────┘

3. CALCULATE INNOVATION (RESIDUAL)
   "How wrong was my prediction?"
   ┌──────────────┐
   │ Innovation   │ ──→ y = z_observed - z_predicted
   │      y       │
   └──────────────┘

4. CALCULATE INNOVATION COVARIANCE
   "How uncertain is this innovation?"
   ┌──────────────┐
   │      S       │ ──→ S = H @ P @ H.T + R
   └──────────────┘

5. CALCULATE KALMAN GAIN
   "How much should I trust the observation?"
   ┌──────────────┐
   │      K       │ ──→ K = P @ H.T @ S^(-1)
   └──────────────┘

6. UPDATE STATE
   "Correct my beliefs"
   ┌──────────────┐
   │  xEst_new    │ ──→ xEst = xEst + K @ y
   └──────────────┘

7. UPDATE COVARIANCE
   "Reduce my uncertainty"
   ┌──────────────┐
   │  PEst_new    │ ──→ PEst = (I - K @ H) @ PEst
   └──────────────┘
```

---

## Step 1: Data Association

### Purpose
Before updating, we need to know: **Is this observation of a landmark we've seen before, or is it a new landmark?**

### Code
```python
min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])
nLM = calc_n_lm(xEst)

if min_id == nLM:
    # NEW LANDMARK - add to state
    xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
    PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                      np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
    xEst = xAug
    PEst = PAug
```

### Detailed Explanation

#### Finding the Match
```python
min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])
```

**What happens inside**:
1. For each existing landmark `i` (0 to nLM-1):
   - Get landmark position from state: `lm = xEst[STATE_SIZE + 2*i : STATE_SIZE + 2*(i+1)]`
   - Calculate predicted observation: `z_predicted = h(lm, xEst)`
   - Calculate innovation: `y = z_observed - z_predicted`
   - Calculate Mahalanobis distance: `d² = y.T @ S^(-1) @ y`
   
2. Compare all distances:
   - If minimum distance < threshold (2.0) → match to that landmark
   - If all distances ≥ threshold → it's a new landmark

**Example**:
```python
# Current state has 2 landmarks
# Observation: z = [10.5m, 0.2rad]

# Check landmark 0:
#   Predicted: [10.0m, 0.15rad]
#   Innovation: [0.5m, 0.05rad]
#   Mahalanobis distance: 0.8

# Check landmark 1:
#   Predicted: [15.0m, 1.2rad]
#   Innovation: [-4.5m, -1.0rad]
#   Mahalanobis distance: 5.2

# Threshold: 2.0
# min_id = 0 (matches landmark 0, distance 0.8 < 2.0)
```

#### Adding a New Landmark

If `min_id == nLM`, we need to extend the state:

**Step 1: Calculate Landmark Position**
```python
calc_landmark_position(xEst, z[iz, :])
```

**Input**: 
- `xEst = [robot_x, robot_y, robot_yaw, ...]`
- `z = [distance, angle, id]`

**Calculation**:
```python
lm_x = robot_x + distance * cos(robot_yaw + angle)
lm_y = robot_y + distance * sin(robot_yaw + angle)
```

**Example**:
```python
# Robot at (5.0, 3.0) with yaw 0.5 rad
# Observation: distance = 10.0m, angle = 0.2 rad

lm_x = 5.0 + 10.0 * cos(0.5 + 0.2) = 5.0 + 10.0 * 0.764 = 12.64
lm_y = 3.0 + 10.0 * sin(0.5 + 0.2) = 3.0 + 10.0 * 0.644 = 9.44

# New landmark at (12.64, 9.44)
```

**Step 2: Extend State Vector**
```python
xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
```

**Before**: `xEst = [5.0, 3.0, 0.5, 10.0, -2.0]` (robot + 1 landmark)
**After**: `xEst = [5.0, 3.0, 0.5, 10.0, -2.0, 12.64, 9.44]` (robot + 2 landmarks)

**Step 3: Extend Covariance Matrix**
```python
PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                  np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
```

**Structure**:
```
PAug = [PEst        |  0  ]
       [------------|-----]
       [    0       | initP]
```

**Before** (5×5):
```
PEst = [P_robot    P_robot_lm1]
       [P_lm1_robot  P_lm1    ]
```

**After** (7×7):
```
PAug = [P_robot      P_robot_lm1    0    ]
       [P_lm1_robot  P_lm1          0    ]
       [    0            0        initP  ]
```

**Why zeros?**: New landmark is initially uncorrelated with existing state
**initP**: `[[1.0, 0.0], [0.0, 1.0]]` - moderate initial uncertainty

---

## Step 2: Calculate Predicted Observation

### Purpose
Given our current state estimate, what should we observe?

### Code
```python
lm = get_landmark_position_from_state(xEst, min_id)
y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)
```

### Detailed Breakdown

#### Get Landmark Position
```python
lm = get_landmark_position_from_state(xEst, min_id)
```

**What it does**: Extracts landmark's [x, y] from state vector

**Example**:
```python
# xEst = [5.0, 3.0, 0.5, 10.0, -2.0, 12.0, 8.0]
# min_id = 1 (second landmark, index 1)

# Landmark 0: xEst[3:5] = [10.0, -2.0]
# Landmark 1: xEst[5:7] = [12.0, 8.0]

lm = [12.0, 8.0]  # Landmark 1's position
```

#### Calculate Predicted Observation

Inside `calc_innovation()`:

**Step 1: Calculate Delta Vector**
```python
delta = lm - xEst[0:2]
```

**What it is**: Vector from robot to landmark in global coordinates

**Example**:
```python
# Robot at (5.0, 3.0)
# Landmark at (12.0, 8.0)

delta = [12.0 - 5.0] = [7.0]
        [8.0 - 3.0]   [5.0]
```

**Step 2: Calculate Distance**
```python
q = (delta.T @ delta)[0, 0]
```

**Formula**: `q = delta_x² + delta_y²`

**Example**:
```python
q = 7.0² + 5.0² = 49 + 25 = 74
distance = sqrt(74) ≈ 8.60m
```

**Step 3: Calculate Predicted Angle**
```python
z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
```

**Breaking it down**:
- `atan2(5.0, 7.0)`: Absolute angle to landmark ≈ 0.620 rad (35.5°)
- `xEst[2, 0]`: Robot's yaw = 0.5 rad (28.6°)
- `z_angle = 0.620 - 0.5 = 0.120 rad` (6.9° to the right)

**Step 4: Form Predicted Observation**
```python
zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
```

**Result**: `zp = [8.60m, 0.120rad]`

**Meaning**: Based on our current state, we expect to see the landmark at:
- Distance: 8.60m
- Angle: 0.120 rad to the right

---

## Step 3: Calculate Innovation (Residual)

### Purpose
Measure how wrong our prediction was!

### Code
```python
y = (z - zp).T
y[1] = pi_2_pi(y[1])
```

### Detailed Explanation

**Formula**: `y = z_observed - z_predicted`

**Example**:
```python
# Observed: z = [9.0m, 0.15rad]
# Predicted: zp = [8.60m, 0.12rad]

y = [9.0 - 8.60] = [0.40]   (0.4m further than expected)
    [0.15 - 0.12]   [0.03]  (0.03rad more to the right)
```

### What Does Innovation Mean?

**Small Innovation** (y ≈ 0):
- Our prediction was accurate!
- State estimate is good
- Small correction needed

**Large Innovation**:
- Our prediction was wrong
- State estimate needs significant correction
- Could indicate:
  - Wrong data association
  - Large uncertainty
  - Bad observation

### Normalize Angle
```python
y[1] = pi_2_pi(y[1])
```

**Purpose**: Keep angle difference in [-π, π] range

**Why needed**: Angles wrap around (2π = 0)
- If innovation is 3.0 rad, normalize to -0.14 rad (3.0 - 2π)
- Prevents angle from growing unbounded

---

## Step 4: Calculate Innovation Covariance

### Purpose
Quantify how uncertain we are about the innovation!

### Code
```python
H = jacob_h(q, delta, xEst, LMid + 1)
S = H @ PEst @ H.T + Cx[0:2, 0:2]
```

### Detailed Explanation

**Formula**: `S = H @ P @ H.T + R`

**Breaking it down**:
- `H @ P @ H.T`: Projects state uncertainty to observation space
- `R` (or `Cx[0:2, 0:2]`): Observation noise
- **Result**: Total uncertainty of the innovation

### Matrix Dimensions

**H (Observation Jacobian)**:
- Size: [2 × N] where N = state size
- Row 1: How observation (distance) changes w.r.t. each state variable
- Row 2: How observation (angle) changes w.r.t. each state variable

**PEst (State Covariance)**:
- Size: [N × N]
- Uncertainty and correlations in state

**S (Innovation Covariance)**:
- Size: [2 × 2]
- Uncertainty in the innovation
- Diagonal: Uncertainty in distance and angle
- Off-diagonal: Correlation between distance and angle errors

### Example Calculation

**Simplified case** (robot + 1 landmark, state size = 5):

```python
# H is [2 × 5]
H = [[-0.81, -0.58, 0.0, 0.81, 0.58],   # distance w.r.t. [rx, ry, ryaw, lx, ly]
     [0.07, -0.10, -1.0, -0.07, 0.10]]  # angle w.r.t. [rx, ry, ryaw, lx, ly]

# PEst is [5 × 5]
PEst = [[0.5, 0.1, 0.0, 0.2, 0.1],
        [0.1, 0.5, 0.0, 0.1, 0.2],
        [0.0, 0.0, 0.3, 0.0, 0.0],
        [0.2, 0.1, 0.0, 1.0, 0.3],
        [0.1, 0.2, 0.0, 0.3, 1.0]]

# H @ PEst @ H.T
temp = H @ PEst  # [2 × 5]
S_projection = temp @ H.T  # [2 × 2]

# Add observation noise
R = [[0.25, 0.0],    # Cx[0:2, 0:2] = diag([0.5², 0.5²])
     [0.0, 0.25]]

S = S_projection + R  # [2 × 2]
```

**Result S**:
```
S = [0.85  0.12]   (uncertainty in innovation)
    [0.12  0.55]
```

**Interpretation**:
- `S[0,0] = 0.85`: Uncertainty in distance innovation (m²)
- `S[1,1] = 0.55`: Uncertainty in angle innovation (rad²)
- `S[0,1] = 0.12`: Correlation between distance and angle errors

### Observation Jacobian (H) - Deep Dive

The observation Jacobian `H` tells us: **"If I change the state by a small amount, how does the observation change?"**

#### Calculating H

**Step 1: Intermediate Matrix (G)**

```python
sq = math.sqrt(q)
G = np.array([[-sq * delta[0], -sq * delta[1], 0, sq * delta[0], sq * delta[1]],
              [delta[1], -delta[0], -q, -delta[1], delta[0]]])
G = G / q
```

**What G represents**: Partial derivatives of observation w.r.t. [robot_x, robot_y, robot_yaw, lm_x, lm_y]

**Row 1 (Distance)**:
- `∂(distance)/∂(robot_x) = -delta_x / distance`
- `∂(distance)/∂(robot_y) = -delta_y / distance`
- `∂(distance)/∂(robot_yaw) = 0` (yaw doesn't affect distance)
- `∂(distance)/∂(lm_x) = delta_x / distance`
- `∂(distance)/∂(lm_y) = delta_y / distance`

**Row 2 (Angle)**:
- `∂(angle)/∂(robot_x) = delta_y / distance²`
- `∂(angle)/∂(robot_y) = -delta_x / distance²`
- `∂(angle)/∂(robot_yaw) = -1` (direct effect - if robot rotates, angle changes)
- `∂(angle)/∂(lm_x) = -delta_y / distance²`
- `∂(angle)/∂(lm_y) = delta_x / distance²`

**Example**:
```python
# delta = [7.0, 5.0], q = 74, sq = 8.60

G = [[-8.60*7.0, -8.60*5.0, 0, 8.60*7.0, 8.60*5.0],
     [5.0, -7.0, -74, -5.0, 7.0]] / 74

G = [[-0.81, -0.58, 0.0, 0.81, 0.58],
     [0.068, -0.095, -1.0, -0.068, 0.095]]
```

**Step 2: Selection Matrix (F)**

```python
F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))  # Select robot state
F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))  # Select landmark i
F = np.vstack((F1, F2))
```

**What F does**: Selects [robot_x, robot_y, robot_yaw, lm_i_x, lm_i_y] from full state

**Example** (1 landmark, matching landmark 0):
```
F = [1  0  0  0  0]   (select robot_x)
    [0  1  0  0  0]   (select robot_y)
    [0  0  1  0  0]   (select robot_yaw)
    [0  0  0  1  0]   (select lm_0_x)
    [0  0  0  0  1]   (select lm_0_y)
```

**Step 3: Final Jacobian**
```python
H = G @ F
```

**Result**: Projects intermediate Jacobian to full state space
- G: [2 × 5] (w.r.t. robot + one landmark)
- F: [5 × N] (selection matrix)
- H: [2 × N] (w.r.t. full state)

---

## Step 5: Calculate Kalman Gain

### Purpose
Determine how much to trust the observation vs. the prediction!

### Code
```python
K = (PEst @ H.T) @ np.linalg.inv(S)
```

### Detailed Explanation

**Formula**: `K = P @ H.T @ S^(-1)`

**Breaking it down**:
- `P @ H.T`: Cross-covariance between state and observation
- `S^(-1)`: Inverse of innovation covariance
- **Result**: Optimal blending factor (Kalman gain)

### Matrix Dimensions

**K (Kalman Gain)**:
- Size: [N × 2] where N = state size, 2 = observation size
- Each row: How much to correct that state variable
- Each column: Correction from distance and angle observations

### What Does Kalman Gain Mean?

**High K**:
- Trust observation more
- Make large corrections
- Happens when:
  - Observation is very certain (small S)
  - State is very uncertain (large P)

**Low K**:
- Trust prediction more
- Make small corrections
- Happens when:
  - Observation is uncertain (large S)
  - State is certain (small P)

### Example Calculation

```python
# PEst @ H.T
# PEst: [5 × 5], H.T: [5 × 2]
# Result: [5 × 2] - cross-covariance

cross_cov = PEst @ H.T
# cross_cov = [[0.45, -0.12],
#              [0.38, -0.15],
#              [0.00, -0.30],
#              [0.85, -0.08],
#              [0.72, -0.10]]

# S^(-1)
S_inv = np.linalg.inv(S)
# S_inv = [[1.22, -0.27],
#          [-0.27, 1.85]]

# K = cross_cov @ S_inv
K = [[0.45, -0.12],   @  [[1.22, -0.27],   =  [[0.52, -0.23],
     [0.38, -0.15],       [-0.27, 1.85]]       [0.44, -0.28],
     [0.00, -0.30],                             [0.08, -0.56],
     [0.85, -0.08],                             [1.05, -0.13],
     [0.72, -0.10]]                             [0.89, -0.16]]
```

**Interpretation**:
- `K[0, 0] = 0.52`: Robot x correction per meter of distance innovation
- `K[2, 1] = -0.56`: Robot yaw correction per radian of angle innovation
- `K[3, 0] = 1.05`: Landmark x correction per meter of distance innovation

**Key Insight**: Both robot AND landmark states get corrected!

---

## Step 6: Update State

### Purpose
Correct our state estimate based on the innovation!

### Code
```python
xEst = xEst + (K @ y)
```

### Detailed Explanation

**Formula**: `x_new = x_predicted + K @ y`

**Breaking it down**:
- `K @ y`: Correction term (how much to adjust each state variable)
- Adds correction to predicted state
- Updates BOTH robot and landmark positions simultaneously!

### Example Calculation

```python
# Innovation
y = [0.40]   (0.4m further than expected)
    [0.03]   (0.03rad more to the right)

# Kalman gain (from previous example)
K = [[0.52, -0.23],
     [0.44, -0.28],
     [0.08, -0.56],
     [1.05, -0.13],
     [0.89, -0.16]]

# Correction
correction = K @ y
correction = [[0.52, -0.23],   @  [0.40]   =  [0.19]   (robot_x correction)
              [0.44, -0.28],       [0.03]       [0.17]   (robot_y correction)
              [0.08, -0.56],                     [-0.01]  (robot_yaw correction)
              [1.05, -0.13],                     [0.42]   (lm_x correction)
              [0.89, -0.16]]                     [0.35]   (lm_y correction)

# Update state
xEst_before = [5.0,  3.0,  0.5,  12.0,  8.0]
correction   = [0.19, 0.17, -0.01, 0.42,  0.35]
xEst_after   = [5.19, 3.17, 0.49, 12.42, 8.35]
```

### What Gets Updated?

**Robot State**:
- Robot x, y, yaw all get corrected
- Correction depends on innovation and Kalman gain

**Landmark State**:
- Landmark x, y get corrected
- **Key insight**: Observing a landmark updates its position estimate!
- This is how SLAM builds the map

**Why Both?**: 
- If we observe a landmark at a different position than expected:
  - Either the robot is in the wrong place, OR
  - The landmark is in the wrong place, OR
  - Both!
- EKF automatically distributes the correction between robot and landmark based on their uncertainties

### Physical Interpretation

**Scenario**: Robot thinks it's at (5.0, 3.0), landmark at (12.0, 8.0)
**Observation**: Landmark is 0.4m further than expected

**What happened**:
1. **Robot correction**: Moves robot slightly (0.19m in x, 0.17m in y)
2. **Landmark correction**: Moves landmark more (0.42m in x, 0.35m in y)

**Why landmark moves more?**: 
- If landmark uncertainty is higher than robot uncertainty
- Kalman gain is larger for landmark
- More correction applied to landmark

---

## Step 7: Update Covariance

### Purpose
Reduce our uncertainty after making an observation!

### Code
```python
PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst
```

### Detailed Explanation

**Formula**: `P_new = (I - K @ H) @ P_old`

**Breaking it down**:
- `K @ H`: "Information gain" - how much we learned
- `I - K @ H`: Reduction factor
- **Result**: Uncertainty decreases (we're more certain now!)

### Why Does Uncertainty Decrease?

**Before observation**:
- We're uncertain about robot position
- We're uncertain about landmark position
- These uncertainties are correlated

**After observation**:
- Observation provides information
- Reduces uncertainty in both robot and landmark
- Reduces correlation (we know their relative positions better)

### Matrix Dimensions

**I (Identity)**:
- Size: [N × N] where N = state size

**K @ H**:
- K: [N × 2], H: [2 × N]
- K @ H: [N × N]
- Represents how much each state variable affects the observation correction

**I - K @ H**:
- Size: [N × N]
- Reduction factor applied to covariance

### Example Calculation

```python
# K @ H
KH = K @ H
# KH = [[0.42, 0.30, 0.00, -0.42, -0.30],
#       [0.35, 0.25, 0.00, -0.35, -0.25],
#       [0.06, 0.04, 0.56, -0.06, -0.04],
#       [0.85, 0.61, 0.00, 0.15, -0.61],
#       [0.72, 0.51, 0.00, -0.72, 0.49]]

# I - K @ H
I_minus_KH = np.eye(5) - KH
# I_minus_KH = [[0.58, -0.30, 0.00, 0.42, 0.30],
#               [-0.35, 0.75, 0.00, 0.35, 0.25],
#               [-0.06, -0.04, 0.44, 0.06, 0.04],
#               [-0.85, -0.61, 0.00, 0.85, 0.61],
#               [-0.72, -0.51, 0.00, 0.72, 0.51]]

# Update covariance
PEst_new = I_minus_KH @ PEst_old
```

### What Gets Reduced?

**Diagonal Elements** (Variances):
- `PEst[0,0]`: Robot x uncertainty decreases
- `PEst[1,1]`: Robot y uncertainty decreases
- `PEst[3,3]`: Landmark x uncertainty decreases
- `PEst[4,4]`: Landmark y uncertainty decreases

**Off-Diagonal Elements** (Covariances):
- Correlation between robot and landmark changes
- After observation, we know their relative positions better
- Some correlations increase, some decrease

### Physical Interpretation

**Before update**:
```
PEst = [0.5, 0.1, 0.0, 0.2, 0.1]   (robot x very uncertain)
       [0.1, 0.5, 0.0, 0.1, 0.2]   (robot y very uncertain)
       [0.0, 0.0, 0.3, 0.0, 0.0]   (yaw moderately uncertain)
       [0.2, 0.1, 0.0, 1.0, 0.3]   (landmark x very uncertain)
       [0.1, 0.2, 0.0, 0.3, 1.0]   (landmark y very uncertain)
```

**After update**:
```
PEst = [0.29, 0.08, 0.00, 0.17, 0.12]   (robot x less uncertain)
       [0.08, 0.38, 0.00, 0.09, 0.16]   (robot y less uncertain)
       [0.00, 0.00, 0.13, 0.00, 0.00]   (yaw much less uncertain)
       [0.17, 0.09, 0.00, 0.85, 0.27]   (landmark x less uncertain)
       [0.12, 0.16, 0.00, 0.27, 0.51]   (landmark y less uncertain)
```

**Key observations**:
- All diagonal elements decreased (uncertainty reduced)
- Yaw uncertainty decreased significantly (angle observation is informative)
- Landmark uncertainty decreased (we observed it!)

---

## Complete Update Step Example

Let's walk through a complete example with numbers:

### Initial State
```python
# Robot at (5.0, 3.0) with yaw 0.5 rad
# One landmark at (12.0, 8.0)
xEst = [5.0, 3.0, 0.5, 12.0, 8.0]

# Covariance (moderate uncertainty)
PEst = [[0.5, 0.1, 0.0, 0.2, 0.1],
        [0.1, 0.5, 0.0, 0.1, 0.2],
        [0.0, 0.0, 0.3, 0.0, 0.0],
        [0.2, 0.1, 0.0, 1.0, 0.3],
        [0.1, 0.2, 0.0, 0.3, 1.0]]
```

### Observation
```python
# We observe: distance = 9.0m, angle = 0.15 rad
z = [9.0, 0.15]
```

### Step 1: Data Association
```python
# Match to existing landmark 0
min_id = 0
```

### Step 2: Calculate Predicted Observation
```python
# Delta vector
delta = [12.0 - 5.0, 8.0 - 3.0] = [7.0, 5.0]

# Distance
q = 7.0² + 5.0² = 74
distance_predicted = sqrt(74) = 8.60m

# Angle
angle_absolute = atan2(5.0, 7.0) = 0.620 rad
angle_predicted = 0.620 - 0.5 = 0.120 rad

# Predicted observation
zp = [8.60, 0.120]
```

### Step 3: Calculate Innovation
```python
# Innovation
y = [9.0 - 8.60, 0.15 - 0.120] = [0.40, 0.030]
```

### Step 4: Calculate Innovation Covariance
```python
# Observation Jacobian H (simplified)
H = [[-0.81, -0.58, 0.0, 0.81, 0.58],
     [0.068, -0.095, -1.0, -0.068, 0.095]]

# Innovation covariance
S = H @ PEst @ H.T + R
S = [[0.85, 0.12],
     [0.12, 0.55]]
```

### Step 5: Calculate Kalman Gain
```python
# Kalman gain
K = PEst @ H.T @ inv(S)
K = [[0.52, -0.23],
     [0.44, -0.28],
     [0.08, -0.56],
     [1.05, -0.13],
     [0.89, -0.16]]
```

### Step 6: Update State
```python
# Correction
correction = K @ y = [0.19, 0.17, -0.01, 0.42, 0.35]

# Updated state
xEst_new = [5.0, 3.0, 0.5, 12.0, 8.0] + [0.19, 0.17, -0.01, 0.42, 0.35]
xEst_new = [5.19, 3.17, 0.49, 12.42, 8.35]
```

### Step 7: Update Covariance
```python
# Reduction factor
I_minus_KH = I - K @ H

# Updated covariance
PEst_new = (I - K @ H) @ PEst
PEst_new = [[0.29, 0.08, 0.00, 0.17, 0.12],
            [0.08, 0.38, 0.00, 0.09, 0.16],
            [0.00, 0.00, 0.13, 0.00, 0.00],
            [0.17, 0.09, 0.00, 0.85, 0.27],
            [0.12, 0.16, 0.00, 0.27, 0.51]]
```

### Results

**State Updates**:
- Robot moved from (5.0, 3.0) to (5.19, 3.17)
- Robot yaw changed from 0.5 to 0.49 rad
- Landmark moved from (12.0, 8.0) to (12.42, 8.35)

**Uncertainty Reduction**:
- Robot x uncertainty: 0.5 → 0.29 (42% reduction)
- Robot y uncertainty: 0.5 → 0.38 (24% reduction)
- Robot yaw uncertainty: 0.3 → 0.13 (57% reduction!)
- Landmark x uncertainty: 1.0 → 0.85 (15% reduction)
- Landmark y uncertainty: 1.0 → 0.51 (49% reduction)

**Key Insight**: The observation corrected both robot and landmark, and reduced uncertainty in both!

---

## Key Takeaways

1. **Both Update**: Observations update both robot AND landmark positions
2. **Uncertainty Decreases**: Each observation reduces uncertainty
3. **Kalman Gain Balances**: Automatically balances trust between prediction and observation
4. **Correlations Matter**: Off-diagonal elements in P track how uncertainties are related
5. **Jacobians are Critical**: Linearization allows EKF to work with nonlinear models
6. **Data Association is Key**: Wrong association leads to wrong updates
7. **Innovation Tells the Story**: Large innovation = large correction needed

---

## Common Pitfalls

1. **Wrong Data Association**: Matching observation to wrong landmark causes bad updates
2. **Forgetting to Extend State**: New landmarks must be added before updating
3. **Angle Wrapping**: Angles must be normalized to [-π, π]
4. **Matrix Dimensions**: Ensure all matrix multiplications have compatible dimensions
5. **Covariance Not Positive Definite**: Numerical errors can make P invalid (add regularization if needed)

---

## Next Steps

- Practice with different scenarios
- Visualize how state and covariance change
- Experiment with different noise levels
- Try implementing your own version!

