# EKF SLAM - Step-by-Step Walkthrough for Homework 3

## Homework Setup

- **Environment**: 10ft × 10ft square
- **Landmarks**: 8 landmarks (2 on each side of the square)
- **Robot**: Starts at unknown pose, no prior knowledge of landmarks
- **Task**: Discover landmarks and estimate both robot trajectory and landmark positions

---

## Initial Setup

### Ground Truth (What Actually Exists - Robot Doesn't Know This!)
```
Landmark positions (in feet):
  LM0: (0, 2)      LM1: (0, 8)      (left side)
  LM2: (2, 10)     LM3: (8, 10)     (top side)
  LM4: (10, 8)     LM5: (10, 2)     (right side)
  LM6: (8, 0)      LM7: (2, 0)      (bottom side)

Robot starting position: (5, 5) facing 0° (north)
```

### Initial EKF State (What Robot Knows)
```
xEst = [5.0, 5.0, 0.0]  (robot x, robot y, robot yaw in radians)
       └─── Only robot state, no landmarks yet!
       
PEst = [[1.0, 0.0, 0.0],   (uncertainty: 1ft in x, y, 0.1 rad in yaw)
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.1]]
        
nLM = 0  (no landmarks discovered yet)
```

---

## Timestep 1: First Movement and Observation

### Step 1.1: Predict Step

**Control Input**:
```
u = [v, ω] = [0.5 ft/s, 0.0 rad/s]  (moving forward, no turning)
DT = 0.1s
```

**Motion Model**:
```
x_predicted = motion_model(xEst, u)

x_new = [5.0 + 0.1*0.5*cos(0.0)]   [5.05]
        [5.0 + 0.1*0.5*sin(0.0)] = [5.00]
        [0.0 + 0.1*0.0]            [0.00]
        
Robot predicted at: (5.05, 5.00) facing 0.0 rad
```

**Update Covariance**:
```
PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx

After predict:
PEst = [[1.1, 0.0, 0.0],   (uncertainty increased slightly)
        [0.0, 1.1, 0.0],
        [0.0, 0.0, 0.12]]
```

### Step 1.2: Observation

**What Robot Sees**:
```
Observing landmark at:
  distance = 7.07 ft
  angle = 0.785 rad (45° to the right)
  
This is actually LM3 at (8, 10)!
```

**Observation Vector**:
```
z = [7.07, 0.785]  (distance in ft, angle in rad)
```

### Step 1.3: Data Association

**Check Existing Landmarks**:
```
nLM = 0  (no landmarks yet)

Since nLM = 0, this MUST be a new landmark!
min_id = 0  (will be landmark index 0)
```

**Decision**: **NEW LANDMARK** - Add to state!

### Step 1.4: Add New Landmark to State

**Calculate Landmark Position**:
```
Robot at: (5.05, 5.00) facing 0.0 rad
Observation: distance = 7.07 ft, angle = 0.785 rad

lm_x = 5.05 + 7.07 * cos(0.0 + 0.785) = 5.05 + 7.07 * 0.707 = 10.05 ft
lm_y = 5.00 + 7.07 * sin(0.0 + 0.785) = 5.00 + 7.07 * 0.707 = 10.00 ft

Estimated landmark position: (10.05, 10.00)
(Ground truth LM3 is at (8, 10), so we're close but not perfect!)
```

**Extend State Vector**:
```
xEst_before = [5.05, 5.00, 0.00]
xEst_after  = [5.05, 5.00, 0.00, 10.05, 10.00]
              └─── Robot ───┘  └─── LM0 ───┘
              
State size: 3 → 5
```

**Extend Covariance Matrix**:
```
PEst_before = [3 × 3]
PEst_after  = [5 × 5]

PEst = [[1.1,  0.0,  0.0,  0.0,  0.0 ],   (robot-robot)
        [0.0,  1.1,  0.0,  0.0,  0.0 ],
        [0.0,  0.0,  0.12, 0.0,  0.0 ],
        [0.0,  0.0,  0.0,  2.0,  0.0 ],   (new landmark, initial uncertainty)
        [0.0,  0.0,  0.0,  0.0,  2.0 ]]
        
New landmark starts with uncertainty = 2.0 ft (moderate uncertainty)
No correlation yet with robot (zeros in off-diagonal)
```

### Step 1.5: Update Step (First Observation of New Landmark)

**Note**: For a brand new landmark, we typically don't update immediately after adding it. 
The landmark position is initialized from the observation, and we'll update it in subsequent observations.

**Final State After Timestep 1**:
```
xEst = [5.05, 5.00, 0.00, 10.05, 10.00]
       Robot at (5.05, 5.00)    LM0 at (10.05, 10.00)
       
PEst = [5 × 5] with moderate uncertainty
nLM = 1
```

---

## Timestep 2: Observe Same Landmark Again

### Step 2.1: Predict Step

**Control Input**:
```
u = [0.5 ft/s, 0.0 rad/s]  (continue forward)
```

**Motion Model**:
```
x_predicted = [5.05 + 0.05, 5.00 + 0.0, 0.0 + 0.0]
            = [5.10, 5.00, 0.00]
            
Robot predicted at: (5.10, 5.00) facing 0.0 rad
```

**Update Covariance** (uncertainty increases):
```
PEst[0:3, 0:3] = increased uncertainty for robot
Landmark uncertainty unchanged (landmarks don't move)
```

### Step 2.2: Observation

**What Robot Sees**:
```
Observing landmark at:
  distance = 6.40 ft  (closer now, as robot moved)
  angle = 0.674 rad (38.6° to the right)
  
This is the SAME landmark (LM3/LM0)!
```

### Step 2.3: Data Association

**Check Existing Landmarks**:

**Landmark 0** (the one we just added):
```
Get landmark position from state:
  lm = xEst[3:5] = [10.05, 10.00]
  
Calculate predicted observation:
  Robot at: (5.10, 5.00) facing 0.0 rad
  Delta = [10.05 - 5.10, 10.00 - 5.00] = [4.95, 5.00]
  q = 4.95² + 5.00² = 49.0
  distance_predicted = sqrt(49.0) = 7.00 ft
  angle_predicted = atan2(5.00, 4.95) - 0.0 = 0.789 rad
  
Predicted: [7.00, 0.789]
Observed:  [6.40, 0.674]
Innovation: [6.40 - 7.00, 0.674 - 0.789] = [-0.60, -0.115]
```

**Calculate Mahalanobis Distance**:
```
Innovation: y = [-0.60, -0.115]
Innovation covariance: S = H @ PEst @ H.T + Q
Mahalanobis distance: d² = y.T @ S^(-1) @ y = 0.85

Threshold: 2.0
0.85 < 2.0 → MATCH! ✓
```

**Decision**: Match to Landmark 0 (min_id = 0)

### Step 2.4: Calculate Innovation

**Innovation**:
```
y = z_observed - z_predicted
  = [6.40, 0.674] - [7.00, 0.789]
  = [-0.60, -0.115]
  
Interpretation:
  - Landmark is 0.60 ft closer than expected
  - Landmark is 0.115 rad more to the left than expected
```

### Step 2.5: Calculate Innovation Covariance

**Observation Jacobian (H)**:
```
H = [2 × 5] matrix
    Row 1: How distance changes w.r.t. [rx, ry, ryaw, lm_x, lm_y]
    Row 2: How angle changes w.r.t. [rx, ry, ryaw, lm_x, lm_y]
    
H = [[-0.707, -0.707, 0.0, 0.707, 0.707],   (distance)
     [0.141, -0.141, -1.0, -0.141, 0.141]]  (angle)
```

**Innovation Covariance**:
```
S = H @ PEst @ H.T + Q
  = [[0.85, 0.12],
     [0.12, 0.35]]
     
Uncertainty in innovation:
  - Distance innovation: ±0.92 ft (sqrt(0.85))
  - Angle innovation: ±0.59 rad (sqrt(0.35))
```

### Step 2.6: Calculate Kalman Gain

**Kalman Gain**:
```
K = PEst @ H.T @ S^(-1)

K = [[0.45, -0.08],   (robot x corrections)
     [0.45, -0.08],   (robot y corrections)
     [0.02, -0.28],   (robot yaw corrections)
     [0.55, -0.05],   (landmark x corrections)
     [0.55, -0.05]]   (landmark y corrections)
     
Interpretation:
  - Robot x/y: 0.45 ft correction per ft of distance innovation
  - Landmark x/y: 0.55 ft correction per ft (more correction for landmark)
  - Robot yaw: -0.28 rad correction per rad of angle innovation
```

### Step 2.7: Update State

**Correction**:
```
correction = K @ y
           = K @ [-0.60, -0.115]
           
correction = [0.45, -0.08]   @  [-0.60]   =  [-0.27]   (robot x)
            [0.45, -0.08]       [-0.115]      [-0.27]   (robot y)
            [0.02, -0.28]                        [0.03]   (robot yaw)
            [0.55, -0.05]                        [-0.33]  (landmark x)
            [0.55, -0.05]                        [-0.33]  (landmark y)
```

**Updated State**:
```
xEst_before = [5.10, 5.00, 0.00, 10.05, 10.00]
correction   = [-0.27, -0.27, 0.03, -0.33, -0.33]
xEst_after   = [4.83, 4.73, 0.03, 9.72, 9.67]

Robot: (5.10, 5.00) → (4.83, 4.73)  (moved back slightly)
Robot yaw: 0.00 → 0.03 rad  (turned slightly right)
Landmark: (10.05, 10.00) → (9.72, 9.67)  (moved closer to ground truth!)
```

**Key Insight**: Both robot and landmark positions were corrected!
- Robot position adjusted based on observation
- Landmark position improved (closer to true position (8, 10))

### Step 2.8: Update Covariance

**Covariance Update**:
```
PEst_new = (I - K @ H) @ PEst_old

Result: Uncertainty decreases!
  
Before:
  P_robot_x = 1.1 ft
  P_lm_x = 2.0 ft
  
After:
  P_robot_x = 0.85 ft  (23% reduction)
  P_lm_x = 1.45 ft     (28% reduction)
  
We're more confident now!
```

**Final State After Timestep 2**:
```
xEst = [4.83, 4.73, 0.03, 9.72, 9.67]
       Robot at (4.83, 4.73)    LM0 at (9.72, 9.67)
       
PEst = Reduced uncertainty
nLM = 1
```

---

## Timestep 3: Discover Second Landmark

### Step 3.1: Predict Step

**Control Input**:
```
u = [0.5 ft/s, 0.1 rad/s]  (moving forward and turning right)
```

**Motion Model**:
```
x_predicted = [4.83 + 0.05*cos(0.03), 4.73 + 0.05*sin(0.03), 0.03 + 0.01]
            = [4.88, 4.73, 0.04]
            
Robot predicted at: (4.88, 4.73) facing 0.04 rad (turned right)
```

### Step 3.2: Observation

**What Robot Sees**:
```
Observing landmark at:
  distance = 5.39 ft
  angle = 1.107 rad (63.4° to the right)
  
This is a DIFFERENT landmark - LM2 at (2, 10)!
```

### Step 3.3: Data Association

**Check Existing Landmarks**:

**Landmark 0**:
```
lm = [9.72, 9.67]
Robot at: (4.88, 4.73) facing 0.04 rad

Delta = [9.72 - 4.88, 9.67 - 4.73] = [4.84, 4.94]
distance_predicted = sqrt(4.84² + 4.94²) = 6.91 ft
angle_predicted = atan2(4.94, 4.84) - 0.04 = 0.789 rad

Observed: [5.39, 1.107]
Predicted: [6.91, 0.789]
Innovation: [-1.52, 0.318]

Mahalanobis distance: 3.8 > 2.0 → NO MATCH
```

**Check Threshold**:
```
min_dist = [3.8, 2.0]  (3.8 for LM0, 2.0 threshold for new)
min_id = 1  (new landmark, since 3.8 > 2.0)
```

**Decision**: **NEW LANDMARK** - Add to state!

### Step 3.4: Add New Landmark to State

**Calculate Landmark Position**:
```
Robot at: (4.88, 4.73) facing 0.04 rad
Observation: distance = 5.39 ft, angle = 1.107 rad

lm_x = 4.88 + 5.39 * cos(0.04 + 1.107) = 4.88 + 5.39 * 0.447 = 7.29 ft
lm_y = 4.73 + 5.39 * sin(0.04 + 1.107) = 4.73 + 5.39 * 0.894 = 9.55 ft

Estimated landmark position: (7.29, 9.55)
(Ground truth LM2 is at (2, 10), so initial estimate is off)
```

**Extend State Vector**:
```
xEst_before = [4.88, 4.73, 0.04, 9.72, 9.67]
xEst_after  = [4.88, 4.73, 0.04, 9.72, 9.67, 7.29, 9.55]
              └─── Robot ───┘  └─── LM0 ───┘  └─── LM1 ───┘
              
State size: 5 → 7
```

**Extend Covariance Matrix**:
```
PEst_before = [5 × 5]
PEst_after  = [7 × 7]

PEst = [[robot-robot    robot-LM0    robot-LM1  ]
        [LM0-robot      LM0-LM0      LM0-LM1    ]
        [LM1-robot      LM1-LM0      LM1-LM1    ]]

New landmark (LM1) starts with:
  - Diagonal: [2.0, 2.0] (initial uncertainty)
  - No correlation with robot or LM0 (zeros)
```

**Final State After Timestep 3**:
```
xEst = [4.88, 4.73, 0.04, 9.72, 9.67, 7.29, 9.55]
       Robot          LM0            LM1
       
nLM = 2
```

---

## Timestep 4: Observe LM1 Again (Update Both)

### Step 4.1: Predict Step

**Control Input**:
```
u = [0.5 ft/s, 0.1 rad/s]  (continue turning)
```

**Motion Model**:
```
x_predicted = [4.88 + 0.05*cos(0.04), 4.73 + 0.05*sin(0.04), 0.04 + 0.01]
            = [4.93, 4.73, 0.05]
            
Robot predicted at: (4.93, 4.73) facing 0.05 rad
```

### Step 4.2: Observation

**What Robot Sees**:
```
Observing landmark at:
  distance = 4.47 ft
  angle = 1.249 rad (71.6° to the right)
  
This is LM1 (the one we just added)!
```

### Step 4.3: Data Association

**Check Landmark 0**:
```
Mahalanobis distance: 4.2 > 2.0 → NO MATCH
```

**Check Landmark 1**:
```
lm = [7.29, 9.55]
Robot at: (4.93, 4.73) facing 0.05 rad

Delta = [7.29 - 4.93, 9.55 - 4.73] = [2.36, 4.82]
distance_predicted = sqrt(2.36² + 4.82²) = 5.37 ft
angle_predicted = atan2(4.82, 2.36) - 0.05 = 1.108 rad

Observed: [4.47, 1.249]
Predicted: [5.37, 1.108]
Innovation: [-0.90, 0.141]

Mahalanobis distance: 1.2 < 2.0 → MATCH! ✓
```

**Decision**: Match to Landmark 1 (min_id = 1)

### Step 4.4: Calculate Innovation

```
y = [4.47, 1.249] - [5.37, 1.108]
  = [-0.90, 0.141]
  
Landmark is 0.90 ft closer and 0.141 rad more to the right than expected
```

### Step 4.5: Calculate Innovation Covariance

**Observation Jacobian (H)**:
```
H = [2 × 7] matrix
    Non-zero elements for robot state and LM1:
    
H = [[-0.441, -0.897, 0.0, 0.0, 0.0, 0.441, 0.897],   (distance)
     [0.904, -0.427, -1.0, 0.0, 0.0, -0.904, 0.427]]  (angle)
```

**Innovation Covariance**:
```
S = H @ PEst @ H.T + Q
  = [[1.2, 0.3],
     [0.3, 0.5]]
```

### Step 4.6: Calculate Kalman Gain

**Kalman Gain**:
```
K = [[0.35, -0.12],   (robot x)
     [0.71, -0.24],   (robot y)
     [0.05, -0.35],   (robot yaw)
     [0.15, -0.05],   (LM0 x) - small correction
     [0.15, -0.05],   (LM0 y) - small correction
     [0.65, -0.08],   (LM1 x) - larger correction
     [0.65, -0.08]]   (LM1 y) - larger correction
     
Key insight: LM1 gets more correction (higher uncertainty)
            LM0 gets less correction (lower uncertainty, further away)
```

### Step 4.7: Update State

**Correction**:
```
correction = K @ [-0.90, 0.141]

correction = [-0.32, -0.64, 0.05, -0.14, -0.14, -0.59, -0.59]
            Robot corrections    LM0 corrections  LM1 corrections
```

**Updated State**:
```
xEst_before = [4.93, 4.73, 0.05, 9.72, 9.67, 7.29, 9.55]
correction   = [-0.32, -0.64, 0.05, -0.14, -0.14, -0.59, -0.59]
xEst_after   = [4.61, 4.09, 0.10, 9.58, 9.53, 6.70, 8.96]

Robot: (4.93, 4.73) → (4.61, 4.09)  (adjusted)
Robot yaw: 0.05 → 0.10 rad
LM0: (9.72, 9.67) → (9.58, 9.53)  (small adjustment)
LM1: (7.29, 9.55) → (6.70, 8.96)  (larger adjustment, getting closer to (2, 10)!)
```

**Key Insight**: 
- LM1 position improved significantly
- LM0 also got a small correction (even though we're observing LM1!)
- Robot position adjusted

### Step 4.8: Update Covariance

```
Uncertainty reduced for:
  - Robot: 0.85 → 0.65 ft (24% reduction)
  - LM1: 2.0 → 1.3 ft (35% reduction)
  - LM0: 1.45 → 1.40 ft (small reduction)
  
Correlations updated:
  - Robot-LM1 correlation increased (we observed their relationship)
  - LM0-LM1 correlation introduced (they're both in the map now)
```

**Final State After Timestep 4**:
```
xEst = [4.61, 4.09, 0.10, 9.58, 9.53, 6.70, 8.96]
       Robot          LM0            LM1
       
nLM = 2
```

---

## Timestep 5: Observe LM0 Again (Refine Estimate)

### Step 5.1: Predict Step

**Control Input**:
```
u = [0.5 ft/s, 0.0 rad/s]  (moving forward, stopped turning)
```

**Motion Model**:
```
x_predicted = [4.61 + 0.05*cos(0.10), 4.09 + 0.05*sin(0.10), 0.10 + 0.0]
            = [4.66, 4.09, 0.10]
```

### Step 5.2: Observation

**What Robot Sees**:
```
Observing landmark at:
  distance = 6.32 ft
  angle = 0.674 rad
  
This is LM0 again!
```

### Step 5.3: Data Association

**Check Landmarks**:
- LM0: Mahalanobis distance = 0.6 < 2.0 → MATCH! ✓
- LM1: Mahalanobis distance = 3.1 > 2.0 → NO MATCH

**Decision**: Match to Landmark 0

### Step 5.4-5.8: Update Steps

**Innovation**:
```
y = [6.32, 0.674] - [6.28, 0.682]
  = [0.04, -0.008]
  
Very small innovation! Our estimate is getting good!
```

**Kalman Gain**:
```
K = [[0.42, -0.06],   (robot x)
     [0.42, -0.06],   (robot y)
     [0.02, -0.25],   (robot yaw)
     [0.58, -0.04],   (LM0 x) - larger correction
     [0.58, -0.04],   (LM0 y) - larger correction
     [0.08, -0.02],   (LM1 x) - small correction
     [0.08, -0.02]]   (LM1 y) - small correction
```

**Updated State**:
```
xEst_after = [4.68, 4.09, 0.10, 9.60, 9.50, 6.71, 8.95]

Robot: (4.66, 4.09) → (4.68, 4.09)  (tiny adjustment)
LM0: (9.58, 9.53) → (9.60, 9.50)  (refined, closer to true (8, 10))
LM1: (6.70, 8.96) → (6.71, 8.95)  (tiny adjustment)
```

**Uncertainty**:
```
LM0 uncertainty: 1.40 → 1.15 ft (18% reduction)
Robot uncertainty: 0.65 → 0.55 ft (15% reduction)
```

---

## Summary: Key Patterns Across Timesteps

### Pattern 1: Landmark Discovery
- **First observation**: Add landmark to state with initial estimate
- **Subsequent observations**: Update both robot and landmark positions
- **Multiple observations**: Refine landmark position over time

### Pattern 2: State Growth
- Timestep 1: State size = 3 (robot only)
- Timestep 2: State size = 5 (robot + 1 landmark)
- Timestep 3: State size = 7 (robot + 2 landmarks)
- Continues growing as more landmarks discovered

### Pattern 3: Uncertainty Evolution
- **Predict step**: Uncertainty increases (motion adds noise)
- **Update step**: Uncertainty decreases (observation provides information)
- **New landmarks**: Start with high uncertainty, reduce over time
- **Old landmarks**: Uncertainty continues to decrease with more observations

### Pattern 4: Corrections
- **High uncertainty landmarks**: Get larger corrections
- **Low uncertainty landmarks**: Get smaller corrections
- **Robot**: Always gets corrected (its uncertainty affects all observations)
- **All landmarks**: Get small corrections even when observing different landmark (through correlations)

### Pattern 5: Data Association
- **First observation of area**: Always new landmark
- **Subsequent observations**: Compare to all existing landmarks
- **Mahalanobis distance**: Accounts for uncertainty (not just Euclidean distance)
- **Threshold**: If all distances > 2.0, create new landmark

---

## Important Notes for Homework

### 1. No Prior Knowledge
- Robot doesn't know there are 8 landmarks
- Must discover landmarks as they're observed
- Landmark count grows dynamically

### 2. Unknown Starting Pose
- Robot starts at unknown position
- Must localize itself while mapping
- Initial uncertainty is high

### 3. Two-Phase Trajectory
- **Phase 1**: Square path (discover landmarks on sides)
- **Phase 2**: Octagon path (refine estimates, discover more landmarks)

### 4. Evaluation
- Compare estimated landmark positions to ground truth
- Compare estimated robot trajectory to actual trajectory
- Analyze uncertainty evolution over time

### 5. Challenges
- **Data association**: Matching observations to correct landmarks
- **Initial estimates**: First observation of landmark may be inaccurate
- **Convergence**: Multiple observations needed for accurate estimates
- **Computational complexity**: State and covariance grow with number of landmarks

---

## Next Steps for Implementation

1. **Initialize EKF**: Start with robot state only
2. **Implement predict step**: Motion model and covariance propagation
3. **Implement data association**: Mahalanobis distance calculation
4. **Implement update step**: Innovation, Kalman gain, state and covariance update
5. **Handle new landmarks**: Extend state and covariance when new landmark detected
6. **Test with simulation**: Verify with known ground truth
7. **Test with real robot**: Use actual sensor data
8. **Evaluate results**: Compare estimates to ground truth

Good luck with your homework! 🚀

