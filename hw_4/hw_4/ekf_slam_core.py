"""
EKF SLAM Core Algorithm
Adapted from PythonRobotics EKF SLAM implementation
"""

import math
import numpy as np

# Constants
ROBOT_STATE_SIZE = 3  # State size [x, y, yaw]
LM_SIZE = 2  # Landmark state size [x, y]


def pi_2_pi(angle):
    """
    Normalize angle to [-π, π]
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-π, π]
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def calc_n_lm(x):
    """
    Calculate number of landmarks in state vector
    
    Args:
        x: State vector [x, y, yaw, lm1_x, lm1_y, ...]
        
    Returns:
        Number of landmarks
    """
    n = int((len(x) - ROBOT_STATE_SIZE) / LM_SIZE)
    return n


def motion_model(x, u, dt):
    """
    Motion model for robot
    
    Args:
        x: Current state [x, y, yaw]
        u: Control input [v, yaw_rate]
        dt: Time step
        
    Returns:
        Updated state vector
    """
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
    
    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt]])
    
    x_new = (F @ x) + (B @ u)
    return x_new


def jacob_motion(x, u, dt):
    """
    Compute Jacobian of motion model
    
    Args:
        x: Current state vector [x, y, yaw, ...]
        u: Control input [v, yaw_rate]
        dt: Time step
        
    Returns:
        G: State transition matrix
        Fx: Selection matrix for robot state
    """
    nLM = calc_n_lm(x)
    Fx = np.hstack((np.eye(ROBOT_STATE_SIZE), np.zeros(
        (ROBOT_STATE_SIZE, LM_SIZE * nLM))))
    
    jF = np.array([[0.0, 0.0, -dt * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, dt * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)
    
    G = np.eye(len(x)) + Fx.T @ jF @ Fx
    
    return G, Fx


def get_landmark_position_from_state(x, ind):
    """
    Extract landmark position from state vector
    
    Args:
        x: State vector
        ind: Landmark index
        
    Returns:
        Landmark position [x, y] as 2x1 numpy array
    """
    lm = x[ROBOT_STATE_SIZE + LM_SIZE * ind: ROBOT_STATE_SIZE + LM_SIZE * (ind + 1), :]
    return lm


def calc_landmark_position(x, z):
    """
    Initialize new landmark position from measurement
    
    Args:
        x: Robot state [x, y, yaw] as (3, 1) array
        z: Measurement [range, bearing] as (2, 1) array
        
    Returns:
        Landmark position [x, y] as 2x1 numpy array
    """
    # Ensure z is properly shaped and extract scalars
    if z.shape == (2, 1):
        range_val = float(z[0, 0])
        bearing_val = float(z[1, 0])
    elif z.shape == (2,):
        range_val = float(z[0])
        bearing_val = float(z[1])
    else:
        # Flatten and take first two elements
        z_flat = z.flatten()
        range_val = float(z_flat[0])
        bearing_val = float(z_flat[1])
    
    zp = np.zeros((2, 1))
    zp[0, 0] = x[0, 0] + range_val * math.cos(x[2, 0] + bearing_val)
    zp[1, 0] = x[1, 0] + range_val * math.sin(x[2, 0] + bearing_val)
    
    return zp


def jacob_h(q, delta, x, i):
    """
    Compute measurement model Jacobian
    
    Args:
        q: Squared distance (delta.T @ delta)
        delta: Vector from robot to landmark [dx, dy]
        x: State vector
        i: Landmark index (1-indexed)
        
    Returns:
        H: Measurement Jacobian matrix
    """
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], -sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], -delta[0, 0], -q, -delta[1, 0], delta[0, 0]]])
    
    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))
    
    F = np.vstack((F1, F2))
    
    H = G @ F
    
    return H


def calc_innovation(lm, xEst, PEst, z, LMid, R):
    """
    Compute measurement innovation
    
    Args:
        lm: Landmark position [x, y]
        xEst: Estimated state vector
        PEst: State covariance matrix
        z: Measurement [range, bearing]
        LMid: Landmark index (0-indexed)
        R: Measurement noise covariance
        
    Returns:
        y: Innovation vector
        S: Innovation covariance
        H: Measurement Jacobian
    """
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    # Ensure zp is (2, 1) column vector to match z shape
    zp = np.array([[math.sqrt(q)], [pi_2_pi(z_angle)]])
    
    # Ensure z is (2, 1) column vector
    if z.shape != (2, 1):
        z = z.reshape(2, 1)
    
    y = z - zp
    y[1, 0] = pi_2_pi(y[1, 0])
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + R
    
    return y, S, H


def search_correspond_landmark_id(xAug, PAug, zi, m_dist_th, R):
    """
    Landmark association with Mahalanobis distance
    
    Args:
        xAug: Augmented state vector
        PAug: Augmented covariance matrix
        zi: Measurement [range, bearing]
        m_dist_th: Mahalanobis distance threshold
        R: Measurement noise covariance
        
    Returns:
        min_id: Index of corresponding landmark (or nLM for new landmark)
    """
    nLM = calc_n_lm(xAug)
    
    min_dist = []
    
    for i in range(nLM):
        lm = get_landmark_position_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i, R)
        mahal_dist = float((y.T @ np.linalg.inv(S) @ y)[0,0])
        min_dist.append(mahal_dist)
    
    min_dist.append(m_dist_th)  # new landmark
    
    min_id = min_dist.index(min(min_dist))
    
    return min_id


def ekf_slam(xEst, PEst, u, z, dt, Q, R, Cx, m_dist_th, landmark_indices=None, is_turning=False):
    """
    EKF SLAM main function
    
    Args:
        xEst: Estimated state vector
        PEst: State covariance matrix
        u: Control input [v, yaw_rate]
        z: Measurements array (Nx2) where each row is [range, bearing]
        dt: Time step
        Q: Process noise covariance (2x2)
        R: Measurement noise covariance (2x2)
        Cx: State noise covariance (3x3)
        m_dist_th: Mahalanobis distance threshold
        landmark_indices: Optional list of landmark indices for each measurement.
                         If None, uses Mahalanobis distance for data association.
                         If provided, must have same length as z.
        is_turning: If True, robot is turning in place - don't update position
        
    Returns:
        xEst: Updated state vector
        PEst: Updated covariance matrix
    """
    # Predict
    # Ensure xEst maintains column vector shape
    if len(xEst.shape) != 2 or xEst.shape[1] != 1:
        xEst = xEst.reshape(-1, 1)
    
    G, Fx = jacob_motion(xEst, u, dt)
    robot_state = xEst[0:ROBOT_STATE_SIZE].reshape(ROBOT_STATE_SIZE, 1)
    updated_robot = motion_model(robot_state, u, dt)
    
    # Ensure updated_robot maintains column vector shape
    if updated_robot.shape != (ROBOT_STATE_SIZE, 1):
        updated_robot = updated_robot.reshape(ROBOT_STATE_SIZE, 1)
    
    # Store predicted position for use in measurement updates
    predicted_robot_x = updated_robot[0, 0]
    predicted_robot_y = updated_robot[1, 0]
    
    xEst[0:ROBOT_STATE_SIZE] = updated_robot
    PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx
    
    # Initial covariance for new landmarks (high uncertainty)
    # Use large initial uncertainty for x and y of new landmarks
    initP = np.eye(LM_SIZE) * 0.0009 # set initial variance to 0.0009 --> standard deviation 0.03
    
    # Update
    if z is not None and len(z) > 0:
        for iz in range(len(z)):  # for each observation
            zi = z[iz, :].reshape(2, 1)  # [range, bearing]
            
            # Ensure xEst maintains column vector shape
            if len(xEst.shape) != 2 or xEst.shape[1] != 1:
                xEst = xEst.reshape(-1, 1)
            
            # Data association: use provided landmark index or Mahalanobis distance
            nLM = calc_n_lm(xEst)
            if landmark_indices is not None and iz < len(landmark_indices):
                # Use known landmark index from tag ID
                min_id = landmark_indices[iz]
                # If landmark doesn't exist yet, it's a new landmark
                if min_id >= nLM:
                    min_id = nLM  # Will create new landmark
            else:
                # Use Mahalanobis distance for data association
                min_id = search_correspond_landmark_id(xEst, PEst, zi, m_dist_th, R)
            
            if min_id == nLM:
                # New landmark - extend state and covariance
                # Current state size
                current_size = len(xEst)
                
                # Ensure robot state slice is column vector
                robot_state = xEst[0:ROBOT_STATE_SIZE].reshape(ROBOT_STATE_SIZE, 1)
                new_lm = calc_landmark_position(robot_state, zi)
                
                # Ensure new_lm is column vector (2, 1)
                if new_lm.shape != (2, 1):
                    new_lm = new_lm.reshape(2, 1)
                
                # Ensure xEst is column vector before vstack
                if len(xEst.shape) != 2 or xEst.shape[1] != 1:
                    xEst = xEst.reshape(-1, 1)
                
                # Verify PEst matches current state size before augmentation
                if PEst.shape[0] != current_size or PEst.shape[1] != current_size:
                    print(f"WARNING: Before augmentation, PEst {PEst.shape} != xEst size {current_size}")
                    PEst = np.eye(current_size) * np.trace(PEst) / max(PEst.shape[0], 1)
                
                # Augment state: add new landmark [x, y]
                xAug = np.vstack((xEst, new_lm))
                new_size = len(xAug)  # Should be current_size + 2
                
                # Augment covariance matrix
                # Structure: [PEst    0  ]
                #           [  0   initP]
                top_row = np.hstack((PEst, np.zeros((current_size, LM_SIZE))))
                bottom_row = np.hstack((np.zeros((LM_SIZE, current_size)), initP))
                PAug = np.vstack((top_row, bottom_row))
                
                # Verify augmentation worked correctly
                if PAug.shape[0] != new_size or PAug.shape[1] != new_size:
                    print(f"ERROR: After augmentation, PAug {PAug.shape} != expected ({new_size}, {new_size})")
                    # Fix it
                    PAug = np.eye(new_size) * 0.5
                
                # Update state and covariance
                xEst = xAug
                PEst = PAug
            
            # Ensure xEst and PEst maintain correct shapes and matching sizes
            if len(xEst.shape) != 2 or xEst.shape[1] != 1:
                xEst = xEst.reshape(-1, 1)
            
            state_size = len(xEst)
            
            # CRITICAL: Verify PEst matches xEst size - if not, something went wrong
            if PEst.shape[0] != state_size or PEst.shape[1] != state_size:
                print(f"ERROR: PEst shape {PEst.shape} doesn't match xEst size {state_size} after landmark processing")
                # This is a critical error - reinitialize with proper size
                # Use a large initial covariance for new landmarks
                old_size = min(PEst.shape[0], state_size)
                new_PEst = np.eye(state_size) * 10.0  # Large initial uncertainty
                if old_size > 0:
                    new_PEst[:old_size, :old_size] = PEst[:old_size, :old_size]
                PEst = new_PEst
            
            lm = get_landmark_position_from_state(xEst, min_id)
            y, S, H = calc_innovation(lm, xEst, PEst, zi, min_id, R)
            
            # Ensure y is (2, 1) column vector
            if y.shape != (2, 1):
                y = y.reshape(2, 1)
            
            # Verify H has correct dimensions: (2, state_size)
            if H.shape[0] != 2 or H.shape[1] != state_size:
                print(f"ERROR: H shape {H.shape} doesn't match expected (2, {state_size})")
                # Skip this update if dimensions don't match
                continue
            
            # Verify PEst and H dimensions are compatible
            if PEst.shape[0] != H.shape[1]:
                print(f"ERROR: PEst shape {PEst.shape} incompatible with H shape {H.shape}")
                continue
            
            K = (PEst @ H.T) @ np.linalg.inv(S)
            
            # Verify K dimensions: should be (state_size, 2)
            if K.shape[0] != state_size or K.shape[1] != 2:
                print(f"ERROR: K shape {K.shape} doesn't match expected ({state_size}, 2)")
                continue
            
            # Ensure K @ y produces (state_size, 1) column vector
            update = K @ y
            if len(update.shape) != 2 or update.shape[1] != 1:
                update = update.reshape(-1, 1)
            
            # Verify update has correct size
            if len(update) != state_size:
                print(f"ERROR: update size {len(update)} doesn't match state size {state_size}")
                continue

            # FREEZE LANDMARKS: Only update robot pose (first 3 elements)
            # Zero out landmark updates to keep them fixed at initial positions
            # This prevents landmarks from drifting and causing localization jumps
            if len(update) > ROBOT_STATE_SIZE:
                update[ROBOT_STATE_SIZE:] = 0

            # Apply update
            xEst = xEst + update

            # LANDMARK LOCALIZATION DURING TURNING: ENABLED
            # Previously, position was frozen during turns to prevent drift
            # Now allowing landmark observations to correct position even while turning
            # This enables more accurate localization when AprilTags are visible
            # if is_turning:
            #     # Reset position to predicted position (no change during pure rotation)
            #     # Only allow orientation and landmark updates
            #     xEst[0, 0] = predicted_robot_x
            #     xEst[1, 0] = predicted_robot_y
            
            # Ensure xEst maintains column vector shape after update
            if len(xEst.shape) != 2 or xEst.shape[1] != 1:
                xEst = xEst.reshape(-1, 1)
            
            # Before covariance update, verify all dimensions
            final_state_size = len(xEst)
            if final_state_size != state_size:
                print(f"WARNING: State size changed from {state_size} to {final_state_size}")
                state_size = final_state_size
            
            # CRITICAL: PEst MUST match state_size before covariance update
            if PEst.shape[0] != state_size or PEst.shape[1] != state_size:
                print(f"CRITICAL ERROR: Before covariance update, PEst {PEst.shape} != state size {state_size}")
                # Emergency fix: create identity matrix with proper size
                PEst = np.eye(state_size) * 10.0
            
            # Verify H and K dimensions are correct
            if H.shape != (2, state_size):
                print(f"ERROR: H shape {H.shape} != (2, {state_size})")
                continue
            if K.shape != (state_size, 2):
                print(f"ERROR: K shape {K.shape} != ({state_size}, 2)")
                continue
            
            # Compute covariance update: P = (I - K*H) * P
            # I - K*H should be (state_size, state_size)
            I_KH = np.eye(state_size) - (K @ H)
            
            # Verify I_KH is square
            if I_KH.shape != (state_size, state_size):
                print(f"ERROR: I_KH shape {I_KH.shape} != ({state_size}, {state_size})")
                continue
            
            # Update covariance
            PEst = I_KH @ PEst
            
            # Final verification: PEst should still be (state_size, state_size)
            if PEst.shape != (state_size, state_size):
                print(f"ERROR: After covariance update, PEst {PEst.shape} != ({state_size}, {state_size})")
                PEst = np.eye(state_size) * 10.0
    
    # Normalize yaw angle and ensure column vector shape
    if len(xEst.shape) != 2 or xEst.shape[1] != 1:
        xEst = xEst.reshape(-1, 1)
    xEst[2, 0] = pi_2_pi(xEst[2, 0])
    
    return xEst, PEst

