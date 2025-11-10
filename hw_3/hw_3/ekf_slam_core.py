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
        x: Robot state [x, y, yaw]
        z: Measurement [range, bearing]
        
    Returns:
        Landmark position [x, y] as 2x1 numpy array
    """
    zp = np.zeros((2, 1))
    
    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
    
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
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])
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
        min_dist.append(y.T @ np.linalg.inv(S) @ y)
    
    min_dist.append(m_dist_th)  # new landmark
    
    min_id = min_dist.index(min(min_dist))
    
    return min_id


def ekf_slam(xEst, PEst, u, z, dt, Q, R, Cx, m_dist_th):
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
        
    Returns:
        xEst: Updated state vector
        PEst: Updated covariance matrix
    """
    # Predict
    G, Fx = jacob_motion(xEst, u, dt)
    xEst[0:ROBOT_STATE_SIZE] = motion_model(xEst[0:ROBOT_STATE_SIZE], u, dt)
    PEst = G.T @ PEst @ G + Fx.T @ Cx @ Fx
    initP = np.eye(2)
    
    # Update
    if z is not None and len(z) > 0:
        for iz in range(len(z)):  # for each observation
            zi = z[iz, :].reshape(2, 1)  # [range, bearing]
            min_id = search_correspond_landmark_id(xEst, PEst, zi, m_dist_th, R)
            
            nLM = calc_n_lm(xEst)
            if min_id == nLM:
                # New landmark - extend state and covariance
                xAug = np.vstack((xEst, calc_landmark_position(xEst[0:ROBOT_STATE_SIZE], zi)))
                PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                                  np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
                xEst = xAug
                PEst = PAug
            
            lm = get_landmark_position_from_state(xEst, min_id)
            y, S, H = calc_innovation(lm, xEst, PEst, zi, min_id, R)
            
            K = (PEst @ H.T) @ np.linalg.inv(S)
            xEst = xEst + (K @ y)
            PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst
    
    xEst[2] = pi_2_pi(xEst[2])
    
    return xEst, PEst

