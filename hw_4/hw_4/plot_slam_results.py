#!/usr/bin/env python3
"""
SLAM Results Visualization Script
Loads saved SLAM data and generates plots showing trajectory and landmark map
"""

import pickle
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Ellipse
import argparse
import os
from pathlib import Path
import math

# Try to import from package, fallback to direct import
try:
    from .ekf_slam_core import ROBOT_STATE_SIZE, LM_SIZE, calc_n_lm, get_landmark_position_from_state
except ImportError:
    from ekf_slam_core import ROBOT_STATE_SIZE, LM_SIZE, calc_n_lm, get_landmark_position_from_state


def load_slam_data(data_file_path):
    """Load SLAM data from pickle file"""
    with open(data_file_path, 'rb') as f:
        data = pickle.load(f)
    return data


def load_ground_truth(gt_file_path):
    """Load ground truth landmarks from YAML file"""
    with open(gt_file_path, 'r') as f:
        gt_data = yaml.safe_load(f)
    
    landmarks = {}
    for lm in gt_data['landmarks']:
        tag_id = lm['tag_id']
        landmarks[tag_id] = {
            'x': lm['x'],
            'y': lm['y']
        }
    return landmarks


def plot_covariance_ellipse(ax, mean, cov, n_std=2.0, **kwargs):
    """
    Plot covariance ellipse
    
    Args:
        ax: matplotlib axes
        mean: (x, y) mean position
        cov: 2x2 covariance matrix
        n_std: number of standard deviations (default 2.0 for 95% confidence)
    """
    # Eigenvalue decomposition
    eigenvals, eigenvecs = np.linalg.eigh(cov)
    
    # Order eigenvalues and eigenvectors
    order = eigenvals.argsort()[::-1]
    eigenvals = eigenvals[order]
    eigenvecs = eigenvecs[:, order]
    
    # Angle of rotation
    angle = np.degrees(np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0]))
    
    # Width and height of ellipse (2 * n_std * sqrt(eigenvalue))
    width = 2 * n_std * np.sqrt(eigenvals[0])
    height = 2 * n_std * np.sqrt(eigenvals[1])
    
    # Create ellipse - ensure edgecolor is set if not provided
    if 'edgecolor' not in kwargs:
        kwargs['edgecolor'] = kwargs.get('facecolor', 'green')
    if 'linewidth' not in kwargs:
        kwargs['linewidth'] = 1.5
    
    # Create ellipse
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle, **kwargs)
    ax.add_patch(ellipse)


def plot_trajectory(ax, trajectory_1hz, waypoint_timestamps):
    """
    Plot robot trajectory with yaw arrows at waypoints
    
    Args:
        ax: matplotlib axes
        trajectory_1hz: List of (timestamp, x, y, yaw) tuples
        waypoint_timestamps: List of (waypoint_idx, timestamp, x, y, yaw) tuples
    """
    if len(trajectory_1hz) == 0:
        print("Warning: No trajectory data to plot")
        return
    
    # Extract trajectory data
    timestamps = [t[0] for t in trajectory_1hz]
    x_coords = [t[1] for t in trajectory_1hz]
    y_coords = [t[2] for t in trajectory_1hz]
    yaws = [t[3] for t in trajectory_1hz]
    
    # Normalize timestamps for color gradient
    if len(timestamps) > 1:
        t_min = min(timestamps)
        t_max = max(timestamps)
        t_range = t_max - t_min if t_max > t_min else 1.0
        colors = [(t - t_min) / t_range for t in timestamps]
    else:
        colors = [0.5]
    
    # Plot trajectory with color gradient
    scatter = ax.scatter(x_coords, y_coords, c=colors, cmap='viridis', 
                         s=20, alpha=0.6, edgecolors='none', zorder=2)
    
    # Plot trajectory line
    ax.plot(x_coords, y_coords, 'b-', alpha=0.3, linewidth=1, zorder=1, label='Trajectory')
    
    # Add start marker
    if len(x_coords) > 0:
        ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, 
               label='Start', zorder=5, markeredgecolor='black', markeredgewidth=1)
    
    # Add end marker
    if len(x_coords) > 0:
        ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, 
               label='End', zorder=5, markeredgecolor='black', markeredgewidth=1)
    
    # Add yaw arrows at waypoints only
    arrow_length = 0.15  # meters
    for wp_idx, wp_t, wp_x, wp_y, wp_yaw in waypoint_timestamps:
        dx = arrow_length * math.cos(wp_yaw)
        dy = arrow_length * math.sin(wp_yaw)
        ax.arrow(wp_x, wp_y, dx, dy, head_width=0.08, head_length=0.06,
                fc='red', ec='red', zorder=4, linewidth=2)
        # Add waypoint number
        ax.text(wp_x + 0.1, wp_y + 0.1, f'WP{wp_idx}', fontsize=8,
               bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
               zorder=6)
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Robot Trajectory (1Hz sampling)', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    ax.set_aspect('equal', adjustable='box')


def plot_map_with_landmarks(ax, slam_data, ground_truth):
    """
    Plot map with estimated landmarks, covariance ellipses, and ground truth
    
    Args:
        ax: matplotlib axes
        slam_data: Dictionary containing SLAM data
        ground_truth: Dictionary mapping tag_id to {'x': x, 'y': y}
    """
    xEst = slam_data['final_state_vector']
    PEst = slam_data['final_covariance']
    tag_id_to_landmark_index = slam_data['tag_id_to_landmark_index']
    
    # Get final robot position
    robot_x = float(xEst[0, 0])
    robot_y = float(xEst[1, 0])
    robot_yaw = float(xEst[2, 0])
    
    # Plot final robot position
    ax.plot(robot_x, robot_y, 'bs', markersize=12, label='Final Robot Pose', 
           zorder=5, markeredgecolor='black', markeredgewidth=1)
    
    # Plot robot orientation arrow
    arrow_length = 0.2
    dx = arrow_length * math.cos(robot_yaw)
    dy = arrow_length * math.sin(robot_yaw)
    ax.arrow(robot_x, robot_y, dx, dy, head_width=0.1, head_length=0.08,
            fc='blue', ec='blue', zorder=4, linewidth=2)
    
    # Plot estimated landmarks with covariance ellipses
    estimated_landmarks = {}
    for tag_id, landmark_idx in tag_id_to_landmark_index.items():
        lm = get_landmark_position_from_state(xEst, landmark_idx)
        lm_x = float(lm[0, 0])
        lm_y = float(lm[1, 0])
        estimated_landmarks[tag_id] = (lm_x, lm_y)
        
        # Get covariance for this landmark
        lm_start_idx = ROBOT_STATE_SIZE + LM_SIZE * landmark_idx
        cov_xx = PEst[lm_start_idx, lm_start_idx]
        cov_yy = PEst[lm_start_idx + 1, lm_start_idx + 1]
        cov_xy = PEst[lm_start_idx, lm_start_idx + 1]
        
        # Create 2x2 covariance matrix
        cov_matrix = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
        
        # Calculate standard deviations and correlation
        std_x = math.sqrt(cov_xx)
        std_y = math.sqrt(cov_yy)
        correlation = cov_xy / (std_x * std_y) if (std_x * std_y) > 0 else 0.0
        
        # Plot covariance ellipse (2σ = 95% confidence)
        plot_covariance_ellipse(ax, (lm_x, lm_y), cov_matrix, n_std=2.0,
                               facecolor='lightgreen', edgecolor='green', linewidth=1.5, alpha=0.4)
        
        # Plot estimated landmark position
        ax.plot(lm_x, lm_y, 'go', markersize=8, zorder=3, 
               markeredgecolor='darkgreen', markeredgewidth=1)
        
        # Add tag ID label with position
        ax.text(lm_x + 0.05, lm_y + 0.05, f'Tag {tag_id}', fontsize=8,
               bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.7),
               zorder=6)
        
        # Add covariance values annotation
        # Position it below and to the right of the landmark
        cov_text = f'σ_x={std_x:.4f}m\nσ_y={std_y:.4f}m\nρ={correlation:.3f}'
        ax.text(lm_x + 0.15, lm_y - 0.15, cov_text, fontsize=7,
               bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8, edgecolor='green'),
               zorder=6, ha='left', va='top')
    
    # Plot ground truth landmarks
    for tag_id, gt_pos in ground_truth.items():
        gt_x = gt_pos['x']
        gt_y = gt_pos['y']
        ax.plot(gt_x, gt_y, 'r*', markersize=12, zorder=3,
               markeredgecolor='darkred', markeredgewidth=0.5)
        # Add ground truth label
        ax.text(gt_x + 0.05, gt_y - 0.1, f'GT{tag_id}', fontsize=7,
               bbox=dict(boxstyle='round,pad=0.2', facecolor='pink', alpha=0.7),
               zorder=6, color='darkred')
    
    # Calculate and display average error
    errors = []
    for tag_id in ground_truth.keys():
        if tag_id in estimated_landmarks:
            est_x, est_y = estimated_landmarks[tag_id]
            gt_x = ground_truth[tag_id]['x']
            gt_y = ground_truth[tag_id]['y']
            error = math.sqrt((est_x - gt_x)**2 + (est_y - gt_y)**2)
            errors.append(error)
    
    if len(errors) > 0:
        avg_error = np.mean(errors)
        max_error = np.max(errors)
        min_error = np.min(errors)
        error_text = f'Average Error: {avg_error:.4f}m\nMax Error: {max_error:.4f}m\nMin Error: {min_error:.4f}m'
        ax.text(0.02, 0.98, error_text, transform=ax.transAxes,
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('SLAM Map: Estimated vs Ground Truth Landmarks', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    ax.set_aspect('equal', adjustable='box')


def main():
    parser = argparse.ArgumentParser(description='Plot SLAM results')
    parser.add_argument('--data-file', type=str, 
                       default=os.path.join(os.path.expanduser('~'), '.ros', 'slam_data.pkl'),
                       help='Path to saved SLAM data pickle file')
    parser.add_argument('--gt-file', type=str,
                       default=None,
                       help='Path to ground truth landmarks YAML file')
    parser.add_argument('--output', type=str,
                       default='slam_results.png',
                       help='Output PNG file path')
    
    args = parser.parse_args()
    
    # Default ground truth file path
    if args.gt_file is None:
        # Try to find it relative to package
        script_dir = Path(__file__).parent
        gt_file = script_dir.parent / 'configs' / 'ground_truth_landmarks.yaml'
        if not gt_file.exists():
            # Try absolute path
            gt_file = Path('/home/ubuntu/ros2_ws/rubikpi_ros2/hw_4/configs/ground_truth_landmarks.yaml')
        args.gt_file = str(gt_file)
    
    # Load data
    print(f"Loading SLAM data from: {args.data_file}")
    slam_data = load_slam_data(args.data_file)
    
    print(f"Loading ground truth from: {args.gt_file}")
    ground_truth = load_ground_truth(args.gt_file)
    
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot trajectory
    plot_trajectory(ax1, slam_data['trajectory_1hz'], slam_data['waypoint_timestamps'])
    
    # Plot map with landmarks
    plot_map_with_landmarks(ax2, slam_data, ground_truth)
    
    # Adjust layout
    plt.tight_layout()
    
    # Save figure
    print(f"Saving plot to: {args.output}")
    plt.savefig(args.output, dpi=300, bbox_inches='tight')
    print(f"Plot saved successfully!")
    
    # Also show plot
    plt.show()


if __name__ == '__main__':
    main()

