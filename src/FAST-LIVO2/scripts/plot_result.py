#!/usr/bin/env python3
"""
Plot SLAM Evaluation Results

This script creates visualizations of SLAM trajectory and errors:
- 3D trajectory comparison
- 2D top-down view
- Error distribution histogram
- Error over time/distance

Usage:
    python3 plot_results.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to avoid Qt issues
import pandas as pd
import zipfile
import json
import os
from scipy.interpolate import interp1d

# File paths - UPDATE THESE FOR YOUR RUN
GT_FILE = '/home/fibo5/fast_ws/src/FAST-LIVO2/Log/result/ntu_viral/eee_03_gt.txt'
SLAM_FILE = '/home/fibo5/fast_ws/my_eee_03_prism.txt'  # YOUR run
RESULTS_ZIP = '/home/fibo5/fast_ws/my_eee_03_final.zip'  # YOUR results
OUTPUT_DIR = '/home/fibo5/fast_ws/plots_my_eee_03'

os.makedirs(OUTPUT_DIR, exist_ok=True)

# Load trajectories
print("Loading trajectories...")
gt_data = pd.read_csv(GT_FILE, sep=' ', header=None)
slam_data = pd.read_csv(SLAM_FILE, sep=' ', header=None)

gt_pos = gt_data.iloc[:, 1:4].values
slam_pos = slam_data.iloc[:, 1:4].values

print(f"Ground truth: {len(gt_pos)} poses")
print(f"SLAM: {len(slam_pos)} poses")

# Load evo results
print("Loading evo results...")
with zipfile.ZipFile(RESULTS_ZIP, 'r') as z:
    with z.open('stats.json') as f:
        stats = json.load(f)
    error_array = np.load(z.open('error_array.npy'))
    distances = np.load(z.open('distances_from_start.npy'))
    timestamps = np.load(z.open('seconds_from_start.npy'))

print(f"Loaded {len(error_array)} error values")
print(f"RMSE: {stats['rmse']*100:.2f} cm")

# ============================================================================
# Figure 1: 3D Trajectory Comparison
# ============================================================================
print("\nCreating 3D trajectory plot...")
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Subsample for clarity (plot every Nth point)
skip = max(1, len(gt_pos) // 1000)

ax.plot(gt_pos[::skip, 0], gt_pos[::skip, 1], gt_pos[::skip, 2], 
        'b-', linewidth=1.5, alpha=0.7, label='Ground Truth')
ax.plot(slam_pos[::skip, 0], slam_pos[::skip, 1], slam_pos[::skip, 2], 
        'r-', linewidth=1.5, alpha=0.7, label='SLAM')

# Mark start and end
ax.scatter(gt_pos[0, 0], gt_pos[0, 1], gt_pos[0, 2], 
          c='green', s=100, marker='o', label='Start')
ax.scatter(gt_pos[-1, 0], gt_pos[-1, 1], gt_pos[-1, 2], 
          c='red', s=100, marker='s', label='End')

ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_zlabel('Z (m)', fontsize=12)
ax.set_title(f'3D Trajectory Comparison\nRMSE: {stats["rmse"]*100:.2f} cm', 
             fontsize=14, fontweight='bold')
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)

plt.savefig(os.path.join(OUTPUT_DIR, '1_trajectory_3d.png'), dpi=150, bbox_inches='tight')
print(f"Saved: {OUTPUT_DIR}/1_trajectory_3d.png")
plt.close()

# ============================================================================
# Figure 2: Top-Down View (XY plane)
# ============================================================================
print("Creating top-down view...")
fig, ax = plt.subplots(figsize=(12, 10))

ax.plot(gt_pos[:, 0], gt_pos[:, 1], 'b-', linewidth=2, alpha=0.7, label='Ground Truth')
ax.plot(slam_pos[:, 0], slam_pos[:, 1], 'r-', linewidth=2, alpha=0.7, label='SLAM')

# Mark start and end
ax.scatter(gt_pos[0, 0], gt_pos[0, 1], c='green', s=150, marker='o', 
          edgecolors='black', linewidths=2, label='Start', zorder=5)
ax.scatter(gt_pos[-1, 0], gt_pos[-1, 1], c='red', s=150, marker='s', 
          edgecolors='black', linewidths=2, label='End', zorder=5)

ax.set_xlabel('X (m)', fontsize=14)
ax.set_ylabel('Y (m)', fontsize=14)
ax.set_title(f'Top-Down View (XY Plane)\nRMSE: {stats["rmse"]*100:.2f} cm', 
             fontsize=16, fontweight='bold')
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3)
ax.axis('equal')

plt.savefig(os.path.join(OUTPUT_DIR, '2_trajectory_topdown.png'), dpi=150, bbox_inches='tight')
print(f"Saved: {OUTPUT_DIR}/2_trajectory_topdown.png")
plt.close()

# ============================================================================
# Figure 3: Error Distribution Histogram
# ============================================================================
print("Creating error histogram...")
fig, ax = plt.subplots(figsize=(10, 6))

ax.hist(error_array * 100, bins=50, color='steelblue', alpha=0.7, edgecolor='black')

# Add statistics lines
ax.axvline(stats['rmse']*100, color='red', linestyle='--', linewidth=2, 
          label=f'RMSE: {stats["rmse"]*100:.2f} cm')
ax.axvline(stats['mean']*100, color='orange', linestyle='--', linewidth=2, 
          label=f'Mean: {stats["mean"]*100:.2f} cm')
ax.axvline(stats['median']*100, color='green', linestyle='--', linewidth=2, 
          label=f'Median: {stats["median"]*100:.2f} cm')

ax.set_xlabel('Position Error (cm)', fontsize=14)
ax.set_ylabel('Frequency', fontsize=14)
ax.set_title('Error Distribution', fontsize=16, fontweight='bold')
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3, axis='y')

plt.savefig(os.path.join(OUTPUT_DIR, '3_error_histogram.png'), dpi=150, bbox_inches='tight')
print(f"Saved: {OUTPUT_DIR}/3_error_histogram.png")
plt.close()

# ============================================================================
# Figure 4: Error Over Distance
# ============================================================================
print("Creating error over distance plot...")
fig, ax = plt.subplots(figsize=(12, 6))

ax.plot(distances, error_array * 100, 'b-', linewidth=1, alpha=0.7)
ax.axhline(stats['rmse']*100, color='red', linestyle='--', linewidth=2, 
          label=f'RMSE: {stats["rmse"]*100:.2f} cm')
ax.axhline(stats['mean']*100, color='orange', linestyle='--', linewidth=2, 
          label=f'Mean: {stats["mean"]*100:.2f} cm')

ax.set_xlabel('Distance Traveled (m)', fontsize=14)
ax.set_ylabel('Position Error (cm)', fontsize=14)
ax.set_title('Position Error vs Distance Traveled', fontsize=16, fontweight='bold')
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3)

plt.savefig(os.path.join(OUTPUT_DIR, '4_error_vs_distance.png'), dpi=150, bbox_inches='tight')
print(f"Saved: {OUTPUT_DIR}/4_error_vs_distance.png")
plt.close()

# ============================================================================
# Figure 5: Error Over Time
# ============================================================================
print("Creating error over time plot...")
fig, ax = plt.subplots(figsize=(12, 6))

ax.plot(timestamps, error_array * 100, 'b-', linewidth=1, alpha=0.7)
ax.axhline(stats['rmse']*100, color='red', linestyle='--', linewidth=2, 
          label=f'RMSE: {stats["rmse"]*100:.2f} cm')
ax.axhline(stats['mean']*100, color='orange', linestyle='--', linewidth=2, 
          label=f'Mean: {stats["mean"]*100:.2f} cm')

ax.set_xlabel('Time (s)', fontsize=14)
ax.set_ylabel('Position Error (cm)', fontsize=14)
ax.set_title('Position Error vs Time', fontsize=16, fontweight='bold')
ax.legend(fontsize=12)
ax.grid(True, alpha=0.3)

plt.savefig(os.path.join(OUTPUT_DIR, '5_error_vs_time.png'), dpi=150, bbox_inches='tight')
print(f"Saved: {OUTPUT_DIR}/5_error_vs_time.png")
plt.close()

# ============================================================================
# Figure 6: Error Heatmap on Trajectory
# ============================================================================
print("Creating error heatmap on trajectory...")
fig, ax = plt.subplots(figsize=(12, 10))

# Load aligned trajectories from evo results to match error array size
try:
    with zipfile.ZipFile(RESULTS_ZIP, 'r') as z:
        # Try to load the aligned SLAM positions
        # These will match the error array size
        timestamps_synced = np.load(z.open('timestamps.npy'))
    
    # Interpolate SLAM positions to match error array timestamps
    from scipy.interpolate import interp1d
    
    slam_timestamps = slam_data.iloc[:, 0].values
    
    # Create interpolation functions
    interp_x = interp1d(slam_timestamps, slam_pos[:, 0], kind='linear', 
                       bounds_error=False, fill_value='extrapolate')
    interp_y = interp1d(slam_timestamps, slam_pos[:, 1], kind='linear',
                       bounds_error=False, fill_value='extrapolate')
    
    # Interpolate at synced timestamps
    slam_x_synced = interp_x(timestamps_synced)
    slam_y_synced = interp_y(timestamps_synced)
    
    # Now create scatter plot with matching sizes
    scatter = ax.scatter(slam_x_synced, slam_y_synced, 
                        c=error_array * 100, cmap='jet', 
                        s=20, alpha=0.6, edgecolors='none')
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Position Error (cm)', fontsize=12)
    
    # Plot ground truth as reference
    ax.plot(gt_pos[:, 0], gt_pos[:, 1], 'k--', linewidth=1, alpha=0.3, label='Ground Truth')
    
    ax.set_xlabel('X (m)', fontsize=14)
    ax.set_ylabel('Y (m)', fontsize=14)
    ax.set_title(f'Error Heatmap on Trajectory\nRMSE: {stats["rmse"]*100:.2f} cm', 
                 fontsize=16, fontweight='bold')
    ax.legend(fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.savefig(os.path.join(OUTPUT_DIR, '6_error_heatmap.png'), dpi=150, bbox_inches='tight')
    print(f"Saved: {OUTPUT_DIR}/6_error_heatmap.png")
    
except Exception as e:
    print(f"Warning: Could not create error heatmap: {e}")
    print("Skipping heatmap plot...")

plt.close()

# ============================================================================
# Summary
# ============================================================================
print("\n" + "="*80)
print("PLOTTING COMPLETE")
print("="*80)
print(f"\nAll plots saved to: {OUTPUT_DIR}/")
print("\nGenerated plots:")
print("  1. 1_trajectory_3d.png       - 3D trajectory comparison")
print("  2. 2_trajectory_topdown.png  - Top-down view (XY plane)")
print("  3. 3_error_histogram.png     - Error distribution")
print("  4. 4_error_vs_distance.png   - Error over distance traveled")
print("  5. 5_error_vs_time.png       - Error over time")
print("  6. 6_error_heatmap.png       - Error heatmap on trajectory")
print("\nStatistics:")
print(f"  RMSE:   {stats['rmse']*100:.2f} cm")
print(f"  Mean:   {stats['mean']*100:.2f} cm")
print(f"  Median: {stats['median']*100:.2f} cm")
print(f"  Std:    {stats['std']*100:.2f} cm")
print(f"  Min:    {stats['min']*100:.2f} cm")
print(f"  Max:    {stats['max']*100:.2f} cm")
print("\nView plots with:")
print(f"  eog {OUTPUT_DIR}/*.png")
print("="*80)