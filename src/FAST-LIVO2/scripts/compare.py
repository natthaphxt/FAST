#!/usr/bin/env python3
"""
NTU-VIRAL SLAM Evaluation Script

This script evaluates your FAST-LIVO2 SLAM output against NTU-VIRAL ground truth.
It handles:
- PRISM coordinate frame transformation
- Time synchronization
- Trajectory alignment
- RMSE calculation

Usage:
    python3 evaluate_my_slam.py

Author: Created for FAST-LIVO2 NTU-VIRAL evaluation
"""

import os
import sys
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d
import subprocess

# ============================================================================
# Configuration
# ============================================================================

# Input files (CHANGE THESE TO YOUR PATHS)
SLAM_OUTPUT = '/home/fibo5/fast_ws/src/FAST-LIVO2/Log/result/eee_01.txt'
GROUND_TRUTH = '/home/fibo5/fast_ws/src/FAST-LIVO2/Log/result/ntu_viral/eee_01_gt.txt'
ALREADY_IN_PRISM = True
# Output files
OUTPUT_DIR = '/home/fibo5/fast_ws/eval_results'
SLAM_PRISM = os.path.join(OUTPUT_DIR, 'my_slam_prism.txt')
SLAM_SYNCED = os.path.join(OUTPUT_DIR, 'my_slam_synced.txt')
GT_SYNCED = os.path.join(OUTPUT_DIR, 'gt_synced.txt')

# PRISM transformation parameters (from NTU-VIRAL dataset)
TRANS_B2PRISM = np.array([-0.293656, -0.012288, -0.273095])

# Evaluation parameters
MAX_TIME_DIFF = 0.01  # Maximum time difference for matching (seconds)
INTERPOLATION = True   # Use interpolation for timestamp matching


# ============================================================================
# Helper Functions
# ============================================================================

def quat_rotate_vector(quat, vec):
    """
    Rotate vector using quaternion(s)
    :param quat: Quaternion array (N x 4, order [x, y, z, w])
    :param vec: 3D vector to rotate
    :return: Rotated vectors (N x 3)
    """
    rot = Rotation.from_quat(quat[:, :4])
    return rot.apply(vec)


def load_tum_trajectory(filename):
    """
    Load trajectory in TUM format
    :param filename: Path to TUM file
    :return: timestamps, positions, quaternions
    """
    if not os.path.exists(filename):
        raise FileNotFoundError(f"File not found: {filename}")
    
    print(f"Loading: {filename}")
    data = pd.read_csv(filename, sep=' ', header=None)
    
    timestamps = data.iloc[:, 0].values
    positions = data.iloc[:, 1:4].values
    quaternions = data.iloc[:, 4:8].values
    
    print(f"  - Loaded {len(timestamps)} poses")
    print(f"  - Time range: {timestamps[0]:.3f} to {timestamps[-1]:.3f} seconds")
    print(f"  - Duration: {timestamps[-1] - timestamps[0]:.3f} seconds")
    
    return timestamps, positions, quaternions


def save_tum_trajectory(filename, timestamps, positions, quaternions):
    """
    Save trajectory in TUM format
    :param filename: Output file path
    :param timestamps: Array of timestamps
    :param positions: Array of positions (N x 3)
    :param quaternions: Array of quaternions (N x 4, x y z w)
    """
    output_data = np.column_stack((timestamps, positions, quaternions))
    np.savetxt(filename, output_data, fmt='%.6f', delimiter=' ')
    print(f"Saved: {filename}")


def transform_to_prism(positions, quaternions):
    """
    Transform trajectory from IMU/Body frame to PRISM coordinate system
    :param positions: Position array (N x 3)
    :param quaternions: Quaternion array (N x 4)
    :return: Transformed positions
    """
    print("Transforming to PRISM coordinate frame...")
    positions_prism = positions + quat_rotate_vector(quaternions, TRANS_B2PRISM)
    
    # Calculate transformation magnitude
    delta = np.linalg.norm(positions_prism - positions, axis=1)
    print(f"  - Average position shift: {np.mean(delta):.4f} m")
    print(f"  - Max position shift: {np.max(delta):.4f} m")
    
    return positions_prism


def synchronize_trajectories(t_ref, pos_ref, quat_ref, 
                             t_test, pos_test, quat_test,
                             max_diff=0.01, use_interpolation=True):
    """
    Synchronize two trajectories by matching timestamps
    
    :param t_ref: Reference timestamps
    :param pos_ref: Reference positions
    :param quat_ref: Reference quaternions
    :param t_test: Test timestamps
    :param pos_test: Test positions
    :param quat_test: Test quaternions
    :param max_diff: Maximum time difference for matching (seconds)
    :param use_interpolation: Use interpolation instead of nearest neighbor
    :return: Synchronized trajectories
    """
    print("\nSynchronizing trajectories...")
    print(f"  Reference trajectory: {len(t_ref)} poses from {t_ref[0]:.3f} to {t_ref[-1]:.3f}s")
    print(f"  Test trajectory: {len(t_test)} poses from {t_test[0]:.3f} to {t_test[-1]:.3f}s")
    
    # Calculate time offset
    time_offset = t_ref[0] - t_test[0]
    print(f"  Time offset: {time_offset:.6f} seconds")
    
    # Apply time offset to test trajectory
    t_test_offset = t_test + time_offset
    
    if use_interpolation:
        print("  Using interpolation for timestamp matching...")
        
        # Create interpolation functions for test trajectory
        interp_x = interp1d(t_test_offset, pos_test[:, 0], kind='linear', 
                           bounds_error=False, fill_value='extrapolate')
        interp_y = interp1d(t_test_offset, pos_test[:, 1], kind='linear',
                           bounds_error=False, fill_value='extrapolate')
        interp_z = interp1d(t_test_offset, pos_test[:, 2], kind='linear',
                           bounds_error=False, fill_value='extrapolate')
        
        # Interpolate quaternions (simplified - should use SLERP for production)
        interp_qx = interp1d(t_test_offset, quat_test[:, 0], kind='linear',
                            bounds_error=False, fill_value='extrapolate')
        interp_qy = interp1d(t_test_offset, quat_test[:, 1], kind='linear',
                            bounds_error=False, fill_value='extrapolate')
        interp_qz = interp1d(t_test_offset, quat_test[:, 2], kind='linear',
                            bounds_error=False, fill_value='extrapolate')
        interp_qw = interp1d(t_test_offset, quat_test[:, 3], kind='linear',
                            bounds_error=False, fill_value='extrapolate')
        
        # Find overlapping time range
        t_start = max(t_ref[0], t_test_offset[0])
        t_end = min(t_ref[-1], t_test_offset[-1])
        
        # Filter reference trajectory to overlap range
        mask = (t_ref >= t_start) & (t_ref <= t_end)
        t_synced = t_ref[mask]
        pos_ref_synced = pos_ref[mask]
        quat_ref_synced = quat_ref[mask]
        
        # Interpolate test trajectory at reference timestamps
        pos_test_synced = np.column_stack([
            interp_x(t_synced),
            interp_y(t_synced),
            interp_z(t_synced)
        ])
        
        quat_test_synced = np.column_stack([
            interp_qx(t_synced),
            interp_qy(t_synced),
            interp_qz(t_synced),
            interp_qw(t_synced)
        ])
        
        # Normalize quaternions
        quat_norms = np.linalg.norm(quat_test_synced, axis=1, keepdims=True)
        quat_test_synced = quat_test_synced / quat_norms
        
    else:
        print("  Using nearest neighbor matching...")
        
        # Match timestamps using nearest neighbor
        synced_indices_ref = []
        synced_indices_test = []
        
        for i, t_r in enumerate(t_ref):
            # Find closest timestamp in test trajectory
            time_diffs = np.abs(t_test_offset - t_r)
            min_diff_idx = np.argmin(time_diffs)
            min_diff = time_diffs[min_diff_idx]
            
            if min_diff < max_diff:
                synced_indices_ref.append(i)
                synced_indices_test.append(min_diff_idx)
        
        t_synced = t_ref[synced_indices_ref]
        pos_ref_synced = pos_ref[synced_indices_ref]
        quat_ref_synced = quat_ref[synced_indices_ref]
        pos_test_synced = pos_test[synced_indices_test]
        quat_test_synced = quat_test[synced_indices_test]
    
    print(f"  Synchronized trajectory: {len(t_synced)} poses")
    print(f"  Coverage: {(len(t_synced) / len(t_ref)) * 100:.1f}% of reference trajectory")
    
    if len(t_synced) < 10:
        print("  WARNING: Very few synchronized poses! Check time offset.")
    
    return t_synced, pos_ref_synced, quat_ref_synced, pos_test_synced, quat_test_synced


def run_evo_evaluation(gt_file, slam_file, output_prefix):
    """
    Run evo_ape evaluation
    :param gt_file: Ground truth TUM file
    :param slam_file: SLAM output TUM file
    :param output_prefix: Prefix for output files
    """
    print("\n" + "="*80)
    print("Running evo_ape evaluation...")
    print("="*80)
    
    result_file = f"{output_prefix}_results.zip"
    
    cmd = [
        'evo_ape', 'tum',
        gt_file, slam_file,
        '--align',
        '--save_results', result_file
    ]
    
    print(f"Command: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(result.stdout)
        
        # Try to extract and show results
        try:
            import json
            import zipfile
            
            with zipfile.ZipFile(result_file, 'r') as z:
                with z.open('stats.json') as f:
                    stats = json.load(f)
                    
            print("\n" + "="*80)
            print("EVALUATION RESULTS")
            print("="*80)
            print(f"RMSE:   {stats['rmse']*100:.2f} cm")
            print(f"Mean:   {stats['mean']*100:.2f} cm")
            print(f"Median: {stats['median']*100:.2f} cm")
            print(f"Std:    {stats['std']*100:.2f} cm")
            print(f"Min:    {stats['min']*100:.2f} cm")
            print(f"Max:    {stats['max']*100:.2f} cm")
            print("="*80)
            
            return stats
            
        except Exception as e:
            print(f"Could not extract results: {e}")
            return None
            
    except subprocess.CalledProcessError as e:
        print(f"Error running evo_ape: {e}")
        print(e.stderr)
        return None
    except FileNotFoundError:
        print("ERROR: evo_ape not found. Install with:")
        print("  pip install evo --upgrade --no-binary evo --break-system-packages")
        return None


# ============================================================================
# Main Evaluation Pipeline
# ============================================================================

def main():
    """Main evaluation pipeline"""
    
    print("="*80)
    print("NTU-VIRAL SLAM EVALUATION SCRIPT")
    print("="*80)
    print()
    
    # Create output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Output directory: {OUTPUT_DIR}\n")
    
    # ========================================================================
    # Step 1: Load trajectories
    # ========================================================================
    print("STEP 1: Loading trajectories")
    print("-" * 80)
    
    try:
        t_slam, pos_slam, quat_slam = load_tum_trajectory(SLAM_OUTPUT)
        t_gt, pos_gt, quat_gt = load_tum_trajectory(GROUND_TRUTH)
    except FileNotFoundError as e:
        print(f"\nERROR: {e}")
        print("\nPlease update the file paths in the script:")
        print(f"  SLAM_OUTPUT = '{SLAM_OUTPUT}'")
        print(f"  GROUND_TRUTH = '{GROUND_TRUTH}'")
        return 1
    
    # ========================================================================
    # Step 2: Transform SLAM output to PRISM frame
    # ========================================================================
    print("\n" + "="*80)
    print("STEP 2: Transform to PRISM coordinate frame")
    print("-" * 80)
    
    pos_slam_prism = transform_to_prism(pos_slam, quat_slam)
    save_tum_trajectory(SLAM_PRISM, t_slam, pos_slam_prism, quat_slam)
    
    # ========================================================================
    # Step 3: Synchronize trajectories
    # ========================================================================
    print("\n" + "="*80)
    print("STEP 3: Synchronize trajectories")
    print("-" * 80)
    
    t_synced, pos_gt_synced, quat_gt_synced, pos_slam_synced, quat_slam_synced = \
        synchronize_trajectories(
            t_gt, pos_gt, quat_gt,
            t_slam, pos_slam_prism, quat_slam,
            max_diff=MAX_TIME_DIFF,
            use_interpolation=INTERPOLATION
        )
    
    if len(t_synced) < 10:
        print("\nERROR: Not enough synchronized poses!")
        print("This usually means:")
        print("  1. Time offset is wrong")
        print("  2. Trajectories don't overlap in time")
        print("  3. SLAM run was incomplete")
        return 1
    
    # Save synchronized trajectories
    save_tum_trajectory(GT_SYNCED, t_synced, pos_gt_synced, quat_gt_synced)
    save_tum_trajectory(SLAM_SYNCED, t_synced, pos_slam_synced, quat_slam_synced)
    
    # ========================================================================
    # Step 4: Calculate basic statistics
    # ========================================================================
    print("\n" + "="*80)
    print("STEP 4: Calculate error statistics")
    print("-" * 80)
    
    # Calculate position errors
    errors = np.linalg.norm(pos_slam_synced - pos_gt_synced, axis=1)
    
    print(f"Position errors (before alignment):")
    print(f"  RMSE:   {np.sqrt(np.mean(errors**2))*100:.2f} cm")
    print(f"  Mean:   {np.mean(errors)*100:.2f} cm")
    print(f"  Median: {np.median(errors)*100:.2f} cm")
    print(f"  Std:    {np.std(errors)*100:.2f} cm")
    print(f"  Min:    {np.min(errors)*100:.2f} cm")
    print(f"  Max:    {np.max(errors)*100:.2f} cm")
    
    # ========================================================================
    # Step 5: Run evo evaluation (with SE3 alignment)
    # ========================================================================
    print("\n" + "="*80)
    print("STEP 5: Run evo_ape evaluation (with SE3 alignment)")
    print("-" * 80)
    
    output_prefix = os.path.join(OUTPUT_DIR, 'evaluation')
    stats = run_evo_evaluation(GT_SYNCED, SLAM_SYNCED, output_prefix)
    
    # ========================================================================
    # Summary
    # ========================================================================
    print("\n" + "="*80)
    print("EVALUATION COMPLETE")
    print("="*80)
    print("\nGenerated files:")
    print(f"  1. {SLAM_PRISM}")
    print(f"     - Your SLAM trajectory in PRISM frame")
    print(f"  2. {SLAM_SYNCED}")
    print(f"     - Synchronized SLAM trajectory")
    print(f"  3. {GT_SYNCED}")
    print(f"     - Synchronized ground truth")
    print(f"  4. {output_prefix}_results.zip")
    print(f"     - Detailed evaluation results")
    
    if stats:
        rmse_cm = stats['rmse'] * 100
        print(f"\nFinal RMSE: {rmse_cm:.2f} cm")
        
        if rmse_cm < 5:
            print("✅ EXCELLENT! Your SLAM is highly accurate.")
        elif rmse_cm < 10:
            print("✅ GOOD! Your SLAM is working well.")
        elif rmse_cm < 50:
            print("⚠️  MODERATE. Check for issues in SLAM processing.")
        else:
            print("❌ POOR. Something is wrong with the evaluation or SLAM run.")
            print("\nPossible issues:")
            print("  - SLAM run was incomplete")
            print("  - Wrong transformation applied")
            print("  - Time synchronization failed")
            print("  - Sensors were not calibrated properly")
    
    print("\n" + "="*80)
    return 0


if __name__ == '__main__':
    sys.exit(main())