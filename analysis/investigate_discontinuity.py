#!/usr/bin/env python3
"""
Diagnostic script to investigate the two-line pattern in CPU cycles vs ROS time scatter plot.
"""

import sys
sys.path.append("/workspace/analysis")

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import rosbag_loader

# Configuration
BAG_PATH = "/workspace/rosbags/2025-12-17-16-02-22.bag"
START_TIME_OFFSET = 31.5
DURATION = 15.0

print("="*80)
print("DISCONTINUITY INVESTIGATION")
print("="*80)

# Load data
print(f"\nLoading {BAG_PATH}...")
data = rosbag_loader.load_bag_topics(BAG_PATH, verbose=False)

# Filter data
t_start = data.start_time + START_TIME_OFFSET
t_end = t_start + DURATION

data.radar_velocity = [radar for radar in data.radar_velocity 
                       if radar.timestamp >= t_start and radar.timestamp <= t_end]

print(f"Radar frames after filtering: {len(data.radar_velocity)}")

# Extract CPU cycles and ROS timestamps
radar_times_ros = []
radar_cpu_cycles = []

for frame in data.radar_velocity:
    radar_times_ros.append(frame.timestamp)
    
    if frame.time_cpu_cycles is not None and len(frame.time_cpu_cycles) > 0:
        radar_cpu_cycles.append(frame.time_cpu_cycles[0])
    else:
        radar_cpu_cycles.append(0)

radar_times_ros = np.array(radar_times_ros)
radar_cpu_cycles = np.array(radar_cpu_cycles)

# Filter out zero CPU cycles
valid_mask = radar_cpu_cycles > 0
radar_times_ros = radar_times_ros[valid_mask]
radar_cpu_cycles = radar_cpu_cycles[valid_mask]

print(f"Valid frames with CPU cycles: {len(radar_times_ros)}")

# ============================================================================
# INVESTIGATION 1: Check monotonicity
# ============================================================================
print("\n" + "="*80)
print("1. MONOTONICITY CHECK")
print("="*80)

cpu_diffs = np.diff(radar_cpu_cycles)
ros_diffs = np.diff(radar_times_ros)

cpu_monotonic = np.all(cpu_diffs > 0)
ros_monotonic = np.all(ros_diffs > 0)

print(f"CPU cycles monotonic: {cpu_monotonic}")
print(f"ROS times monotonic:  {ros_monotonic}")

if not cpu_monotonic:
    backwards_cpu = np.where(cpu_diffs <= 0)[0]
    print(f"  CPU goes backwards at {len(backwards_cpu)} indices: {backwards_cpu[:10]}")

if not ros_monotonic:
    backwards_ros = np.where(ros_diffs <= 0)[0]
    print(f"  ROS goes backwards at {len(backwards_ros)} indices: {backwards_ros[:10]}")

# ============================================================================
# INVESTIGATION 2: Identify the two segments
# ============================================================================
print("\n" + "="*80)
print("2. SEGMENT IDENTIFICATION")
print("="*80)

# Find large jumps in the relationship between CPU cycles and ROS time
# Compute expected ROS time based on local slope
window = 10
slopes = []
for i in range(window, len(radar_cpu_cycles) - window):
    local_slope = (radar_times_ros[i+window] - radar_times_ros[i-window]) / \
                  (radar_cpu_cycles[i+window] - radar_cpu_cycles[i-window])
    slopes.append(local_slope)

slopes = np.array(slopes)
slope_median = np.median(slopes)

print(f"Median local slope: {slope_median:.10e} s/cycle")
print(f"Corresponds to: {1.0/slope_median/1e6:.2f} MHz clock")

# Look for points that deviate significantly from expected linear relationship
# Fit global linear model
from scipy.stats import linregress
slope_global, intercept_global, r, p, se = linregress(radar_cpu_cycles, radar_times_ros)

print(f"\nGlobal linear fit:")
print(f"  Slope: {slope_global:.10e} s/cycle ({1.0/slope_global/1e6:.2f} MHz)")
print(f"  Intercept: {intercept_global:.4f} s")
print(f"  R²: {r**2:.6f}")

# Compute residuals
expected_ros = slope_global * radar_cpu_cycles + intercept_global
residuals = radar_times_ros - expected_ros

print(f"\nResiduals:")
print(f"  Mean: {np.mean(residuals)*1000:.3f} ms")
print(f"  Std: {np.std(residuals)*1000:.3f} ms")
print(f"  Min: {np.min(residuals)*1000:.3f} ms")
print(f"  Max: {np.max(residuals)*1000:.3f} ms")

# Find outliers (points far from the fit)
residual_threshold = 3 * np.std(residuals)
outlier_mask = np.abs(residuals) > residual_threshold
outlier_indices = np.where(outlier_mask)[0]

print(f"\nOutliers (>{residual_threshold*1000:.1f} ms from fit): {len(outlier_indices)}")

# ============================================================================
# INVESTIGATION 3: Look for distinct clusters
# ============================================================================
print("\n" + "="*80)
print("3. CLUSTER ANALYSIS")
print("="*80)

# Use CPU cycle ranges to identify clusters
cpu_min, cpu_max = radar_cpu_cycles.min(), radar_cpu_cycles.max()
cpu_range = cpu_max - cpu_min
cpu_bins = 20

hist, bin_edges = np.histogram(radar_cpu_cycles, bins=cpu_bins)

print(f"CPU cycle range: [{cpu_min/1e9:.3f}, {cpu_max/1e9:.3f}] billion")
print(f"\nHistogram of CPU cycles (20 bins):")
print(f"{'Bin Range (B)':>25} | Count")
print("-" * 40)
for i in range(len(hist)):
    if hist[i] > 0:
        print(f"[{bin_edges[i]/1e9:6.3f}, {bin_edges[i+1]/1e9:6.3f}] | {hist[i]:>5}")

# Find gaps (bins with zero counts)
gap_mask = hist == 0
gap_indices = np.where(gap_mask)[0]

if len(gap_indices) > 0:
    print(f"\n⚠️ Found {len(gap_indices)} gaps in CPU cycle distribution!")
    print("Gaps at:")
    for idx in gap_indices:
        print(f"  [{bin_edges[idx]/1e9:.3f}, {bin_edges[idx+1]/1e9:.3f}] billion cycles")
    
    # Find the largest gap
    largest_gap_idx = gap_indices[0]
    for i in range(len(gap_indices) - 1):
        if gap_indices[i+1] != gap_indices[i] + 1:
            break
        largest_gap_idx = gap_indices[i+1]
    
    gap_boundary = bin_edges[largest_gap_idx + 1]
    
    print(f"\nUsing gap at {gap_boundary/1e9:.3f}B cycles to split data")
    
    # Split data into two segments
    segment1_mask = radar_cpu_cycles < gap_boundary
    segment2_mask = radar_cpu_cycles >= gap_boundary
    
    seg1_cpu = radar_cpu_cycles[segment1_mask]
    seg1_ros = radar_times_ros[segment1_mask]
    seg2_cpu = radar_cpu_cycles[segment2_mask]
    seg2_ros = radar_times_ros[segment2_mask]
    
    print(f"\nSegment 1 (smaller CPU cycles):")
    print(f"  Points: {len(seg1_cpu)}")
    print(f"  CPU range: [{seg1_cpu.min()/1e9:.3f}, {seg1_cpu.max()/1e9:.3f}] B")
    print(f"  ROS range: [{seg1_ros.min():.3f}, {seg1_ros.max():.3f}] s")
    
    print(f"\nSegment 2 (larger CPU cycles):")
    print(f"  Points: {len(seg2_cpu)}")
    print(f"  CPU range: [{seg2_cpu.min()/1e9:.3f}, {seg2_cpu.max()/1e9:.3f}] B")
    print(f"  ROS range: [{seg2_ros.min():.3f}, {seg2_ros.max():.3f}] s")
    
    # Check if segments have different slopes
    if len(seg1_cpu) > 10 and len(seg2_cpu) > 10:
        slope1, int1, r1, _, _ = linregress(seg1_cpu, seg1_ros)
        slope2, int2, r2, _, _ = linregress(seg2_cpu, seg2_ros)
        
        print(f"\nSegment 1 fit:")
        print(f"  Slope: {slope1:.10e} s/cycle ({1.0/slope1/1e6:.2f} MHz)")
        print(f"  Intercept: {int1:.4f} s")
        print(f"  R²: {r1**2:.6f}")
        
        print(f"\nSegment 2 fit:")
        print(f"  Slope: {slope2:.10e} s/cycle ({1.0/slope2/1e6:.2f} MHz)")
        print(f"  Intercept: {int2:.4f} s")
        print(f"  R²: {r2**2:.6f}")
        
        print(f"\nSlope difference: {(slope2-slope1)/slope1*100:.4f}%")
        
        # ========================================================================
        # STITCHING TEST: Add offset to later segment to make CPU cycles continuous
        # ========================================================================
        print(f"\n" + "="*80)
        print("STITCHING TEST: Reconstruct Continuous CPU Timeline")
        print("="*80)
        
        # Determine which segment is chronologically first based on ROS time
        seg1_ros_start = seg1_ros[0]
        seg2_ros_start = seg2_ros[0]
        
        if seg2_ros_start < seg1_ros_start:
            # Segment 2 comes first chronologically
            first_seg_cpu = seg2_cpu
            first_seg_ros = seg2_ros
            second_seg_cpu = seg1_cpu
            second_seg_ros = seg1_ros
            first_seg_name = "Segment 2"
            second_seg_name = "Segment 1"
        else:
            # Segment 1 comes first chronologically
            first_seg_cpu = seg1_cpu
            first_seg_ros = seg1_ros
            second_seg_cpu = seg2_cpu
            second_seg_ros = seg2_ros
            first_seg_name = "Segment 1"
            second_seg_name = "Segment 2"
        
        print(f"\nChronological order:")
        print(f"  First:  {first_seg_name} (ROS time {first_seg_ros[0]:.3f} - {first_seg_ros[-1]:.3f}s)")
        print(f"  Second: {second_seg_name} (ROS time {second_seg_ros[0]:.3f} - {second_seg_ros[-1]:.3f}s)")
        
        # Calculate offset: make second segment continue from where first segment ended
        # Add a small gap based on ROS time difference to account for the time gap
        cpu_offset = first_seg_cpu[-1]
        ros_gap = second_seg_ros[0] - first_seg_ros[-1]
        
        # Estimate additional CPU cycles during the gap using the first segment's slope
        # (first segment has more data points and represents the longer time period)
        first_slope = slope1 if seg2_ros_start < seg1_ros_start else slope2
        estimated_gap_cycles = ros_gap / first_slope
        
        print(f"\nStitching parameters:")
        print(f"  Last CPU cycle of first segment: {first_seg_cpu[-1]/1e9:.3f}B")
        print(f"  First CPU cycle of second segment: {second_seg_cpu[0]/1e9:.3f}B")
        print(f"  ROS time gap: {ros_gap*1000:.3f} ms")
        print(f"  Estimated CPU cycles during gap: {estimated_gap_cycles/1e9:.3f}B")
        print(f"  Offset to apply: {(cpu_offset + estimated_gap_cycles)/1e9:.3f}B")
        
        # Apply offset to second segment
        # Calculate the total offset: continue from where first segment ended, plus the time gap
        total_offset = cpu_offset + estimated_gap_cycles
        
        print(f"\n  Offset calculation:")
        print(f"    cpu_offset = {cpu_offset/1e9:.6f}B (last CPU of first segment)")
        print(f"    estimated_gap_cycles = {estimated_gap_cycles/1e9:.6f}B (gap duration)")
        print(f"    Total offset = {total_offset/1e9:.6f}B")
        print(f"    second_seg_cpu[0] = {second_seg_cpu[0]/1e9:.6f}B (before correction)")
        print(f"    Expected after correction: {(second_seg_cpu[0] + total_offset)/1e9:.6f}B")
        
        # Apply the offset
        second_seg_cpu_corrected = second_seg_cpu.copy() + total_offset
        
        print(f"\n  Second segment after correction:")
        print(f"    second_seg_cpu_corrected[0] = {second_seg_cpu_corrected[0]/1e9:.6f}B")
        print(f"    second_seg_cpu_corrected[-1] = {second_seg_cpu_corrected[-1]/1e9:.6f}B")
        print(f"    First 3 values (B): {second_seg_cpu_corrected[:3]/1e9}")
        print(f"    Last 3 values (B): {second_seg_cpu_corrected[-3:]/1e9}")
        
        # Verify the stitching makes sense
        gap_in_cpu = second_seg_cpu_corrected[0] - first_seg_cpu[-1]
        gap_in_time = second_seg_ros[0] - first_seg_ros[-1]
        effective_freq = gap_in_cpu / gap_in_time if gap_in_time > 0 else 0
        
        print(f"\n  Stitching verification:")
        print(f"    Gap in CPU cycles: {gap_in_cpu/1e9:.6f}B")
        print(f"    Gap in ROS time: {gap_in_time*1000:.3f} ms")
        print(f"    Implied frequency: {effective_freq/1e6:.2f} MHz")
        
        # Combine segments in chronological order
        cpu_cycles_stitched = np.concatenate([first_seg_cpu, second_seg_cpu_corrected])
        ros_times_stitched = np.concatenate([first_seg_ros, second_seg_ros])
        
        print(f"\nStitched data (before sorting):")
        print(f"  Total points: {len(cpu_cycles_stitched)}")
        print(f"  CPU range: [{cpu_cycles_stitched[0]/1e9:.3f}, {cpu_cycles_stitched[-1]/1e9:.3f}] B")
        print(f"  ROS range: [{ros_times_stitched[0]:.3f}, {ros_times_stitched[-1]:.3f}] s")
        print(f"  Duration: {ros_times_stitched[-1] - ros_times_stitched[0]:.3f} s")
        
        # IMPORTANT: The original data was sorted by CPU cycles, not chronologically!
        # We need to re-sort by ROS time to get chronological order
        sort_indices = np.argsort(ros_times_stitched)
        cpu_cycles_stitched = cpu_cycles_stitched[sort_indices]
        ros_times_stitched = ros_times_stitched[sort_indices]
        
        print(f"\nStitched data (after sorting by ROS time):")
        print(f"  Total points: {len(cpu_cycles_stitched)}")
        print(f"  CPU range: [{cpu_cycles_stitched[0]/1e9:.3f}, {cpu_cycles_stitched[-1]/1e9:.3f}] B")
        print(f"  ROS range: [{ros_times_stitched[0]:.3f}, {ros_times_stitched[-1]:.3f}] s")
        print(f"  Duration: {ros_times_stitched[-1] - ros_times_stitched[0]:.3f} s")
        
        # Check monotonicity
        cpu_stitched_monotonic = np.all(np.diff(cpu_cycles_stitched) > 0)
        ros_stitched_monotonic = np.all(np.diff(ros_times_stitched) > 0)
        
        print(f"\nMonotonicity check after stitching:")
        print(f"  CPU cycles monotonic: {cpu_stitched_monotonic}")
        print(f"  ROS times monotonic:  {ros_stitched_monotonic}")
        
        # Fit stitched data
        slope_stitched, int_stitched, r_stitched, _, _ = linregress(cpu_cycles_stitched, ros_times_stitched)
        
        print(f"\nStitched data fit:")
        print(f"  Slope: {slope_stitched:.10e} s/cycle ({1.0/slope_stitched/1e6:.2f} MHz)")
        print(f"  Intercept: {int_stitched:.4f} s")
        print(f"  R²: {r_stitched**2:.6f}")
        
        # Calculate residuals for stitched data
        expected_stitched = slope_stitched * cpu_cycles_stitched + int_stitched
        residuals_stitched = ros_times_stitched - expected_stitched
        
        print(f"\nStitched fit quality:")
        print(f"  Residual std: {np.std(residuals_stitched)*1000:.3f} ms")
        print(f"  Residual max: {np.max(np.abs(residuals_stitched))*1000:.3f} ms")
        
        # Compare with original segment fits
        print(f"\nComparison:")
        print(f"  First segment R²:  {(r1 if seg2_ros_start < seg1_ros_start else r2)**2:.6f}")
        print(f"  Second segment R²: {(r2 if seg2_ros_start < seg1_ros_start else r1)**2:.6f}")
        print(f"  Stitched R²:       {r_stitched**2:.6f}")
        
        if r_stitched**2 > 0.999:
            print(f"\n✅ Stitching successful! Linear fit quality maintained.")
        else:
            print(f"\n⚠️ Stitching degraded fit quality. May need adjustment.")
        
else:
    print("\n✅ No significant gaps found - data appears continuous")
    segment1_mask = None
    segment2_mask = None
    cpu_cycles_stitched = None
    ros_times_stitched = None

# ============================================================================
# INVESTIGATION 4: Array index analysis
# ============================================================================
print("\n" + "="*80)
print("4. ARRAY INDEX ANALYSIS")
print("="*80)

# Check how array index relates to CPU cycles and ROS time
indices = np.arange(len(radar_cpu_cycles))

print(f"First 10 entries:")
print(f"{'Index':>6} | {'CPU (B)':>10} | {'ROS (s)':>10} | {'CPU Diff':>12} | {'ROS Diff (ms)':>15}")
print("-" * 70)
for i in range(min(10, len(radar_cpu_cycles))):
    cpu_diff = cpu_diffs[i-1] if i > 0 else 0
    ros_diff = ros_diffs[i-1] if i > 0 else 0
    print(f"{i:6} | {radar_cpu_cycles[i]/1e9:10.3f} | {radar_times_ros[i]:10.3f} | "
          f"{cpu_diff/1e9:12.6f} | {ros_diff*1000:15.3f}")

print(f"\nLast 10 entries:")
print(f"{'Index':>6} | {'CPU (B)':>10} | {'ROS (s)':>10} | {'CPU Diff':>12} | {'ROS Diff (ms)':>15}")
print("-" * 70)
for i in range(max(0, len(radar_cpu_cycles)-10), len(radar_cpu_cycles)):
    cpu_diff = cpu_diffs[i-1] if i > 0 else 0
    ros_diff = ros_diffs[i-1] if i > 0 else 0
    print(f"{i:6} | {radar_cpu_cycles[i]/1e9:10.3f} | {radar_times_ros[i]:10.3f} | "
          f"{cpu_diff/1e9:12.6f} | {ros_diff*1000:15.3f}")

# ============================================================================
# VISUALIZATION
# ============================================================================
print("\n" + "="*80)
print("5. GENERATING DIAGNOSTIC PLOTS")
print("="*80)

fig, axes = plt.subplots(2, 3, figsize=(20, 12))

# Plot 1: Scatter with color = index (original data)
scatter = axes[0, 0].scatter(radar_cpu_cycles/1e9, radar_times_ros, 
                             c=indices, s=10, alpha=0.6, cmap='viridis')
axes[0, 0].set_xlabel('CPU Cycles (billions)')
axes[0, 0].set_ylabel('ROS Time (s)')
axes[0, 0].set_title('Original: CPU Cycles vs ROS Time\n(color = array index)')
axes[0, 0].grid(True, alpha=0.3)
plt.colorbar(scatter, ax=axes[0, 0], label='Array Index')

# Plot 2: Scatter with segments highlighted (original data)
if segment1_mask is not None:
    axes[0, 1].scatter(seg1_cpu/1e9, seg1_ros, s=10, alpha=0.6, label=f'Seg 1 (n={len(seg1_cpu)})', color='red')
    axes[0, 1].scatter(seg2_cpu/1e9, seg2_ros, s=10, alpha=0.6, label=f'Seg 2 (n={len(seg2_cpu)})', color='blue')
    axes[0, 1].legend()
else:
    axes[0, 1].scatter(radar_cpu_cycles/1e9, radar_times_ros, s=10, alpha=0.6)
axes[0, 1].set_xlabel('CPU Cycles (billions)')
axes[0, 1].set_ylabel('ROS Time (s)')
axes[0, 1].set_title('Original: Segments Identified')
axes[0, 1].grid(True, alpha=0.3)

# Plot 3: Stitched data (if available)
if cpu_cycles_stitched is not None:
    axes[0, 2].scatter(cpu_cycles_stitched/1e9, ros_times_stitched, 
                      s=10, alpha=0.6, c=np.arange(len(cpu_cycles_stitched)), cmap='plasma')
    axes[0, 2].plot(cpu_cycles_stitched/1e9, 
                   slope_stitched * cpu_cycles_stitched + int_stitched,
                   'r-', linewidth=2, alpha=0.7, label=f'Fit (R²={r_stitched**2:.6f})')
    axes[0, 2].set_xlabel('CPU Cycles (billions)')
    axes[0, 2].set_ylabel('ROS Time (s)')
    axes[0, 2].set_title('Stitched: Continuous CPU Timeline\n(color = chronological order)')
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)
else:
    axes[0, 2].text(0.5, 0.5, 'No stitching needed', ha='center', va='center', transform=axes[0, 2].transAxes)
    axes[0, 2].set_title('Stitched Data (N/A)')

# Plot 4: Index vs CPU cycles (original)
axes[1, 0].plot(indices, radar_cpu_cycles/1e9, 'o-', markersize=3, linewidth=0.5, alpha=0.7)
axes[1, 0].set_xlabel('Array Index')
axes[1, 0].set_ylabel('CPU Cycles (billions)')
axes[1, 0].set_title('Original: CPU Cycles vs Array Index')
axes[1, 0].grid(True, alpha=0.3)

# Plot 5: Index vs ROS time (original)
axes[1, 1].plot(indices, radar_times_ros, 'o-', markersize=3, linewidth=0.5, alpha=0.7)
axes[1, 1].set_xlabel('Array Index')
axes[1, 1].set_ylabel('ROS Time (s)')
axes[1, 1].set_title('Original: ROS Time vs Array Index')
axes[1, 1].grid(True, alpha=0.3)

# Plot 6: Residuals comparison
if cpu_cycles_stitched is not None:
    axes[1, 2].scatter(cpu_cycles_stitched/1e9, residuals_stitched * 1000, 
                      s=10, alpha=0.5, color='green')
    axes[1, 2].axhline(0, color='r', linestyle='--', linewidth=2)
    axes[1, 2].set_xlabel('CPU Cycles (billions)')
    axes[1, 2].set_ylabel('Residual (ms)')
    axes[1, 2].set_title(f'Stitched Fit Residuals\n(σ={np.std(residuals_stitched)*1000:.3f} ms)')
    axes[1, 2].grid(True, alpha=0.3)
else:
    axes[1, 2].text(0.5, 0.5, 'No residuals to show', ha='center', va='center', transform=axes[1, 2].transAxes)
    axes[1, 2].set_title('Residuals (N/A)')

plt.tight_layout()
plt.savefig('/workspace/analysis/discontinuity_investigation.png', dpi=150)
print(f"✅ Saved plot to /workspace/analysis/discontinuity_investigation.png")

print("\n" + "="*80)
print("INVESTIGATION COMPLETE")
print("="*80)
