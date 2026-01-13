#!/usr/bin/env python3
"""
Debug script for CPU counter stitching with multiple resets.
"""

import sys
sys.path.append("/workspace/analysis")

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
from scipy.stats import linregress
from sklearn.cluster import DBSCAN

import rosbag_loader

# Configuration
BAG_PATH = "/workspace/rosbags/2025-12-17-16-02-22.bag"

print("="*70)
print("CPU Counter Stitching Debug Script (FULL DATASET)")
print("="*70)

# Load data
print(f"\n1. Loading {BAG_PATH}...")
data = rosbag_loader.load_bag_topics(BAG_PATH, verbose=False)

print(f"   Bag duration: {data.duration:.2f} s")
print(f"   Total frames: {len(data.radar_velocity)}")
radar_frames = data.radar_velocity

# Extract CPU cycles and ROS times
print("\n2. Extracting CPU cycle data...")
valid_data = []
for idx, frame in enumerate(radar_frames):
    if frame.time_cpu_cycles is not None and len(frame.time_cpu_cycles) > 0:
        valid_data.append((idx, frame.timestamp, frame.time_cpu_cycles[0]))

if len(valid_data) == 0:
    print("   ERROR: No frames with CPU cycles!")
    sys.exit(1)

indices, ros_times, cpu_cycles = map(np.array, zip(*valid_data))

print(f"   Valid frames: {len(valid_data)}")
print(f"   CPU cycle range: [{cpu_cycles.min()/1e9:.3f}, {cpu_cycles.max()/1e9:.3f}] billion")
print(f"   ROS time range: [{ros_times.min():.2f}, {ros_times.max():.2f}] s")

# Detect resets using derivative
print("\n3. Detecting CPU counter resets...")
diffs = np.diff(cpu_cycles.astype(float))
reset_threshold = -1e9  # Drop of more than 1 billion cycles
reset_indices = np.where(diffs < reset_threshold)[0]

print(f"   Reset threshold: {reset_threshold/1e9:.1f} billion cycles")
print(f"   Resets detected: {len(reset_indices)}")
if len(reset_indices) > 0:
    print(f"   Reset locations (frame indices): {reset_indices}")
    for i, idx in enumerate(reset_indices):
        print(f"      Reset {i+1}: frame {idx} -> {idx+1}, "
              f"CPU drop = {diffs[idx]/1e9:.3f}B cycles")

# Cluster analysis to find linear segments
print("\n4. Clustering analysis (DBSCAN on normalized CPU vs ROS)...")

# Normalize data for clustering
cpu_norm = (cpu_cycles - cpu_cycles.mean()) / cpu_cycles.std()
ros_norm = (ros_times - ros_times.mean()) / ros_times.std()
X = np.column_stack([cpu_norm, ros_norm])

# DBSCAN clustering
db = DBSCAN(eps=0.5, min_samples=5)
labels = db.fit_predict(X)

n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
n_noise = list(labels).count(-1)

print(f"   Clusters found: {n_clusters}")
print(f"   Noise points: {n_noise}")

for cluster_id in range(n_clusters):
    cluster_mask = labels == cluster_id
    cluster_size = cluster_mask.sum()
    cluster_cpu_range = [cpu_cycles[cluster_mask].min()/1e9, cpu_cycles[cluster_mask].max()/1e9]
    cluster_ros_range = [ros_times[cluster_mask].min(), ros_times[cluster_mask].max()]
    print(f"   Cluster {cluster_id}: {cluster_size} points, "
          f"CPU=[{cluster_cpu_range[0]:.3f}, {cluster_cpu_range[1]:.3f}]B, "
          f"ROS=[{cluster_ros_range[0]:.2f}, {cluster_ros_range[1]:.2f}]s")

# Visualize before stitching
print("\n5. Visualizing data before stitching...")
fig, axes = plt.subplots(2, 2, figsize=(16, 10))

# Plot 1: CPU vs ROS (colored by cluster)
axes[0, 0].scatter(cpu_cycles/1e9, ros_times, c=labels, cmap='tab10', s=10, alpha=0.7)
axes[0, 0].set_xlabel('CPU Cycles (billions)')
axes[0, 0].set_ylabel('ROS Time (s)')
axes[0, 0].set_title(f'CPU Cycles vs ROS Time (Before Stitching)\n{n_clusters} clusters detected')
axes[0, 0].grid(True, alpha=0.3)

# Plot 2: CPU diff (to show resets)
axes[0, 1].plot(np.arange(len(diffs)), diffs/1e9, 'b-', linewidth=0.5, alpha=0.7)
axes[0, 1].axhline(reset_threshold/1e9, color='r', linestyle='--', linewidth=2, label=f'Reset threshold ({reset_threshold/1e9:.1f}B)')
for idx in reset_indices:
    axes[0, 1].axvline(idx, color='orange', linestyle='--', alpha=0.5)
axes[0, 1].set_xlabel('Frame Index')
axes[0, 1].set_ylabel('CPU Cycle Diff (billions)')
axes[0, 1].set_title(f'CPU Cycle Differences (Resets: {len(reset_indices)})')
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)

# Plot 3: Residuals for each cluster
for cluster_id in range(n_clusters):
    cluster_mask = labels == cluster_id
    if cluster_mask.sum() < 5:
        continue
    
    seg_cpu = cpu_cycles[cluster_mask]
    seg_ros = ros_times[cluster_mask]
    
    slope, intercept, r, _, _ = linregress(seg_cpu, seg_ros)
    residuals = seg_ros - (slope * seg_cpu + intercept)
    
    axes[1, 0].scatter(seg_cpu/1e9, residuals*1000, s=5, alpha=0.5, label=f'Cluster {cluster_id}')

axes[1, 0].axhline(0, color='k', linestyle='--', linewidth=1)
axes[1, 0].set_xlabel('CPU Cycles (billions)')
axes[1, 0].set_ylabel('Residual (ms)')
axes[1, 0].set_title('Linear Fit Residuals per Cluster')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

# Plot 4: Timeline visualization
axes[1, 1].scatter(ros_times, cpu_cycles/1e9, c=labels, cmap='tab10', s=10, alpha=0.7)
axes[1, 1].set_xlabel('ROS Time (s)')
axes[1, 1].set_ylabel('CPU Cycles (billions)')
axes[1, 1].set_title('Timeline View (ROS time is monotonic)')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/workspace/analysis/cpu_stitching_before.png', dpi=150)
print(f"   Saved: cpu_stitching_before.png")

# Apply stitching
print("\n6. Applying stitching function...")
stitched_frames, diagnostics = rosbag_loader.stitch_cpu_counter_resets_improved(
    radar_frames, 
    verbose=True,
    reset_drop_threshold=-1e9
)

print(f"\n   Diagnostics: {diagnostics}")

# Extract stitched data
print("\n7. Extracting stitched data...")
stitched_valid_data = []
for idx, frame in enumerate(stitched_frames):
    if frame.time_cpu_cycles is not None and len(frame.time_cpu_cycles) > 0:
        stitched_valid_data.append((idx, frame.timestamp, frame.time_cpu_cycles[0]))

indices_s, ros_times_s, cpu_cycles_s = map(np.array, zip(*stitched_valid_data))

print(f"   Valid frames: {len(stitched_valid_data)}")
print(f"   CPU cycle range: [{cpu_cycles_s.min()/1e9:.3f}, {cpu_cycles_s.max()/1e9:.3f}] billion")

# Check stitching quality
print("\n8. Checking stitching quality...")
slope, intercept, r, _, _ = linregress(cpu_cycles_s, ros_times_s)
residuals = ros_times_s - (slope * cpu_cycles_s + intercept)

print(f"   Linear fit R²: {r**2:.8f}")
print(f"   Clock frequency: {1.0/slope/1e6:.2f} MHz")
print(f"   Residual std: {np.std(residuals)*1000:.3f} ms")
print(f"   Residual max: {np.max(np.abs(residuals))*1000:.3f} ms")

# Check monotonicity
is_monotonic = np.all(np.diff(cpu_cycles_s) > 0)
print(f"   CPU cycles monotonic: {is_monotonic}")
if not is_monotonic:
    backwards = np.where(np.diff(cpu_cycles_s) <= 0)[0]
    print(f"      ❌ Non-monotonic at indices: {backwards[:10]}")

# Visualize after stitching
print("\n9. Visualizing stitched data...")
fig, axes = plt.subplots(2, 2, figsize=(16, 10))

# Plot 1: Stitched CPU vs ROS
axes[0, 0].scatter(cpu_cycles_s/1e9, ros_times_s, s=10, alpha=0.5, c=np.arange(len(cpu_cycles_s)), cmap='viridis')
axes[0, 0].plot(cpu_cycles_s/1e9, slope * cpu_cycles_s + intercept, 'r-', linewidth=2, alpha=0.7, label=f'Linear fit (R²={r**2:.6f})')
axes[0, 0].set_xlabel('CPU Cycles (billions)')
axes[0, 0].set_ylabel('ROS Time (s)')
axes[0, 0].set_title('CPU Cycles vs ROS Time (After Stitching)')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

# Plot 2: Residuals
axes[0, 1].scatter(cpu_cycles_s/1e9, residuals*1000, s=5, alpha=0.5)
axes[0, 1].axhline(0, color='r', linestyle='--', linewidth=1)
axes[0, 1].set_xlabel('CPU Cycles (billions)')
axes[0, 1].set_ylabel('Residual (ms)')
axes[0, 1].set_title(f'Linear Fit Residuals (σ={np.std(residuals)*1000:.3f} ms)')
axes[0, 1].grid(True, alpha=0.3)

# Plot 3: Residual histogram
axes[1, 0].hist(residuals*1000, bins=50, alpha=0.7, edgecolor='black')
axes[1, 0].axvline(0, color='r', linestyle='--', linewidth=2)
axes[1, 0].set_xlabel('Residual (ms)')
axes[1, 0].set_ylabel('Count')
axes[1, 0].set_title('Residual Distribution')
axes[1, 0].grid(True, alpha=0.3)

# Plot 4: CPU diff after stitching
diffs_s = np.diff(cpu_cycles_s)
axes[1, 1].plot(np.arange(len(diffs_s)), diffs_s/1e6, 'b-', linewidth=0.5, alpha=0.7)
axes[1, 1].set_xlabel('Frame Index')
axes[1, 1].set_ylabel('CPU Cycle Diff (millions)')
axes[1, 1].set_title('CPU Cycle Differences (After Stitching)')
axes[1, 1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/workspace/analysis/cpu_stitching_after.png', dpi=150)
print(f"   Saved: cpu_stitching_after.png")

# Summary
print("\n" + "="*70)
print("SUMMARY")
print("="*70)
print(f"Resets detected:     {len(reset_indices)}")
print(f"Clusters found:      {n_clusters}")
print(f"Stitching R²:        {r**2:.8f}")
print(f"CPU monotonic:       {is_monotonic}")
print(f"Residual std:        {np.std(residuals)*1000:.3f} ms")
print(f"Clock frequency:     {1.0/slope/1e6:.2f} MHz")

if r**2 > 0.999 and is_monotonic:
    print("\n✅ Stitching SUCCESSFUL!")
else:
    print("\n❌ Stitching FAILED - needs debugging")

print("="*70)
