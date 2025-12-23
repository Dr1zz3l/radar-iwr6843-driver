#!/usr/bin/env python3
"""
Quick test: Compare TI's filtered point cloud vs our raw data.
Tests if TI's angle filtering improves ego-velocity estimation.
"""

import sys
sys.path.append("/workspace/analysis")

import numpy as np
from scipy.signal import butter, filtfilt
from scipy import stats
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import rosbag_loader

# Configuration (same as notebook)
BAG_PATH = "/workspace/rosbags/2025-12-17-16-02-22.bag"
START_TIME_OFFSET = 31.5
DURATION = 15.0

MIN_INTENSITY = 5.0
MIN_RANGE = 0.2
MIN_POINTS = 5

IMU_HIGHPASS_CUTOFF = 0.2
RADAR_LOWPASS_CUTOFF = 3.0


def solve_ego_velocity_weighted(positions, velocities, intensities):
    """Weighted least squares ego-velocity estimation."""
    H = []
    z = []
    weights = []
    
    for i in range(len(positions)):
        x, y, z_coord = positions[i]
        v_rad = velocities[i]
        intensity = intensities[i]
        
        r = np.sqrt(x**2 + y**2 + z_coord**2)
        
        if intensity < MIN_INTENSITY or r < MIN_RANGE:
            continue
        
        dir_vec = np.array([x/r, y/r, z_coord/r])
        H.append(dir_vec)
        z.append(v_rad)
        weights.append(intensity)
    
    if len(z) < MIN_POINTS:
        return None
    
    H = np.array(H)
    z = np.array(z)
    W = np.diag(weights)
    
    try:
        lhs = H.T @ W @ H
        rhs = H.T @ W @ z
        v_body = np.linalg.solve(lhs, rhs)
        return v_body
    except np.linalg.LinAlgError:
        return None


def load_ti_filtered_data(bag_path, t_start, t_end):
    """Load TI's filtered point cloud from /ti_mmwave/radar_scan_pcl_0."""
    print("Loading TI filtered radar data...")
    
    bag = rosbag.Bag(bag_path)
    
    radar_times = []
    radar_vx = []
    radar_vy = []
    radar_vz = []
    
    for topic, msg, t in bag.read_messages(topics=['/ti_mmwave/radar_scan_pcl_0']):
        timestamp = msg.header.stamp.to_sec()
        
        if timestamp < t_start or timestamp > t_end:
            continue
        
        # Parse PointCloud2
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "velocity"), skip_nans=True))
        
        if len(points) == 0:
            continue
        
        positions = [(p[0], p[1], p[2]) for p in points]
        velocities = [p[4] for p in points]
        intensities = [p[3] for p in points]
        
        v_body = solve_ego_velocity_weighted(positions, velocities, intensities)
        
        if v_body is not None:
            radar_times.append(timestamp)
            radar_vx.append(v_body[0])
            radar_vy.append(v_body[1])
            radar_vz.append(v_body[2])
    
    bag.close()
    
    return (
        np.array(radar_times),
        np.array(radar_vx),
        np.array(radar_vy),
        np.array(radar_vz)
    )


def process_imu_data(imu_data):
    """Process IMU data: integrate and filter."""
    imu_times = np.array([imu.timestamp for imu in imu_data])
    imu_ax = np.array([imu.linear_acceleration[0] for imu in imu_data])
    
    # Remove bias
    ax_bias = np.mean(imu_ax)
    imu_ax_unbiased = imu_ax - ax_bias
    
    # Integrate
    dt_imu = np.diff(imu_times)
    dt_imu = np.insert(dt_imu, 0, dt_imu[0])
    imu_vx_raw = np.cumsum(imu_ax_unbiased * dt_imu)
    
    # High-pass filter
    fs_imu = 1.0 / np.mean(dt_imu)
    b_hp, a_hp = butter(2, IMU_HIGHPASS_CUTOFF, btype='high', fs=fs_imu)
    imu_vx_filt = filtfilt(b_hp, a_hp, imu_vx_raw)
    
    return imu_times, imu_vx_filt


def filter_radar_velocity(radar_times, radar_vx):
    """Apply low-pass filter to radar velocity."""
    fs_radar = len(radar_times) / (radar_times[-1] - radar_times[0])
    
    if fs_radar > 2 * RADAR_LOWPASS_CUTOFF:
        b_lp, a_lp = butter(2, RADAR_LOWPASS_CUTOFF, btype='low', fs=fs_radar)
        radar_vx_filt = filtfilt(b_lp, a_lp, radar_vx)
    else:
        radar_vx_filt = radar_vx.copy()
    
    return radar_vx_filt


def compute_correlation(imu_times, imu_vx_filt, radar_times, radar_vx_filt):
    """Compute correlation between IMU and radar velocity."""
    radar_vx_interp = np.interp(imu_times, radar_times, radar_vx_filt)
    correlation = np.corrcoef(imu_vx_filt, radar_vx_interp)[0, 1]
    rms_error = np.sqrt(np.mean((imu_vx_filt - radar_vx_interp)**2))
    return correlation, rms_error


def main():
    print("="*60)
    print("TI FILTERED vs RAW DATA COMPARISON")
    print("="*60)
    
    # Load data using rosbag_loader
    print(f"\nLoading bag: {BAG_PATH}")
    data = rosbag_loader.load_bag_topics(BAG_PATH, verbose=False)
    
    t_start = data.start_time + START_TIME_OFFSET
    t_end = t_start + DURATION
    
    print(f"Analysis window: {t_start:.2f} - {t_end:.2f} s")
    
    # Filter data
    data.imu_data = [imu for imu in data.imu_data if t_start <= imu.timestamp <= t_end]
    data.radar_velocity = [r for r in data.radar_velocity if t_start <= r.timestamp <= t_end]
    
    print(f"\nIMU samples: {len(data.imu_data)}")
    print(f"Raw radar frames: {len(data.radar_velocity)}")
    
    # Process IMU
    imu_times, imu_vx_filt = process_imu_data(data.imu_data)
    
    # === Test 1: Our raw data ===
    print("\n" + "="*60)
    print("TEST 1: RAW DATA (from /mmWaveDataHdl/RScanVelocity)")
    print("="*60)
    
    radar_times_raw = []
    radar_vx_raw = []
    
    for frame in data.radar_velocity:
        if frame.velocities is None or frame.intensities is None:
            continue
        
        v_body = solve_ego_velocity_weighted(
            frame.positions,
            frame.velocities,
            frame.intensities
        )
        
        if v_body is not None:
            radar_times_raw.append(frame.timestamp)
            radar_vx_raw.append(v_body[0])
    
    radar_times_raw = np.array(radar_times_raw)
    radar_vx_raw = np.array(radar_vx_raw)
    
    print(f"Frames processed: {len(radar_times_raw)}")
    print(f"Vx range: [{radar_vx_raw.min():.2f}, {radar_vx_raw.max():.2f}] m/s")
    
    radar_vx_raw_filt = filter_radar_velocity(radar_times_raw, radar_vx_raw)
    corr_raw, rms_raw = compute_correlation(imu_times, imu_vx_filt, radar_times_raw, radar_vx_raw_filt)
    
    print(f"Correlation with IMU: {corr_raw:.4f}")
    print(f"RMS error: {rms_raw:.3f} m/s")
    
    # === Test 2: TI filtered data ===
    print("\n" + "="*60)
    print("TEST 2: TI FILTERED (from /ti_mmwave/radar_scan_pcl_0)")
    print("="*60)
    
    radar_times_ti, radar_vx_ti, _, _ = load_ti_filtered_data(BAG_PATH, t_start, t_end)
    
    print(f"Frames processed: {len(radar_times_ti)}")
    
    if len(radar_times_ti) > 0:
        print(f"Vx range: [{radar_vx_ti.min():.2f}, {radar_vx_ti.max():.2f}] m/s")
        
        radar_vx_ti_filt = filter_radar_velocity(radar_times_ti, radar_vx_ti)
        corr_ti, rms_ti = compute_correlation(imu_times, imu_vx_filt, radar_times_ti, radar_vx_ti_filt)
        
        print(f"Correlation with IMU: {corr_ti:.4f}")
        print(f"RMS error: {rms_ti:.3f} m/s")
        
        # === Comparison ===
        print("\n" + "="*60)
        print("COMPARISON SUMMARY")
        print("="*60)
        print(f"\nRAW DATA:")
        print(f"  Frames: {len(radar_times_raw)}")
        print(f"  Correlation: {corr_raw:.4f}")
        print(f"  RMS error: {rms_raw:.3f} m/s")
        
        print(f"\nTI FILTERED:")
        print(f"  Frames: {len(radar_times_ti)}")
        print(f"  Correlation: {corr_ti:.4f}")
        print(f"  RMS error: {rms_ti:.3f} m/s")
        
        print(f"\nIMPROVEMENT:")
        print(f"  Δ Correlation: {corr_ti - corr_raw:+.4f} ({(corr_ti - corr_raw)/corr_raw*100:+.2f}%)")
        print(f"  Δ RMS error: {rms_ti - rms_raw:+.3f} m/s")
        
        if corr_ti > corr_raw + 0.01:
            print("\n✅ TI filtering SIGNIFICANTLY improves results!")
            print("   Recommendation: Use TI's filtered points for odometry")
        elif corr_ti > corr_raw:
            print("\n🟡 TI filtering slightly improves results")
            print("   Recommendation: Minor gain, either approach works")
        else:
            print("\n❌ TI filtering does NOT improve results")
            print("   Recommendation: Stick with raw data + CPU timestamps")
    else:
        print("❌ ERROR: No TI filtered data found!")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
