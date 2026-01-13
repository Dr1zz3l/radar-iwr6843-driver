"""
Radar velocity estimation and signal processing utilities.

This module provides functions for:
- Weighted least squares ego-velocity estimation from radar point clouds
- Signal filtering (highpass, lowpass)
- IMU acceleration integration
"""

import numpy as np
from scipy.signal import butter, filtfilt, detrend
from scipy.integrate import cumulative_trapezoid


def solve_ego_velocity_weighted(positions, velocities, intensities, 
                                  min_intensity=5.0, min_range=0.2, min_points=5):
    """
    Solve for 3D body velocity using Weighted Least Squares.
    
    Minimizes: Σ w_i (r̂_i · v - v_rad,i)²
    where w_i = intensity_i (trust brighter returns more)
    
    Args:
        positions: Array of shape (N, 3) with [x, y, z] positions
        velocities: Array of shape (N,) with radial velocities
        intensities: Array of shape (N,) with signal intensities
        min_intensity: Minimum intensity threshold
        min_range: Minimum range threshold (meters)
        min_points: Minimum number of points required
        
    Returns:
        v_body: 3D velocity vector [vx, vy, vz] or None if insufficient data
    """
    H = []
    z = []
    weights = []
    
    for i in range(len(positions)):
        x, y, z_coord = positions[i]
        v_rad = velocities[i]
        intensity = intensities[i]
        
        r = np.sqrt(x**2 + y**2 + z_coord**2)
        
        # Filter weak/close returns
        if intensity < min_intensity or r < min_range:
            continue
        
        # Unit direction vector
        dir_vec = np.array([x/r, y/r, z_coord/r])
        
        H.append(dir_vec)
        z.append(v_rad)
        weights.append(intensity)
    
    if len(z) < min_points:
        return None
    
    H = np.array(H)
    z = np.array(z)
    W = np.diag(weights)
    
    try:
        # Weighted Least Squares: (H^T W H)^-1 H^T W z
        lhs = H.T @ W @ H
        rhs = H.T @ W @ z
        v_body = np.linalg.solve(lhs, rhs)
        return v_body
    except np.linalg.LinAlgError:
        return None


def process_radar_frames(radar_frames, min_intensity=5.0, min_range=0.2, min_points=5):
    """
    Process all radar frames to extract ego-velocity estimates.
    
    Args:
        radar_frames: List of RadarVelocityFrame objects
        min_intensity: Minimum intensity threshold
        min_range: Minimum range threshold (meters)
        min_points: Minimum number of points required
        
    Returns:
        Dictionary with arrays:
            'times_ros': ROS timestamps
            'cpu_cycles': CPU cycle counts
            'vx', 'vy', 'vz': Velocity components
    """
    times_ros = []
    cpu_cycles = []
    vx_list = []
    vy_list = []
    vz_list = []
    
    for frame in radar_frames:
        if frame.velocities is None or frame.intensities is None:
            continue
        
        v_body = solve_ego_velocity_weighted(
            frame.positions,
            frame.velocities,
            frame.intensities,
            min_intensity=min_intensity,
            min_range=min_range,
            min_points=min_points
        )
        
        if v_body is not None:
            times_ros.append(frame.timestamp)
            vx_list.append(v_body[0])
            vy_list.append(v_body[1])
            vz_list.append(v_body[2])
            
            if frame.time_cpu_cycles is not None and len(frame.time_cpu_cycles) > 0:
                cpu_cycles.append(frame.time_cpu_cycles[0])
            else:
                cpu_cycles.append(0)
    
    return {
        'times_ros': np.array(times_ros),
        'cpu_cycles': np.array(cpu_cycles),
        'vx': np.array(vx_list),
        'vy': np.array(vy_list),
        'vz': np.array(vz_list)
    }


def integrate_imu_acceleration(imu_data, axis='x'):
    """
    Integrate IMU acceleration to velocity with detrending.
    
    This method integrates raw acceleration and then applies linear detrending
    to the velocity. This is more robust than pre-removing bias for oscillatory
    motion, as it handles both:
    - Integration constant (initial velocity offset)
    - Constant acceleration bias (manifests as linear drift in velocity)
    
    Args:
        imu_data: List of IMUData objects
        axis: Axis to integrate ('x', 'y', or 'z')
        
    Returns:
        Dictionary with:
            'times': Timestamps
            'acceleration': Raw acceleration
            'velocity_raw': Integrated and detrended velocity
            'bias': 0.0 (bias implicitly handled by detrending)
    """
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    idx = axis_map[axis.lower()]
    
    times = np.array([imu.timestamp for imu in imu_data])
    accel = np.array([imu.linear_acceleration[idx] for imu in imu_data])
    
    # Integrate raw acceleration immediately (no pre-bias removal)
    velocity_integrated = cumulative_trapezoid(accel, times, initial=0.0)
    
    # Linear detrend on velocity removes:
    # - Integration constant (initial velocity offset)
    # - Linear trend from constant acceleration bias
    velocity_raw = detrend(velocity_integrated, type='linear')
    
    return {
        'times': times,
        'acceleration': accel,
        'velocity_raw': velocity_raw,
        'bias': 0.0,  # Bias implicitly handled by detrending
    }


def apply_highpass_filter(signal, cutoff_hz, sample_rate_hz, order=2):
    """
    Apply highpass Butterworth filter to remove low-frequency drift.
    
    Args:
        signal: Input signal
        cutoff_hz: Cutoff frequency in Hz
        sample_rate_hz: Sample rate in Hz
        order: Filter order (default: 2)
        
    Returns:
        Filtered signal
    """
    b, a = butter(order, cutoff_hz, btype='high', fs=sample_rate_hz)
    return filtfilt(b, a, signal)


def apply_lowpass_filter(signal, cutoff_hz, sample_rate_hz, order=2):
    """
    Apply lowpass Butterworth filter to reduce high-frequency noise.
    
    Args:
        signal: Input signal
        cutoff_hz: Cutoff frequency in Hz
        sample_rate_hz: Sample rate in Hz
        order: Filter order (default: 2)
        
    Returns:
        Filtered signal
    """
    b, a = butter(order, cutoff_hz, btype='low', fs=sample_rate_hz)
    return filtfilt(b, a, signal)


def filter_valid_cpu_cycles(data_dict):
    """
    Filter out entries with invalid (zero) CPU cycle counts.
    
    Args:
        data_dict: Dictionary with 'cpu_cycles' and other aligned arrays
        
    Returns:
        Filtered dictionary with only valid entries
    """
    valid_mask = data_dict['cpu_cycles'] > 0
    
    return {
        key: value[valid_mask] if isinstance(value, np.ndarray) else value
        for key, value in data_dict.items()
    }
