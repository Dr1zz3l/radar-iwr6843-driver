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
from scipy.optimize import least_squares


def solve_ego_velocity_weighted(positions, velocities, intensities, 
                                  min_intensity=5.0, min_range=0.2, min_points=5,
                                  use_huber=False, huber_delta=1.0):
    """
    Solve for 3D body velocity using Weighted Least Squares or robust Huber loss.
    
    Standard WLS minimizes: Σ w_i (r̂_i · v - v_rad,i)²
    Huber loss minimizes: Σ w_i * ρ((r̂_i · v - v_rad,i) / δ)
    where ρ(x) = x² for |x|≤1, 2|x|-1 for |x|>1
    
    Args:
        positions: Array of shape (N, 3) with [x, y, z] positions
        velocities: Array of shape (N,) with radial velocities
        intensities: Array of shape (N,) with signal intensities
        min_intensity: Minimum intensity threshold
        min_range: Minimum range threshold (meters)
        min_points: Minimum number of points required (still needed for Huber!)
        use_huber: If True, use Huber loss instead of L2
        huber_delta: Huber loss threshold parameter (m/s)
        
    Returns:
        v_body: 3D velocity vector [vx, vy, vz] or None if insufficient data
        
    Note:
        min_points is still required with Huber loss because we need at least
        3 points to solve for 3D velocity (more for numerical stability).
        Huber loss handles outliers better but doesn't eliminate the need for
        sufficient measurements.
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
    weights = np.array(weights)
    
    if use_huber:
        # Robust estimation using Huber loss
        def residual_func(v):
            residuals = H @ v - z
            # Weight residuals by intensity
            weighted_residuals = np.sqrt(weights) * residuals
            return weighted_residuals
        
        try:
            # Initial guess from standard WLS
            W = np.diag(weights)
            lhs = H.T @ W @ H
            rhs = H.T @ W @ z
            v_init = np.linalg.solve(lhs, rhs)
            
            # Optimize with Huber loss
            result = least_squares(
                residual_func, 
                v_init, 
                loss='huber',
                f_scale=huber_delta,
                method='trf'
            )
            return result.x
        except (np.linalg.LinAlgError, ValueError):
            return None
    else:
        # Standard Weighted Least Squares
        W = np.diag(weights)
        
        try:
            # Weighted Least Squares: (H^T W H)^-1 H^T W z
            lhs = H.T @ W @ H
            rhs = H.T @ W @ z
            v_body = np.linalg.solve(lhs, rhs)
            return v_body
        except np.linalg.LinAlgError:
            return None


def process_radar_frames(radar_frames, min_intensity=5.0, min_range=0.2, min_points=5,
                         use_huber=False, huber_delta=1.0):
    """
    Process all radar frames to extract ego-velocity estimates.
    
    Args:
        radar_frames: List of RadarVelocityFrame objects
        min_intensity: Minimum intensity threshold
        min_range: Minimum range threshold (meters)
        min_points: Minimum number of points required
        use_huber: If True, use Huber loss instead of L2
        huber_delta: Huber loss threshold parameter (m/s)
        
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
            min_points=min_points,
            use_huber=use_huber,
            huber_delta=huber_delta
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


def find_time_shift(t_reference, v_reference, t_sensor, v_sensor, 
                    search_window=(-0.2, 0.2), crop_duration=1.0):
    """
    Find optimal time shift between two velocity signals.
    
    Finds the time shift 'dt' such that v_sensor(t + dt) ~ v_reference(t).
    A positive dt means the sensor data is DELAYED (arrives late).
    
    Args:
        t_reference: Reference timestamps (e.g., IMU)
        v_reference: Reference velocity signal
        t_sensor: Sensor timestamps (e.g., radar)
        v_sensor: Sensor velocity signal
        search_window: Search range for time shift in seconds (min, max)
        crop_duration: Duration in seconds to crop from start/end (0 or False to disable)
        
    Returns:
        Tuple of (optimal_dt, min_rmse, correlation_at_optimum)
    """
    from scipy.interpolate import interp1d
    from scipy.optimize import minimize_scalar
    
    # Create interpolator for sensor data
    sensor_interp = interp1d(t_sensor, v_sensor, kind='linear', 
                            fill_value="extrapolate", bounds_error=False)
    
    # Compute crop indices (0 if cropping disabled)
    if crop_duration:
        sample_rate = len(t_reference) / (t_reference[-1] - t_reference[0])
        crop_samples = int(crop_duration * sample_rate)
    else:
        crop_samples = 0
    
    # Crop edges to avoid filter artifacts (or use full signal if crop_samples=0)
    if crop_samples > 0:
        t_ref_cropped = t_reference[crop_samples:-crop_samples]
        v_ref_cropped = v_reference[crop_samples:-crop_samples]
    else:
        t_ref_cropped = t_reference
        v_ref_cropped = v_reference
    
    # Define cost function (RMSE)
    def cost_function(dt):
        # Query sensor at (t_reference - dt)
        # If dt is positive (delay), we look BACK in sensor time
        v_sensor_shifted = sensor_interp(t_ref_cropped - dt)
        
        # Calculate RMSE
        err = v_ref_cropped - v_sensor_shifted
        return np.sqrt(np.mean(err**2))
    
    # Optimize with higher precision
    result = minimize_scalar(
        cost_function, 
        bounds=search_window, 
        method='bounded',
        options={'xatol': 1e-6}  # Aim for microsecond precision
    )
    
    # Compute correlation at optimum
    v_sensor_optimal = sensor_interp(t_ref_cropped - result.x)
    correlation = np.corrcoef(v_ref_cropped, v_sensor_optimal)[0, 1]
    
    return result.x, result.fun, correlation


def compute_alignment_metrics(t_reference, v_reference, t_sensor, v_sensor, dt_shift, crop_duration=1.0):
    """
    Compute alignment quality metrics after applying time shift.
    
    Args:
        t_reference: Reference timestamps
        v_reference: Reference velocity
        t_sensor: Sensor timestamps
        v_sensor: Sensor velocity
        dt_shift: Time shift to apply (positive = sensor delayed)
        crop_duration: Duration in seconds to crop from edges (0 or False to disable)
        
    Returns:
        Dictionary with rmse, correlation, residuals, aligned signal, and crop_samples
    """
    from scipy.interpolate import interp1d
    
    # Apply shift and interpolate
    sensor_interp = interp1d(t_sensor, v_sensor, kind='linear', 
                            fill_value="extrapolate", bounds_error=False)
    v_sensor_aligned = sensor_interp(t_reference - dt_shift)
    
    # Compute crop indices (0 if cropping disabled)
    if crop_duration:
        sample_rate = len(t_reference) / (t_reference[-1] - t_reference[0])
        crop_samples = int(crop_duration * sample_rate)
    else:
        crop_samples = 0
    
    # Compute metrics on cropped data (or full data if crop_samples=0)
    if crop_samples > 0:
        v_ref_cropped = v_reference[crop_samples:-crop_samples]
        v_sensor_cropped = v_sensor_aligned[crop_samples:-crop_samples]
    else:
        v_ref_cropped = v_reference
        v_sensor_cropped = v_sensor_aligned
    
    residuals_cropped = v_ref_cropped - v_sensor_cropped
    rmse = np.sqrt(np.mean(residuals_cropped**2))
    correlation = np.corrcoef(v_ref_cropped, v_sensor_cropped)[0, 1]
    
    # Full residuals for plotting
    residuals_full = v_reference - v_sensor_aligned
    
    return {
        'rmse': rmse,
        'correlation': correlation,
        'residuals': residuals_full,
        'v_sensor_aligned': v_sensor_aligned,
        'crop_samples': crop_samples
    }
